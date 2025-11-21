#!/usr/bin/env python3
from __future__ import annotations

import threading
from typing import Any, Dict, List, Optional

from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import TransitionCallbackReturn
from sensor_msgs.msg import Image, JointState

from .base import PolicyConfig
from .registry import make_policy


class SO101PolicyRunner(LifecycleNode):
    """Lifecycle node for running a LeRobot policy with separate
    inference and publish timers.
    """

    JOINT_NAMES = [
        'shoulder_pan',
        'shoulder_lift',
        'elbow_flex',
        'wrist_flex',
        'wrist_roll',
        'gripper',
    ]

    def __init__(self) -> None:
        super().__init__('policy_runner')

        # Latest synced observation
        self._latest_msgs: Optional[Dict[str, Any]] = None
        self._obs_lock = threading.Lock()

        # Action buffer
        self._action_buffer: List[List[float]] = []
        self._action_index: int = 0
        self._action_lock = threading.Lock()

        # Velocity state
        self._last_filtered_velocity: Optional[List[float]] = None
        self._last_cmd_position: Optional[List[float]] = None

        # Policy
        self._policy = None

        # Rates
        self._inference_rate: float = 5.0
        self._publish_rate: float = 25.0
        self._inference_period: float = 1.0 / self._inference_rate
        self._publish_period: float = 1.0 / self._publish_rate
        self._action0_index: int = 7
        self._vel_alpha: float = 0.2

        # Timers
        self._inference_timer = None
        self._publish_timer = None

        # ROS I/O
        self._sub_cam1 = None
        self._sub_cam2 = None
        self._sub_joints = None
        self._sync = None
        self._cmd_pub = None

        self._cb_group = ReentrantCallbackGroup()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring policy_runner...')

        # Core parameters
        self.declare_parameter('policy_name', 'smolvla')
        self.declare_parameter(
            'checkpoint_path',
            '/home/nimrod/005000/pretrained_model',
        )
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter(
            'task',
            'First, identify the cube position. Then, reach out and grasp the cube. Finally, move the cube and release it into the bowl',
        )

        # Sync + timing
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.02)
        self.declare_parameter('inference_rate', 1.0)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('action0_index', 30)
        self.declare_parameter('velocity_lowpass_alpha', 0.2)

        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        slop = self.get_parameter('slop').get_parameter_value().double_value
        policy_name = self.get_parameter('policy_name').get_parameter_value().string_value
        self._inference_rate = (
            self.get_parameter('inference_rate').get_parameter_value().double_value
        )
        self._publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._action0_index = (
            self.get_parameter('action0_index').get_parameter_value().integer_value
        )
        self._vel_alpha = (
            self.get_parameter('velocity_lowpass_alpha').get_parameter_value().double_value
        )

        self._inference_period = 1.0 / max(self._inference_rate, 1e-3)
        self._publish_period = 1.0 / max(self._publish_rate, 1e-3)

        self.get_logger().info(
            f'Timing config: inference_rate={self._inference_rate:.2f} Hz '
            f'(period={self._inference_period:.3f}s), '
            f'publish_rate={self._publish_rate:.2f} Hz '
            f'(period={self._publish_period:.3f}s)'
        )

        # Instantiate policy
        cfg = PolicyConfig.from_ros_params(self)
        print('DEBUG: PolicyConfig:', cfg)
        self._policy = make_policy(policy_name, cfg, self)
        self.get_logger().info(
            f"Using policy '{cfg.policy_name}' on device '{cfg.device}', "
            f"checkpoint='{cfg.checkpoint_path}'"
        )

        # Subscribers + approximate time sync
        self._sub_cam1 = Subscriber(
            self,
            Image,
            '/follower/cam_front/image_raw',
        )
        self._sub_cam2 = Subscriber(
            self,
            Image,
            '/static_camera/cam_side/color/image_raw',
        )
        self._sub_joints = Subscriber(
            self,
            JointState,
            '/follower/joint_states',
        )

        self._sync = ApproximateTimeSynchronizer(
            [self._sub_cam1, self._sub_cam2, self._sub_joints],
            queue_size=queue_size,
            slop=slop,
        )
        self._sync.registerCallback(self._synced_cb)

        self._cmd_pub = self.create_publisher(
            JointState,
            '/leader/joint_states',
            10,
        )

        self.get_logger().info(f'Configured sync with queue_size={queue_size}, slop={slop}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Activating policy_runner ...')

        self._inference_timer = self.create_timer(
            self._inference_period, self._inference_step, self._cb_group
        )

        self._publish_timer = self.create_timer(
            self._publish_period, self._publish_step, self._cb_group
        )

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating policy_runner ...')

        if self._inference_timer is not None:
            self._inference_timer.cancel()
            self._inference_timer = None

        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self._publish_timer = None

        with self._action_lock:
            self._action_buffer = []
            self._action_index = 0

        self._send_safe_stop()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up policy_runner...')
        self._sync = None
        self._sub_cam1 = None
        self._sub_cam2 = None
        self._sub_joints = None
        self._cmd_pub = None
        self._policy = None

        with self._obs_lock:
            self._latest_msgs = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down policy_runner...')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _synced_cb(
        self,
        cam1: Image,
        cam2: Image,
        joint_state: JointState,
    ) -> None:
        """ApproximateTimeSynchronizer callback that updates latest observation."""

        with self._obs_lock:
            self._latest_msgs = {
                'observation.images.camera1': cam1,
                'observation.images.camera2': cam2,
                'observation.state': joint_state,
            }

    def _publish_step(self) -> None:
        """Publish next JointState command at publish_rate Hz."""
        if self._cmd_pub is None:
            return

        # 1) Get next joint position vector from the policy
        action = self._policy.get_action() if self._policy is not None else None
        if action is None:
            self.get_logger().warn('[PUBLISH] No action available from policy.')
            self._send_safe_stop()
            return

        current_pos = list(action)
        n = len(current_pos)
        dt = self._publish_period

        # 2) Compute velocities from commanded positions
        if self._last_cmd_position is None or len(self._last_cmd_position) != n:
            raw_vel = [0.0] * n
        else:
            raw_vel = [(current_pos[i] - self._last_cmd_position[i]) / dt for i in range(n)]

        if self._last_filtered_velocity is None or len(self._last_filtered_velocity) != n:
            filt_vel = raw_vel
        else:
            alpha = self._vel_alpha
            prev_filt = self._last_filtered_velocity
            filt_vel = [alpha * raw_vel[i] + (1.0 - alpha) * prev_filt[i] for i in range(n)]

        self._last_cmd_position = current_pos
        self._last_filtered_velocity = filt_vel

        # 3) Use fixed JOINT_NAMES or infer from last observation; here we keep JOINT_NAMES
        js_cmd = JointState()
        js_cmd.header.stamp = self.get_clock().now().to_msg()
        js_cmd.name = self.JOINT_NAMES[:n]
        js_cmd.position = current_pos
        # js_cmd.velocity = filt_vel  # enable once ready

        self._cmd_pub.publish(js_cmd)

    def _inference_step(self) -> None:
        """Timer: ask the policy to refresh its internal action queue/buffer."""
        if self._policy is None:
            self.get_logger().warn('Inference step: no policy instantiated.')
            return

        with self._obs_lock:
            ros_obs = self._latest_msgs

        if ros_obs is None:
            self.get_logger().warn('Inference step: no observation received yet.')
            return

        # The policy handles RTC vs simple buffering internally
        self._policy.infer(ros_obs=ros_obs, time_per_action=self._publish_period)

    def _send_safe_stop(self) -> None:
        """Send a safe stop command."""

        if self._cmd_pub is None:
            return

        with self._obs_lock:
            msgs = self._latest_msgs

        if msgs is None:
            self.get_logger().warn('Safe stop: no last observation, no command published.')
            return

        last_joint_state: JointState = msgs['observation.state']
        positions = list(last_joint_state.position)
        if not positions:
            self.get_logger().warn(
                'Safe stop: last JointState has no positions, no command published.'
            )
            return

        js_cmd = JointState()
        js_cmd.header.stamp = self.get_clock().now().to_msg()
        js_cmd.name = list(last_joint_state.name)
        js_cmd.position = positions

        self._cmd_pub.publish(js_cmd)
        # Throttle logging slightly
        self.get_logger().warn('Safe stop: holding last observed joint positions.')
