#!/usr/bin/env python3
from __future__ import annotations

import threading
from re import A
from typing import List, Optional, Tuple

from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import TransitionCallbackReturn
from sensor_msgs.msg import Image, JointState

# For now, do NOT instantiate any policy
from .base import PolicyConfig
from .registry import make_policy


class SO101PolicyRunner(LifecycleNode):
    """Lifecycle node to debug ApproximateTimeSynchronizer + synced_cb only."""

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

        # Latest synced observation: (cam_front, cam_side, joint_state)
        self._latest_msgs: Optional[Tuple[Image, Image, JointState]] = None
        self._obs_lock = threading.Lock()

        self._action_buffer: List[List[float]] = []
        self._action_index: int = 0
        self._action_lock = threading.Lock()

        self._last_filtered_velocity: Optional[List[float]] = None
        self._last_cmd_position: Optional[List[float]] = None

        self._policy = None

        # self._inference_rate: float = 5.0
        self._publish_rate: float = 25.0
        # self._inference_period: float = 0.2
        self._publish_period: float = 1 / self._publish_rate
        # self._action0_index: int = 0
        self._vel_alpha: float = 0.2

        # self._inference_timer = None
        self._publish_timer = None

        # ROS I/O
        self._sub_cam1 = None
        self._sub_cam2 = None
        self._sub_joints = None
        self._sync = None

        # For now we don't publish anything
        self._cmd_pub = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def on_configure(self, state) -> TransitionCallbackReturn:
        """Configure subscribers and approximate time sync only."""
        self.get_logger().info('Configuring policy_runner (SYNC DEBUG MODE)...')

        # Parameters only needed for sync
        self.declare_parameter('policy_name', 'smolvla')
        self.declare_parameter(
            'checkpoint_path',
            '/home/nimrod/.cache/huggingface/lerobot/nimiCurtis/train/isaac_pick_and_place_train',
        )
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('task', 'Pick and Place')

        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.02)
        self.declare_parameter('inference_rate', 5.0)
        self.declare_parameter('publish_rate', 25.0)
        self.declare_parameter('action0_index', 0)
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

        # Instantiate policy from registry using ROS params
        cfg = PolicyConfig.from_ros_params(self)

        print('DEBUG: PolicyConfig:', cfg)

        self._policy = make_policy(policy_name, cfg, self)
        self.get_logger().info(
            f"Using policy '{cfg.policy_name}' on device '{cfg.device}', "
            f"checkpoint='{cfg.checkpoint_path}'"
        )

        # Subscribers + approximate time sync
        self._sub_cam1 = Subscriber(self, Image, '/follower/cam_front/image_raw')
        self._sub_cam2 = Subscriber(self, Image, '/static_camera/cam_side/color/image_raw')
        self._sub_joints = Subscriber(self, JointState, '/follower/joint_states')

        self._sync = ApproximateTimeSynchronizer(
            [self._sub_cam1, self._sub_cam2, self._sub_joints],
            queue_size=queue_size,
            slop=slop,
        )
        self._sync.registerCallback(self._synced_cb)

        # No publisher / timers for now
        self._cmd_pub = self.create_publisher(JointState, '/leader/joint_states', 10)

        self.get_logger().info(f'Configured sync with queue_size={queue_size}, slop={slop}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        """Nothing to start: just let synced_cb run."""
        self.get_logger().info('Activating policy_runner ...')

        # No timers in this phase
        # self._inference_timer = self.create_timer(...)

        self._publish_timer = self.create_timer(
            self._publish_period,
            self._publish_step,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        """Just log deactivation; no timers / commands."""
        self.get_logger().info('Deactivating policy_runner ...')

        # if self._inference_timer is not None:
        #     self._inference_timer.cancel()
        #     self._inference_timer = None
        #
        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self._publish_timer = None

        # Clear action buffer
        with self._action_lock:
            self._action_buffer = []
            self._action_index = 0

        # Send safe stop command
        self._send_safe_stop()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        """Cleanup resources."""
        self.get_logger().info('Cleaning up policy_runner (SYNC DEBUG MODE)...')

        self._sync = None
        self._sub_cam1 = None
        self._sub_cam2 = None
        self._sub_joints = None
        # self._cmd_pub = None
        # self._policy = None

        with self._obs_lock:
            self._latest_msgs = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        """Handle shutdown."""
        self.get_logger().info('Shutting down policy_runner (SYNC DEBUG MODE)...')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Callbacks (only synced_cb is active now)
    # ------------------------------------------------------------------
    def _synced_cb(
        self,
        img_top: Image,
        img_wrist: Image,
        joint_state: JointState,
    ) -> None:
        """ApproximateTimeSynchronizer callback that updates latest observation."""
        with self._obs_lock:
            self._latest_msgs = (img_top, img_wrist, joint_state)

    def _publish_step(self) -> None:
        """Publish next JointState command at publish_rate Hz.

        Positions are taken from the planned sequence.
        Velocities are derived from consecutive commanded positions and low-pass filtered.
        """
        if self._cmd_pub is None:
            return

        # Get current planned position
        with self._action_lock:
            if not self._action_buffer:
                self._send_safe_stop()
                return

            if self._action_index >= len(self._action_buffer):
                self._action_index = len(self._action_buffer) - 1

            current_pos = list(self._action_buffer[self._action_index])

            # Advance index for next tick (or hold last)
            if self._action_index < len(self._action_buffer) - 1:
                self._action_index += 1

        n = len(current_pos)
        dt = self._publish_period

        # Compute raw velocity from commanded positions only
        if self._last_cmd_position is None or len(self._last_cmd_position) != n:
            raw_vel = [0.0] * n
        else:
            raw_vel = [(current_pos[i] - self._last_cmd_position[i]) / dt for i in range(n)]

        # Low-pass filter: v_filt = alpha * v_raw + (1 - alpha) * v_prev_filt
        if self._last_filtered_velocity is None or len(self._last_filtered_velocity) != n:
            filt_vel = raw_vel
        else:
            alpha = self._vel_alpha
            prev_filt = self._last_filtered_velocity
            filt_vel = [alpha * raw_vel[i] + (1.0 - alpha) * prev_filt[i] for i in range(n)]

        self._last_cmd_position = current_pos
        self._last_filtered_velocity = filt_vel

        js_cmd = JointState()
        js_cmd.header.stamp = self.get_clock().now().to_msg()
        js_cmd.name = self.JOINT_NAMES
        js_cmd.position = current_pos
        js_cmd.velocity = filt_vel

        self._cmd_pub.publish(js_cmd)

    def _send_safe_stop(self) -> None:
        """Publish 'hold' JointState if we have a last observation; otherwise publish nothing."""
        if self._cmd_pub is None:
            return

        with self._obs_lock:
            msgs = self._latest_msgs

        if msgs is None:
            self.get_logger().warn('Safe stop: no last observation, no command published.')
            return

        _, _, last_joint_state = msgs
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
        self.get_logger().info('Safe stop: holding last observed joint positions.')


# #!/usr/bin/env python3
# from __future__ import annotations

# import threading
# from typing import List, Optional, Tuple


# from message_filters import ApproximateTimeSynchronizer, Subscriber

# from rclpy.lifecycle import LifecycleNode
# from rclpy.lifecycle.node import TransitionCallbackReturn
# from sensor_msgs.msg import Image, JointState

# from so101_ros2_bridge.policy import PolicyConfig, make_policy


# class PolicyRunner(LifecycleNode):
#     """Lifecycle node that runs a sequence-based policy with ROS 2 I/O.

#     - Syncs 2x Image + JointState via ApproximateTimeSynchronizer.
#     - Inference timer: computes a sequence of future joint positions.
#     - Publish timer: streams JointState commands from that sequence.
#     - Velocity command is derived from commanded positions, with low-pass filter.
#     """

#     def __init__(self) -> None:
#         super().__init__('policy_runner')

#         # Latest synced observation: (top_image, wrist_image, joint_state)
#         self._latest_msgs: Optional[Tuple[Image, Image, JointState]] = None
#         self._obs_lock = threading.Lock()

#         # Planned action buffer: list of joint position vectors [T, n_joints]
#         self._action_buffer: List[List[float]] = []
#         self._action_index: int = 0
#         self._action_lock = threading.Lock()

#         # Velocity-related state
#         self._last_filtered_velocity: Optional[List[float]] = None
#         self._last_cmd_position: Optional[List[float]] = None

#         # Policy
#         self._policy = None  # type: ignore

#         # Rates / periods
#         self._inference_rate: float = 5.0
#         self._publish_rate: float = 25.0
#         self._inference_period: float = 0.2
#         self._publish_period: float = 0.04
#         self._action0_index: int = 0
#         self._vel_alpha: float = 0.2  # low-pass factor in [0, 1]

#         # Timers
#         self._inference_timer = None
#         self._publish_timer = None

#         # ROS I/O
#         self._sub_cam1 = None
#         self._sub_cam2 = None
#         self._sub_joints = None
#         self._sync = None
#         self._cmd_pub = None

#     # ------------------------------------------------------------------
#     # Lifecycle
#     # ------------------------------------------------------------------
#     def on_configure(self, state) -> TransitionCallbackReturn:
#         """Configure subscribers, sync, publisher and policy adapter."""
#         self.get_logger().info('Configuring policy_runner...')

#         # Shared parameters (overridden by YAML)
#         self.declare_parameter('policy_name', 'smolvla')
#         self.declare_parameter('inference_rate', 5.0)
#         self.declare_parameter('publish_rate', 25.0)
#         self.declare_parameter('queue_size', 10)
#         self.declare_parameter('slop', 0.05)
#         self.declare_parameter('action0_index', 0)
#         self.declare_parameter('velocity_lowpass_alpha', 0.2)

#         policy_name = self.get_parameter('policy_name').get_parameter_value().string_value
#         self._inference_rate = (
#             self.get_parameter('inference_rate').get_parameter_value().double_value
#         )
#         self._publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
#         queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
#         slop = self.get_parameter('slop').get_parameter_value().double_value
#         self._action0_index = (
#             self.get_parameter('action0_index').get_parameter_value().integer_value
#         )
#         self._vel_alpha = (
#             self.get_parameter('velocity_lowpass_alpha').get_parameter_value().double_value
#         )

#         self._inference_period = 1.0 / max(self._inference_rate, 1e-3)
#         self._publish_period = 1.0 / max(self._publish_rate, 1e-3)

#         # Instantiate adapter (plug your own config & factory here)
#         # Instantiate policy from registry using ROS params
#         cfg = PolicyConfig.from_ros_params(self)
#         self._policy = make_policy(policy_name, cfg, self)
#         self.get_logger().info(
#             f"Using policy '{cfg.policy_name}' on device '{cfg.device}', "
#             f"checkpoint='{cfg.checkpoint_path}'"
#         )

#         # Subscribers + approximate time sync
#         self._sub_cam1 = Subscriber(self, Image, '/follower/cam_front/image_raw')
#         self._sub_cam2 = Subscriber(self, Image, '/static_camera/image_raw/cam_side/image_raw')
#         self._sub_joints = Subscriber(self, JointState, '/follower/joint_states')

#         self._sync = ApproximateTimeSynchronizer(
#             [self._sub_cam1, self._sub_cam2, self._sub_joints],
#             queue_size=queue_size,
#             slop=slop,
#         )
#         self._sync.registerCallback(self._synced_cb)

#         # Publisher: JointState command for follower
#         self._cmd_pub = self.create_publisher(
#             JointState,
#             '/leader/joint_states',
#             10,
#         )

#         self.get_logger().info(
#             f'Configured: inference_rate={self._inference_rate} Hz, '
#             f'publish_rate={self._publish_rate} Hz, '
#             f'action0_index={self._action0_index}, '
#             f'vel_lowpass_alpha={self._vel_alpha}'
#         )
#         return TransitionCallbackReturn.SUCCESS

#     def on_activate(self, state) -> TransitionCallbackReturn:
#         """Start inference and publish timers."""
#         self.get_logger().info('Activating policy_runner...')

#         self._inference_timer = self.create_timer(
#             self._inference_period,
#             self._inference_step,
#         )
#         self._publish_timer = self.create_timer(
#             self._publish_period,
#             self._publish_step,
#         )

#         # if self._policy is not None: ??
#         #     self._policy.reset()

#         return TransitionCallbackReturn.SUCCESS

#     def on_deactivate(self, state) -> TransitionCallbackReturn:
#         """Stop timers and send a safe stop command."""
#         self.get_logger().info('Deactivating policy_runner...')

#         if self._inference_timer is not None:
#             self._inference_timer.cancel()
#             self._inference_timer = None

#         if self._publish_timer is not None:
#             self._publish_timer.cancel()
#             self._publish_timer = None

#         # Clear action buffer
#         with self._action_lock:
#             self._action_buffer = []
#             self._action_index = 0

#         # Send safe stop command
#         self._send_safe_stop()
#         return TransitionCallbackReturn.SUCCESS

#     def on_cleanup(self, state) -> TransitionCallbackReturn:
#         """Cleanup resources."""
#         self.get_logger().info('Cleaning up policy_runner...')

#         self._sync = None
#         self._sub_cam1 = None
#         self._sub_cam2 = None
#         self._sub_joints = None
#         self._cmd_pub = None
#         self._policy = None

#         with self._obs_lock:
#             self._latest_msgs = None
#         with self._action_lock:
#             self._action_buffer = []
#             self._action_index = 0

#         self._last_filtered_velocity = None
#         self._last_cmd_position = None

#         return TransitionCallbackReturn.SUCCESS

#     def on_shutdown(self, state) -> TransitionCallbackReturn:
#         """Handle shutdown."""
#         self.get_logger().info('Shutting down policy_runner...')
#         return TransitionCallbackReturn.SUCCESS

#     # ------------------------------------------------------------------
#     # Callbacks
#     # ------------------------------------------------------------------
#     def _synced_cb(
#         self,
#         img_top: Image,
#         img_wrist: Image,
#         joint_state: JointState,
#     ) -> None:
#         """ApproximateTimeSynchronizer callback that updates latest observation."""
#         with self._obs_lock:
#             self._latest_msgs = (img_top, img_wrist, joint_state)

#     def _inference_step(self) -> None:
#         """Run policy at inference_rate Hz to compute new future positions."""
#         if self._policy is None:
#             return

#         # Copy latest observation
#         with self._obs_lock:
#             msgs = self._latest_msgs

#         if msgs is None:
#             return

#         img_top, img_wrist, joint_state = msgs

#         obs = self._policy.make_observation(
#             img_top=img_top,
#             img_wrist=img_wrist,
#             joint_state=joint_state,
#         )

#         # Adapter returns a sequence of joint positions: List[List[float]] (T, n_joints)
#         action_seq = self._policy.act_sequence(obs)
#         if not action_seq:
#             return

#         start = max(0, self._action0_index)
#         if start >= len(action_seq):
#             start = len(action_seq) - 1

#         sliced_seq = action_seq[start:]

#         with self._action_lock:
#             self._action_buffer = sliced_seq
#             self._action_index = 0

#         # Optional: reset velocity filter when a new sequence arrives
#         self._last_filtered_velocity = None

#     def _publish_step(self) -> None:
#         """Publish next JointState command at publish_rate Hz.

#         Positions are taken from the planned sequence.
#         Velocities are derived from consecutive commanded positions and low-pass filtered.
#         """
#         if self._cmd_pub is None:
#             return

#         # Get current planned position
#         with self._action_lock:
#             if not self._action_buffer:
#                 self._send_safe_stop()
#                 return

#             if self._action_index >= len(self._action_buffer):
#                 self._action_index = len(self._action_buffer) - 1

#             current_pos = list(self._action_buffer[self._action_index])

#             # Advance index for next tick (or hold last)
#             if self._action_index < len(self._action_buffer) - 1:
#                 self._action_index += 1

#         n = len(current_pos)
#         dt = self._publish_period

#         # Compute raw velocity from commanded positions only
#         if self._last_cmd_position is None or len(self._last_cmd_position) != n:
#             raw_vel = [0.0] * n
#         else:
#             raw_vel = [(current_pos[i] - self._last_cmd_position[i]) / dt for i in range(n)]

#         # Low-pass filter: v_filt = alpha * v_raw + (1 - alpha) * v_prev_filt
#         if self._last_filtered_velocity is None or len(self._last_filtered_velocity) != n:
#             filt_vel = raw_vel
#         else:
#             alpha = self._vel_alpha
#             prev_filt = self._last_filtered_velocity
#             filt_vel = [alpha * raw_vel[i] + (1.0 - alpha) * prev_filt[i] for i in range(n)]

#         self._last_cmd_position = current_pos
#         self._last_filtered_velocity = filt_vel

#         # Get joint names from last observation
#         with self._obs_lock:
#             msgs = self._latest_msgs

#         if msgs is None:
#             # No observation → do not publish
#             return

#         _, _, last_joint_state = msgs
#         joint_names = list(last_joint_state.name)
#         if not joint_names:
#             return

#         # Align names length with command length
#         if len(joint_names) < n:
#             # Truncate positions as a safety fallback
#             current_pos = current_pos[: len(joint_names)]
#             filt_vel = filt_vel[: len(joint_names)]
#             n = len(joint_names)
#         else:
#             joint_names = joint_names[:n]

#         js_cmd = JointState()
#         js_cmd.header.stamp = self.get_clock().now().to_msg()
#         js_cmd.name = joint_names
#         js_cmd.position = current_pos
#         js_cmd.velocity = filt_vel

#         self._cmd_pub.publish(js_cmd)

#     def _send_safe_stop(self) -> None:
#         """Publish 'hold' JointState if we have a last observation; otherwise publish nothing."""
#         if self._cmd_pub is None:
#             return

#         with self._obs_lock:
#             msgs = self._latest_msgs

#         if msgs is None:
#             self.get_logger().warn('Safe stop: no last observation, no command published.')
#             return

#         _, _, last_joint_state = msgs
#         positions = list(last_joint_state.position)
#         if not positions:
#             self.get_logger().warn(
#                 'Safe stop: last JointState has no positions, no command published.'
#             )
#             return

#         js_cmd = JointState()
#         js_cmd.header.stamp = self.get_clock().now().to_msg()
#         js_cmd.name = list(last_joint_state.name)
#         js_cmd.position = positions
#         js_cmd.velocity = [0.0] * len(positions)

#         self._cmd_pub.publish(js_cmd)
#         self.get_logger().info('Safe stop: holding last observed joint positions.')
