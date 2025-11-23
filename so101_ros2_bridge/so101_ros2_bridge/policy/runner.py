#!/usr/bin/env python3
from __future__ import annotations

import threading
from importlib import import_module
from pathlib import Path
from typing import Any, Dict, List, Optional, Type

from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState  # for type hints in _send_safe_stop

from .base import PolicyConfig
from .registry import make_policy


class SO101PolicyRunner(LifecycleNode):
    """Lifecycle node for running a LeRobot policy with separate
    inference and publish timers.
    """

    ROBOT_PROPERTIES = {
        'robot_type': 'so101_follower',
        'joint_names': [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'gripper_finger',
        ],
    }

    def __init__(self) -> None:
        super().__init__('policy_runner')

        # Scalar params declared once (can still be overridden from YAML)
        self.declare_parameter('policy_name', 'smolvla')
        self.declare_parameter('checkpoint_path', '005000')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('task', '...')

        self.declare_parameter('inference_rate', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('inference_delay', 0.4)
        self.declare_parameter('use_delay_compensation', True)
        self.declare_parameter('sync.queue_size', 10)
        self.declare_parameter('sync.slop', 0.05)

        # --- Complex parameters (dicts) – allow dynamic typing ---
        dyn_desc = ParameterDescriptor(dynamic_typing=True)

        # IMPORTANT: use `None` here, not {}.
        self.declare_parameter('observations', None, descriptor=dyn_desc)
        self.declare_parameter('action', None, descriptor=dyn_desc)

        # Latest synced observation
        self._latest_msgs: Optional[Dict[str, Any]] = None
        self._obs_lock = threading.Lock()

        # Policy
        self._policy = None

        # Timers
        self._inference_timer = None
        self._publish_timer = None

        # ROS I/O
        self._sync = None
        self._cmd_pub = None
        self._cb_group = ReentrantCallbackGroup()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _resolve_msg_type(self, type_str: str) -> Type:
        """Convert e.g. 'sensor_msgs/msg/Image' -> sensor_msgs.msg.Image class."""
        pkg, submod, cls_name = type_str.split('/')
        module = import_module(f'{pkg}.{submod}')  # e.g. sensor_msgs.msg
        return getattr(module, cls_name)

    def _make_sync_cb(self, keys: List[str]):
        """Build a callback for ApproximateTimeSynchronizer that maps
        msgs -> self._latest_msgs using the observation keys.
        """

        def cb(*msgs):
            if len(msgs) != len(keys):
                self.get_logger().warn(
                    f'Sync callback got {len(msgs)} msgs but expected {len(keys)}'
                )
                return
            with self._obs_lock:
                self._latest_msgs = {k: m for k, m in zip(keys, msgs)}

        return cb

    def _build_config_from_params(self) -> PolicyConfig:
        """Read ROS params and construct a PolicyConfig via PolicyConfig.create."""
        policy_name = self.get_parameter('policy_name').value
        device = self.get_parameter('device').value
        checkpoint_path_str = self.get_parameter('checkpoint_path').value
        task = self.get_parameter('task').value

        # Start from default robot properties and allow overrides from YAML
        robot_properties: Dict[str, Any] = dict(self.ROBOT_PROPERTIES)

        # --- Optional state joint names from observations.observation.state.names ---
        obs_param: Parameter = self.get_parameter('observations')
        obs_cfg = obs_param.value or {}
        if isinstance(obs_cfg, dict):
            state_cfg = obs_cfg.get('observation.state')
            if isinstance(state_cfg, dict) and 'names' in state_cfg:
                state_names = list(state_cfg['names'])
                robot_properties['state_joint_names'] = state_names

        # --- Optional action joint names from action.names / action.action.names ---
        action_param: Parameter = self.get_parameter('action')
        action_cfg = action_param.value or {}

        if isinstance(action_cfg, dict):
            # Support both:
            # action: {topic: ..., msg_type: ..., names: [...]}
            # and:
            # action: {action: {topic:..., msg_type:..., names:[...]}}
            if 'topic' in action_cfg:
                action_entry = action_cfg
            else:
                _, action_entry = next(iter(action_cfg.items()), (None, {}))

            if isinstance(action_entry, dict) and 'names' in action_entry:
                action_names = list(action_entry['names'])
                robot_properties['action_joint_names'] = action_names

        cfg = PolicyConfig.create(
            policy_name=policy_name,
            device=device,
            checkpoint_path=Path(checkpoint_path_str),
            task=task,
            robot_properties=robot_properties,
        )
        return cfg

    # ---------- lifecycle ----------
    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring policy_runner...')

        queue_size = int(self.get_parameter('sync.queue_size').value)
        slop = float(self.get_parameter('sync.slop').value)

        self._inference_rate = float(self.get_parameter('inference_rate').value)
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._inference_delay = float(self.get_parameter('inference_delay').value)
        self._use_delay_compensation = bool(self.get_parameter('use_delay_compensation').value)

        self._inference_period = 1.0 / max(self._inference_rate, 1e-3)
        self._publish_period = 1.0 / max(self._publish_rate, 1e-3)

        self.get_logger().info(
            f'Timing config: inference_rate={self._inference_rate:.2f} Hz '
            f'(period={self._inference_period:.3f}s), '
            f'publish_rate={self._publish_rate:.2f} Hz '
            f'(period={self._publish_period:.3f}s), '
            f'inference_delay={self._inference_delay:.3f}s, '
            f'use_delay_compensation={self._use_delay_compensation}'
        )

        # --- Build PolicyConfig from ROS params + YAML ---
        cfg = self._build_config_from_params()

        # Action joint names
        self._action_joint_names = cfg.robot_properties.get(
            'action_joint_names',
            cfg.robot_properties.get('joint_names', []),
        )

        self.get_logger().info(f'DEBUG: PolicyConfig: {cfg}')
        self._policy = make_policy(cfg.policy_name, cfg, self)
        self.get_logger().info(
            f"Using policy '{cfg.policy_name}' on device '{cfg.device}', "
            f"checkpoint='{cfg.checkpoint_path}' with task prompt='{cfg.task}'"
        )

        # --- Build subscribers + sync generically from `observations` ---
        obs_param: Parameter = self.get_parameter('observations')
        obs_cfg = obs_param.value or {
            'observation.images.camera1': {
                'topic': '/follower/cam_front/image_raw',
                'msg_type': 'sensor_msgs/msg/Image',
            },
            'observation.state': {
                'topic': '/follower/joint_states',
                'msg_type': 'sensor_msgs/msg/JointState',
            },
        }
        if not isinstance(obs_cfg, dict):
            self.get_logger().error(f'Expected `observations` to be a dict, got {type(obs_cfg)}')
            return TransitionCallbackReturn.FAILURE

        obs_keys = list(obs_cfg.keys())  # e.g. ['observation.images.camera1', ...]
        subscribers = []

        for key in obs_keys:
            entry = obs_cfg[key]
            topic = entry['topic']
            msg_type_str = entry['msg_type']
            msg_cls = self._resolve_msg_type(msg_type_str)
            self.get_logger().info(f'Observations[{key}] -> {topic} ({msg_type_str})')
            subscribers.append(Subscriber(self, msg_cls, topic))

        # Create generic ApproximateTimeSynchronizer
        self._sync = ApproximateTimeSynchronizer(
            subscribers,
            queue_size=queue_size,
            slop=slop,
        )
        self._sync.registerCallback(self._make_sync_cb(obs_keys))

        # --- Build action publisher from `action` param ---
        action_param: Parameter = self.get_parameter('action')
        action_cfg = action_param.value or {
            'topic': '/leader/joint_states',
            'msg_type': 'sensor_msgs/msg/JointState',
        }

        if 'topic' in action_cfg:
            action_entry = action_cfg
        else:
            _, action_entry = next(iter(action_cfg.items()))
        act_topic = action_entry['topic']
        act_msg_type_str = action_entry['msg_type']
        act_msg_cls = self._resolve_msg_type(act_msg_type_str)

        self._cmd_pub = self.create_publisher(act_msg_cls, act_topic, 10)
        self.get_logger().info(f'Action publisher -> {act_topic} ({act_msg_type_str})')

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

        self._send_safe_stop()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up policy_runner...')
        self._sync = None
        self._cmd_pub = None
        self._policy = None

        with self._obs_lock:
            self._latest_msgs = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down policy_runner...')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------
    def _publish_step(self) -> None:
        """Publish next JointState command at publish_rate Hz."""
        if self._cmd_pub is None or self._policy is None:
            return

        action = self._policy.get_action()
        if action is None:
            self.get_logger().debug('[PUBLISH] No action available from policy.')
            return

        current_pos = list(action)
        n = len(current_pos)

        js_cmd = JointState()
        js_cmd.header.stamp = self.get_clock().now().to_msg()
        js_cmd.name = self._action_joint_names[:n]
        js_cmd.position = current_pos

        self._cmd_pub.publish(js_cmd)

    def _inference_step(self) -> None:
        """Timer: ask the policy to refresh its internal action buffer/buffer."""
        if self._policy is None:
            self.get_logger().warn('Inference step: no policy instantiated.')
            return

        with self._obs_lock:
            ros_obs = self._latest_msgs

        if ros_obs is None:
            self.get_logger().warn('Inference step: no observation received yet.')
            return

        delay = self._inference_delay if self._use_delay_compensation else 0.0

        self._policy.infer(
            ros_obs=ros_obs,
            time_per_action=self._publish_period,
            inference_delay=delay,
        )

    def _send_safe_stop(self) -> None:
        """Send a safe stop command (hold last observed positions)."""
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
        self.get_logger().warn('Safe stop: holding last observed joint positions.')
