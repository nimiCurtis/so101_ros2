#!/usr/bin/env python3
from __future__ import annotations

import threading
from importlib import import_module
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import yaml
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.logging import LoggingSeverity
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from so101_ros2_bridge import POLICY_BASE_DIR

from ..utils.utils import resolve_msg_type
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
            'gripper',
        ],
    }

    def __init__(self) -> None:
        super().__init__('policy_runner')

        # Scalar params declared once (can still be overridden from YAML)
        self.declare_parameter('policy_name', 'smolvla')
        self.declare_parameter(
            'checkpoint_path',
            'abs_path_to_pretrained_model_checkpoint_dir',
        )
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter(
            'task',
            'Pick and Place',
        )

        self.declare_parameter('inference_rate', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('inference_delay', 0.4)
        self.declare_parameter('use_delay_compensation', True)
        self.declare_parameter('sync.queue_size', 10)
        self.declare_parameter('sync.slop', 0.05)

        # Internal state
        self._obs_cfg: Dict[str, Any] = None
        self._action_cfg: Dict[str, Any] = None

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

        self.get_logger().info(f'{self.get_name()} node initialized.')

    def _load_obs_and_action_cfg(self) -> tuple[Dict[str, Any], Dict[str, Any]]:
        """Load observation and action configs from a YAML file.

        The YAML is expected to have the structure:

        observations:
          <obs_key>:
            topic: ...
            msg_type: ...
        action:
          topic: ...
          msg_type: ...

        If the file is missing or malformed, fall back to hardcoded defaults.
        """

        # ---------- Defaults ----------
        default_observations = {
            'observation.images.camera1': {
                'topic': '/follower/cam_front/image_raw',
                'msg_type': 'sensor_msgs/msg/Image',
            },
            'observation.images.camera2': {
                'topic': '/static_camera/cam_side/color/image_raw',
                'msg_type': 'sensor_msgs/msg/Image',
            },
            'observation.state': {
                'topic': '/follower/joint_states',
                'msg_type': 'sensor_msgs/msg/JointState',
            },
        }

        default_action = {
            'topic': '/leader/joint_states',
            'msg_type': 'sensor_msgs/msg/JointState',
        }

        cfg_path = POLICY_BASE_DIR / 'io.yaml'

        try:
            self.get_logger().info(f'Loading I/O config from {cfg_path}')
            with cfg_path.open('r') as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            self.get_logger().warn(f'I/O config file not found at {cfg_path}, using defaults.')
            return default_observations, default_action
        except yaml.YAMLError as e:
            self.get_logger().warn(
                f'Failed to parse I/O config YAML at {cfg_path}: {e}. Using defaults.'
            )
            return default_observations, default_action

        if not isinstance(data, dict):
            self.get_logger().warn(
                f'I/O config at {cfg_path} is not a dict (got {type(data)}), using defaults.'
            )
            return default_observations, default_action

        obs_cfg = data.get('observations')
        action_cfg = data.get('action')

        # Validate types, otherwise fall back
        if not isinstance(obs_cfg, dict):
            self.get_logger().warn(
                f'"observations" in {cfg_path} is not a dict (got {type(obs_cfg)}), using defaults.'
            )
            obs_cfg = default_observations

        if not isinstance(action_cfg, dict):
            self.get_logger().warn(
                f'"action" in {cfg_path} is not a dict (got {type(action_cfg)}), using defaults.'
            )
            action_cfg = default_action

        self.get_logger().info(f'Loaded observations config: {list(obs_cfg.keys())}')
        self.get_logger().info(f'Loaded action config: {action_cfg}')
        return obs_cfg, action_cfg

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

            # Debug: confirm that sync is working
            self.get_logger().log(
                f'[SYNC] Updated latest_msgs with keys={list(self._latest_msgs.keys())}',
                severity=LoggingSeverity.DEBUG,
                throttle_duration_sec=2.0,
            )

        return cb

    def _build_config_from_params(self) -> PolicyConfig:
        """Read ROS params and construct a PolicyConfig via PolicyConfig.create."""
        policy_name = self.get_parameter('policy_name').value
        device = self.get_parameter('device').value
        checkpoint_path_str = self.get_parameter('checkpoint_path').value
        task = self.get_parameter('task').value

        # Start from default robot properties and allow overrides from YAML
        robot_properties: Dict[str, Any] = dict(self.ROBOT_PROPERTIES)

        state_cfg = self._obs_cfg.get('observation.state')
        if isinstance(state_cfg, dict) and 'names' in state_cfg:
            state_names = list(state_cfg['names'])
            robot_properties['state_joint_names'] = state_names

        # Support both:
        # action: {topic: ..., msg_type: ..., names: [...]}
        # and:
        # action: {action: {topic:..., msg_type:..., names:[...]}}
        if 'topic' in self._action_cfg:
            action_entry = self._action_cfg
        else:
            _, action_entry = next(iter(self._action_cfg.items()), (None, {}))

        if isinstance(action_entry, dict) and 'names' in action_entry:
            action_names = list(action_entry['names'])
            robot_properties['action_joint_names'] = action_names

        try:
            cfg = PolicyConfig.create(
                policy_name=policy_name,
                device=device,
                checkpoint_path=Path(checkpoint_path_str),
                task=task,
                robot_properties=robot_properties,
            )
        except ValueError as e:
            self.get_logger().error(f'Error building PolicyConfig: {e}')
            return None
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

        try:
            obs_cfg, action_cfg = self._load_obs_and_action_cfg()
        except TypeError as e:
            self.get_logger().error(str(e))
            return TransitionCallbackReturn.FAILURE

        self._obs_cfg = obs_cfg
        self._action_cfg = action_cfg

        # Build PolicyConfig from ROS params + YAML
        cfg = self._build_config_from_params()
        if cfg is None:
            self.get_logger().error('Failed to build PolicyConfig, cannot configure node.')
            return TransitionCallbackReturn.FAILURE

        # Action joint names
        self._action_joint_names = cfg.robot_properties.get(
            'action_joint_names',
            cfg.robot_properties.get('joint_names', []),
        )

        self.get_logger().info(f'[PolicyConfig] Init with: {cfg}')
        self._policy = make_policy(cfg.policy_name, cfg, self)

        # Build subscribers + sync generically from `observations`
        if not isinstance(self._obs_cfg, dict):
            self.get_logger().error(
                f'Expected `observations` to be a dict, got {type(self._obs_cfg)}'
            )
            return TransitionCallbackReturn.FAILURE

        obs_keys = list(self._obs_cfg.keys())
        subscribers = []

        self.get_logger().info('**** Timing Configuration: ****')
        self.get_logger().info(
            f'inference_rate={self._inference_rate:.2f} Hz '
            f'(period={self._inference_period:.3f}s), '
            f'publish_rate={self._publish_rate:.2f} Hz '
            f'(period={self._publish_period:.3f}s), '
            f'inference_delay={self._inference_delay:.3f}s, '
            f'use_delay_compensation={self._use_delay_compensation}'
        )

        self.get_logger().info('**** Setting up observation subscribers: ****')
        for key in obs_keys:
            entry = obs_cfg[key]
            topic = entry['topic']
            msg_type_str = entry['msg_type']
            msg_cls = resolve_msg_type(msg_type_str)
            self.get_logger().info(f'* Observations[{key}] -> {topic} ({msg_type_str})')
            subscribers.append(
                Subscriber(self, msg_cls, topic, qos_profile=qos_profile_sensor_data)
            )

        # Create generic ApproximateTimeSynchronizer
        self._sync = ApproximateTimeSynchronizer(
            subscribers,
            queue_size=queue_size,
            slop=slop,
        )
        self._sync.registerCallback(self._make_sync_cb(obs_keys))
        self.get_logger().info(f'* Configured sync with queue_size={queue_size}, slop={slop}')

        if 'topic' in self._action_cfg:
            action_entry = self._action_cfg
        else:
            _, action_entry = next(iter(self._action_cfg.items()))
        act_topic = action_entry['topic']
        act_msg_type_str = action_entry['msg_type']
        act_msg_cls = resolve_msg_type(act_msg_type_str)

        self.get_logger().info('**** Setting up action publisher: ****')
        self._cmd_pub = self.create_publisher(act_msg_cls, act_topic, 10)
        self.get_logger().info(f'* Action publisher -> {act_topic} ({act_msg_type_str})')
        self.get_logger().info(f'* Action joint names: {self._action_joint_names}')
        self.get_logger().info('************************************************')

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

        self._policy.reset()

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

        # 1. Cancel timers if they somehow survived deactivate
        if self._inference_timer is not None:
            self._inference_timer.cancel()
            self._inference_timer = None

        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self._publish_timer = None

        # 2. Drop the sync object and any subscribers it holds
        self._sync = None  # message_filters cleanup is via GC

        # 3. Destroy publisher if it exists
        if self._cmd_pub is not None:
            self.destroy_publisher(self._cmd_pub)
            self._cmd_pub = None

        # 4. Reset / drop the policy
        if self._policy is not None:
            # self._policy.close()
            self._policy = None

        # 5. Clear latest observation
        with self._obs_lock:
            self._latest_msgs = None

        self.get_logger().info('policy_runner cleanup complete.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('Shutting down policy_runner...')

        # cancel timers, as safety
        if self._inference_timer:
            self._inference_timer.cancel()
        if self._publish_timer:
            self._publish_timer.cancel()

        # send safe stop to the robot
        try:
            self._send_safe_stop()
        except Exception as e:
            self.get_logger().warn(f'Failed safe stop on shutdown: {e}')

        # drop policy and buffers
        self._policy = None
        with self._obs_lock:
            self._latest_msgs = None

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
            self.get_logger().log(
                'No action available from policy.',
                severity=LoggingSeverity.WARN,
                throttle_duration_sec=2.0,
            )
            return

        current_pos = list(action)
        n = len(current_pos)

        js_cmd = JointState()
        js_cmd.header.stamp = self.get_clock().now().to_msg()
        js_cmd.name = self._action_joint_names[:n]
        js_cmd.position = current_pos

        self.get_logger().log(
            f'Policy is running. Publishing joint command -> {np.around(current_pos, 3).tolist()}',
            severity=LoggingSeverity.INFO,
            throttle_duration_sec=5.0,
        )

        self._cmd_pub.publish(js_cmd)

    def _inference_step(self) -> None:
        """Timer: ask the policy to refresh its internal action buffer/buffer."""
        if self._policy is None:
            self.get_logger().log(
                'Inference step: no policy instantiated.',
                severity=LoggingSeverity.WARN,
                throttle_duration_sec=2.0,
            )
            return

        with self._obs_lock:
            ros_obs = self._latest_msgs

        if ros_obs is None:
            self.get_logger().log(
                'Inference step: no observation received yet.',
                severity=LoggingSeverity.WARN,
                throttle_duration_sec=2.0,
            )
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
            self.get_logger().log(
                'Safe stop: no last observation, no command published.',
                severity=LoggingSeverity.WARN,
                throttle_duration_sec=2.0,
            )
            return

        last_joint_state: JointState = msgs['observation.state']
        positions = list(last_joint_state.position)
        if not positions:
            self.get_logger().log(
                'Safe stop: last JointState has no positions, no command published.',
                severity=LoggingSeverity.WARN,
                throttle_duration_sec=2.0,
            )
            return

        js_cmd = JointState()
        js_cmd.header.stamp = self.get_clock().now().to_msg()
        js_cmd.name = list(last_joint_state.name)
        js_cmd.position = positions

        self._cmd_pub.publish(js_cmd)

        self.get_logger().log(
            'Safe stop: holding last observed joint positions.',
            severity=LoggingSeverity.WARN,
            throttle_duration_sec=5.0,
        )
