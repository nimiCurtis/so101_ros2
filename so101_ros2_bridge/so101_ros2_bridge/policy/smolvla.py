#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np
import torch
from lerobot.configs.types import PolicyFeature, RTCAttentionSchedule
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.rtc.action_queue import ActionQueue
from lerobot.policies.rtc.configuration_rtc import RTCConfig
from lerobot.policies.rtc.latency_tracker import LatencyTracker
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.utils import build_inference_frame
from lerobot.processor.core import EnvTransition
from rclpy.node import Node
from rclpy.parameter import ParameterType
from sensor_msgs.msg import Image, JointState

from so101_ros2_bridge.utils import ensure_conda_site_packages_from_env

from .base import BasePolicy, PolicyConfig
from .registry import register_policy

ensure_conda_site_packages_from_env()


@register_policy('smolvla')
class SmolVLA(BasePolicy):
    """SmolVLA policy adapter with optional Real-Time Chunking (RTC).

    Responsibilities:
    - Convert ROS observations -> LeRobot EnvTransition.
    - Run SmolVLA predict_action_chunk() with or without RTC.
    - Maintain an internal buffer / action queue.
    - Provide a simple get_action() API returning one joint-position vector.
    """

    JOINT_NAMES = [
        'shoulder_pan',
        'shoulder_lift',
        'elbow_flex',
        'wrist_flex',
        'wrist_roll',
        'gripper',
    ]

    def __init__(self, cfg: PolicyConfig, node: Node) -> None:
        super().__init__(cfg, node)

        node.get_logger().info(
            f'[SmolVLA] Init with device={cfg.device}, checkpoint={cfg.checkpoint_path}'
        )

        # ------------------------------------------------------------------
        # Core model + pre/post processors
        # ------------------------------------------------------------------
        self.model: SmolVLAPolicy = SmolVLAPolicy.from_pretrained(cfg.checkpoint_path).to(
            cfg.device
        )
        self.model.eval()

        self._pre_processor, self._post_processor = make_pre_post_processors(
            self.model.config,
            cfg.checkpoint_path,
            preprocessor_overrides={'device_processor': {'device': str(cfg.device)}},
        )

        self._device = cfg.device
        self._task = cfg.task

        # Feature specs from the model
        self._input_features: Dict[str, PolicyFeature] = self.model.config.input_features
        self._output_features: Dict[str, PolicyFeature] = self.model.config.output_features

        node.get_logger().info(f'[SmolVLA] Input features: {self._input_features}')
        node.get_logger().info(f'[SmolVLA] Output features: {self._output_features}')

        # Dataset feature description (used by build_inference_frame)
        self.dataset_features: Dict[str, Dict] = {}
        self._setup_dataset_features()

        # ------------------------------------------------------------------
        # RTC / buffering state
        # ------------------------------------------------------------------
        # NOTE: We read RTC flags from cfg.extra to keep PolicyConfig generic.
        self._rtc_enabled: bool = bool(cfg.extra.get('rtc_enabled', True))
        execution_horizon = int(
            cfg.extra.get('rtc_execution_horizon', 10)
        )  # Number of actions to predict per chunk
        queue_threshold = int(
            cfg.extra.get('rtc_queue_threshold', 30)
        )  # Minimum actions in queue before requesting more

        self._rtc_cfg: Optional[RTCConfig] = None
        self._action_queue: Optional[ActionQueue] = None
        self._latency_tracker: Optional[LatencyTracker] = None
        self._rtc_queue_threshold: int = queue_threshold

        if self._rtc_enabled:
            # TODO: change hardcoded RTCConfig params to be read from cfg.extra
            self._rtc_cfg = RTCConfig(
                execution_horizon=execution_horizon,
                max_guidance_weight=float(cfg.extra.get('rtc_max_guidance_weight', 10.0)),
                prefix_attention_schedule=RTCAttentionSchedule[
                    cfg.extra.get('rtc_prefix_attention_schedule', 'EXP')
                ],
            )
            # Attach RTC config to the SmolVLA policy and init its internal RTC processor
            self.model.config.rtc_config = self._rtc_cfg
            self.model.init_rtc_processor()

            self._action_queue = ActionQueue(self._rtc_cfg)
            self._latency_tracker = LatencyTracker()

            node.get_logger().info(
                f'[SmolVLA] RTC enabled: execution_horizon={execution_horizon}, '
                f'queue_threshold={self._rtc_queue_threshold}'
            )
        else:
            node.get_logger().info('[SmolVLA] RTC disabled: using simple chunk buffering')

        # Non-RTC fallback: simple Python list buffer
        self._buffer_actions: List[List[float]] = []
        self._buffer_index: int = 0

    # ------------------------------------------------------------------
    # Dataset feature description
    # ------------------------------------------------------------------
    def _setup_dataset_features(self) -> None:
        """Setup dataset feature metadata based on the model's config."""
        state_names = [f'{joint}.pos' for joint in self.JOINT_NAMES]

        action_names = [f'{joint}.pos' for joint in self.JOINT_NAMES]

        for key, feature in self._input_features.items():
            if feature.type.value == 'STATE':
                state_dim = feature.shape[0]
                self.dataset_features[key] = {
                    'dtype': 'float32',
                    'shape': feature.shape,
                    'names': state_names
                    if state_names is not None
                    else [f'j{i}' for i in range(state_dim)],
                }
            elif feature.type.value == 'VISUAL':
                self.dataset_features[key] = {
                    'dtype': 'image',
                    'shape': feature.shape,
                    'names': None,
                }

        for key, feature in self._output_features.items():
            action_dim = feature.shape[0]
            self.dataset_features[key] = {
                'dtype': 'float32',
                'shape': feature.shape,
                'names': action_names
                if action_names is not None
                else [f'j{i}' for i in range(action_dim)],
            }

    # ------------------------------------------------------------------
    # ROS → EnvTransition
    # ------------------------------------------------------------------
    def make_observation(
        self,
        ros_obs: Mapping[str, Any],
    ) -> EnvTransition:
        """Convert ROS messages into a LeRobot EnvTransition."""
        raw_obs_features: Dict[str, Any] = ros_to_dataset_features(
            ros_obs=ros_obs,
            input_features=self._input_features,
        )

        # LeRobot helper: build a single-frame "inference frame" dict
        inference_frame = build_inference_frame(
            observation=raw_obs_features,
            ds_features=self.dataset_features,
            device=self._device,
            task=self._task,
            robot_type='so101_follower',
        )

        # Preprocess → EnvTransition
        observation: EnvTransition = self._pre_processor(inference_frame)
        return observation

    # ------------------------------------------------------------------
    # NEW: RTC-aware inference API
    # ------------------------------------------------------------------
    def infer(self, ros_obs: Mapping[str, Any], time_per_action: float) -> None:
        """Update internal action buffer / queue from the latest observation.

        This does NOT return actions directly. Instead:
        - If RTC is enabled: fills an ActionQueue using predict_action_chunk
          with RTC parameters (inference_delay, prev_chunk_left_over, etc.).
        - If RTC is disabled: fills a simple Python list buffer.
        """
        if ros_obs is None or 'observation.state' not in ros_obs:
            self.node.get_logger().warn('[SmolVLA] infer: observation is missing.')
            return

        # Build EnvTransition
        observation = self.make_observation(ros_obs=ros_obs)

        if self._rtc_enabled and self._action_queue is not None:
            self._infer_rtc(observation, time_per_action)
        else:
            self._infer_simple(observation)

    def get_action(self) -> Optional[List[float]]:
        """Return the next joint position vector, or None if not available."""
        if self._rtc_enabled and self._action_queue is not None:
            # RTC path: pop one action from ActionQueue
            action = self._action_queue.get()
            if action is None:
                return None
            return action.cpu().tolist()

        # Non-RTC path: simple Python list buffer
        if not self._buffer_actions:
            return None

        if self._buffer_index >= len(self._buffer_actions):
            self._buffer_index = len(self._buffer_actions) - 1

        current = self._buffer_actions[self._buffer_index]
        if self._buffer_index < len(self._buffer_actions) - 1:
            self._buffer_index += 1

        return current

    # ------------------------------------------------------------------
    # Internal inference modes
    # ------------------------------------------------------------------
    def _infer_rtc(self, observation: EnvTransition, time_per_action: float) -> None:
        """RTC mode: request a new action chunk if queue is below threshold."""
        assert self._action_queue is not None

        # If queue has enough actions, do nothing this tick
        if self._action_queue.qsize() > self._rtc_queue_threshold:
            return

        now = time.perf_counter()

        action_index_before = self._action_queue.get_action_index()
        prev_leftover = self._action_queue.get_left_over()

        # Estimate inference_delay from latency history
        if self._latency_tracker is not None:
            max_latency = self._latency_tracker.max()
        else:
            max_latency = None

        if max_latency is None:
            inference_delay = 0
        else:
            inference_delay = math.ceil(max_latency / time_per_action)

        # Call RTC-aware predict_action_chunk on the HF SmolVLA policy
        actions = self.model.predict_action_chunk(
            observation,
            inference_delay=inference_delay,
            prev_chunk_left_over=prev_leftover,
        )

        # actions: [B=1, T, n_joints]
        original_actions = actions.squeeze(0).detach()
        post_actions = self._post_processor(actions).squeeze(0)

        new_latency = time.perf_counter() - now
        if self._latency_tracker is not None:
            self._latency_tracker.add(new_latency)

        new_delay = math.ceil(new_latency / time_per_action)

        if self._rtc_cfg is not None:
            if self._rtc_queue_threshold < (self._rtc_cfg.execution_horizon + new_delay):
                self.node.get_logger().warn(
                    '[SmolVLA] rtc_queue_threshold is too small: '
                    'should be higher than execution_horizon + inference_delay.'
                )

        # Merge into RTC queue
        self._action_queue.merge(
            original_actions,
            post_actions,
            new_delay,
            action_index_before,
        )

    def _infer_simple(self, observation: EnvTransition) -> None:
        """Non-RTC: produce a full sequence and store in a Python list buffer."""
        # [B=1, T, n_joints]
        actions = self.model.predict_action_chunk(observation)
        actions = self._post_processor(actions)
        actions_np = actions[0].cpu().numpy().tolist()

        if not actions_np:
            self.node.get_logger().warn('[SmolVLA] infer_simple: empty action sequence returned.')
            return

        self._buffer_actions = actions_np
        self._buffer_index = 0

    def reset(self, context: Optional[Mapping[str, Any]] = None) -> None:
        self.model.reset()
        self._buffer_actions = []
        self._buffer_index = 0
        if self._rtc_enabled and self._action_queue is not None:
            self._action_queue = ActionQueue(self._rtc_cfg)
            assert self._action_queue.empty()

    def close(self) -> None:
        return super().close()


# ----------------------------------------------------------------------
# Helper functions (unchanged)
# ----------------------------------------------------------------------
def ros_to_dataset_features(
    ros_obs: Mapping[str, Any],
    input_features: Dict[str, PolicyFeature],
) -> Dict[str, Any]:
    """Convert ROS observation messages to raw values dict for build_inference_frame."""
    values: Dict[str, Any] = {}

    # 1) STATE features -> per-joint keys like "shoulder_pan.pos"
    if 'observation.state' in ros_obs:
        joint_state_msg: JointState = ros_obs['observation.state']

        joint_vec, joint_names = ros_jointstate_to_vec6(
            js_msg=joint_state_msg,
            joint_order=SmolVLA.JOINT_NAMES,
            use_lerobot_ranges_norms=False,
        )

        for name, val in zip(joint_names, joint_vec):
            values[f'{name}.pos'] = float(val)

    # 2) VISUAL features -> keys like "observation.images.camera1", ...
    for key, feature in input_features.items():
        if feature.type.value != 'VISUAL':
            continue

        if key not in ros_obs:
            raise ValueError(f"Image for VISUAL feature key '{key}' not found in ROS observation.")

        img_msg: Image = ros_obs[key]
        np_img = ros_image_to_hwc_float01(img_msg)

        cam_id = key.split('.images.', 1)[1]  # "camera1"
        values[cam_id] = np_img

    return values


def ros_image_to_hwc_float01(msg: Image) -> np.ndarray:
    """Convert a ROS image message to an HWC float array scaled to [0, 1]."""
    enc = msg.encoding
    if enc not in ('rgb8', 'bgr8', 'mono8'):
        raise ValueError(f'Unsupported encoding: {enc}')
    ch = 3 if enc in ('rgb8', 'bgr8') else 1
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
    if enc == 'bgr8':
        arr = arr[..., ::-1]
    if ch == 1:
        arr = np.repeat(arr, 3, axis=2)
    return arr.astype(np.float32) / 255.0


def radians_to_normalized(joint_name: str, rad: float) -> float:
    """Convert a radian command into the normalized SO101 joint range."""
    if joint_name == 'gripper':
        return (rad / math.pi) * 100.0
    return (rad / math.pi) * 100.0


def ros_jointstate_to_vec6(
    js_msg: JointState,
    joint_order: Optional[List[str]] = None,
    use_lerobot_ranges_norms: bool = False,
) -> Tuple[np.ndarray, List[str]]:
    """Convert a JointState message to a 6D vector in joint_order."""
    pos = list(getattr(js_msg, 'position', []))
    names = list(getattr(js_msg, 'name', []))
    out = np.zeros((6,), dtype=np.float32)

    if joint_order:
        if len(joint_order) != 6:
            raise ValueError(f'joint_order must have 6 names, got {len(joint_order)}')
        name_to_idx = {n: i for i, n in enumerate(names)}
        try:
            vals = [pos[name_to_idx[n]] for n in joint_order]
            if use_lerobot_ranges_norms:
                vals = [
                    radians_to_normalized(joint_name, val)
                    for joint_name, val in zip(joint_order, vals)
                ]
        except KeyError as e:
            missing = set(joint_order) - set(names)
            raise KeyError(f'Joint(s) {missing} not found in JointState.name') from e
        out[:] = np.array(vals, dtype=np.float32)
        joint_names = joint_order
    else:
        if len(pos) < 6:
            raise ValueError(f'JointState.position has {len(pos)} values, need >= 6')
        vals = pos[:6]
        if use_lerobot_ranges_norms:
            joint_names = names[:6] if len(names) >= 6 else [f'joint_{i}' for i in range(6)]
            vals = [
                radians_to_normalized(joint_name, val) for joint_name, val in zip(joint_names, vals)
            ]
        out[:] = np.array(vals, dtype=np.float32)
        joint_names = names[:6] if names else [f'joint_{i}' for i in range(6)]

    return out, joint_names
