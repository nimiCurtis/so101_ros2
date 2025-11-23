#!/usr/bin/env python3
from __future__ import annotations

from so101_ros2_bridge.utils.core import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()

import math
from typing import Any, Dict, List, Mapping, Optional

import numpy as np
from lerobot.configs.types import PolicyFeature
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.utils import build_inference_frame
from lerobot.processor.core import EnvTransition
from rclpy.node import Node

from ..utils.conversion import ros_to_dataset_features
from ..utils.filtering import LowPassFilter
from .base import BasePolicy, PolicyConfig
from .registry import register_policy


@register_policy('smolvla')
class SmolVLA(BasePolicy):
    """SmolVLA policy adapter with simple chunk-based inference."""

    def __init__(self, cfg: PolicyConfig, node: Node) -> None:
        super().__init__(cfg, node)

        self.node = node

        node.get_logger().info(
            f'[{__class__.__name__}] Init with device={cfg.device}, '
            f'checkpoint={cfg.checkpoint_path}, task={cfg.task}'
        )

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

        # Robot / joint info from config
        robot_props = cfg.robot_properties
        self._robot_type: str = robot_props.get('robot_type', 'so101_follower')

        self._default_joint_names: List[str] = robot_props.get(
            'joint_names',
            [
                'shoulder_pan',
                'shoulder_lift',
                'elbow_flex',
                'wrist_flex',
                'wrist_roll',
                'gripper_finger',
            ],
        )
        self._state_joint_names: List[str] = robot_props.get(
            'state_joint_names', self._default_joint_names
        )
        self._action_joint_names: List[str] = robot_props.get(
            'action_joint_names', self._default_joint_names
        )

        # Feature specs from the model
        self._input_features: Dict[str, PolicyFeature] = self.model.config.input_features
        self._output_features: Dict[str, PolicyFeature] = self.model.config.output_features

        # Dataset feature description (used by build_inference_frame)
        self.dataset_features: Dict[str, Dict] = {}
        self._setup_dataset_features()

        # Chunk buffer
        self._buffer_actions: List[List[float]] = []
        self._buffer_index: int = 0

        self.filter = LowPassFilter(alpha=0.5)

    def _setup_dataset_features(self) -> None:
        """Setup dataset feature metadata based on the model's config."""
        state_pos_names = [f'{j}.pos' for j in self._state_joint_names]
        action_pos_names = [f'{j}.pos' for j in self._action_joint_names]

        for key, feature in self._input_features.items():
            if feature.type.value == 'STATE':
                dim = feature.shape[0]
                if len(state_pos_names) == dim:
                    names = state_pos_names
                else:
                    names = [f'j{i}' for i in range(dim)]

                self.dataset_features[key] = {
                    'dtype': 'float32',
                    'shape': feature.shape,
                    'names': names,
                }

            elif feature.type.value == 'VISUAL':
                self.dataset_features[key] = {
                    'dtype': 'image',
                    'shape': feature.shape,
                    'names': None,
                }

        for key, feature in self._output_features.items():
            dim = feature.shape[0]
            if len(action_pos_names) == dim:
                names = action_pos_names
            else:
                names = [f'j{i}' for i in range(dim)]

            self.dataset_features[key] = {
                'dtype': 'float32',
                'shape': feature.shape,
                'names': names,
            }

    def make_observation(
        self,
        ros_obs: Mapping[str, Any],
    ) -> EnvTransition:
        """Convert ROS messages into a LeRobot EnvTransition."""
        raw_obs_features: Dict[str, Any] = ros_to_dataset_features(
            ros_obs=ros_obs,
            joint_order=self._state_joint_names,
            input_features=self._input_features,
        )

        inference_frame = build_inference_frame(
            observation=raw_obs_features,
            ds_features=self.dataset_features,
            device=self._device,
            task=self._task,
            robot_type=self._robot_type,
        )

        observation: EnvTransition = self._pre_processor(inference_frame)
        return observation

    def infer(self, ros_obs, time_per_action, inference_delay=0):
        return super().infer(ros_obs, time_per_action, inference_delay)

    def predict_action_chunk(
        self,
        observation: EnvTransition,
        time_per_action: float,
        inference_delay: float,
    ) -> None:
        """Run policy and update internal action buffer with predicted joint positions."""
        actions = self.model.predict_action_chunk(observation)  # [B, T, n_joints]
        actions = self._post_processor(actions)
        actions_list: List[List[float]] = actions[0].cpu().numpy().tolist()

        if not actions_list:
            self.node.get_logger().warn('[SmolVLA] Empty action sequence returned.')
            return

        if inference_delay > 0.0 and time_per_action > 0.0:
            skip_actions = math.ceil(inference_delay / time_per_action)
        else:
            skip_actions = 0

        if skip_actions >= len(actions_list):
            self.node.get_logger().warn(
                f'[SmolVLA] predict_actions: skip_actions={skip_actions} >= len(chunk)={len(actions_list)}. '
                f'Starting from last action instead.'
            )
            self._buffer_index = len(actions_list) - 1
        elif skip_actions < 0:
            self.node.get_logger().warn(
                f'[SmolVLA] predict_actions: negative skip_actions={skip_actions}, starting from 0.'
            )
            self._buffer_index = 0
        else:
            self._buffer_index = skip_actions

        self._buffer_actions = actions_list
        self.node.get_logger().info(
            f'[SmolVLA] predict_actions: new chunk with {len(actions_list)} actions, '
            f'start_index={self._buffer_index} (skipped {self._buffer_index})'
        )

    def get_action(self) -> Optional[List[float]]:
        """Return the next joint position vector, or None if not available."""
        if not self._buffer_actions:
            return None

        if self._buffer_index >= len(self._buffer_actions):
            self._buffer_index = len(self._buffer_actions) - 1

        current = self._buffer_actions[self._buffer_index]
        if self._buffer_index < len(self._buffer_actions) - 1:
            self._buffer_index += 1

        return self.filter.filter(np.array(current)).tolist()

    def reset(self, context: Optional[Mapping[str, Any]] = None) -> None:
        self.model.reset()
        self._buffer_actions = []
        self._buffer_index = 0
        self.filter.reset()
