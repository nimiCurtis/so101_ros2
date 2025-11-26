#!/usr/bin/env python3

# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from __future__ import annotations

from so101_ros2_bridge.utils.core import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()

import math
from typing import Any, Dict, List, Mapping, Optional

from lerobot.configs.types import PolicyFeature
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.utils import build_inference_frame
from lerobot.processor.core import EnvTransition
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from ..utils.buffer import ActionBuffer
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

        # Load model
        self.model: SmolVLAPolicy = SmolVLAPolicy.from_pretrained(
            cfg.checkpoint_path, local_files_only=True
        ).to(cfg.device)
        self.model.eval()

        # Pre- and post-processors
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

        # Joint names
        self._state_joint_names: List[str] = robot_props.get(
            'state_joint_names', robot_props.get('joint_names')
        )
        self._action_joint_names: List[str] = robot_props.get(
            'action_joint_names', robot_props.get('joint_names')
        )

        # Feature specs from the model
        self._input_features: Dict[str, PolicyFeature] = self.model.config.input_features
        self._output_features: Dict[str, PolicyFeature] = self.model.config.output_features

        # Dataset feature description (used by build_inference_frame)
        self.dataset_features: Dict[str, Dict] = {}
        self._setup_dataset_features()

        # Low-pass filter for smoothing actions
        lpf_cfg: Dict[str, Any] = cfg.extra.get('lpf_filtering', {})

        self._filter = None
        if lpf_cfg.get('enable', False):
            alpha = lpf_cfg.get('alpha', 0.2)
            self._filter = LowPassFilter(alpha=alpha)
            node.get_logger().info(
                f'[{__class__.__name__}] Low-pass filtering enabled with alpha={alpha}'
            )
        else:
            node.get_logger().info(f'[{__class__.__name__}] Low-pass filtering disabled.')

        # Chunk buffer
        self._action_buffer = ActionBuffer(lpf=self._filter)

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
        """Convert ROS messages into a LeRobot EnvTransition.
        Args:
            ros_obs: A mapping of ROS observation messages.
        Returns:
            An EnvTransition object suitable for policy inference.
        """
        raw_obs_features: Dict[str, Any] = ros_to_dataset_features(
            ros_obs=ros_obs,
            joint_order=self._state_joint_names,
            input_features=self._input_features,
        )

        # Build model-ready inference frame
        inference_frame = build_inference_frame(
            observation=raw_obs_features,
            ds_features=self.dataset_features,
            device=self._device,
            task=self._task,
            robot_type=self._robot_type,
        )

        observation: EnvTransition = self._pre_processor(inference_frame)
        return observation

    def predict_action_chunk(
        self,
        observation: EnvTransition,
        time_per_action: float,
        inference_delay: float,
    ) -> None:
        """Run policy and update internal action buffer with predicted joint positions.
        Args:
            observation: The preprocessed observation for the policy.
            time_per_action: The time duration each action is expected to cover.
            inference_delay: The expected delay (in seconds) between inference
                and action execution, used to skip ahead in the action chunk.
        Returns:
            None. Updates internal action buffer.
        """

        # Run model inference
        actions = self.model.predict_action_chunk(observation)  # [B, T, n_joints]
        actions = self._post_processor(actions)
        actions_list: List[List[float]] = actions[0].cpu().numpy().tolist()

        if not actions_list:
            self.node.get_logger().log(
                f'[{__class__.__name__}] Empty action sequence returned.',
                severity=LoggingSeverity.WARN,
                throttle_duration_sec=2.0,
            )
            return

        # Determine how many actions to skip based on inference delay
        if inference_delay > 0.0 and time_per_action > 0.0:
            skip_actions = math.ceil(inference_delay / time_per_action)
        else:
            skip_actions = 0

        # Clamp start_index to valid range (0 .. len-1)
        if skip_actions >= len(actions_list):
            self.node.get_logger().log(
                f'[{__class__.__name__}] predict_action_chunk: skip_actions={skip_actions} '
                f'>= len(chunk)={len(actions_list)}. Starting from last action instead.',
                severity=LoggingSeverity.WARN,
                throttle_duration_sec=2.0,
            )
            start_index = len(actions_list) - 1
        elif skip_actions < 0:
            self.node.get_logger().log(
                f'[{__class__.__name__}] predict_action_chunk: negative skip_actions={skip_actions}, '
                f'starting from 0.',
                severity=LoggingSeverity.WARN,
                throttle_duration_sec=2.0,
            )
            start_index = 0
        else:
            start_index = skip_actions

        # Update action buffer
        self._action_buffer.set(actions_list, start_index=start_index)

        self.node.get_logger().log(
            f'[{__class__.__name__}] predict_action_chunk: new chunk with {len(actions_list)} actions, '
            f'start_index={start_index} (requested_skip={skip_actions})',
            severity=LoggingSeverity.INFO,
            throttle_duration_sec=5.0,
        )

    def get_action(self) -> Optional[List[float]]:
        """Return the next joint position vector, or None if not available.
        Returns:
            A list of joint positions, or None if no actions are buffered.
        """
        return self._action_buffer.get()

    def reset(self, context: Optional[Mapping[str, Any]] = None) -> None:
        self.node.get_logger().info(f'[{__class__.__name__}] Resetting policy.')
        self.model.reset()
        self._action_buffer.reset()  # clear action buffer
