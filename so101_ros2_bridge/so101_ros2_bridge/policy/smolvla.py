from __future__ import annotations

from curses import raw

# Ensure the conda site-packages directory is in the system path
from so101_ros2_bridge.utils import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()

from typing import Any, Dict, List, Mapping

import numpy as np
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.utils import build_inference_frame, make_robot_action
from lerobot.processor.core import EnvAction, EnvTransition
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from .base import BasePolicy, PolicyConfig
from .registry import register_policy


@register_policy('smolvla')
class SmolVLA(BasePolicy):
    """SmolVLA policy."""

    def __init__(self, cfg: PolicyConfig, node: Node) -> None:
        super().__init__(cfg, node)

        node.get_logger().info(
            f'[SmolVLAPolicy] Init with device={cfg.device}, checkpoint={cfg.checkpoint_path}'
        )

        self.model = SmolVLAPolicy.from_pretrained(cfg.checkpoint_path).to(cfg.device)
        self.model.eval()

        self._pre_processor, self._post_processor = make_pre_post_processors(
            self.model.config,
            cfg.checkpoint_path,
            preprocessor_overrides={'device_processor': {'device': str(cfg.device)}},
        )

        self._device = cfg.device
        self._task = cfg.task

        # Get the actual feature names from the model config
        self._input_features = self.model.config.input_features
        self._output_features = self.model.config.output_features

        # Log what features the model expects
        node.get_logger().info(f'Input features: {list(self._input_features.keys())}')
        node.get_logger().info(f'Output features: {list(self._output_features.keys())}')

    # ------------------------------------------------------------------
    # Observation construction
    # ------------------------------------------------------------------
    # def make_observation(
    #     self,
    #     ros_obs: Mapping[str, Any],  # e.g., images: Dict[str, Image], joint_state: JointState
    # ) -> EnvTransition:
    #     """Convert ROS messages into a LeRobot-style observation dict."""
    #     images = ros_obs.get('images', {})
    #     joint_state = ros_obs.get('joint_state', None)

    #     raw_obs: Dict[str, Any] = {}
    #     # convert ros msgs to datafeatures # TODO:
    #     inference_frame = build_inference_frame(
    #             observation=raw_obs,
    #             ds_features=self.dataset_features,
    #             device=self.device,
    #             task=self.task,
    #             robot_type=self.robot_type
    #         )

    #     observation: EnvTransition = self._pre_processor(inference_frame)
    #     return observation

    # def act_sequence(self, observation: EnvTransition) -> List[List[float]]:
    #     """Run SmolVLA to get a sequence of future joint positions.

    #     This is a stub implementation. Replace the internals with real model
    #     inference that returns positions in the correct joint order and units.
    #     """
    #     action_chunk = self.model.predict_action_chunk(observation)
    #     action_chunk = self._post_processor(action_chunk)
    #     return action_chunk
