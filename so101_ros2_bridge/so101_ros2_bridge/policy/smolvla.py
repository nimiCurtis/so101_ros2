from __future__ import annotations

from typing import Any, Dict, List, Mapping

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

# If you have torch / lerobot, import them here:
# import torch
# from lerobot import load_policy
from .base import BasePolicy, PolicyConfig
from .registry import register_policy


@register_policy('smolvla')
class SmolVLAPolicy(BasePolicy):
    """SmolVLA policy."""

    def __init__(self, cfg: PolicyConfig, node: Node) -> None:
        super().__init__(cfg, node)

        # Example placeholders for actual model and preprocessing
        # self.model = None  # replace with real model
        # self.image_size = cfg.extra.get("observation_image_size", [256, 256])
        # self.normalize_obs = bool(cfg.extra.get("normalize_observations", True))

        # node.get_logger().info(
        #     f"[SmolVLAPolicy] Init with device={cfg.device}, "
        #     f"checkpoint={cfg.checkpoint_path}"
        # )

        # TODO: implement actual loading:
        # self.model = load_policy(
        #     name="smolvla",
        #     checkpoint_path=cfg.checkpoint_path,
        #     device=cfg.device,
        # )
        # self.model.eval()

    # ------------------------------------------------------------------
    # Observation construction
    # ------------------------------------------------------------------
    def make_observation(
        self,
        ros_obs: Mapping[str, Any],  # e.g., images: Dict[str, Image], joint_state: JointState
    ) -> Dict[str, Any]:
        """Convert ROS messages into a LeRobot-style observation dict."""
        images = ros_obs.get('images', {})
        joint_state = ros_obs.get('joint_state', None)

        # Preprocess images and joint_state as needed
        # For example:
        # images = {key: self.preprocess_image(img) for key, img in images.items()}
        # joint_state = self.preprocess_joint_state(joint_state)

        return {
            'images': images,
            'joint_state': joint_state,
        }

    def act_sequence(self, obs: Mapping[str, Any]) -> List[List[float]]:
        """Run SmolVLA to get a sequence of future joint positions.

        This is a stub implementation. Replace the internals with real model
        inference that returns positions in the correct joint order and units.
        """
        # Example stub: return zeros
        n_joints = len(obs['joint_state'].position) if obs['joint_state'] else 7
        T = 10  # e.g., predict 10 future steps
        return [[0.0] * n_joints for _ in range(T)]
