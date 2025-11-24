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

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

import yaml
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from so101_ros2_bridge import POLICY_BASE_DIR


@dataclass
class PolicyConfig:
    """Configuration for a policy adapter.

    Attributes:
        policy_name: Name of the policy.
        device: Device to run the policy on (e.g., 'cuda:0' or 'cpu').
        checkpoint_path: Path to the model checkpoint file.
        task: Task type (e.g., 'pick_and_place', 'navigation').
        extra: Additional configuration parameters.
    """

    policy_name: str
    device: str
    checkpoint_path: str
    task: str
    robot_properties: Dict[str, Any] = field(default_factory=dict)
    extra: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def create(
        cls,
        policy_name: str,
        device: str,
        checkpoint_path: Path,
        task: str,
        robot_properties: Dict[str, Any],
    ) -> 'PolicyConfig':
        """Create PolicyConfig from a YAML file in the policy config directory.
        Args:
            policy_name: Name of the policy.
            device: Device to run the policy on.
            checkpoint_path: Path to the model checkpoint file.
            task: Task type.
            robot_properties: Robot-specific properties.
        Returns:
            An instance of PolicyConfig.
        Raises:
            ValueError: If required fields are missing or invalid.
        """

        # device should be non-empty, and fallback to 'cpu' if not specified
        if not device:
            device = 'cpu'

        # Checkpoint should be an absolute path
        if checkpoint_path:
            if not checkpoint_path.is_absolute():
                raise ValueError(f'Checkpoint path must be absolute: {checkpoint_path}')
        else:
            raise ValueError('Checkpoint path must be provided and non-empty.')

        # Load extra config from YAML file if it exists
        policy_config_path = POLICY_BASE_DIR / f'{policy_name}.yaml'
        extra = {}
        if policy_config_path.exists():
            with policy_config_path.open('r') as f:
                extra = yaml.safe_load(f) or {}

        return cls(
            policy_name=policy_name,
            device=device,
            checkpoint_path=str(checkpoint_path),
            task=task,
            robot_properties=robot_properties,
            extra=extra,
        )


class BasePolicy:
    """Base interface for all policy classes."""

    def __init__(self, cfg: PolicyConfig, node: Node) -> None:
        """Initialize the policy with configuration and ROS node.

        Subclasses usually:
        - load model weights,
        - build preprocessing pipelines,
        - move model to the correct device.
        """
        self.cfg = cfg
        self.node = node

    def make_observation(
        self,
        ros_obs: Mapping[str, Any],
    ) -> Any:
        """Convert ROS messages into model-ready observation.

        Subclasses must override this method to:
        - convert Image to tensors,
        - reorder / normalize joint_state,
        - build a dict matching the LeRobot policy expectations.
        """
        raise NotImplementedError

    def infer(
        self,
        ros_obs: Mapping[str, Any],
        time_per_action: float,
        inference_delay: float = 0.0,
    ) -> None:
        """Update internal action buffer / queue from the latest observation."""
        if ros_obs is None or 'observation.state' not in ros_obs:
            self.node.get_logger().log(
                f'[{__class__.__name__}] Observation is missing.',
                severity=LoggingSeverity.WARN,
                throttle_duration_sec=2.0,
            )
            return

        # Build model-ready observation
        observation = self.make_observation(ros_obs=ros_obs)

        # Run policy to fill internal action buffer
        self.predict_action_chunk(observation, time_per_action, inference_delay)

    def predict_action_chunk(
        self, ros_obs: Mapping[str, Any], time_per_action: float, inference_delay: float
    ) -> None:
        """Run policy and return a sequence of future joint positions.
        Subclasses must override this method to:
        - run model inference,
        - fill internal action buffer with predicted actions.
        """
        raise NotImplementedError

    def get_action(self) -> Optional[List[float]]:
        """Return the next action to execute.

        Subclasses must override this method to:
        - return the next action from internal buffer,
        - return None if no action is available.
        """
        raise NotImplementedError

    def reset(self, context: Optional[Mapping[str, Any]] = None) -> None:
        """Reset any internal state for a new episode.

        Subclasses may override to handle language prompts, goals, etc.
        """
        raise NotImplementedError
