from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

from rclpy.node import Node

from so101_ros2_bridge import POLICY_BASE_DIR, POLICY_SHARE_BASE_DIR


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
        """Create PolicyConfig from a YAML file in the policy config directory."""
        import yaml

        extra_config_path = POLICY_SHARE_BASE_DIR / f'{policy_name}' / 'config.yaml'

        # check that the config file exists
        if not extra_config_path.exists():
            raise FileNotFoundError(f'Policy config file not found: {extra_config_path}')

        # Load YAML config
        with open(extra_config_path, 'r') as f:
            extra_config = yaml.safe_load(f)

        # If checkpoint_path is relative, make it relative to the POLICY_BASE_DIR
        if checkpoint_path and not checkpoint_path.startswith('/'):
            checkpoint_path = (POLICY_BASE_DIR / checkpoint_path / 'pretrained_model').resolve()
            checkpoint_path = str(checkpoint_path)

        return cls(
            policy_name=policy_name,
            device=device,
            checkpoint_path=str(checkpoint_path),
            task=task,
            robot_properties=robot_properties,
            extra=extra_config.get('extra', {}),
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
            self.node.get_logger().warn(f'[{self.cfg.policy_name}] infer: observation is missing.')
            return

        observation = self.make_observation(ros_obs=ros_obs)
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
