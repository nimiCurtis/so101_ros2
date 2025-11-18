from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Type

from rclpy.node import Node
from rclpy.parameter import ParameterType
from sensor_msgs.msg import Image, JointState


@dataclass
class PolicyConfig:
    """Configuration for a policy adapter.

    Attributes:
        policy_name: Name used for registry lookup (e.g., 'smolvla', 'pi0').
        device: Target device string (e.g., 'cuda:0', 'cpu').
        checkpoint_path: Path or identifier for the model weights.
        extra: Additional free-form configuration parameters.
    """

    policy_name: str
    device: str = 'cuda:0'
    checkpoint_path: Optional[str] = None
    extra: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_ros_params(cls, node: Node) -> 'PolicyConfig':
        """Build configuration from ROS parameters on the given node."""
        policy_name = node.get_parameter('policy_name').get_parameter_value().string_value
        device_param = node.get_parameter_or(
            'device',
            node._parameters.get('device', None),  # type: ignore[attr-defined]
        )
        if device_param is not None:
            device = device_param.get_parameter_value().string_value  # type: ignore[union-attr]
        else:
            device = 'cuda:0'

        checkpoint_param = node.get_parameter_or(
            'checkpoint_path',
            node._parameters.get('checkpoint_path', None),  # type: ignore[attr-defined]
        )
        if checkpoint_param is not None:
            checkpoint_path = (
                checkpoint_param.get_parameter_value().string_value  # type: ignore[union-attr]
            )
        else:
            checkpoint_path = None

        # Optional: read a small set of known extras
        extra: Dict[str, Any] = {}

        for name in [
            'normalize_observations',
            'observation_image_size',
            'max_joint_delta',
            'max_joint_velocity',
            'action_scaling',
            'text_prompt',
            'use_language_conditioning',
            'horizon',
            'temperature',
        ]:
            if node.has_parameter(name):
                p = node.get_parameter(name)
                v = p.get_parameter_value()

                # Match rclpy ParameterType enums
                if v.type == ParameterType.PARAMETER_DOUBLE:
                    extra[name] = v.double_value

                elif v.type == ParameterType.PARAMETER_INTEGER:
                    extra[name] = v.integer_value

                elif v.type == ParameterType.PARAMETER_STRING:
                    extra[name] = v.string_value

                elif v.type == ParameterType.PARAMETER_BOOL:
                    extra[name] = v.bool_value

                elif v.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    extra[name] = list(v.double_array_value)

                elif v.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                    extra[name] = list(v.integer_array_value)

                elif v.type == ParameterType.PARAMETER_STRING_ARRAY:
                    extra[name] = list(v.string_array_value)

                elif v.type == ParameterType.PARAMETER_BOOL_ARRAY:
                    extra[name] = list(v.bool_array_value)

                else:
                    node.get_logger().warn(
                        f"Parameter '{name}' has unsupported type {v.type}, skipping."
                    )

        return cls(
            policy_name=policy_name,
            device=device,
            checkpoint_path=checkpoint_path,
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

    # ---- Episode control -------------------------------------------------

    def reset(self, context: Optional[Mapping[str, Any]] = None) -> None:
        """Reset any internal state for a new episode.

        Subclasses may override to handle language prompts, goals, etc.
        """
        # Default: stateless
        _ = context

    # ---- Core API --------------------------------------------------------

    def make_observation(
        self,
        ros_obs: Mapping[str, Any],
    ) -> Dict[str, Any]:
        """Convert ROS messages into model-ready observation.

        Subclasses must override this method to:
        - convert Image to tensors,
        - reorder / normalize joint_state,
        - build a dict matching the LeRobot policy expectations.
        """
        raise NotImplementedError

    def act_sequence(self, obs: Mapping[str, Any]) -> List[List[float]]:
        """Run policy and return a sequence of future joint positions.

        The result should be a list of length T, each element a list
        of length n_joints:

            [
              [q_0_joint_0, ..., q_0_joint_N],
              [q_1_joint_0, ..., q_1_joint_N],
              ...
            ]

        The PolicyRunnerNode computes joint velocities from these
        commanded positions internally.
        """
        raise NotImplementedError

    # ---- Optional hooks --------------------------------------------------

    def close(self) -> None:
        """Release any resources such as GPU memory or file handles."""
        # Default: nothing to do
        return
