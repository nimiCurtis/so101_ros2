from __future__ import annotations

from curses import raw
from shlex import join

# Ensure the conda site-packages directory is in the system path
from so101_ros2_bridge.utils import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()

import math
from typing import Any, Dict, List, Mapping, Optional

import numpy as np
from lerobot.configs.types import FeatureType, PolicyFeature
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
        node.get_logger().info(f'Input features: {self._input_features}')
        node.get_logger().info(f'Output features: {self._output_features}')

        self.setup_dataset_features()

    def setup_dataset_features(self) -> None:
        """Setup dataset features based on the model's config."""
        self.dataset_features: Dict[str, Dict] = {}

        state_names = [f'{joint}.pos' for joint in self.JOINT_NAMES] if self.JOINT_NAMES else None

        action_names = [f'{joint}.pos' for joint in self.JOINT_NAMES] if self.JOINT_NAMES else None

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
    # Observation construction
    # ------------------------------------------------------------------
    def make_observation(
        self,
        ros_obs: Mapping[str, Any],  # e.g., images: Dict[str, Image], joint_state: JointState
    ) -> EnvTransition:
        """Convert ROS messages into a LeRobot-style observation dict."""

        raw_obs_features: dict[str, dict] = ros_to_dataset_features(
            ros_obs=ros_obs, input_features=self._input_features
        )

        # Build the inference frame expected by SmolVLA
        inference_frame = build_inference_frame(
            observation=raw_obs_features,
            ds_features=self.dataset_features,
            device=self._device,
            task=self._task,
            robot_type='so101_follower',
        )

        # Preprocess the inference frame (e.g., to tensors, normalization, etc.)
        observation: EnvTransition = self._pre_processor(inference_frame)

        return observation

    def act_sequence(self, observation: EnvTransition) -> List[List[float]]:
        """Run SmolVLA to get a sequence of future joint positions.

        This is a stub implementation. Replace the internals with real model
        inference that returns positions in the correct joint order and units.
        """
        action_chunk = self.model.predict_action_chunk(observation)
        action_chunk = self._post_processor(action_chunk)
        return action_chunk

    def reset(self, context: Optional[Mapping[str, Any]] = None) -> None:
        self.model.reset()

    def close(self):
        return super().close()


def ros_to_dataset_features(
    ros_obs: Mapping[str, Any],
    input_features: dict[str, PolicyFeature],
) -> dict[str, Any]:
    """Convert ROS observation messages to the raw values dict
    expected by LeRobot's build_dataset_frame.
    """

    values: dict[str, Any] = {}

    # 1) STATE features -> per-joint keys like "shoulder_pan.pos"
    if 'observation.state' in ros_obs:
        joint_state_msg: JointState = ros_obs['observation.state']

        # Make sure we use the same order as in SmolVLA.JOINT_NAMES
        joint_vec, joint_names = ros_jointstate_to_vec6(
            js_msg=joint_state_msg,
            joint_order=SmolVLA.JOINT_NAMES,
            use_lerobot_ranges_norms=False,  # or True if you want normalized values
        )

        for name, val in zip(joint_names, joint_vec):
            values[f'{name}.pos'] = float(val)

    # 2) VISUAL features -> keys like "camera1", "camera2"
    for key, feature in input_features.items():
        if feature.type.value != 'VISUAL':
            continue

        # key is e.g. "observation.images.camera1"
        if key not in ros_obs:
            raise ValueError(f"Image for VISUAL feature key '{key}' not found in ROS observation.")

        img_msg: Image = ros_obs[key]
        np_img = ros_image_to_hwc_float01(img_msg)

        # LeRobot's build_dataset_frame expects values["camera1"], "camera2", ...
        cam_id = key.split('.images.', 1)[1]  # "camera1"
        values[cam_id] = np_img

    return values


def ros_image_to_hwc_float01(msg) -> np.ndarray:
    """Convert a ROS image message to an HWC float array scaled to ``[0, 1]``.

    Args:
        msg: A ROS ``sensor_msgs/Image`` like object with ``encoding``, ``height``,
            ``width`` and ``data`` attributes.

    Returns:
        ``numpy.ndarray`` with shape ``(height, width, 3)`` containing float32 values in
        the range ``[0, 1]``.

    Raises:
        ValueError: If the encoding is not one of ``rgb8``, ``bgr8`` or ``mono8``.
    """

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
    """Convert a radian command into the normalized SO101 joint range.

    Args:
        joint_name: Name of the joint for which the command is expressed. The gripper has
            a special conversion.
        rad: Command expressed in radians as provided by MoveIt.

    Returns:
        The normalized joint value expected by the SO101 API.
    """
    if joint_name == 'gripper':
        # Convert radian command [0, pi] to the robot's expected gripper range [0, 100]
        normalized = (rad / math.pi) * 100.0
    else:
        # Convert radians to normalized range [-100, 100]
        normalized = (rad / math.pi) * 100.0
    return normalized


def ros_jointstate_to_vec6(
    js_msg,
    joint_order: Optional[List[str]] = None,
    use_lerobot_ranges_norms: bool = False,
) -> np.ndarray:
    """Convert a ``sensor_msgs/JointState`` message to a six element vector.

    Args:
        js_msg: Message providing ``position`` and optionally ``name`` attributes.
        joint_order: Optional explicit joint ordering for the returned vector.
        use_lerobot_ranges_norms: Whether to map the values to LeRobot's normalized
            ranges using :func:`radians_to_normalized`.

    Returns:
        ``numpy.ndarray`` shaped ``(6,)`` containing the joint positions.

    Raises:
        ValueError: If the provided message does not contain enough joint positions or
            the ``joint_order`` does not contain exactly six joints.
        KeyError: If a name specified in ``joint_order`` is missing in the message.
    """

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
            # When no joint_order is provided, use the joint names from the message
            joint_names = names[:6] if len(names) >= 6 else [f'joint_{i}' for i in range(6)]
            print(joint_names)
            vals = [
                radians_to_normalized(joint_name, val) for joint_name, val in zip(joint_names, vals)
            ]
        out[:] = np.array(vals, dtype=np.float32)

    return out, joint_names
