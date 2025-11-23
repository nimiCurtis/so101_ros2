#!/usr/bin/env python3
from __future__ import annotations

from so101_ros2_bridge.utils import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()


import math
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np
from lerobot.configs.types import PolicyFeature
from sensor_msgs.msg import Image, JointState


def ros_image_to_hwc_uint8(msg: Image) -> np.ndarray:
    """Convert a ROS image message to HWC uint8 (0–255)."""
    enc = msg.encoding
    if enc not in ('rgb8', 'bgr8', 'mono8'):
        raise ValueError(f'Unsupported encoding: {enc}')
    ch = 3 if enc in ('rgb8', 'bgr8') else 1
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
    if enc == 'bgr8':
        arr = arr[..., ::-1]
    if ch == 1:
        arr = np.repeat(arr, 3, axis=2)
    return arr.astype(np.uint8)  # / 255.0


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


def ros_to_dataset_features(
    ros_obs: Mapping[str, Any],
    joint_order: Optional[List[str]],
    input_features: Dict[str, PolicyFeature],
) -> Dict[str, Any]:
    """Convert ROS observation messages to raw values dict for build_inference_frame."""
    values: Dict[str, Any] = {}

    if 'observation.state' in ros_obs:
        joint_state_msg: JointState = ros_obs['observation.state']

        joint_vec, joint_names = ros_jointstate_to_vec6(
            js_msg=joint_state_msg,
            joint_order=joint_order,
            use_lerobot_ranges_norms=False,
        )

        for name, val in zip(joint_names, joint_vec):
            values[f'{name}.pos'] = float(val)

    for key, feature in input_features.items():
        if feature.type.value != 'VISUAL':
            continue

        if key not in ros_obs:
            raise ValueError(f"Image for VISUAL feature key '{key}' not found in ROS observation.")

        img_msg: Image = ros_obs[key]
        np_img = ros_image_to_hwc_uint8(img_msg)

        cam_id = key.split('.images.', 1)[1]  # "camera1"
        values[cam_id] = np_img

    return values
