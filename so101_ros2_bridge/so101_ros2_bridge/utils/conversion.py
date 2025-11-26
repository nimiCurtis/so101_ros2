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
    return (rad / math.pi) * 100.0


def ros_jointstate_to_vec(
    js_msg: JointState,
    joint_order: Optional[List[str]] = None,
    use_lerobot_ranges_norms: bool = False,
) -> Tuple[np.ndarray, List[str]]:
    """Convert a JointState message to a vector in the given joint_order.

    If joint_order is provided, return joints in that order.
    Otherwise, return all joints in the message.
    """
    pos = list(getattr(js_msg, 'position', []))
    names = list(getattr(js_msg, 'name', []))

    if joint_order:
        # Use ordered subset from joint_order
        name_to_idx = {n: i for i, n in enumerate(names)}
        try:
            vals = [pos[name_to_idx[n]] for n in joint_order]
        except KeyError as e:
            missing = set(joint_order) - set(names)
            raise KeyError(f'Joint(s) {missing} not found in JointState.name') from e

        if use_lerobot_ranges_norms:
            vals = [radians_to_normalized(j_name, v) for j_name, v in zip(joint_order, vals)]

        out = np.asarray(vals, dtype=np.float32)
        joint_names = list(joint_order)
    else:
        # Use everything as-is
        if not pos:
            raise ValueError('JointState.position is empty')

        joint_names = names if names else [f'joint_{i}' for i in range(len(pos))]
        vals = pos

        if use_lerobot_ranges_norms:
            vals = [radians_to_normalized(j_name, v) for j_name, v in zip(joint_names, vals)]

        out = np.asarray(vals, dtype=np.float32)

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

        joint_vec, joint_names = ros_jointstate_to_vec(
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
