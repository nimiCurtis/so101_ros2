# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

PKG_BRINGUP = 'so101_bringup'
SHARE_DIR = get_package_share_directory(PKG_BRINGUP)

# Map "type" from the cameras YAML to (package, executable)
TYPE_REGISTRY: Dict[str, Tuple[str, str]] = {
    'usb_camera': ('usb_cam', 'usb_cam_node_exe'),
    'realsense2_camera': ('realsense2_camera', 'realsense2_camera_node'),
}


def load_yaml_from_pkg(package_name: str, rel_path: str) -> Any:
    """Load YAML at <share(package)>/<rel_path>."""
    package_path = get_package_share_directory(package_name)
    abs_path = os.path.join(package_path, rel_path)
    with open(abs_path, 'r') as f:
        return yaml.safe_load(f)


@dataclass
class CameraSpec:
    name: str
    camera_type: str
    param_path: Path
    namespace: Optional[str] = None
    remappings: Optional[List[Tuple[str, str]]] = None  # optional per-camera overrides


def _resolve_param_path(relative_param_path: str) -> Path:
    # Your param files live under PKG_BRINGUP/config/<file>.yaml
    p = Path(SHARE_DIR) / 'config' / relative_param_path
    if not p.exists():
        raise FileNotFoundError(f'Parameter file not found: {p}')
    return p


def parse_cameras_config(
    config_file_rel: str = 'config/so101_cameras.yaml',
) -> List[CameraSpec]:
    cfg = load_yaml_from_pkg(PKG_BRINGUP, config_file_rel)
    if not cfg or 'cameras' not in cfg:
        raise RuntimeError(f"No valid 'cameras' list found in {config_file_rel}")

    cams: List[CameraSpec] = []
    for cam in cfg['cameras']:
        name = cam.get('name')
        cam_type = cam.get('camera_type')
        rel_param = cam.get('param_path')
        namespace = cam.get('namespace')
        remappings = cam.get('remappings')  # optional

        if not name or not cam_type or not rel_param:
            raise ValueError(
                f"Camera entry is missing one of 'name'/'camera_type'/'param_path': {cam}"
            )
        if cam_type not in TYPE_REGISTRY:
            raise ValueError(
                f"Unknown camera type '{cam_type}'. Supported: {list(TYPE_REGISTRY.keys())}"
            )

        cams.append(
            CameraSpec(
                name=name,
                camera_type=cam_type,
                param_path=_resolve_param_path(rel_param),
                namespace=namespace,
                remappings=remappings,  # may be None
            )
        )
    return cams


def build_node_for_camera(spec: CameraSpec) -> Node:
    pkg, exe = TYPE_REGISTRY[spec.camera_type]

    # Common override: ensure camera_name matches the camera "name" from the list
    # Both usb_cam and realsense2_camera honor a 'camera_name' parameter.
    overrides: Dict[str, Any] = {
        'camera_name': spec.name,
    }

    # Optional: sensible defaults that often help (comment/uncomment as needed)
    # For usb_cam you might also want to use a frame_id matching the camera name:
    if spec.camera_type == 'usb_camera':
        overrides.setdefault('frame_id', spec.name)

    # Build parameters list: first the YAML file path, then the overrides dict.
    # ROS 2 merges later entries over earlier ones, so our overrides win.
    parameters = [str(spec.param_path), overrides]

    # Remappings:
    # - If you didn’t define custom remappings in the YAML, you can keep defaults.
    # - Example for usb_cam (single stream):
    default_usb_cam_remaps = [
        ('image_raw', f'{spec.name}/image_raw'),
        ('image_raw/compressed', f'{spec.name}/image_compressed'),
        ('image_raw/compressedDepth', f'{spec.name}/compressedDepth'),
        ('image_raw/theora', f'{spec.name}/image_raw/theora'),
        ('camera_info', f'{spec.name}/camera_info'),
    ]

    # For RealSense, topic structure is richer; if you want uniform names,
    # define them per-camera in so101_cameras.yaml under 'remappings'.
    remappings = spec.remappings
    if remappings is None and spec.camera_type == 'usb_camera':
        remappings = default_usb_cam_remaps

    return Node(
        package=pkg,
        executable=exe,
        name=spec.name,  # node name (distinct per camera)
        namespace=spec.namespace,  # logical grouping per your config
        output='screen',
        parameters=parameters,
        remappings=remappings or [],
    )


def generate_launch_description():
    camera_specs = parse_cameras_config()

    # Build one Node per camera
    nodes = [build_node_for_camera(spec) for spec in camera_specs]

    # GroupAction keeps the launch tree tidy; you could also just return the nodes directly
    return LaunchDescription([GroupAction(nodes)])
