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
from pathlib import Path
from typing import List, Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

USB_CAM_DIR = get_package_share_directory('so101_bringup')


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError as e:
        print(f"Failed to load {absolute_file_path}: {e}")
        return None


class CameraConfig:
    def __init__(
        self,
        name: str,
        param_path: Path,
        remappings: Optional[List] = None,
        namespace: Optional[str] = None,
    ):
        self.name = name
        self.namespace = namespace
        self.param_path = param_path
        self._validate_param_path()

        if remappings is None:
            remappings = [
                ('image_raw', f'{name}/image_raw'),
                ('image_raw/compressed', f'{name}/image_compressed'),
                ('image_raw/compressedDepth', f'{name}/compressedDepth'),
                ('image_raw/theora', f'{name}/image_raw/theora'),
                ('camera_info', f'{name}/camera_info'),
            ]
        self.remappings = remappings

    def _validate_param_path(self):
        if not self.param_path.exists():
            raise FileNotFoundError(f'Could not find parameter file: {self.param_path}')


def get_camera_configs(
    config_file: str = 'config/so101_cameras.yaml',
) -> List[CameraConfig]:
    yaml_config = load_yaml('so101_bringup', config_file)
    cameras: List[CameraConfig] = []

    if yaml_config and 'cameras' in yaml_config:
        for cam in yaml_config['cameras']:
            name = cam.get('name')
            relative_param_path = cam.get('param_path')
            full_param_path = Path(USB_CAM_DIR, 'config', relative_param_path)
            namespace = cam.get('namespace', None)
            cameras.append(
                CameraConfig(name=name, param_path=full_param_path, namespace=namespace)
            )
    else:
        raise RuntimeError(f"No valid 'cameras' list found in {config_file}")

    return cameras


def generate_launch_description():
    camera_configs = get_camera_configs()

    camera_nodes = [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[camera.param_path],
            remappings=camera.remappings,
        )
        for camera in camera_configs
    ]

    return LaunchDescription([GroupAction(camera_nodes)])
