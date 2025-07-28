# MIT License
#
# Copyright (c) 2025 nimiCurtis
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

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():

    # Declare launch argument for mode
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="real", description="Execution mode: real, gazebo, isaac"
    )
    mode = LaunchConfiguration("mode")

    # Paths
    robot_description_share_dir = get_package_share_directory("so101_description")
    robot_description_f = os.path.join(
        robot_description_share_dir, "urdf", "so101_new_calib.urdf.xacro"
    )
    joint_limits_f = os.path.join(
        robot_description_share_dir, "config", "joint_limits.yaml"
    )
    moveit_controllers_f = "config/moveit_controllers.yaml"
    srdf_f = "config/so101_new_calib.srdf"

    # MoveIt config with dynamic Xacro arg
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so101_moveit")
        .robot_description(file_path=robot_description_f, mappings={"mode": mode})
        .robot_description_semantic(file_path=srdf_f)
        .trajectory_execution(file_path=moveit_controllers_f)
        .joint_limits(file_path=joint_limits_f)
        .to_moveit_configs()
    )

    # Load Servo YAML
    servo_yaml = load_yaml("so101_moveit", "config/so101_servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Launch the Servo node in a composable container
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            mode_arg,
            container,
        ]
    )
