from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the policy runner node with params and policy_type-based name."""
    policy_type = LaunchConfiguration('policy_type')

    policy_type_arg = DeclareLaunchArgument(
        'policy_type',
        default_value='smolvla',
        description='Policy type / node name (smolvla, pi0, pi05, groot, ...).',
    )

    pkg_share = get_package_share_directory('so101_ros2_bridge')
    params_file = os.path.join(
        pkg_share,
        'config',
        'so101_policy_params.yaml',
    )

    policy_node = Node(
        package='so101_ros2_bridge',
        executable='so101_policy_runner',  # entry point for PolicyRunnerNode
        name=policy_type,  # node name == policy_type
        output='screen',
        parameters=[params_file],
        # Add remappings if needed for your actual topics:
        # remappings=[
        #     ("/camera_top/image_raw", "/so101/cam_top/image_raw"),
        #     ("/camera_wrist/image_raw", "/so101/cam_wrist/image_raw"),
        #     ("/follower/joint_states", "/so101/follower/joint_states"),
        # ],
    )

    return LaunchDescription(
        [
            policy_type_arg,
            policy_node,
        ]
    )
