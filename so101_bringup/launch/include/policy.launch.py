from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the policy runner node with params and policy_name-based name."""
    bridge_pkg = get_package_share_directory('so101_ros2_bridge')

    policy_name = LaunchConfiguration('policy_name')
    checkpoint_path = LaunchConfiguration('checkpoint_path')
    task = LaunchConfiguration('task')

    policy_name_arg = DeclareLaunchArgument(
        'policy_name',
        default_value='smolvla',
        description='Policy name (smolvla, pi0, pi05, groot, ...).',
    )

    checkpoint_path_arg = DeclareLaunchArgument(
        'checkpoint_path',
        default_value='',
        description='Path to the policy checkpoint.',
    )

    task_arg = DeclareLaunchArgument(
        'task',
        default_value='Pick the cube and place it inside the bowl.',
        description='Task prompt (e.g., pick and place, navigation).',
    )

    params_file = os.path.join(
        bridge_pkg,
        'config',
        'so101_policy_params.yaml',
    )

    policy_node = Node(
        package='so101_ros2_bridge',
        executable='so101_policy_runner',  # entry point for PolicyRunnerNode
        name='policy_runner',  # node name == policy_name
        output='screen',
        parameters=[
            params_file,
            # Override parameters from launch args
            {
                'policy_name': policy_name,
                'checkpoint_path': checkpoint_path,
                'task': task,
            },
        ],
        # Add remappings if needed
        # remappings=[
        # ],
    )

    return LaunchDescription(
        [
            policy_name_arg,
            checkpoint_path_arg,
            task_arg,
            policy_node,
        ]
    )
