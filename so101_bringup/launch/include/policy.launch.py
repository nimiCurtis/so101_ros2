from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bridge_pkg = get_package_share_directory('so101_ros2_bridge')

    # Resolve launch arguments *in this context*
    policy_name = LaunchConfiguration('policy_name').perform(context)
    checkpoint_path = LaunchConfiguration('checkpoint_path').perform(context)
    task = LaunchConfiguration('task').perform(context)

    params_file = os.path.join(
        bridge_pkg,
        'config',
        'so101_policy_params.yaml',
    )

    # Start with the YAML params
    params = [params_file]

    # Only override if the user actually provided something non-empty
    if policy_name:
        params.append({'policy_name': policy_name})

    if checkpoint_path:
        # If this is empty string, we skip it and keep the YAML value
        params.append({'checkpoint_path': checkpoint_path})

    if task:
        params.append({'task': task})

    policy_node = Node(
        package='so101_ros2_bridge',
        executable='policy_runner_ros2_node',
        name='policy_runner',
        output='screen',
        parameters=params,
    )

    return [policy_node]


def generate_launch_description() -> LaunchDescription:
    policy_name_arg = DeclareLaunchArgument(
        'policy_name',
        default_value='smolvla',
        description='Policy name (smolvla, pi0, pi05, groot, ...).',
    )

    checkpoint_path_arg = DeclareLaunchArgument(
        'checkpoint_path',
        default_value='',
        description='Path to the policy checkpoint. '
        'If empty, the value from the YAML will be used.',
    )

    task_arg = DeclareLaunchArgument(
        'task',
        default_value='Pick the cube and place it inside the bowl.',
        description='Task prompt (e.g., pick and place, navigation).',
    )

    return LaunchDescription(
        [
            policy_name_arg,
            checkpoint_path_arg,
            task_arg,
            # Node is created via OpaqueFunction so we can decide params dynamically
            OpaqueFunction(function=launch_setup),
        ]
    )
