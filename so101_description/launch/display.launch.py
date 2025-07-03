import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description():
    so101_description_share_dir = get_package_share_directory("so101_description")

    # Declare arguments
    joint_states_gui_arg = DeclareLaunchArgument(
        "joint_states_gui",
        default_value="false",
        description="Use joint_state_publisher_gui",
    )

    display_config_arg = DeclareLaunchArgument(
        "display_config",
        default_value=os.path.join(so101_description_share_dir, "rviz", "display.rviz"),
        description="Path to the RViz display config file",
    )

    joint_states_gui = LaunchConfiguration("joint_states_gui")
    display_config = LaunchConfiguration("display_config")

    # Only launch joint_state_publisher_gui if enabled
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(joint_states_gui),
    )

    # RViz node with configurable display config
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", display_config],
    )

    return LaunchDescription(
        [
            joint_states_gui_arg,
            display_config_arg,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
