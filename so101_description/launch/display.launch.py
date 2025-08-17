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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
        namespace="follower",
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
