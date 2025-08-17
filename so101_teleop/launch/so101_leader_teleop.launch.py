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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Launch configuration of "mode"
    mode = LaunchConfiguration("mode")

    # Launch Leader as a component
    container = ComposableNodeContainer(
        name="leader_teleop_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",  # Use multithreaded for best performance
        composable_node_descriptions=[
            ComposableNode(
                package="so101_teleop",
                plugin="so101_teleop::LeaderTeleopComponent",
                name="leader_teleop_component",  # This name must match the one in the YAML file
                parameters=[
                    os.path.join(
                        get_package_share_directory("so101_teleop"),
                        "config",
                        "so101_leader_teleop.yaml",
                    ),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True, "use_sim_time": mode == "gazebo"}
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
