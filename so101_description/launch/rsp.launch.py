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
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    so101_description_share_dir = get_package_share_directory("so101_description")

    # Declare arguments
    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="real",
        description="Execution mode: gazebo / isaac / real",
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            so101_description_share_dir, "urdf", "so101_new_calib.urdf.xacro"
        ),
        description="Absolute path to the robot URDF/xacro file",
    )

    use_sim_arg = DeclareLaunchArgument(
        name="use_sim",
        default_value="false",
        description="Use simulation time (true for sim environments)",
    )

    # Launch configurations
    model = LaunchConfiguration("model")
    mode = LaunchConfiguration("mode")
    use_sim = LaunchConfiguration("use_sim")

    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                model,
                " ",
                "mode:=",
                mode,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim},
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            mode_arg,
            use_sim_arg,
            robot_state_publisher_node,
        ]
    )
