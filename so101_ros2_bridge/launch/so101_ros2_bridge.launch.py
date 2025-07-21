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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare top-level launch argument for robot type
    robot_type_arg = DeclareLaunchArgument(
        "type",
        default_value="follower",
        description="Robot type: follower / leader",
    )

    robot_type = LaunchConfiguration("type")

    # Use PathJoinSubstitution to build the path dynamically at launch time
    config_path = PathJoinSubstitution(
        [
            FindPackageShare("so101_ros2_bridge"),
            "config",
            [
                TextSubstitution(text='so101_'),
                robot_type,
                TextSubstitution(text='_params.yaml'),
            ],
        ]
    )

    so101_bridge_node = Node(
        package='so101_ros2_bridge',
        executable='so101_ros2_bridge_node',
        name=[
            TextSubstitution(text='so101_'),
            robot_type,
            TextSubstitution(text='_interface'),
        ],
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription(
        [robot_type_arg, so101_bridge_node]  # expose the 'type' arg to command line
    )
