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
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare the 'type' argument so it can be set from the command line
    # type_arg = DeclareLaunchArgument(
    #     "type",
    #     default_value="leader",
    #     description="The type of the robot bridge to launch (leader or follower)."
    # )

    robot_type = LaunchConfiguration("type")

    # Use PathJoinSubstitution to build the path to the config file dynamically
    config_path = PathJoinSubstitution(
        [
            FindPackageShare("so101_ros2_bridge"),
            "config",
            [
                TextSubstitution(text="so101_"),
                robot_type,
                TextSubstitution(text="_params.yaml"),
            ],
        ]
    )

    # Define the node, dynamically setting the executable, name, and parameters
    so101_bridge_node = Node(
        package='so101_ros2_bridge',
        executable=[
            robot_type,
            TextSubstitution(text='_ros2_node'),
        ],
        name=[
            TextSubstitution(text='so101_'),
            robot_type,
            TextSubstitution(text='_interface'),
        ],
        output='screen',
        parameters=[
            config_path,
            # Pass the robot_type as a ROS parameter named 'type'
            {"type": robot_type},
        ],
        namespace=robot_type,
    )

    # This event handler will now only shut down the launch script if the node
    # crashes (exits with a non-zero code). A clean exit (Ctrl+C) will be ignored.
    # node_monitor = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=so101_bridge_node,
    #         on_exit=[
    #             Shutdown(
    #                 reason=f"Node '{so101_bridge_node.name}' crashed, shutting down launch."
    #             )
    #         ],
    #         # This condition ensures the shutdown only happens on a crash
    #         # condition=lambda event: event.returncode != 0,
    #     )
    # )

    return LaunchDescription(
        [
            # type_arg,
            so101_bridge_node,
            # node_monitor,
        ]
    )
