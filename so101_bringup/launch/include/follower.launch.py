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
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Paths
    robot_ros2_bridge_pkg = get_package_share_directory("so101_ros2_bridge")
    controller_pkg = get_package_share_directory("so101_controller")
    description_pkg = get_package_share_directory("so101_description")

    model = LaunchConfiguration("model")

    # Include RSP with sim settings
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "model": model,
            "mode": "real",
            "use_sim": "false",
            "type": "follower",
        }.items(),
    )

    # Include robot ros2 bridge
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_ros2_bridge_pkg, "launch", "so101_ros2_bridge.launch.py")
        ),
        launch_arguments={"type": "follower"}.items(),
    )

    # Include controller manger
    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, "launch", "controller_manager.launch.py")
        ),
        launch_arguments={"type": "follower"}.items(),
    )

    # Include controller spawners
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, "launch", "so101_controllers.launch.py")
        ),
        launch_arguments={"type": "follower"}.items(),
    )

    delayed_controller_manager = TimerAction(
        period=3.0, actions=[controller_manager_launch]
    )

    delayed_spawn_controllers = TimerAction(
        period=6.0, actions=[spawn_controllers_launch]
    )

    return LaunchDescription(
        [
            rsp_launch,
            robot_launch,
            delayed_controller_manager,
            delayed_spawn_controllers,
        ]
    )
