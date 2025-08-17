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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    args = []
    actions = []

    # Paths
    description_pkg = get_package_share_directory("so101_description")
    bringup_pkg = get_package_share_directory("so101_bringup")

    # --- Declare arguments ---
    display_config_arg = DeclareLaunchArgument(
        "display_config",
        default_value=os.path.join(
            description_pkg,
            "rviz",
            "display.rviz",
        ),
    )
    args.append(display_config_arg)

    display_arg = DeclareLaunchArgument(
        "display", default_value="false", description="Launch RViz or not"
    )
    args.append(display_arg)

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            description_pkg, "urdf", "so101_new_calib.urdf.xacro"
        ),
    )
    args.append(model_arg)

    model = LaunchConfiguration("model")
    display_config = LaunchConfiguration("display_config")
    display = LaunchConfiguration("display")

    # Include gazebo simulation
    sim_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "model": model,
            "display_config": display_config,  ## Not in use
        }.items(),
    )
    actions.append(sim_gazebo_launch)

    # Include display.launch.py
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, "launch", "display.launch.py")
        ),
        launch_arguments={
            "joint_states_gui": "false",
            "display_config": display_config,
        }.items(),
    )

    delayed_display = TimerAction(
        period=3.0, actions=[display_launch], condition=IfCondition(display)
    )
    actions.append(delayed_display)

    return LaunchDescription(args + actions)
