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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    # Launch description lists
    args = []
    actions = []

    # --- Paths ---
    bringup_pkg = get_package_share_directory("so101_bringup")
    moveit_pkg = get_package_share_directory("so101_moveit")
    description_pkg = get_package_share_directory("so101_description")

    display_config_arg = DeclareLaunchArgument(
        "display_config",
        default_value=os.path.join(
            moveit_pkg,
            "rviz",
            "moveit.rviz",
        ),
    )
    args.append(display_config_arg)

    mode_arg = DeclareLaunchArgument(
        "mode", default_value="real", description="System mode: gazebo / real / sim"
    )
    args.append(mode_arg)

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

    # --- LaunchConfigurations ---
    mode = LaunchConfiguration("mode")
    display = LaunchConfiguration("display")
    model = LaunchConfiguration("model")
    display_config = LaunchConfiguration("display_config")

    # Launch follower
    # Include follower robot launch if mode == "real"
    # Include sim_gazebo.launch if mode == "gazebo"
    follower_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "follower.launch.py")
        ),
        launch_arguments={
            "model": model,
        }.items(),
        condition=LaunchConfigurationEquals("mode", "real"),
    )
    actions.append(follower_robot_launch)

    # Include sim_gazebo.launch if mode == "gazebo"
    sim_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "model": model,
            "display_config": display_config,  ## Not in use
        }.items(),
        condition=LaunchConfigurationEquals("mode", "gazebo"),
    )
    actions.append(sim_gazebo_launch)

    # --- Always include MoveIt launch file ---
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "moveit.launch.py")
        ),
        launch_arguments={
            "display": display,
            "display_config": display_config,
            "use_sim": PythonExpression(
                ["'", mode, "' in ['gazebo', 'isaac']"]
            ),  # evaluates to "true"/"false"
        }.items(),
    )
    actions.append(moveit_launch)

    return LaunchDescription(args + actions)
