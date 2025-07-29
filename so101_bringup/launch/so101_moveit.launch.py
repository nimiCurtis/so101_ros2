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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)


def generate_launch_description():

    # --- Paths ---
    bringup_pkg = get_package_share_directory("so101_bringup")
    moveit_pkg = get_package_share_directory("so101_moveit")
    description_pkg = get_package_share_directory("so101_description")

    # --- Declare arguments ---
    robot_type_arg = DeclareLaunchArgument(
        "type",
        default_value="follower",
        description="Robot type: follower / leader",
    )

    display_config_arg = DeclareLaunchArgument(
        "display_config",
        default_value=os.path.join(
            get_package_share_directory("so101_moveit"),
            "rviz",
            "moveit.rviz",
        ),
    )
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="real", description="System mode: gazebo / real / sim"
    )
    display_arg = DeclareLaunchArgument(
        "display", default_value="false", description="Launch RViz or not"
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            description_pkg, "urdf", "so101_new_calib.urdf.xacro"
        ),
    )

    # --- LaunchConfigurations ---
    mode = LaunchConfiguration("mode")
    display = LaunchConfiguration("display")
    model = LaunchConfiguration("model")
    display_config = LaunchConfiguration("display_config")
    robot_type = LaunchConfiguration("type")

    # --- Conditionally include Gazebo sim if mode == gazebo ---
    sim_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "sim_gazebo.launch.py")
        ),
        condition=IfCondition(EqualsSubstitution(mode, "gazebo")),
        launch_arguments={
            "model": model,
            "display_config": display_config,  ## Not in use
        }.items(),
    )

    # --- Conditionally include Real Ros2 Robot Brodge if mode == real ---
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "robot.launch.py")
        ),
        launch_arguments={"type": robot_type, "model": model}.items(),
        condition=IfCondition(EqualsSubstitution(mode, "real")),
    )

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

    return LaunchDescription(
        [
            robot_type_arg,
            mode_arg,
            display_arg,
            display_config_arg,
            model_arg,
            sim_gazebo_launch,
            robot_launch,
            moveit_launch,
        ]
    )
