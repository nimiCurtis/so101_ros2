# Copyright 2025 nimiCurtis
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
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # Launch description lists
    args = []
    groups = []

    # Paths
    bringup_pkg = get_package_share_directory("so101_bringup")
    description_pkg = get_package_share_directory("so101_description")

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

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="real",
        description="Execution mode: real, gazebo",
    )
    args.append(mode_arg)

    model = LaunchConfiguration("model")
    display_config = LaunchConfiguration("display_config")
    display = LaunchConfiguration("display")
    mode = LaunchConfiguration("mode")

    # Launch leader
    leader_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "robot.launch.py")
        ),
        launch_arguments={
            "type": "leader",
            "model": model,
            # CORRECTED: Use PythonExpression to conditionally set the value
            "use_sim_time": PythonExpression(
                ["'true' if '", mode, "' == 'gazebo' else 'false'"]
            ),
        }.items(),
    )

    # Launch follower
    # Include follower robot launch if mode == "real"
    # Include sim_gazebo.launch if mode == "gazebo"
    follower_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "robot.launch.py")
        ),
        launch_arguments={"type": "follower", "model": model}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
    )

    # Include sim_gazebo.launch if mode == "gazebo"
    follower_sim_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "sim_gazebo.launch.py")
        ),
        launch_arguments={"model": model}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'gazebo'"])),
    )

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

    # Include leader teleop
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_teleop"),
                "launch",
                "so101_leader_teleop.launch.py",
            )
        ),
        launch_arguments={
            "mode": LaunchConfiguration("mode"),
        }.items(),
    )

    # Prepare groups
    groups.append(
        GroupAction(
            [
                PushRosNamespace("leader"),
                leader_robot_launch,
            ]
        )
    )
    groups.append(
        GroupAction(
            [
                PushRosNamespace("follower"),
                follower_robot_launch,
                follower_sim_gazebo_launch,
            ]
        )
    )
    groups.append(
        GroupAction(
            [
                teleop_launch,
                delayed_display,
            ]
        )
    )

    return LaunchDescription(args + groups)
