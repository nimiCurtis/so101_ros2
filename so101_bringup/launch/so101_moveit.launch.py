# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to do so, subject to the
# following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)


def generate_launch_description():
    # Collect in lists (teleop style)
    args = []
    actions = []

    # --- Paths ---
    bringup_pkg = get_package_share_directory("so101_bringup")
    moveit_pkg = get_package_share_directory("so101_moveit")
    description_pkg = get_package_share_directory("so101_description")

    # --- Declare arguments ---
    robot_type_arg = DeclareLaunchArgument(
        "type",
        default_value="follower",
        description="Robot type: follower (only)",
    )
    args.append(robot_type_arg)

    display_config_arg = DeclareLaunchArgument(
        "display_config",
        default_value=os.path.join(moveit_pkg, "rviz", "moveit.rviz"),
        description="Path to RViz config file",
    )
    args.append(display_config_arg)

    moveit_mode_arg = DeclareLaunchArgument(
        "moveit_mode",
        default_value="real",
        description="Execution mode: real, gazebo, isaac",
    )
    args.append(moveit_mode_arg)

    display_arg = DeclareLaunchArgument(
        "display",
        default_value="false",
        description="Launch RViz or not",
    )
    args.append(display_arg)

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(description_pkg, "urdf", "so101_new_calib.urdf.xacro"),
        description="Path to the robot xacro/urdf",
    )
    args.append(model_arg)

    # --- LaunchConfigurations ---
    moveit_mode = LaunchConfiguration("moveit_mode")
    display = LaunchConfiguration("display")
    model = LaunchConfiguration("model")
    display_config = LaunchConfiguration("display_config")
    robot_type = LaunchConfiguration("type")

    # --- Logs (debug/helpful) ---
    actions.append(LogInfo(msg=["[MOVEIT LAUNCH] Mode is set to: ", moveit_mode]))
    actions.append(
        LogInfo(
            msg="[MOVEIT LAUNCH] Launching REAL robot bridge",
            condition=IfCondition(EqualsSubstitution(moveit_mode, "real")),
        )
    )
    actions.append(
        LogInfo(
            msg="[MOVEIT LAUNCH] Launching Gazebo simulation",
            condition=IfCondition(EqualsSubstitution(moveit_mode, "gazebo")),
        )
    )
    actions.append(
        LogInfo(
            msg="[MOVEIT LAUNCH] Launching Isaac simulation",
            condition=IfCondition(EqualsSubstitution(moveit_mode, "isaac")),
        )
    )

    # --- Real robot (bridge) when moveit_mode == real ---
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "follower.launch.py")
        ),
        launch_arguments={"type": robot_type, "model": model}.items(),
        condition=IfCondition(EqualsSubstitution(moveit_mode, "real")),
    )
    # Small delay can help hardware settle before MoveIt; tune if needed
    delayed_follower_launch = TimerAction(
        period=3.0,
        actions=[follower_launch],
        condition=IfCondition(EqualsSubstitution(moveit_mode, "real")),
    )
    actions.append(delayed_follower_launch)

    # --- Cameras only in real mode (mirrors teleop style) ---
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "camera.launch.py")
        ),
        condition=IfCondition(EqualsSubstitution(moveit_mode, "real")),
    )
    actions.append(cameras_launch)

    # --- Gazebo sim when moveit_mode == gazebo ---
    sim_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "model": model,
            "display_config": display_config,  # passthrough if your include supports it
        }.items(),
        condition=IfCondition(EqualsSubstitution(moveit_mode, "gazebo")),
    )
    actions.append(sim_gazebo_launch)

    # --- Isaac sim when moveit_mode == isaac (optional include) ---
    sim_isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "sim_isaac.launch.py")
        ),
        launch_arguments={
            "model": model,
            "display_config": display_config,
        }.items(),
        condition=IfCondition(EqualsSubstitution(moveit_mode, "isaac")),
    )
    # Delay often helps if your Isaac bringup needs a moment
    delayed_isaac_launch = TimerAction(
        period=5.0,
        actions=[sim_isaac_launch],
        condition=IfCondition(EqualsSubstitution(moveit_mode, "isaac")),
    )
    actions.append(delayed_isaac_launch)

    # --- Always include MoveIt; compute use_sim from mode in ['gazebo','isaac'] ---
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "moveit.launch.py")
        ),
        launch_arguments={
            "display": display,
            "display_config": display_config,
            # True when running under sim (gazebo/isaac), false for real
            "use_sim": PythonExpression(["'", moveit_mode, "' in ['gazebo', 'isaac']"]),
        }.items(),
    )
    # Optional: slight delay to let robot/sim init first
    delayed_moveit_launch = TimerAction(period=6.0, actions=[moveit_launch])
    actions.append(delayed_moveit_launch)

    return LaunchDescription(args + actions)
