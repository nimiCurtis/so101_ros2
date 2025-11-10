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
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration


def generate_launch_description():
    # Launch description lists
    args = []
    actions = []

    # Paths
    bringup_pkg = get_package_share_directory('so101_bringup')
    description_pkg = get_package_share_directory('so101_description')
    teleop_pkg = get_package_share_directory('so101_teleop')

    # --- Declare arguments ---
    display_config_arg = DeclareLaunchArgument(
        'display_config',
        default_value=os.path.join(
            teleop_pkg,
            'rviz',
            'teleop_with_camera.rviz',
        ),
    )
    args.append(display_config_arg)

    display_arg = DeclareLaunchArgument(
        'display', default_value='false', description='Launch RViz or not'
    )
    args.append(display_arg)

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_pkg, 'urdf', 'so101_new_calib.urdf.xacro'),
    )
    args.append(model_arg)

    teleop_mode_arg = DeclareLaunchArgument(
        'teleop_mode',
        default_value='real',
        description='Execution mode: real, isaac',
    )
    args.append(teleop_mode_arg)

    model = LaunchConfiguration('model')
    display_config = LaunchConfiguration('display_config')
    display = LaunchConfiguration('display')
    teleop_mode = LaunchConfiguration('teleop_mode')

    # Debug: Log the mode
    mode_log = LogInfo(msg=['[TELEOP LAUNCH] Mode is set to: ', teleop_mode])
    actions.append(mode_log)

    # Launch follower - ONLY in real mode
    follower_log = LogInfo(
        msg='[TELEOP LAUNCH] Launching REAL follower',
        condition=IfCondition(EqualsSubstitution(teleop_mode, 'real')),
    )
    actions.append(follower_log)

    # Launch leader - ALWAYS (in real mode)
    leader_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'leader.launch.py')
        ),
        launch_arguments={
            'model': model,
            'use_sim_time': 'false',
        }.items(),
    )
    # delayed_leader_launch = TimerAction(period=8.0, actions=[leader_robot_launch])
    actions.append(leader_robot_launch)

    follower_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'follower.launch.py')
        ),
        launch_arguments={
            'model': model,
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(EqualsSubstitution(teleop_mode, 'real')),
    )
    delayed_follower_launch = TimerAction(
        period=5.0,
        actions=[follower_robot_launch],
        condition=IfCondition(EqualsSubstitution(teleop_mode, 'real')),
    )
    actions.append(delayed_follower_launch)

    # Include cameras - ONLY in real mode
    camera_log = LogInfo(
        msg='[TELEOP LAUNCH] Launching cameras',
        condition=IfCondition(EqualsSubstitution(teleop_mode, 'real')),
    )
    actions.append(camera_log)

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'camera.launch.py')
        ),
        condition=IfCondition(EqualsSubstitution(teleop_mode, 'real')),
    )
    actions.append(cameras_launch)

    # Include sim_isaac.launch ONLY if teleop_mode == "isaac"
    isaac_log = LogInfo(
        msg='[TELEOP LAUNCH] Launching Isaac simulation',
        condition=IfCondition(EqualsSubstitution(teleop_mode, 'isaac')),
    )
    actions.append(isaac_log)

    sim_isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'sim_isaac.launch.py')
        ),
        launch_arguments={
            'model': model,
            'display_config': display_config,
        }.items(),
        condition=IfCondition(EqualsSubstitution(teleop_mode, 'isaac')),
    )
    delayed_sim_isaac_launch = TimerAction(
        period=5.0,
        actions=[sim_isaac_launch],
        condition=IfCondition(EqualsSubstitution(teleop_mode, 'isaac')),
    )
    actions.append(delayed_sim_isaac_launch)

    # Include display.launch.py
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_pkg, 'launch', 'display.launch.py')),
        launch_arguments={
            'joint_states_gui': 'false',
            'display_config': display_config,
        }.items(),
    )
    delayed_display = TimerAction(
        period=5.0, actions=[display_launch], condition=IfCondition(display)
    )
    actions.append(delayed_display)

    # Include leader teleop
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('so101_teleop'),
                'launch',
                'so101_leader_teleop.launch.py',
            )
        ),
        launch_arguments={
            'mode': teleop_mode,
        }.items(),
    )

    delayed_teleop_launch = TimerAction(period=15.0, actions=[teleop_launch])
    actions.append(delayed_teleop_launch)

    return LaunchDescription(args + actions)
