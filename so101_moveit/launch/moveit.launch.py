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
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    use_sim = LaunchConfiguration("use_sim")
    display = LaunchConfiguration("display")

    display_config = LaunchConfiguration("display_config")

    robot_description_share_dir = get_package_share_directory("so101_description")
    robot_description_f = os.path.join(
        robot_description_share_dir, "urdf", "so101_new_calib.urdf.xacro"
    )
    joint_limits_f = os.path.join(
        robot_description_share_dir, "config", "joint_limits.yaml"
    )
    moveit_controllers_f = "config/moveit_controllers.yaml"
    srdf_f = "config/so101_new_calib.srdf"

    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so101_moveit")
        .robot_description(file_path=robot_description_f)
        .robot_description_semantic(file_path=srdf_f)
        .trajectory_execution(file_path=moveit_controllers_f)
        .joint_limits(file_path=joint_limits_f)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_parameters = [
        moveit_config.joint_limits,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        {"use_sim_time": use_sim},
    ]

    # RViz node with configurable display config
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", display_config],
        parameters=rviz_parameters,
    )

    delayed_display = TimerAction(
        period=5.0, actions=[rviz_node], condition=IfCondition(display)
    )

    return LaunchDescription([move_group_node, delayed_display])
