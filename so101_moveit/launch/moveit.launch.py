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
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch args (robot_ns added; others assumed declared upstream if needed)
    robot_ns = LaunchConfiguration("robot_ns")
    use_sim = LaunchConfiguration("use_sim")
    display = LaunchConfiguration("display")
    display_config = LaunchConfiguration("display_config")

    declare_ns = DeclareLaunchArgument("robot_ns", default_value="follower")

    # Files
    robot_description_share_dir = get_package_share_directory("so101_description")
    robot_description_f = os.path.join(
        robot_description_share_dir, "urdf", "so101_new_calib.urdf.xacro"
    )
    joint_limits_f = os.path.join(
        robot_description_share_dir, "config", "joint_limits.yaml"
    )
    moveit_controllers_f = "config/moveit_controllers.yaml"
    srdf_f = "config/so101_new_calib.srdf"

    # MoveIt configuration (URDF, SRDF, pipelines, controllers, limits)
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so101_moveit")
        .robot_description(file_path=robot_description_f)
        .robot_description_semantic(file_path=srdf_f)
        .trajectory_execution(file_path=moveit_controllers_f)
        .joint_limits(file_path=joint_limits_f)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # move_group under namespace
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

    # Group both under the same namespace (robot_ns)
    moveit_group = GroupAction(
        [
            PushRosNamespace(robot_ns),
            move_group_node,
        ]
    )

    # RViz gets model params directly + hint namespace param
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.joint_limits,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        {"use_sim_time": use_sim},
        {"moveit_ros_visualization.move_group_namespace": robot_ns},
    ]

    # Remap RViz’s MoveIt endpoints to robot_ns (cover services + both action name styles)
    rviz_remaps = [
        # already present…
        ("/monitored_planning_scene", "/follower/monitored_planning_scene"),
        ("/display_planned_path", "/follower/display_planned_path"),
        ("/motion_plan_request", "/follower/motion_plan_request"),
        ("/planned_path", "/follower/planned_path"),
        ("/execute_trajectory", "/follower/execute_trajectory"),
        ("/get_planning_scene", "/follower/get_planning_scene"),
        ("/compute_fk", "/follower/compute_fk"),
        ("/compute_ik", "/follower/compute_ik"),
        ("/query_planners", "/follower/query_planners"),
        ("/clear_octomap", "/follower/clear_octomap"),
        # ✅ Missing ones that RViz uses to populate planners:
        ("/get_planner_params", "/follower/get_planner_params"),
        ("/set_planner_params", "/follower/set_planner_params"),
        # Actions (keep both modern + legacy)
        (
            "/execute_trajectory/_action/goal",
            "/follower/execute_trajectory/_action/goal",
        ),
        (
            "/execute_trajectory/_action/cancel",
            "/follower/execute_trajectory/_action/cancel",
        ),
        (
            "/execute_trajectory/_action/status",
            "/follower/execute_trajectory/_action/status",
        ),
        (
            "/execute_trajectory/_action/feedback",
            "/follower/execute_trajectory/_action/feedback",
        ),
        (
            "/execute_trajectory/_action/result",
            "/follower/execute_trajectory/_action/result",
        ),
        ("/move_group/_action/goal", "/follower/move_group/_action/goal"),
        ("/move_group/_action/cancel", "/follower/move_group/_action/cancel"),
        ("/move_group/_action/status", "/follower/move_group/_action/status"),
        ("/move_group/_action/feedback", "/follower/move_group/_action/feedback"),
        ("/move_group/_action/result", "/follower/move_group/_action/result"),
        ("/move_group/goal", "/follower/move_group/goal"),
        ("/move_group/cancel", "/follower/move_group/cancel"),
        ("/move_group/status", "/follower/move_group/status"),
        ("/move_group/feedback", "/follower/move_group/feedback"),
        ("/move_group/result", "/follower/move_group/result"),
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", display_config],
        parameters=rviz_parameters,
        remappings=rviz_remaps,
    )

    # Start RViz after a short delay so move_group is up
    delayed_display = TimerAction(
        period=5.0, actions=[rviz_node], condition=IfCondition(display)
    )

    return LaunchDescription([declare_ns, moveit_group, delayed_display])
