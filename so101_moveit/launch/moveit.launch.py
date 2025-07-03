import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    use_sim = LaunchConfiguration("use_sim")

    use_sim_arg = DeclareLaunchArgument("use_sim", default_value="False")

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

    return LaunchDescription([use_sim_arg, move_group_node])
