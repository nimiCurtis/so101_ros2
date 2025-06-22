import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    so101_description_share_dir = get_package_share_directory("so101_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            so101_description_share_dir, "urdf", "so101_new_calib.urdf.xacro"
        ),
        description="Absolute path to the robot URDF/xacro file",
    )

    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                LaunchConfiguration("model"),
                " ",
                "display:=true",  # or false if you want ros2_control enabled
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(so101_description_share_dir, "rviz", "display.rviz"),
        ],
    )

    return LaunchDescription(
        [model_arg, robot_state_publisher, joint_state_publisher_gui, rviz_node]
    )
