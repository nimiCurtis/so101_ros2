import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Declare args
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("so101_description"),
            "urdf",
            "so101_new_calib.urdf.xacro",
        ),
    )
    mode_arg = DeclareLaunchArgument("mode", default_value="real")
    model = LaunchConfiguration("model")
    mode = LaunchConfiguration("mode")

    # Build robot_description
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                model,
                " ",
                "mode:=",
                mode,
            ]
        ),
        value_type=str,
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                get_package_share_directory("so101_controller"),
                "config",
                "so101_controllers.yaml",
            ),
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            mode_arg,
            controller_manager_node,
        ]
    )
