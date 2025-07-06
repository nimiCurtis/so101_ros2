import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                get_package_share_directory("so101_controller"),
                "config",
                "so101_controllers.yaml",
            ),
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    return LaunchDescription(
        [
            controller_manager_node,
        ]
    )
