import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Launch configuration of "mode"
    mode = LaunchConfiguration("mode")

    # Launch Leader as a component
    container = ComposableNodeContainer(
        name="leader_teleop_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",  # Use multithreaded for best performance
        composable_node_descriptions=[
            ComposableNode(
                package="so101_teleop",
                plugin="so101_teleop::LeaderTeleopComponent",
                name="leader_teleop_component",  # This name must match the one in the YAML file
                parameters=[
                    os.path.join(
                        get_package_share_directory("so101_teleop"),
                        "config",
                        "so101_leader_teleop.yaml",
                    ),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True, "use_sim_time": mode == "gazebo"}
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
