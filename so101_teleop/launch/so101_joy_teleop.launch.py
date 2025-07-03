import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Declare top-level launch argument for mode
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="real",
        description="Execution mode: real, gazebo, or isaac",
    )
    mode = LaunchConfiguration("mode")

    # Include moveit_servo.launch.py with forwarded mode
    moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_moveit"),
                "launch",
                "moveit_servo.launch.py",
            )
        ),
        launch_arguments={
            "mode": mode,
        }.items(),
    )

    # Launch Joy as components
    container = ComposableNodeContainer(
        name="joy_teleop_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="so101_teleop",
                plugin="moveit_servo::JoyToServoComponent",
                name="controller_to_servo_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([mode_arg, moveit_servo_launch, container])
