from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare top-level launch argument for robot type
    robot_type_arg = DeclareLaunchArgument(
        "type",
        default_value="follower",
        description="Robot type: follower / leader",
    )

    robot_type = LaunchConfiguration("type")

    # Use PathJoinSubstitution to build the path dynamically at launch time
    config_path = PathJoinSubstitution(
        [
            FindPackageShare("so101_ros2_bridge"),
            "config",
            [
                TextSubstitution(text='so101_'),
                robot_type,
                TextSubstitution(text='_params.yaml'),
            ],
        ]
    )

    so101_bridge_node = Node(
        package='so101_ros2_bridge',
        executable='so101_ros2_bridge_node',
        name=[
            TextSubstitution(text='so101_'),
            robot_type,
            TextSubstitution(text='_interface'),
        ],
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription(
        [robot_type_arg, so101_bridge_node]  # expose the 'type' arg to command line
    )
