import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    return LaunchDescription(
        [
            mode_arg,
            moveit_servo_launch,
        ]
    )
