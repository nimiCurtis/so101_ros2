import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_arg = DeclareLaunchArgument("use_sim", default_value="False")
    display_config_arg = DeclareLaunchArgument(
        "display_config",
        default_value=os.path.join(
            get_package_share_directory("so101_moveit"),
            "rviz",
            "moveit.rviz",
        ),
    )

    use_sim = LaunchConfiguration("use_sim")
    display_config = LaunchConfiguration("display_config")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_moveit"),
                "launch",
                "moveit.launch.py",
            )
        ),
        launch_arguments={"use_sim": use_sim}.items(),
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_description"),
                "launch",
                "display.launch.py",
            )
        ),
        launch_arguments={
            "joint_states_gui": "false",
            "display_config": display_config,
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_arg,
            display_config_arg,
            moveit_launch,
            display_launch,
        ]
    )
