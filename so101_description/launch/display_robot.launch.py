import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    so101_description_dir = get_package_share_directory("so101_description")

    model = os.path.join(so101_description_dir, "urdf", "so101_new_calib.urdf.xacro")
    use_joint_states_gui = "true"
    mode = "real"

    display_config_arg = DeclareLaunchArgument(
        "display_config",
        default_value=os.path.join(so101_description_dir, "rviz", "display.rviz"),
        description="RViz config file path",
    )

    # Launch configurations
    display_config = LaunchConfiguration("display_config")

    # Include rsp.launch.py
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_description_dir, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "model": model,
            "mode": mode,
            "joint_states_gui": use_joint_states_gui,
        }.items(),
    )

    # Include display.launch.py
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_description_dir, "launch", "display.launch.py")
        ),
        launch_arguments={
            "joint_states_gui": use_joint_states_gui,
            "display_config": display_config,
        }.items(),
    )

    return LaunchDescription(
        [
            display_config_arg,
            rsp_launch,
            display_launch,
        ]
    )
