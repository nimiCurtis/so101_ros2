import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    so101_description_share_dir = get_package_share_directory("so101_description")

    # Declare model argument
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            so101_description_share_dir, "urdf", "so101_new_calib.urdf.xacro"
        ),
        description="Absolute path to the robot URDF/xacro file",
    )

    # Set Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(so101_description_share_dir).parent.resolve())],
    )

    model = LaunchConfiguration("model")

    # Include RSP with sim settings
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_description_share_dir, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "model": model,
            "mode": "gazebo",
            "use_sim": "true",
        }.items(),
    )

    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments=[("gz_args", [" -v 4 -r empty.sdf"])],
    )

    # Spawn the robot into Gazebo from /robot_description topic
    gazebo_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "so101_new_calib"],
    )

    # Bridge simulation clock
    gazebo_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    return LaunchDescription(
        [
            model_arg,
            gazebo_resource_path,
            rsp_launch,
            gazebo_launch,
            gazebo_spawn_entity,
            gazebo_ros2_bridge,
        ]
    )
