import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    robot_description_share_dir = get_package_share_directory("so101_description")
    robot_description_f = os.path.join(
        robot_description_share_dir, "urdf", "so101_new_calib.urdf.xacro"
    )
    joint_limits_f = os.path.join(
        robot_description_share_dir, "config", "joint_limits.yaml"
    )
    moveit_controllers_f = "config/moveit_controllers.yaml"
    srdf_f = "config/so101_new_calib.srdf"

    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so101_moveit")
        .robot_description(file_path=robot_description_f)
        .robot_description_semantic(file_path=srdf_f)
        .trajectory_execution(file_path=moveit_controllers_f)
        .joint_limits(file_path=joint_limits_f)
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("so101_teleop", "config/so101_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            # ComposableNode(
            #     package="so101_teleop",
            #     plugin="moveit_servo::KeyboardServoComponent",
            #     name="keyboard_to_servo_node",
            # ),
        ],
        output="screen",
    )

    keyboard_node = Node(
        package="so101_teleop",
        executable="keyboard_servo_input",  # <-- Match your target name in CMakeLists
        name="so101_keyboard_teleop",
        output="screen",
    )

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node, container, keyboard_node])
