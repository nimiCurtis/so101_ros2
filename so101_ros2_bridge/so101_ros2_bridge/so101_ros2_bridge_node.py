#!/usr/bin/env python3

# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import sys

# Manually append Conda site-packages to PYTHONPATH
conda_site = '/home/nimrod/miniconda3/envs/lerobot/lib/python3.10/site-packages'
if conda_site not in sys.path:
    sys.path.append(conda_site)
import asyncio  # make sure this is at the top
import math
from pathlib import Path

import rclpy
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray  # Import message type for commands

from so101_ros2_bridge import CALIBRATION_BASE_DIR  # defined in __init__.py


class SO101ROS2Bridge(Node):
    JOINT_NAMES = [
        'shoulder_pan',
        'shoulder_lift',
        'elbow_flex',
        'wrist_flex',
        'wrist_roll',
        'gripper',
    ]

    def __init__(self):
        super().__init__('so101_ros2_bridge')
        params = self.read_parameters()
        self.use_degrees = params["use_degrees"]
        config = self.dict_to_so101config(params)

        self.robot = SO101Follower(config)
        self.robot.connect(calibrate=False)

        if not self.robot.is_connected:
            self.shutdown_hook()
            self.get_logger().error("Failed to connect to so101 arm.")
            rclpy.shutdown()
            sys.exit(1)

        self.joint_pub = self.create_publisher(JointState, '/joint_states_raw', 10)
        rate = params.get("publish_rate", 30.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_states)

        # Subscribe to commands from the ros2_control hardware interface bridge
        self.create_subscription(
            Float64MultiArray,
            '/joint_commands',  # This topic should match the publisher in the C++ bridge
            self.command_callback,
            10,
        )

    def shutdown_hook(self):
        try:
            self.get_logger().info("Shutting down, disconnecting robot...")
            self.robot.disconnect()
        except Exception as e:
            self.get_logger().warn(f"Exception during shutdown: {e}")

    def read_parameters(self) -> dict:
        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("id", "Tzili")
        self.declare_parameter("calibration_dir", str(CALIBRATION_BASE_DIR))
        self.declare_parameter("use_degrees", False)
        self.declare_parameter("max_relative_target", 0)
        self.declare_parameter("disable_torque_on_disconnect", True)
        self.declare_parameter("publish_rate", 30.0)

        max_relative_target = (
            self.get_parameter("max_relative_target")
            .get_parameter_value()
            .integer_value
        )
        max_relative_target = max_relative_target if max_relative_target != 0 else None

        return {
            "port": self.get_parameter("port").get_parameter_value().string_value,
            "id": self.get_parameter("id").get_parameter_value().string_value,
            "calibration_dir": Path(
                self.get_parameter("calibration_dir").get_parameter_value().string_value
            ),
            "use_degrees": (
                self.get_parameter("use_degrees").get_parameter_value().bool_value
            ),
            "max_relative_target": max_relative_target,
            "disable_torque_on_disconnect": (
                self.get_parameter("disable_torque_on_disconnect")
                .get_parameter_value()
                .bool_value
            ),
            "publish_rate": (
                self.get_parameter("publish_rate").get_parameter_value().double_value
            ),
        }

    def dict_to_so101config(self, params: dict) -> SO101FollowerConfig:
        return SO101FollowerConfig(
            port=params["port"],
            calibration_dir=params["calibration_dir"],
            id=params["id"],
            use_degrees=params["use_degrees"],
            max_relative_target=params["max_relative_target"],
            disable_torque_on_disconnect=params["disable_torque_on_disconnect"],
        )

    def publish_joint_states(self):
        try:
            obs = self.robot.get_observation()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.JOINT_NAMES
            positions = []
            for joint in self.JOINT_NAMES:
                if joint == "gripper":
                    # Normalize gripper position to be within [0, pi] for MoveIt
                    pos = ((obs.get(f"{joint}.pos", 0.0) - 10.0) / 100.0) * math.pi
                else:
                    pos = math.radians(obs.get(f"{joint}.pos", 0.0))
                positions.append(pos)
            msg.position = positions
            self.joint_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint states: {e}")

    def command_callback(self, msg: Float64MultiArray):
        """
        Receives joint command goals in radians and sends them to the robot.
        """
        if len(msg.data) != len(self.JOINT_NAMES):
            self.get_logger().error(
                f"Received command with {len(msg.data)} joints, but expected {len(self.JOINT_NAMES)}."
            )
            return

        target_positions = {}
        for i, joint_name in enumerate(self.JOINT_NAMES):
            # Convert the incoming radian command to the format the robot expects (degrees or normalized)
            target_positions[joint_name] = self.radians_to_normalized(
                joint_name, msg.data[i]
            )

        try:
            # Assuming the library has a method like `set_target_positions`
            self.robot.send_action(target_positions)
        except Exception as e:
            self.get_logger().error(f"Failed to send commands to robot: {e}")

    def radians_to_normalized(self, joint_name: str, rad: float) -> float:
        """
        Converts a command in radians from MoveIt to the format expected by the SO101 API.
        """
        if joint_name == "gripper":
            # Convert radian command [0, pi] to the robot's expected gripper range [10, ~110]
            return (rad / math.pi) * 100.0 + 10.0
        else:
            # Convert radians to degrees for all other joints
            return math.degrees(rad)


def main(args=None):
    rclpy.init(args=args)
    node = SO101ROS2Bridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
