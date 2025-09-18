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


import math
import threading
import time
from abc import ABC, abstractmethod
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray  # Import message type for commands

# Ensure the conda site-packages directory is in the system path
from so101_ros2_bridge.utils import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()

from lerobot.robots.so101_follower import SO101Follower
from lerobot.teleoperators.so101_leader import SO101Leader

# from .registry import ROBOT_FACTORY_REGISTRY
from so101_ros2_bridge import CALIBRATION_BASE_DIR  # defined in __init__.py
from so101_ros2_bridge.bridge.registry import ROBOT_FACTORY_REGISTRY


class SO101ROS2Bridge(Node, ABC):
    JOINT_NAMES = [
        'shoulder_pan',
        'shoulder_lift',
        'elbow_flex',
        'wrist_flex',
        'wrist_roll',
        'gripper',
    ]

    def __init__(self, node_name='so101_ros2_bridge'):
        super().__init__(node_name)
        params = self.read_parameters()
        self.use_degrees = params["use_degrees"]
        # config = self.dict_to_so101config(params)

        # Initialize watchdog at the background
        self._is_alive = True
        self._watchdog_interval = 0.01  # 100 hz whatchdog
        self._timeout = 5.0
        self._alive_thread = threading.Thread(target=self._alive, daemon=True)

        self.joint_pub = self.create_publisher(JointState, 'joint_states_raw', 10)
        rate = params.get("publish_rate", 30.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_states)

        # Register the robot type in the factory registry
        if not ROBOT_FACTORY_REGISTRY:
            raise RuntimeError(
                "ROBOT_FACTORY_REGISTRY is empty. Ensure robot types are registered."
            )

        self.declare_parameter("type", "follower")
        robot_type = self.get_parameter("type").get_parameter_value().string_value
        self.get_logger().info(
            f"Initializing SO101 ROS2 Bridge for robot type: {robot_type}"
        )
        factory_fn = ROBOT_FACTORY_REGISTRY.get(robot_type)
        if factory_fn is None:
            raise ValueError(f"Robot type '{robot_type}' not registered.")

        # Initialize robot
        self.robot = factory_fn(params)

        try:
            self.robot.connect(calibrate=False)
        except Exception as e:
            raise RuntimeError(f"Failed to connect to SO101 robot: {e}") from e

        self.last_positions = None
        self.last_time = self.get_clock().now()

        # Start alive
        self._alive_thread.start()

    @abstractmethod
    def read_parameters(self) -> dict:
        """Reads parameters from the ROS2 parameter server."""

    @abstractmethod
    def get_joints_states(self) -> dict:
        """
        Returns the current joint states as a dictionary.
        This method should be implemented by subclasses to return the robot's joint states.
        """

    def publish_joint_states(self):
        try:
            current_time = self.get_clock().now()
            obs = self.get_joints_states()
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

            # Velocity calculation
            velocities = [0.0] * len(self.JOINT_NAMES)
            # On the first run, last_positions will be None, so velocities will be zero.
            if self.last_positions is not None:
                # Calculate time delta in seconds
                dt = (current_time - self.last_time).nanoseconds / 1e9
                # Avoid division by zero if the time delta is too small
                if dt > 0:
                    for i in range(len(self.JOINT_NAMES)):
                        velocities[i] = (positions[i] - self.last_positions[i]) / dt

            # Prepare msg
            msg.position = positions
            msg.velocity = velocities

            # Publish
            self.joint_pub.publish(msg)

            # Store the current state for the next iteration's calculation
            self.last_positions = positions
            self.last_time = current_time

        except Exception as e:
            self.get_logger().error(f"Failed to publish joint states: {e}")

    def _alive(self):
        """
        Watchdog thread running at a fixed rate. Monitors rclpy.ok() and ROS time.
        Triggers shutdown if ROS crashes or time stops progressing.
        """
        interval = self._watchdog_interval
        next_tick = time.monotonic()

        while self._is_alive:
            # Check ROS system state
            if not rclpy.ok():
                self.get_logger().error(
                    "ROS shutdown detected (rclpy.ok() == False). Triggering shutdown."
                )
                self.shutdown_hook()
                rclpy.try_shutdown()
                break

            # Check time progress
            now = self.get_clock().now()
            elapsed = (now - self.last_time).nanoseconds / 1e9
            if elapsed > self._timeout:
                self.get_logger().error(
                    f"ROS time not updating for {elapsed:.1f}s — triggering shutdown."
                )
                self.shutdown_hook()
                rclpy.try_shutdown()
                break

            # Sleep until next tick
            next_tick += interval
            sleep_time = max(0.0, next_tick - time.monotonic())
            time.sleep(sleep_time)

    def shutdown_hook(self):
        if not self._is_alive:
            return  # Prevent double cleanup

        self._is_alive = False
        try:
            self.get_logger().info("Shutting down, disconnecting robot...")
            if hasattr(self, 'robot'):
                (
                    self.robot.disconnect()
                    if self.robot.is_connected
                    else self.get_logger().warn("Robot is already disconnected")
                )
        except Exception as e:
            self.get_logger().warn(f"Exception during shutdown: {e}")

    def radians_to_normalized(self, joint_name: str, rad: float) -> float:
        """
        converts a command in radians from MoveIt to the format expected by the SO101 API.
        """
        if joint_name == "gripper":
            # Convert radian command [0, pi] to the robot's expected gripper range [10, ~110]
            return (rad / math.pi) * 100.0 + 10.0
        else:
            # Convert radians to degrees for all other joints
            return math.degrees(rad)


class FollowerBridge(SO101ROS2Bridge):

    def __init__(self):
        super().__init__("so101_follower_ros2_bridge")

        # Type hint of the robot
        if not isinstance(self.robot, SO101Follower):
            raise TypeError(
                f"Expected robot type SO101Follower, got {type(self.robot).__name__}"
            )
        # Subscribe to commands from the ros2_control hardware interface bridge
        self.create_subscription(
            Float64MultiArray,
            'joint_commands',  # This topic should match the publisher in the C++ bridge
            self.command_callback,
            10,
        )
        self.get_logger().info("SO101 Follower ROS2 Bridge initialized.")

    def read_parameters(self) -> dict:
        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("id", "Tzili")
        self.declare_parameter("calibration_dir", str(CALIBRATION_BASE_DIR))
        self.declare_parameter("use_degrees", True)
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

    def get_joints_states(self) -> dict:
        """
        Returns the current joint states as a dictionary.
        This method should be implemented by subclasses to return the robot's joint states.
        """
        return self.robot.get_observation()

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
        for i, joint in enumerate(self.JOINT_NAMES):
            # Convert the incoming radian command to the format the robot expects (degrees or normalized)
            target_positions[f"{joint}.pos"] = self.radians_to_normalized(
                joint, msg.data[i]
            )

        try:
            self.robot.send_action(target_positions)
        except Exception as e:
            self.get_logger().error(f"Failed to send commands to robot: {e}")


class LeaderBridge(SO101ROS2Bridge):

    def __init__(self):
        super().__init__("so101_leader_ros2_bridge")

        # Type hint of the robot
        if not isinstance(self.robot, SO101Leader):
            raise TypeError(
                f"Expected robot type SO101Leader, got {type(self.robot).__name__}"
            )
        self.get_logger().info("SO101 Leader ROS2 Bridge initialized.")

    def read_parameters(self) -> dict:
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("id", "Gili")
        self.declare_parameter("calibration_dir", str(CALIBRATION_BASE_DIR))
        self.declare_parameter("use_degrees", True)
        self.declare_parameter("publish_rate", 30.0)

        return {
            "port": self.get_parameter("port").get_parameter_value().string_value,
            "id": self.get_parameter("id").get_parameter_value().string_value,
            "calibration_dir": Path(
                self.get_parameter("calibration_dir").get_parameter_value().string_value
            ),
            "use_degrees": (
                self.get_parameter("use_degrees").get_parameter_value().bool_value
            ),
            "publish_rate": (
                self.get_parameter("publish_rate").get_parameter_value().double_value
            ),
        }

    def get_joints_states(self) -> dict:
        """
        Returns the current joint states as a dictionary.
        This method should be implemented by subclasses to return the robot's joint states.
        """
        return self.robot.get_action()
