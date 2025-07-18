#!/usr/bin/env python3

import sys

# Manually append Conda site-packages to PYTHONPATH
conda_site = '/home/nimrod/miniconda3/envs/lerobot/lib/python3.10/site-packages'
if conda_site not in sys.path:
    sys.path.append(conda_site)
import asyncio  # make sure this is at the top
import math
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        rate = params.get("publish_rate", 30.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_states)

        # FollowJointTrajectory Action Server
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_trajectory_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def shutdown_hook(self):
        try:
            self.get_logger().info("Shutting down, disconnecting robot...")
            self.robot.disconnect()
            self.action_server.destroy()
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
                    pos = ((obs.get(f"{joint}.pos", 0.0) - 10.0) / 100.0) * math.pi
                else:
                    pos = math.radians(obs.get(f"{joint}.pos", 0.0))
                positions.append(pos)
            msg.position = positions
            self.joint_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint states: {e}")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received FollowJointTrajectory goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_trajectory_callback(self, goal_handle):
        self.get_logger().info("Executing trajectory...")
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names

        for point in trajectory.points:
            positions = point.positions
            action = {}
            for joint_name, position in zip(joint_names, positions):
                if joint_name in self.JOINT_NAMES:
                    action[joint_name] = self.radians_to_normalized(
                        joint_name, position
                    )

            try:
                print(action)
                self.robot.send_action(action)
            except Exception as e:
                self.get_logger().error(f"Failed to send action: {e}")
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            # ✅ Correct sleep based on duration
            duration = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            await asyncio.sleep(duration)

        goal_handle.succeed()
        self.get_logger().info("Trajectory execution complete")
        return FollowJointTrajectory.Result()

    def radians_to_normalized(self, joint_name: str, rad: float) -> float:
        if joint_name == "gripper":
            return (rad / math.pi) * 100.0 + 10.0
        else:
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
