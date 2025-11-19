#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor

from so101_ros2_bridge.policy.runner import SO101PolicyRunner


def main(args=None) -> None:
    """Run the PolicyRunner Node."""
    rclpy.init(args=args)
    node = SO101PolicyRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
