import os
import sys
from pathlib import Path

# Ensure the conda site-packages directory is in the system path
from so101_ros2_bridge.utils import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig

from so101_ros2_bridge.bridge.registry import register_robot


@register_robot("follower")
def create_follower(params: dict):
    config = SO101FollowerConfig(
        port=params["port"],
        calibration_dir=Path(params["calibration_dir"]),
        id=params["id"],
        use_degrees=params["use_degrees"],
        max_relative_target=params["max_relative_target"],
        disable_torque_on_disconnect=params["disable_torque_on_disconnect"],
    )
    return SO101Follower(config)


@register_robot("leader")
def create_leader(params: dict):
    config = SO101LeaderConfig(
        port=params["port"],
        calibration_dir=Path(params["calibration_dir"]),
        id=params["id"],
        use_degrees=params["use_degrees"],
    )
    return SO101Leader(config)
