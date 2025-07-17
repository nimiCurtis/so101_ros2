import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

# Path to the pkg share directory
PACKAGE_DIR = Path(get_package_share_directory("so101_ros2_bridge"))

# Default calibration path relative to this package
CALIBRATION_BASE_DIR = PACKAGE_DIR / "config" / "calibration"
