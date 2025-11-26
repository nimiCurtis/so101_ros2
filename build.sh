#!/usr/bin/env bash
set -euo pipefail

BASHRC_FILE=${BASHRC_FILE:-$HOME/.bashrc}

# Ensure the user's bashrc exists so we can append to it later.
touch "$BASHRC_FILE"

if command -v conda &>/dev/null; then
  conda deactivate || true
fi

if ! grep -Fqx 'source /opt/ros/humble/setup.bash' "$BASHRC_FILE"; then
  echo "source /opt/ros/humble/setup.bash" >> "$BASHRC_FILE"
fi
if ! grep -Fqx 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' "$BASHRC_FILE"; then
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> "$BASHRC_FILE"
fi

# Enable colored logging
if ! grep -Fqx 'export RCUTILS_COLORIZED_OUTPUT=1' "$BASHRC_FILE"; then
  echo "export RCUTILS_COLORIZED_OUTPUT=1" >> "$BASHRC_FILE"
fi

source "$BASHRC_FILE"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SRC="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$WORKSPACE_SRC/.." && pwd)"

cd "$WORKSPACE_ROOT"

sudo apt update

if ! command -v rosdep &>/dev/null; then
  sudo apt install -y python3-rosdep
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi

rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

if ! grep -Fqx "source $WORKSPACE_ROOT/install/local_setup.bash" "$BASHRC_FILE"; then
  echo "source $WORKSPACE_ROOT/install/local_setup.bash" >> "$BASHRC_FILE"
fi

source "$BASHRC_FILE"

echo "Build so101_ros2 completed.."
