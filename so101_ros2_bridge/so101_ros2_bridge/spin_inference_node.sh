#!/bin/bash

# Source conda
source /home/anton/miniconda3/etc/profile.d/conda.sh
conda activate lerobot_ros2

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/anton/ros2_ws/install/setup.bash

# # CRITICAL: Put system library paths FIRST to override conda's libstdc++
# export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# # Also ensure system libs are used for other potential conflicts
# export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# Run the node


ros2 run so101_ros2_bridge smolvla_inference_node   --ros-args   -p task:="Pick up the white cube and insert it on the green peg"   -p publisher_rate:=2   -p model_id:="lerobot/smolvla_base" "$@"
# ros2 run so101_ros2_bridge smolvla_inference_node "$@"