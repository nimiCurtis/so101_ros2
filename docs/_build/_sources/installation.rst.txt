Installation
============

System requirements
-------------------
- ROS 2 Humble with the CycloneDDS RMW implementation as used throughout the workspace.
- Gazebo Fortress (Ignition) for simulation bring-up.
- A working build toolchain for ``colcon`` workspaces.

Prepare the Lerobot environment
-------------------------------
The ROS 2 bridge relies on the Hugging Face `lerobot` project for the SO101
leader and follower APIs. Create a dedicated Conda environment and install the
forked repository before launching any bridge nodes::

   conda create -n lerobot_ros2 python=3.10
   conda activate lerobot_ros2
   git clone https://github.com/nimiCurtis/lerobot.git
   cd lerobot
   pip install -e ".[all]"

The Lerobot tutorials describe the calibration workflow needed to generate
per-arm JSON calibration files. Keep these accessible; the ROS 2 bridge defaults
to ``so101_ros2_bridge/config/calibration`` but accepts custom paths.

Clone and build
---------------
1. Create or reuse a ROS 2 workspace and clone the repository into the ``src`` folder::

       mkdir -p ~/ros2_ws/src
       cd ~/ros2_ws/src
       git clone --recurse-submodules git@github.com:nimiCurtis/so101_ros2.git

2. Install ROS 2 package dependencies with ``rosdep`` and build the workspace. Be
   sure to deactivate the ``lerobot_ros2`` Conda environment before compiling so
   the build uses the system ROS 2 Python interpreter::

       cd ~/ros2_ws
       rosdep install --from-paths src --ignore-src -r -y
       colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
       source install/setup.bash

Expose Lerobot to ROS 2
-----------------------
At runtime the bridge needs access to the Lerobot installation. Export the
``LECONDA_SITE_PACKAGES`` environment variable in every terminal before
launching ROS 2 nodes so the Conda ``site-packages`` directory is appended to
``sys.path``::

   export LECONDA_SITE_PACKAGES=<conda_root>/envs/lerobot_ros2/lib/python3.10/site-packages

For long-lived deployments you can create a symbolic link from the Lerobot
source tree into the bridge package's ``site-packages`` folder inside the
workspace install. This makes the modules available even when the Conda
environment is inactive::

   export LEROBOT_SRC=<path to lerobot>/src/lerobot
   export SO101BRIDGE_INSTALL_SITE_PACKAGES=<ros2_ws>/install/so101_ros2_bridge/lib/python3.10/site-packages/lerobot
   ln -s $LEROBOT_SRC $SO101BRIDGE_INSTALL_SITE_PACKAGES

Recreate the link after rebuilding the workspace to ensure the path remains
valid.
