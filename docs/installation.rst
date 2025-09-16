Installation
============

System requirements
-------------------
- ROS 2 Humble with the CycloneDDS RMW implementation as used throughout the workspace.  
- Gazebo Fortress (Ignition) for simulation bring-up.  
- A working build toolchain for `colcon` workspaces.

Clone and build
---------------
1. Create or reuse a ROS 2 workspace and clone the repository into the ``src`` folder::

       mkdir -p ~/ros2_ws/src
       cd ~/ros2_ws/src
       git clone https://github.com/<your-org>/so101_ros2.git

2. Install ROS 2 package dependencies with ``rosdep`` and build the workspace::

       cd ~/ros2_ws
       rosdep install --from-paths src --ignore-src -r -y
       colcon build
       source install/setup.bash

Python dependencies
-------------------
The ROS 2 bridge nodes rely on the Hugging Face `lerobot` project for the SO101
leader and follower APIs. Ensure the library is available in the Python
environment used to launch the ROS 2 nodes. If ``lerobot`` is installed in a
separate Conda environment, expose its ``site-packages`` directory through the
``LECONDA_SITE_PACKAGES`` environment variable before running the bridge nodes.
