Build the Workspace
===================

System requirements
-------------------
* ROS 2 Humble with ``rmw_cyclonedds_cpp`` configured as the middleware.
* A working ``colcon`` toolchain with build dependencies installed via ``rosdep``.

Bootstrap the workspace
-----------------------
1. Create or reuse a ROS 2 workspace and clone the repository into the ``src``
   directory::

       mkdir -p ~/ros2_ws/src
       cd ~/ros2_ws/src
       git clone --recurse-submodules https://github.com/nimiCurtis/so101_ros2.git
       cd so101_ros2

2. Run the helper script once to install system dependencies and append the ROS 2
   sourcing statements to your shell configuration::

       ./build.sh

   The script can be safely rerun if you need to refresh dependencies.

Incremental builds
------------------
Deactivate the ``lerobot_ros2`` Conda environment before compiling so ``colcon``
uses the system Python interpreter::

   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash

Expose Lerobot to ROS 2
-----------------------
Export the Conda ``site-packages`` directory so the bridge nodes can import
Lerobot modules at runtime::

   export LECONDA_SITE_PACKAGES=<conda_root>/envs/lerobot_ros2/lib/python3.10/site-packages

For persistent setups create a symbolic link from the Lerobot sources into the
bridge install path inside the workspace::

   export LEROBOT_SRC=<path to lerobot>/src/lerobot
   export SO101BRIDGE_INSTALL_SITE_PACKAGES=<ros2_ws>/install/so101_ros2_bridge/lib/python3.10/site-packages/lerobot
   ln -s $LEROBOT_SRC $SO101BRIDGE_INSTALL_SITE_PACKAGES

Recreate the link after rebuilding to ensure it points to the latest install
folder.

Camera support
--------------
Two camera backends are supported out of the box:

* **USB cameras** – installed automatically through ``rosdep`` when available.
  Install manually if required::

     sudo apt install ros-humble-usb-cam

* **Intel RealSense cameras** – install the librealsense SDK and ROS 2 wrapper::

     sudo apt install ros-humble-realsense2-*

Adjust the camera parameter files in ``so101_bringup/config`` to select the
correct device paths and settings before launching.
