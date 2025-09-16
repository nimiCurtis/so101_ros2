Setup
=====

Bridge configuration
--------------------
The ROS 2 bridge nodes read their default parameters from the YAML files in
``so101_ros2_bridge/config``. Update the serial port, robot identifier and
calibration directory to match the hardware that is connected to the machine::

   so101_ros2_bridge/config/so101_follower_params.yaml
   so101_ros2_bridge/config/so101_leader_params.yaml

Each file defines the ``port``, ``id`` and ``publish_rate`` fields used by the
Python bridge node. The follower configuration additionally exposes
``max_relative_target`` and ``disable_torque_on_disconnect`` to bound commands
sent to the arm.

Calibration assets
------------------
Default calibration JSON files are installed with the package under
``so101_ros2_bridge/config/calibration``. Replace or extend these files with the
calibration data generated for your manipulators and point the parameter files
to the desired directory when launching.

Python environment
------------------
The bridge code dynamically adds the Lerobot Conda environment to ``sys.path``
using the ``LECONDA_SITE_PACKAGES`` environment variable. Ensure the variable
points to the ``site-packages`` directory that contains the ``lerobot``
distribution before starting the ROS 2 bridge nodes.

Cameras
-------
USB camera support is defined in ``so101_bringup/config/so101_cameras.yaml``.
Each entry references a USB camera parameter file such as
``so101_bringup/config/so101_usb_cam_front.yaml`` where the device path,
resolution and exposure settings are declared. Update these files to match the
connected cameras.

System data recorder
--------------------
The data recording launch file loads ``so101_bringup/config/so101_sdr.yaml`` to
configure the ``system_data_recorder`` node. Adjust the ``copy_destination``,
``bag_name_prefix`` and the list of topics to capture the information required
for your workflows before running a recording session.
