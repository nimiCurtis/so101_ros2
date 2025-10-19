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
sent to the arm. Set ``calibration_dir`` to the directory that contains the
calibration JSON files generated during the Lerobot setup. When the field is
omitted the bridge defaults to the bundled assets inside the package.

Calibration assets
------------------
Default calibration JSON files are installed with the package under
``so101_ros2_bridge/config/calibration``. Replace or extend these files with the
calibration data generated for your manipulators and point the parameter files
to the desired directory when launching. The naming convention ``<id>.json`` is
used throughout the workspace and matches the examples provided in the Lerobot
tutorials.

Python environment
------------------
Source the workspace in every terminal before launching any nodes::

   source ~/ros2_ws/install/setup.bash

Then export ``LECONDA_SITE_PACKAGES`` so the Lerobot Conda environment is added
to ``sys.path`` at runtime::

   export LECONDA_SITE_PACKAGES=<conda_root>/envs/lerobot_ros2/lib/python3.10/site-packages

If you created a symbolic link from the Lerobot sources into the
``so101_ros2_bridge`` install directory (see :doc:`installation`), verify that
the link still exists after rebuilding the workspace.

Cameras
-------
USB camera support is defined in ``so101_bringup/config/so101_cameras.yaml``.
Each entry references a USB camera parameter file such as
``so101_bringup/config/so101_usb_cam_front.yaml`` where the device path,
resolution and exposure settings are declared. Update ``video_device`` to match
the enumerated device (for example ``/dev/video4``) so the live feed appears in
RViz during teleoperation.

System data recorder
--------------------
The data recording launch file loads ``so101_bringup/config/so101_sdr.yaml`` to
configure the ``system_data_recorder`` node. Adjust the ``copy_destination``,
``bag_name_prefix`` and the list of topics to capture the information required
for your workflows before running a recording session.
