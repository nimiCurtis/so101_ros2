Getting Started
===============

Bridge configuration
--------------------
Update the leader and follower parameter files in
``so101_ros2_bridge/config`` so they reference the detected USB ports,
calibration directory, and Lerobot identifiers::

   so101_ros2_bridge/config/so101_leader_params.yaml
   so101_ros2_bridge/config/so101_follower_params.yaml

Each file exposes the ``port``, ``id`` and ``publish_rate`` fields that the
Python bridge nodes consume. The follower configuration also provides
``max_relative_target`` and ``disable_torque_on_disconnect`` to bound the
commands sent to the hardware. Set ``calibration_dir`` to the location where you
stored the JSON calibration files generated during the Lerobot setup.

Camera configuration
--------------------
Camera launch parameters live in ``so101_bringup/config/so101_cameras.yaml``. The
file lists the cameras to start, their namespaces, and the specific parameter
files to load. For example::

   cameras:
     - name: cam_front
       camera_type: usb_camera
       param_path: so101_usb_cam.yaml
       namespace: follower

The referenced USB camera configuration declares the per-device settings::

   /follower/cam_front:
     ros__parameters:
       video_device: "/dev/video0"
       frame_id: "cam_front"
       camera_name: "cam_front"
       camera_info_url: "package://usb_cam/config/camera_info.yaml"

Update ``video_device`` to match the enumerated device path (for example
``/dev/video4``) and adjust exposure, resolution, and other parameters to suit
your cameras.

Launch the system
-----------------
Source the workspace and launch the teleoperation pipeline with cameras and RViz
enabled::

   ros2 launch so101_bringup so101_robot_with_cameras.launch.py display:=true

Manipulate the follower arm to verify that the robot model in RViz follows the
real hardware and that the camera streams are visible.

.. image:: ../media/getting_started.png
   :alt: SO101 robot with cameras in RViz

.. image:: ../media/getting_started2.jpeg
   :alt: RViz camera view of the SO101 follower

Teleoperation launch
--------------------
Start a teleoperation session (real hardware) with::

   ros2 launch so101_bringup so101_teleoperate.launch.py mode:=real display:=true

The launch file starts the leader bridge, waits for the follower connection,
opens RViz (when ``display:=true``) and then enables the teleoperation node. Use
``mode:=isaac`` to connect to Isaac Sim transports instead of the physical
follower.
