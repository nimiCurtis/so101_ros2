Data collection
===============

System Data Recorder
--------------------
The repository provides ``so101_bringup/launch/so101_record.launch.py`` to start
the ``system_data_recorder`` node with a preconfigured parameter file. Launching
the bring-up prepares the lifecycle node; use the ROS 2 lifecycle CLI to control
recording sessions while teleoperation is running in another terminal.

1. Start the recorder node::

   ros2 launch so101_bringup so101_record.launch.py

2. Configure and activate the node to begin recording::

   ros2 lifecycle set /sdr configure
   ros2 lifecycle set /sdr activate

3. When the demonstration is complete, stop and cleanly shut down the recorder::

   ros2 lifecycle set /sdr deactivate
   ros2 lifecycle set /sdr shutdown

Configuration
-------------
Parameters for the recorder are stored in ``so101_bringup/config/so101_sdr.yaml``.
The default setup writes bag files prefixed with ``robot_data`` to the path in
``copy_destination`` and records ``/joint_states`` as ``sensor_msgs/msg/JointState``.
Extend the ``topic_names`` and ``topic_types`` arrays to capture additional
datasets and adjust the ``max_file_size`` limit to control bag splitting during
long sessions.

Recorded bags are copied into ``copy_destination`` after the session. Use
``ros2 bag info <bag_path>`` to inspect their contents before converting them
for downstream imitation learning pipelines.
