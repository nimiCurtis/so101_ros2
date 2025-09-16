Data collection
===============

System Data Recorder
--------------------
The repository provides ``so101_bringup/launch/so101_record.launch.py`` to start
the ``system_data_recorder`` node with a preconfigured parameter file. Launching
it will record the topics listed in the configuration while mirroring the output
into the configured destination directory::

   ros2 launch so101_bringup so101_record.launch.py

Configuration
-------------
Parameters for the recorder are stored in ``so101_bringup/config/so101_sdr.yaml``.
The default setup writes bag files prefixed with ``robot_data`` to the path in
``copy_destination`` and records ``/joint_states`` as ``sensor_msgs/msg/JointState``.
Extend the ``topic_names`` and ``topic_types`` arrays to capture additional
datasets and adjust the ``max_file_size`` limit to control bag splitting during
long sessions.
