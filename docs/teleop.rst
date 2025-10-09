Teleoperation
=============

Prerequisites
-------------
- Complete the Lerobot calibration workflow and ensure the resulting JSON files
  are referenced by the leader and follower parameter files (see :doc:`setup`).
- Source the workspace and export ``LECONDA_SITE_PACKAGES`` in every terminal
  that launches bridge components.
- Configure the front camera parameter file with the correct ``video_device`` if
  you want the image stream in RViz during real teleoperation.

Launching
---------
``so101_bringup/launch/so101_teleoperate.launch.py`` orchestrates a complete
teleoperation session. It brings up the leader stack immediately, waits for the
follower to connect, optionally starts RViz and finally loads the teleoperation
component inside a multithreaded container.

To run against the physical robots and show RViz, use::

   ros2 launch so101_bringup so101_teleoperate.launch.py mode:=real display:=true

The launch file emits helpful log messages while connecting to the hardware.
Common issues include missing calibration files or incorrect serial port names.

To exercise the workflow in Gazebo without hardware, switch the mode::

   ros2 launch so101_bringup so101_teleoperate.launch.py mode:=gazebo display:=true

This path boots the simulated follower using ros2_control controllers so that
the teleoperation component can publish to the same interfaces as on the real
robot. The leader bridge still runs locally, letting you practice demonstrations
with the leader arm alone.

Component overview
------------------
The ``so101_teleop`` package provides the ``LeaderTeleopComponent`` ROS 2
component. It subscribes to the leader robot joint states, buffers the joint name
ordering on the first message and publishes streaming ``JointTrajectory``
messages for the follower arm. The trajectory points mirror the current leader
positions with a short ``time_from_start`` so controllers can track the data as a
continuous stream.

Parameters
----------
Default parameters are stored in ``so101_teleop/config/so101_leader_teleop.yaml``.
The ``leader_joint_states_topic`` selects the input topic (``/leader/joint_states``
by default) and ``follower_trajectory_topic`` configures the target controller
interface (``/follower/arm_controller/joint_trajectory`` by default).
