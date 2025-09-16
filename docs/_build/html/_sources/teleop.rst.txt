Teleoperation
=============

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

Launching
---------
``so101_bringup/launch/so101_teleoperate.launch.py`` orchestrates a complete
teleoperation session. It brings up the leader stack first, optionally the
follower hardware, and finally loads the teleoperation component inside a
multithreaded component container. Use the ``mode`` argument to switch between
real hardware and Gazebo simulation::

   ros2 launch so101_bringup so101_teleoperate.launch.py mode:=real

When ``mode:=gazebo`` the launch file skips the follower hardware bring-up and
runs with simulated time while still attaching the teleoperation component to the
leader bridge output.
