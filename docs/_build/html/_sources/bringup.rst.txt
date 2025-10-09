Bring-up
========

Teleoperation bring-up (real hardware)
--------------------------------------
``so101_bringup/launch/so101_teleoperate.launch.py`` is the primary entry point
for running both arms in tandem. It launches the leader bridge first, waits for
the follower bridge to connect, starts cameras when in real mode and optionally
opens RViz after the controllers are ready::

   ros2 launch so101_bringup so101_teleoperate.launch.py mode:=real display:=true

Internally the launch file reuses ``include/leader.launch.py`` and
``include/follower.launch.py`` for the hardware bring-up, plus
``include/camera.launch.py`` when cameras are enabled. The teleoperation
component is injected into a multithreaded container once both bridges publish
joint states. Use the ``display_config`` argument to select a custom RViz
configuration if desired.

Standalone arm bring-up
-----------------------
When you only need one arm, ``so101_bringup/launch/so101_robot.launch.py`` can
start a single stack. Set ``type:=leader`` or ``type:=follower`` and optionally
``display:=true`` to launch RViz. The launch file forwards the ``model`` and
``display_config`` arguments to the description package. A companion launch file
``so101_robot_with_cameras.launch.py`` adds the USB camera pipeline.

Simulation
----------
The teleoperation launch file also supports Gazebo simulation via ``mode:=gazebo``.
It reuses ``include/sim_gazebo.launch.py`` to bring up the simulated follower
robot, ros2_control controllers and Gazebo environment from ``so101_sim`` before
loading the teleoperation component::

   ros2 launch so101_bringup so101_teleoperate.launch.py mode:=gazebo display:=true

For targeted simulation bring-up without teleoperation, use
``so101_bringup/launch/so101_sim_gazebo.launch.py`` directly. It forwards the
``model`` argument to the robot description and spawns the controllers required
for MoveIt and ros2_control integrations.

MoveIt integration
------------------
For motion planning use ``so101_bringup/launch/so101_moveit.launch.py``. It can
switch between real hardware and Gazebo simulation via the ``mode`` argument and
always launches the MoveIt configuration from ``so101_moveit``. When running in
Gazebo, the launch file reuses ``include/sim_gazebo.launch.py`` so the MoveIt
nodes are connected to the simulated controller interface.
