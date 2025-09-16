Bring-up
========

Real hardware
-------------
Use ``so101_bringup/launch/so101_robot.launch.py`` to start either the follower
or leader hardware stack. The launch file exposes the ``type`` argument to pick
the robot role and forwards the ``model`` and ``display`` options to the
underlying description launch files. For example, to start the follower arm with
RViz::

   ros2 launch so101_bringup so101_robot.launch.py type:=follower display:=true

When ``type:=follower`` is selected the launch file loads
``include/follower.launch.py`` which in turn brings up the robot state publisher
for the real hardware, the Python bridge node and the ros2_control controller
manager with timed spawners for the arm and gripper controllers. Selecting
``type:=leader`` activates the analogous stack defined in
``include/leader.launch.py``.

The ``display_config`` argument may be used to pick a custom RViz configuration
and ``model`` can be set to an alternative URDF if needed. For setups with
cameras, ``so101_robot_with_cameras.launch.py`` extends the bring-up with the USB
camera pipeline before optionally launching RViz.

Simulation
----------
Gazebo bring-up is provided by ``so101_bringup/launch/so101_sim_gazebo.launch.py``.
The launch file forwards the ``model`` argument to the robot description and can
start RViz when ``display:=true``. Internally it loads
``include/sim_gazebo.launch.py`` which spawns the robot state publisher in
simulation mode, launches the Gazebo environment from ``so101_sim`` and spawns
the follower ros2_control controllers so that planning and teleoperation stacks
see the same interfaces as the hardware.

MoveIt integration
------------------
For motion planning use ``so101_bringup/launch/so101_moveit.launch.py``. It can
switch between real hardware and Gazebo simulation via the ``mode`` argument and
always launches the MoveIt configuration from ``so101_moveit``. When running in
Gazebo, the launch file reuses ``include/sim_gazebo.launch.py`` so the MoveIt
nodes are connected to the simulated controller interface.
