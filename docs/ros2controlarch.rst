ROS 2 Control Architecture
==========================

Hardware interface
------------------
The ``so101_hardware_interface`` package exposes a ``SystemInterface`` plugin
named ``so101_hardware_interface/So101HardwareBridge``. It creates a dedicated
ROS 2 node that publishes commands on the ``joint_commands`` topic and subscribes
to raw joint states from the Python bridge on ``joint_states_raw``. Incoming
joint states are matched against the URDF joint order before being stored in the
ros2_control state buffers, while outgoing commands stream the latest controller
set-points back to the Python layer. The plugin is exported in
``so101_hardware_bridge_plugins.xml`` so it can be referenced from the URDF.

Controller layout
-----------------
Controller definitions live in ``so101_controller/config/so101_controllers.yaml``.
The follower namespace loads a ``joint_trajectory_controller`` for the arm, a
``GripperActionController`` and a joint state broadcaster. The leader namespace
runs only a joint state broadcaster, reflecting its use as a sensing device. The
configuration keeps both command and state interfaces in position mode to match
the bridge expectations.

Launch files
------------
``so101_controller/launch/controller_manager.launch.py`` starts the
``ros2_control_node`` with the controller configuration. The separate launch file
``so101_controller/launch/so101_controllers.launch.py`` spawns the joint state
broadcaster before the arm and gripper controllers under the namespace provided
through the ``type`` launch argument. The bring-up launch descriptions use timer
actions to delay controller spawning until the hardware interfaces and bridge
nodes are online.
