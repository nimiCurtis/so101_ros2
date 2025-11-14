:orphan:

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
``GripperActionController`` and a ``joint state broadcaster``. The leader namespace
runs only a ``joint state broadcaster``, reflecting its use as a sensing device. The
configuration keeps both command and state interfaces in position mode to match
the bridge expectations.

.. code-block:: yaml

    /follower:
    controller_manager:
        ros__parameters:
        update_rate: 50  # Hz

        arm_controller:
            type: joint_trajectory_controller/JointTrajectoryController

        gripper_controller:
            type: position_controllers/GripperActionController

        joint_state_broadcaster:
            type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
    
        ros__parameters:
        update_rate: 25
        type: joint_trajectory_controller/JointTrajectoryController
        joints:
            - shoulder_pan
            - shoulder_lift
            - elbow_flex
            - wrist_flex
            - wrist_roll
        command_interfaces:
            - position
        state_interfaces:
            - position
            - velocity
        
        state_publish_rate: 50.0  
        action_monitor_rate: 20.0 
        time_scaling: 1.1  

    gripper_controller:
        ros__parameters:
        type: position_controllers/GripperActionController
        joint: gripper

    /leader:
    controller_manager:
        ros__parameters:
        update_rate: 50  # Hz

        joint_state_broadcaster:
            type: joint_state_broadcaster/JointStateBroadcaster

    joint_state_broadcaster:
        ros__parameters:
        type: joint_state_broadcaster/JointStateBroadcaster
        joints:
            - shoulder_pan
            - shoulder_lift
            - elbow_flex
            - wrist_flex
            - wrist_roll
            - gripper

        interfaces:
            - position
