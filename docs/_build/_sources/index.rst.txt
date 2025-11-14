SO101 ROS 2 Workspace
=====================

Summary
-------
The SO101 ROS 2 workspace connects Lerobot's leader and follower controllers to
the ROS 2 ecosystem. It packages the bridge nodes, bring-up launch files,
teleoperation utilities, and documentation for building datasets that support
imitation learning research.

Overview
--------
This documentation walks through the full workflow:

* Preparing the Lerobot and Python environments required by the bridge.
* Building the ROS 2 workspace and enabling camera pipelines.
* Configuring the bridge parameters and launching the system end-to-end.
* Capturing demonstrations and preparing them for imitation learning pipelines.


Packages
--------
- ``ros2_externals/system_data_recording`` – Nodes and launch files for capturing datasets for imitation learning using bag files.
- ``so101_bringup`` – End-to-end bring-up launch files for the SO101 robot.
- ``so101_controller`` – ROS 2 control launches and configurations.
- ``so101_description`` – URDF, xacro, meshes, usd files for the robot + ``robot_state_publisher`` launch file.
- ``so101_hardware_interface`` – Hardware interface for the SO101 robot.
- ``so101_ros2`` – Meta package for the SO101 ROS 2 workspace.
- ``so101_ros2_bridge`` – Bridge nodes connecting Lerobot APIs to ROS 2 topics and services.
- ``so101_teleop`` – Teleoperation component and launch file.

Acknowledgments
---------------
This project builds directly on the excellent `Lerobot <https://github.com/huggingface/lerobot>`_ framework from Hugging Face.

.. toctree::
   :maxdepth: 2
   :caption: Guides

   setup
   build
   ros2controlarch
   getting_started
   imitation_learning
   
