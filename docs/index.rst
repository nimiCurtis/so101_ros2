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

Acknowledgments
---------------
The SO101 ROS 2 bridge builds on the Lerobot project from Hugging Face for the
leader and follower device APIs used throughout the bridge implementation.

.. toctree::
   :maxdepth: 2
   :caption: Guides

   setup
   build
   getting_started
   imitation_learning
