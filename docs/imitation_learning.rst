Imitation Learning
==================

The SO101 ROS 2 workspace integrates teleoperation, recording, and dataset
conversion tools so you can generate demonstrations for downstream imitation
learning pipelines.

Real teleoperation
------------------
Launch the bridge with the physical leader and follower hardware::

   ros2 launch so101_bringup so101_teleoperate.launch.py mode:=real display:=true

Monitor the logs for connection issues and confirm that the follower mirrors the
leader motions while RViz visualises the joint states and camera feeds.

.. admonition:: Demo video (MP4)

   Placeholder for the real teleoperation walkthrough video.

Isaac Sim teleoperation
-----------------------
Use the simulator workflow to stream demonstrations into Isaac Sim::

   ros2 launch so101_bringup so101_teleoperate.launch.py mode:=isaac display:=true

Start Isaac Sim separately, load the SO101 scene, and then enable simulation to
mirror the leader arm in the virtual environment.

.. admonition:: Demo video (MP4)

   Placeholder for the Isaac Sim teleoperation walkthrough video.

Record demonstrations with ``system_data_recorder``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The workspace ships with launch files for the
``system_data_recorder`` package to capture rosbag2 datasets.

1. Adjust ``so101_bringup/config/so101_sdr.yaml`` to list the topics you want to
   record, set ``bag_name_prefix`` and ``copy_destination``.
2. Start your teleoperation session (real or Isaac).
3. Launch the recorder::

       ros2 launch so101_bringup so101_record.launch.py

4. Drive the lifecycle transitions from another terminal::

       ros2 lifecycle set /sdr configure
       ros2 lifecycle set /sdr activate

5. Stop the recording when the demonstration is complete::

       ros2 lifecycle set /sdr deactivate
       ros2 lifecycle set /sdr shutdown

The optional ``SDRKeyboardCommander`` node provides hotkeys for these
transitions.

.. admonition:: Demo video (MP4)

   Placeholder for the system data recorder tutorial video.

Convert rosbag2 datasets
------------------------
Convert captured rosbag2 files into Lerobot datasets with the
``so101_rosbag2lerobot_dataset`` utilities::

   conda activate lerobot_ros2
   git clone https://github.com/nimiCurtis/so101_rosbag2lerobot_dataset.git
   cd so101_rosbag2lerobot_dataset
   pip install so101_rosbag2lerobot_dataset
   so101-rosbag2lerobot --config <path_to_config.yaml>

Prepare a YAML configuration describing the topics and output directory. Inspect
results with ``lerobot`` after conversion.

.. admonition:: Demo video (MP4)

   Placeholder for the rosbag conversion tutorial video.

Training (placeholder)
----------------------
Document the process used to fine-tune a VLA model on the converted dataset.

.. admonition:: Demo video (MP4)

   Placeholder for the training pipeline video.

Deployment (placeholder)
------------------------
Describe how to deploy the trained policy back to the SO101 hardware.

.. admonition:: Demo video (MP4)

   Placeholder for the deployment demonstration video.
