Setup
=====

Lerobot prerequisites
---------------------
Install ROS 2 Humble on Ubuntu 22.04 with the Cyclone DDS RMW implementation.
(Optional) Isaac Sim 5.0 or later is required only for the simulator
teleoperation workflows.

Python environment
------------------
Create an isolated Conda environment for Lerobot and install the forked
repository that the ROS 2 bridge depends on::

   conda create -n lerobot_ros2 python=3.10
   conda activate lerobot_ros2
   conda install -n lerobot_ros2 -c conda-forge "libstdcxx-ng>=12" "libgcc-ng>=12"
   git clone https://github.com/nimiCurtis/lerobot.git
   cd lerobot
   pip install -e ".[all]"

The additional ``libstdcxx-ng`` and ``libgcc-ng`` packages avoid missing GLIBCXX
symbols when importing ROS 2 Python modules from Conda.

USB access
----------
Add your user to the ``dialout`` group so the bridge can communicate with the
serial-connected arms::

   sudo usermod -aG dialout $USER

Log out and back in for the group membership to take effect. As a temporary
fallback you can change the permissions on the detected device paths::

   sudo chmod 666 /dev/<leader port>
   sudo chmod 666 /dev/<follower port>

Calibration workflow
--------------------
Follow the Lerobot SO101 calibration guide to generate JSON calibration files
for each manipulator. Save them to a known directory and reference that path
from the bridge parameter files. The bundled defaults under
``so101_ros2_bridge/config/calibration`` provide an initial reference but should
be replaced with your measured values.

Verification
------------
Use ``lerobot-find-port`` to list the connected leader and follower USB ports
and confirm that the Conda environment can access both arms. Then execute the
Lerobot tutorials to validate communication and calibration before continuing to
the ROS 2 workspace.

Optional: Isaac Lab environment
-------------------------------
Install Isaac Sim 5.0 and Isaac Lab in a separate Conda environment using the
official instructions. Isaac Lab requires Python 3.11, so keeping it separate
from ``lerobot_ros2`` (Python 3.10) avoids dependency conflicts. Activate the
environment when working with the simulator-specific teleoperation tools::

   conda activate lerobot_isaaclab
   pip install -e ".[feetech,smolvla,pi,async]"
