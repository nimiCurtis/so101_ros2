# so101_ros2

A ROS 2 driver for the Lerobot SO101 manipulator.

---

## Overview

This workspace contains several packages that provide description files, controllers,
MoveIt configurations and Gazebo simulation for the SO101 arm. It allows you to
visualise the robot model, control it with ROS 2 control and run a simulation
environment for testing and development.

---

## Dependencies

- ROS2 Humble from the [Official Link](https://docs.ros.org/en/humble/Installation.html)

---

## Setup Lerobot

These docs assume that your SO101 is already assembled and all motor IDs and baud rates are set.

#### Lerobot ROS2 Python Env Setup

1. Create and activate a Conda environment

    ```bash
    conda create -n lerobot_ros2 python=3.10
    conda activate lerobot_ros2
    ```

2. Clone and install the forked Lerobot repository

    ```bash
    git clone https://github.com/nimiCurtis/lerobot.git
    cd lerobot
    pip install -e ".[all]"
    ```

3. Grant access to USB ports (for robot communication)

    Add your user to the dialout group (recommended):

    ```bash
    sudo usermod -aG dialout $USER
    ```

    Then log out and log back in for the changes to take effect.

    Alternatively, you can manually change permissions:

    ```bash
    sudo chmod 666 /dev/ttyACM0
    sudo chmod 666 /dev/ttyACM1
    ```

4. Verify robot connections

    After connecting the robots via USB, detect their ports:

    ```bash
    lerobot-find-port
    ```

#### Calibrate

Check out [this link](https://huggingface.co/docs/lerobot/so101?calibrate_follower=Command#configure-the-motors) and skip to the *Calibrate* section to calibrate your leader/follower arms correctly and save the calibration files in a known directoy.

#### Validate installation and calibration

Try this tutorial from the [official link](https://huggingface.co/docs/lerobot/il_robots) to check env and robots are configured correctly.

#### (Optional) Lerobot IsaacSim/Lab Python Env Setup

1. Follow the instructions of the [this link](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/source_installation.html) to install IsaacSim 5.0 and the most updated IsaacLab.

    **[NOTE]: Install IsaacLab using conda so it can be synced with lerobot dependencies, this is a different conda environment then the lerobot_ros2 installed in the previous steps!!!, because the native python version of isaac is 3.11 and the native python of ros2 humble is 3.10**

2. Activate the environment:

    ```bash
    conda activate lerobot_isaac
    ```

3. Then from the lerobot directory, install the extras required for the IsaacLab
   utilities (adjust the extras list to match the features you use):

    ```bash
    pip install -e ".[feetech,smolvla,pi,async]"
    ```

## Build so101_ros2

Once you can teleoperate so101 leader-follower properly it is time to bridge to ros2 workspace.

1. Clone this repository into the `src` folder of your ROS 2 workspace and build with `colcon`:

    **[NOTE]: Dont forget to deactivate conda environment if was opened before compilation of the ros workspace**

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone --recurse-submodules git@github.com:nimiCurtis/so101_ros2.git
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    ```

2. Because lerebot env is managed by conda package manager, a workaround to compile the ros2 workspace and utilize the lerobot virtual env is to make a symbolic link between lerobot env to ros2_ws by:

    ```bash
    export LECONDA_SITE_PACKAGES="<your path to anaconda/miniconda installation/envs/lerobot_ros2/lib/python3.10/site-packages"
    export LEROBOT_SRC="<your path to lerobot pkg>/src/lerobot"
    export SO101BRIDGE_INSTALL_SITE_PACKAGES="<your path to ros2_ws>/install/so101_ros2_bridge/lib/python3.10/site-packages/lerobot"
    ln -s $LEROBOT_SRC $SO101BRIDGE_INSTALL_SITE_PACKAGES
    ```

---

## Getting Started

To visualise the robot description run:

```bash
ros2 launch so101_description display_robot.launch.py
```

Launch Gazebo simulation with:

```bash
ros2 launch so101_bringup so101_sim_gazebo.launch.py
```

---

## Imitation Learning with so101_ros2

This workspace connects the Lerobot leader/follower stack with ROS 2 so you can
teleoperate the hardware, stream observations into ROS tooling and record
demonstrations for imitation learning pipelines.

### Prerequisites

- Complete the Lerobot calibration procedure for both arms and keep the exported
  JSON files handy. The bridge looks for them in
  `so101_ros2_bridge/config/calibration/` by default (see `Gili.json` and
  `Tzili.json` for the expected naming scheme). Alternatively provide an
  absolute path via the `calibration_dir` parameter.
- Export `LECONDA_SITE_PACKAGES` in every terminal that launches the bridge so
  the Lerobot Python packages are discoverable:

  ```bash
  export LECONDA_SITE_PACKAGES=<path to conda>/envs/lerobot_ros2/lib/python3.10/site-packages
  ```

- Source the workspace in each terminal: `source ~/ros2_ws/install/setup.bash`.
- If you followed the optional symlink workaround in the build section, ensure
  those links are still valid after rebuilding.

### Configure bridge parameters

Edit `so101_ros2_bridge/config/so101_leader_params.yaml` and
`so101_ros2_bridge/config/so101_follower_params.yaml` so they reference the
correct USB ports, calibration directory and Lerobot identifiers:

```yaml
so101_follower_ros2_bridge:
  ros__parameters:
    port: "/dev/ttyACM1"
    id: "Tzili"
    calibration_dir: "/abs/path/to/calibration"
    use_degrees: true
    max_relative_target: 10
    disable_torque_on_disconnect: true
    publish_rate: 30.0
```

If `calibration_dir` is omitted the node falls back to the bundled calibration
files, which is useful for quick smoke testing.

### Camera configuration (real teleoperation)

When using the physical robot, update `so101_bringup/config/so101_usb_cam_front.yaml`
with the correct video device path so the camera feed shows up in RViz:

```yaml
ros__parameters:
  video_device: "/dev/video4"
```

### Run a real teleoperation session

Launch the leader and follower bridges, cameras and RViz in one terminal:

```bash
ros2 launch so101_bringup so101_teleoperate.launch.py mode:=real display:=true
```

The launch file brings up the leader bridge immediately, waits for the follower
to connect, optionally opens RViz (`display:=true`) and starts the teleoperation
node once both arms publish joint states. Watch the log output for any
connection errors—most issues stem from missing calibration files or incorrect
USB port assignments.

### Run a Gazebo teleoperation session (not working yet)

To test the pipeline without hardware, launch the same entry point in Gazebo
mode:

```bash
ros2 launch so101_bringup so101_teleoperate.launch.py mode:=gazebo display:=true
```

This starts the simulated follower, loads ROS 2 controllers and spawns the RViz
configuration so you can practice teleoperation flows before running them on
the real robot. The leader bridge still runs locally, so keep the leader arm
connected if you want to stream human demonstrations into the simulator.

### Run an Isaac teleoperation session

TBD...

### Record demonstrations with `system_data_recorder`

1. Configure the topics you care about in
   `so101_bringup/config/so101_sdr.yaml`.

2. Start teleoperation (real|Gazebo|Isaac) from another terminal.

3. Launch the recorder lifecycle node:

   ```bash
   ros2 launch so101_bringup so101_record.launch.py
   ```

4. Configure and activate the node to begin recording:

   ```bash
   ros2 lifecycle set /sdr configure
   ros2 lifecycle set /sdr activate
   ```

5. When you are done collecting a demonstration, stop the recording:

   ```bash
   ros2 lifecycle set /sdr deactivate
   ros2 lifecycle set /sdr shutdown
   ```

The resulting rosbag2 dataset is stored under the `copy_destination` directory
with the prefix defined in `bag_name_prefix`. Inspect the bag with
`ros2 bag info <bag_path>` or process it with your preferred imitation learning
tooling. For more options see the
[system_data_recorder documentation](https://github.com/nimiCurtis/system_data_recorder).

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for the
full license text.

---

## Contributions

Contributions are welcome. Fork the repository, create a feature branch and run
`pre-commit run --all-files` before opening a pull request. The CI triggers the
same hooks on pushes to `main` and `dev`.

---

## Roadmap

- [x] Add recording keyboard commander node for SDR
- [ ] Check refactored branch for no sim pkg
- [x] Refactor launch configuration for "mode" arg
- [x] Ensure `rw_rate > update_rate`
- [ ] Add appropriate QoS profiles for realtime topics
- [ ] Investigate the MoveIt "No Planning Library Loaded" warning and refactor the configuration
- [ ] Harden Gazebo teleoperation to match the hardware workflow
- [ ] Create rviz display config for each mode
- [ ] Check and fix GitHub Actions workflow
- [ ] General code clean-up and documentation
- [ ] Add realsense top camera