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

This docs assumed that your SO101 is already assembled and all motors ids and boudrates are set.

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
    pip install -e ."[all]
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

3. Then from the lerobot directory, install the neccessary packages to run the isaacsim/lab scripts for the simulation inference deployment and the rl training scripts:

    ```bash
    pip install -e ."[feetech,smolvla,pi,async]" (check later which packages needed)
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

### Teleoperate

### Real teleoperation

First set properly the parameters in  ```so101_ros2_bridge/config/so101_leader_params.yaml``` and ```so101_ros2_bridge/config/so101_follower_params.yaml``` according to the location of your calibration files.

In addition set properly the camera port parameter in ```so101_bringup/config/so101_usb_cam_front.yaml``` here:

```yaml
.
.
ros__parameters:
    video_device: "/dev/video4" # Replace here
```

Then:

```bash
ros2 launch so101_bringup so101_teleoperate.launch.py mode:=real display:=true
```

### Sim teleoperation

TBD...

### Record Dataset with rosbag2 using system_data_recorder package

First set properly the parameters in ```so101_bringup/config/so101_sdr.yaml``` according to your desired topics names and dataset destinations.

Pay attention that each topic name should have it corresponding message type.

```yaml
.
.
# List of topics and their types to record.
topic_names: [
    "/joint_states",
]
topic_types: [
    "sensor_msgs/msg/JointState",
]
```

Then , while the teleopartion is runnning, launch from a second terminal the sdr lifecycle node with:

``` bash
ros2 launch so101_bringup so101_record.launch.py
```

And start recording from a third terminal with:

```bash
ros2 lifecycle set /sdr configure
ros2 lifecycle set /sdr activate
```

Stop recording with:

```bash
ros2 lifecycle set /sdr deactivate
```

For more information see [system_data_recording](https://github.com/nimiCurtis/system_data_recorder).

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for the
full license text.

---

## Contributions

Install the hooks with `pre-commit install` so the checks run automatically
before each commit. The CI executes `pre-commit` on pushes and pull requests
to the `main` and `dev` branches.

[TBD how to fork the repo and contribut]

---

## TODO:

- [ ] Go over movieit code and refactor (NO PLANNING LIBRARY LOADED issue)
- [ ] Clean code and refactoring
- [ ] Add integration with isaacSim/Lab
- [ ] Improve readme and check installation process
- [ ] Fix teleoperate with gazebo
