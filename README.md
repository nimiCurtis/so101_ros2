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

#### Env Setup

Follow the instructions in the [Official Link](https://huggingface.co/docs/lerobot/installation) for installing the lerobot python environment (Install from source is recommended)

You might need to give access to the USB ports by running:

```bash
sudo usermod -aG dialout $USER
```

Or:

```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```

Check lerobot env is activated:

```bash
conda activate lerobot
```

Then Connect the robots and try finding their ports by:

```bash
lerobot-find-port
```

#### Calibrate

Check out [this link](https://huggingface.co/docs/lerobot/so101?calibrate_follower=Command#configure-the-motors) and skip to the *Calibrate* section to calibrate your leader/follower arms correctly and save the calibration files in a known directoy.

#### Validate installation and calibration

Try this tutorial from the [official link](https://huggingface.co/docs/lerobot/il_robots) to check env and robots are configured correctly.

## Build so101_ros2

Once you can teleoperate so101 leader-follower properly it is time to bridge to ros2 workspace.

1. Clone this repository into the `src` folder of your ROS 2 workspace and build
with `colcon`:

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
    export LEROBOT_SRC="<your path to lerobot pkg>/src/lerobot"
    export SO101BRIDGE_INSTALL_SITE_PACKAGES="<your path to ros2_ws>/install/so101_ros2_bridge/lib/python3.10/site-packages/lerobot"
    ln -s $LEROBOT_SRC $SO101BRIDGE_INSTALL_SITE_PACKAGES
    ```

---

## Usage

To visualise the robot description run:

```bash
ros2 launch so101_description display_robot.launch.py
```

Launch the simulation environment with:

```bash
ros2 launch so101_bringup so101_sim_gazebo.launch.py
```

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for the
full license text.

---

## Development

Formatting and linting are handled with
[pre-commit](https://pre-commit.com/) which runs `black` and `flake8`.
Install the hooks with `pre-commit install` so the checks run automatically
before each commit. The CI executes `pre-commit` on pushes and pull requests
to the `main` and `dev` branches.

---

## TODO:

- [ ] Go over movieit code and refactor (NO PLANNING LIBRARY LOADED issue)
- [ ] Clean code and refactoring
- [ ] Add integration with isaacSim/Lab
- [ ] Improve readme and check installation process
- [ ] Fix teleoperate with gazebo
