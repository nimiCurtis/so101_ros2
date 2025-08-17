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

- ROS2 Humble 
- Gazebo fortress-ignition 
- RMW cyclonedds

---

## Build

Clone this repository into the `src` folder of your ROS 2 workspace and build
with `colcon`:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build 
source install/setup.bash
```

---

## Usage

To visualise the robot description run:

```bash
ros2 launch so101_description display_robot.launch.py
```

Launch the simulation environment with:

```bash
ros2 launch so101_sim so101_sim_gazebo.launch.py
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

## TODO

- [ ] Go over movieit code and refactor
- [ ] Fix teleoperate with gazebo
- [ ] Add teleoperate functionalities with joy (and keyboard?)
- [ ] Add integration with isaacSim/Lab
- [ ] Add gripper control using teleop ( joint controller / or using action server)
- [ ] Clean code and refactoring
- [ ] Add externals (sdr pkg)
- [ ] Add Docs with sphyncs
- [ ] Improve readme and check installation process