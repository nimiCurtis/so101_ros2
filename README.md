# so101_ros2
A ROS 2 driver for the Lerobot SO101 manipulator.

## Overview
This workspace contains several packages that provide description files, controllers,
MoveIt configurations and Gazebo simulation for the SO101 arm. It allows you to
visualise the robot model, control it with ROS 2 control and run a simulation
environment for testing and development.

## Build
Clone this repository into the `src` folder of your ROS 2 workspace and build
with `colcon`:

```bash
cd ~/ros2_ws
rosdep install --from-paths src -r -y
colcon build --packages-select so101_description so101_controller so101_moveit so101_sim
source install/setup.bash
```

## Usage
To visualise the robot description run:

```bash
ros2 launch so101_description display.launch.py
```

Launch the simulation environment with:

```bash
ros2 launch so101_sim so101_sim_gazebo.launch.py
```

## License
This project is licensed under the MIT License. See [LICENSE](LICENSE) for the
full license text.

## Development

Linting is checked with [flake8](https://flake8.pycqa.org/) via GitHub Actions.
Run `flake8` locally before submitting changes.