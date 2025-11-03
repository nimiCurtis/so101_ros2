# SmolVLAInferenceNode Interfaces and Model Overview

## Input Interfaces

The node receives inputs primarily through ROS2 topics. These are configured via parameter files and managed using subscriber objects in rclpy.

- **Camera Inputs:**
    - Topics: `camera1topic`, `camera2topic`, `camera3topic`
    - Message Type: `sensor_msgs/Image`
    - Typical usage: 3 camera feeds (front, left, right) per robot are subscribed to, with each image converted to a numpy array and preprocessed. 
- **Joint State Inputs:**
    - Topic: `jointstatetopic`
    - Message Type: `std_msgs/Float64MultiArray`
    - Typical usage: Robot state vector (e.g., joint positions or other state variables) is subscribed to and used for model inference. 
- **Parameter Inputs:**
    - Example parameters: `modelid`, `task`, `robottype`, `usedummyinput`, `publisherrate`, `imagesubscriptionqos`, `jointstatesubscriptionqos`
    - Purpose: Configure which topics to use, which task/scenario, robot type, simulation/dummy input mode, QoS depths, and inference rate. 

| Input | Type | Example Parameter/Topic | ROS2 Message |
| :-- | :-- | :-- | :-- |
| Camera 1 | Image (RGB, BGR, or Mono) | `camera1topic` | sensor_msgs/Image  |
| Camera 2 | Image | `camera2topic` | sensor_msgs/Image  |
| Camera 3 | Image | `camera3topic` | sensor_msgs/Image  |
| Joint State | Float64 vector | `jointstatetopic` | std_msgs/Float64MultiArray  |
| Task/Settings | String, Int, Bool | Node parameters | N/A  |


***

## Output Interfaces

The node performs inference and publishes results to action topics as robot commands.

- **Action Outputs:**
    - Topics: `actiontopic` (single), `actionchunktopic` (multi-step/chunked)
    - Message Type: `std_msgs/Float32MultiArray`
    - Data Structure:
        - Single action: Direct float array representing one robot action. 
        - Chunked actions: Multi-dimensional float array, typically [num_actions, action_dim], published for trajectory prediction or open-loop control. The message uses `MultiArrayDimension` for structuring the array. 
- **Echo/Validation:**
    - Topic: `actionchunktopic` (subscribe, optional)
    - Used to receive back published chunks for validation/demonstration purposes, employing structured array layout.

| Output | Type | Topic | ROS2 Message |
| :-- | :-- | :-- | :-- |
| Single Robot Action | Float32 vector | `actiontopic` | std_msgs/Float32MultiArray  |
| Chunked Robot Actions | Float32 2D array | `actionchunktopic` | std_msgs/Float32MultiArray with layout  |
| Echo Chunk (optional) | Float32 2D array | `actionchunktopic` | std_msgs/Float32MultiArray  |


***

## SmolVLA Model Architecture and Parameters

The SmolVLA model (from `SmolVLAPolicy` class) is loaded via its pretrained identifier and configured by the node's parameters. 

- **Model Parameters:**
    - `modelid`: String, points to which weights/configuration to use (`lerobot.smolvla.base`, etc.)
    - `device`: `cuda` or `cpu`, automatically set depending on GPU availability
    - Input features (`model.config.inputfeatures`): Camera images (potentially multiple), robot state vectors, their shapes and types
    - Output features (`model.config.outputfeatures`): Action vectors, with dimensionality matching the robot
- **Model Stats:**
    - Model size and parameter count are logged on startup for transparency
    - Pre/post-processors: Created from model config and device, managing feature engineering and frame building before/after inference 
- **Inference Details:**
    - Typical forward pass predicts next action(s) based on latest robot state and visual input
    - Supports chunked prediction (e.g., predicting a block of 50 actions for open-loop execution)
    - Uses torch no_grad context for inference performance

***

## API and Data Flow Overview

1. **Startup:** Node loads/initializes SmolVLA model, subscribes to image and joint state topics.
2. **Inference Loop:**
    - On timer callback, node checks for latest images and robot state.
    - Builds an observation frame for the model.
    - Uses preprocessor to format observation, then predicts action(s) with model.
    - Postprocesses predictions and publishes to action topics as Float32MultiArray. 
3. **Validation:** Optionally subscribes to action chunks for demonstration/error checking. 

***

## Useful Snippets

- **Image conversion:** Handles `rgb8`, `bgr8`, `mono8`, `gray8` encodings, reshapes and stacks as needed.
- **Frame Construction:** Handles missing data by alerting and throttling.
- **Action chunk message:** Uses `layout.dim[]` in ROS2 MultiArray for [num_actions, action_dim] arrays.

***

## Example Parameters

```yaml
modelid: "lerobot.smolvla.base"
camera1topic: "/follower/cam_front/image_raw"
camera2topic: "/follower/cam_left/image_raw"
camera3topic: "/follower/cam_right/image_raw"
jointstatetopic: "/follower/robot_state_publisher"
actiontopic: "/smolvla/inference/action"
actionchunktopic: "/smolvla/inference/action_chunk"
task: "Pick up the cube"
robottype: "so100"
usedummyinput: false
publisherrate: 4   # Hz
imagesubscriptionqos: 2
jointstatesubscriptionqos: 2
```


***

## References

- Source code: `smolvla_inference_node.py` 

***

This document should help both technical and non-technical readers understand the interfaces for data exchange with SmolVLAInferenceNode and key aspects of the underlying SmolVLA model. For further details, check specific ROS2 message definitions and the `SmolVLAPolicy` class implementation. 

<div align="center">⁂</div>

 : smolvla_inference_node.py


---

# generate README.md

Below is a template for a README.md file specifically tailored for the SmolVLA inference node and its related interfaces, summarizing the primary usage, configuration, and model details. You can copy this directly and use it as the main README for your repository or project.

***

# SmolVLAInferenceNode

ROS2 node for deploying the SmolVLA policy model for real-time robotic inference.

## Features

- Multi-camera and joint state input support via configurable ROS2 topics[^2_1]
- Chunked or single-step robot action output for closed or open-loop control[^2_1]
- Supports various robot types and task specifications using parameter-driven configuration[^2_1]
- Integrated with pre-trained SmolVLA model, adjustable via `modelid` and runtime parameters[^2_1]

***

## Setup

### Requirements

- ROS2 Foxy or later
- Python 3.8+
- torch (PyTorch)
- numpy
- sensormsgs, stdmsgs (ROS2 message packages)
- SmolVLA model weights available (see below)[^2_1]

***

## Usage

1. **Add SmolVLA model and scripts to your workspace.**
2. **Configure parameters in `.yaml` or launch file:**
```yaml
modelid: "lerobot.smolvla.base"
camera1topic: "/follower/cam_front/image_raw"
camera2topic: "/follower/cam_left/image_raw"
camera3topic: "/follower/cam_right/image_raw"
jointstatetopic: "/follower/robot_state_publisher"
actiontopic: "/smolvla/inference/action"
actionchunktopic: "/smolvla/inference/action_chunk"
task: "Pick up the cube"
robottype: "so100"
usedummyinput: false
publisherrate: 4
imagesubscriptionqos: 2
jointstatesubscriptionqos: 2
```

3. **Launch the node:**
```bash
ros2 run lerobot smolvla_inference_node --ros-args --params-file path/to/config.yaml
```


***

## Interfaces

### Inputs

| Name | Type | ROS2 Message | Description |
| :-- | :-- | :-- | :-- |
| camera1topic | Image | sensor_msgs/Image | Main/front camera feed |
| camera2topic | Image | sensor_msgs/Image | Side/left camera feed |
| camera3topic | Image | sensor_msgs/Image | Side/right camera feed |
| jointstatetopic | Float64 vector | std_msgs/Float64MultiArray | Robot joint or state vector |

### Outputs

| Name | Type | ROS2 Message | Description |
| :-- | :-- | :-- | :-- |
| actiontopic | Float32MultiArray | std_msgs/Float32MultiArray | Single robot action |
| actionchunktopic | Float32MultiArray (2D) | std_msgs/Float32MultiArray | Sequence of robot actions |


***

## SmolVLA Model

- **Policy Class:** SmolVLAPolicy (imported from lerobot.policies.smolvla.modelingsmolvla)[^2_1]
- **Configurable Parameters:** `modelid`, input/output feature spec, device (CPU/CUDA), robot type[^2_1]
- **Input:** Camera image tensors (converted from ROS2 messages) + state vectors
- **Output:** Robot action vectors (dimension matches robot type)[^2_1]
- **Supports:** Chunked prediction, frame preprocessing, feature postprocessing

***

## Advanced

- **Pre-/Post-processors:** Created from the model config, ensure all observations and actions are standardized[^2_1]
- **QoS:** Adjustable QoS depths for image and joint subscriptions[^2_1]
- **Chunked Output:** Action chunk includes `MultiArrayDimension` layout for multi-step trajectories[^2_1]

***

## Notes

- For more details, consult the Python source in `smolvla_inference_node.py` and the SmolVLAPolicy source.[^2_1]
- Ensure all camera/image topics publish using `sensor_msgs/Image` with encodings `rgb8`, `bgr8`, `mono8`, `gray8`.
- All topics and parameters are overridable at launch via ROS2 params or CLI.[^2_1]

***

## References

- Node implementation: smolvla_inference_node.py[^2_1]
- ROS2 documentation: https://docs.ros.org/en/foxy
- PyTorch documentation: https://pytorch.org/docs/stable/index.html

***

This README.md gives a concise guide for using and understanding the SmolVLA inference node, making it straightforward for collaborators to set up and start using the system.[^2_1]

<div align="center">⁂</div>

[^2_1]: smolvla_inference_node.py

