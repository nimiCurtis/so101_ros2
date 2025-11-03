# SmolVLA Inference Node

The `smolvla_inference_node.py` script runs a ROS 2 node that performs inference using a pre-trained SmolVLA (Small Vision-Language-Action) model. It takes multiple camera inputs and the robot's joint states, along with a natural language task description, to predict a sequence of actions.

## How it Works

The node loads a `SmolVLAPolicy` model from the `lerobot` library. It subscribes to image and joint state topics. On a timer, it gathers the latest sensor data, preprocesses it, and feeds it to the model's `predict_action_chunk` method. The resulting action sequence is then published.

## Subscribed Topics

| Topic | Message Type | Description | Default Topic Name |
|---|---|---|---|
| Camera 1 | `sensor_msgs/msg/Image` | Main camera feed. | `/follower/cam_front/image_raw` |
| Camera 2 | `sensor_msgs/msg/Image` | Second camera feed. | `/follower/cam_top1/image_raw` |
| Camera 3 | `sensor_msgs/msg/Image` | Third camera feed. | `/follower/cam_top2/image_raw` |
| Joint States | `sensor_msgs/msg/JointState` | The current state of the robot's joints. | `/isaac/isaac_joint_states` |

## Published Topics

| Topic | Message Type | Description | Default Topic Name |
|---|---|---|---|
| Action | `std_msgs/msg/Float32MultiArray` | The first action from the predicted sequence for immediate execution. | `/smolvla_inference/action` |
| Action Chunk | `std_msgs/msg/Float32MultiArray` | The complete chunk of 50 predicted future actions. | `/smolvla_inference/action_chunk` |

## SmolVLA Integration

### SmolVLA Class and Methods

**Class Used**: `lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy`

**Methods from SmolVLAPolicy:**
- `from_pretrained(model_id)` - Loads the pre-trained model from Hugging Face Hub. Returns a SmolVLAPolicy instance.
- `predict_action_chunk(flattened_transition)` - Main inference method. Takes processed observation data and returns a tensor of predicted actions.
  - Input: Dictionary containing flattened observation data with keys like `observation.images.camera1`, `observation.state`, etc.
  - Output: Tensor of shape `(batch_size, chunk_size, action_dim)` - typically `(1, 50, action_dim)`

**Helper Functions from lerobot:**
- `make_pre_post_processors(config, model_id, preprocessor_overrides)` - Creates preprocessing and postprocessing functions for the model.
- `build_inference_frame(observation, ds_features, device, task, robot_type)` - Builds a properly formatted observation frame from raw sensor data.
- `make_robot_action(processed_action, dataset_features)` - Converts model output to robot-executable action format.

### SmolVLAPolicy Configuration

The model is initialized with the following configuration:
- **model_id**: Hugging Face model identifier (e.g., `lerobot/smolvla_base`)
- **device**: Computation device (`cuda` if GPU available, otherwise `cpu`)

The model's configuration includes:
- **input_features**: Dictionary defining expected observation inputs (images and joint states)
- **output_features**: Dictionary defining action output dimensions

## Input Data Format

### Camera Images (Subscribed Topics)

The node subscribes to three camera image topics:

**Topic Type**: `sensor_msgs/msg/Image`

**Supported Encodings**:
- `rgb8`: 8-bit RGB images (height × width × 3)
- `bgr8`: 8-bit BGR images (automatically converted to RGB)
- `mono8`/`gray8`: Grayscale images (converted to RGB by replicating channels)

**Processing**:
1. ROS Image messages are converted to numpy arrays using `imgmsg_to_numpy()` method
2. Arrays have shape `(height, width, 3)` in RGB format
3. Images are then added to the observation dictionary with keys: `camera1`, `camera2`, `camera3`

**Example Topic Data** (metadata from `ros2 topic echo /follower/cam_front/image_raw --once`):
```
header:
  stamp:
    sec: 123
    nanosec: 456789000
  frame_id: 'camera_front'
height: 480
width: 640
encoding: 'rgb8'
is_bigendian: 0
step: 1920
data: [binary image data...]
```

**Available Camera Topics** (from `ros2 topic list`):
- `/follower/cam_front/image_raw`
- `/follower/cam_top1/image_raw`
- `/follower/cam_top2/image_raw`

### Joint States (Subscribed Topic)

**Topic Type**: `sensor_msgs/msg/JointState`

**Data Used**: Only the `position` field from the JointState message

**Processing**:
1. Joint positions are extracted as a list/array
2. Converted to a dictionary format: `{joint_0: value, joint_1: value, ..., joint_n: value}`
3. The number of joints must match the expected `state_dim` from the model's configuration

**Example Topic Data** (using `ros2 topic echo /isaac/isaac_joint_states --once`):
```
header:
  stamp:
    sec: 484
    nanosec: 283358590
  frame_id: ''
name:
- shoulder_pan
- shoulder_lift
- elbow_flex
- wrist_flex
- wrist_roll
- gripper
position:
- 0.0
- 0.0137
- 0.0044
- 0.0005
- 0.0
- 0.0
velocity:
- 0.7016
- -0.4871
- -0.5483
- 0.9869
- -1.0525
- -0.0449
effort:
- 0.0131
- -0.5392
- -0.4474
- -0.1159
- -0.0001
- 0.0035
```

**Processed Format**: The `position` field is converted to a dictionary for the model:
```python
{
  'joint_0': 0.0,
  'joint_1': 0.0137,
  'joint_2': 0.0044,
  'joint_3': 0.0005,
  'joint_4': 0.0,
  'joint_5': 0.0
}
```

Note: The node only uses the `position` field; `velocity` and `effort` are ignored.

## Output Data Format

### Single Action (Published Topic)

**Topic Type**: `std_msgs/msg/Float32MultiArray`

**Format**: 1D array containing the first action from the predicted chunk

**Structure**:
- Data: `[action_0, action_1, ..., action_n]`
- Length: Determined by the model's output action dimension (typically 7 for a 7-DOF robot)

**Purpose**: Represents the immediate action to be executed by the robot controller

**Example Topic Data** (using `ros2 topic echo /smolvla_inference/action --once`):
```
layout:
  dim: []
  data_offset: 0
data:
- 0.12
- -0.34
- 0.56
- -0.78
- 0.90
- -0.12
- 0.34
```

### Action Chunk (Published Topic)

**Topic Type**: `std_msgs/msg/Float32MultiArray`

**Format**: 2D array flattened to 1D, containing 50 predicted future actions

**Structure**:
- Shape: `(chunk_size, action_dim)` - typically `(50, 7)` for a 7-DOF robot
- Total elements: `chunk_size × action_dim` (e.g., 50 × 7 = 350 floats)
- Layout dimensions:
  - `dim[0]`:
    - label: `"actions"`
    - size: `chunk_size` (50)
    - stride: `chunk_size × action_dim`
  - `dim[1]`:
    - label: `"action_dimensions"`
    - size: `action_dim` (7)
    - stride: `action_dim`

**Purpose**: Provides a sequence of predicted actions for trajectory planning and model predictive control

**Accessing Elements**:
To access action `i` at dimension `j`:
- Index in flattened array: `i × action_dim + j`
- Or reshape the data: `actions = np.array(msg.data).reshape(chunk_size, action_dim)`

**Example Topic Data** (using `ros2 topic echo /smolvla_inference/action_chunk --once`):
```
layout:
  dim:
  - label: actions
    size: 50
    stride: 350
  - label: action_dimensions
    size: 7
    stride: 7
  data_offset: 0
data:
- 0.12    # action[0][0]
- -0.34   # action[0][1]
- 0.56    # action[0][2]
- -0.78   # action[0][3]
- 0.90    # action[0][4]
- -0.12   # action[0][5]
- 0.34    # action[0][6]
- 0.15    # action[1][0]
- -0.28   # action[1][1]
- ...     # (343 more values for remaining 48 actions)
```

**Example Reading Code**:
```python
def action_chunk_callback(self, msg):
    num_actions = msg.layout.dim[0].size  # 50
    action_dim = msg.layout.dim[1].size   # 7

    # Reshape to 2D array
    actions = np.array(msg.data).reshape(num_actions, action_dim)

    # Access specific action
    first_action = actions[0]      # Shape: (7,)
    tenth_action = actions[9]      # Shape: (7,)
    last_action = actions[-1]      # Shape: (7,)
```

## Node Parameters

The node's behavior can be configured through the following ROS 2 parameters:

| Parameter | Type | Default Value | Description |
|---|---|---|---|
| `model_id` | string | `lerobot/smolvla_base` | The Hugging Face ID of the pre-trained model to load. |
| `camera1_topic` | string | `/follower/cam_front/image_raw` | Topic for the first camera. |
| `camera2_topic` | string | `/follower/cam_top1/image_raw` | Topic for the second camera. |
| `camera3_topic` | string | `/follower/cam_top2/image_raw` | Topic for the third camera. |
| `joint_state_topic` | string | `/isaac/isaac_joint_states` | Topic for robot joint states. |
| `action_topic` | string | `/smolvla_inference/action` | Topic for the immediate action. |
| `action_chunk_topic` | string | `/smolvla_inference/action_chunk` | Topic for the full action sequence. |
| `task` | string | `Pick up the cube` | The natural language instruction for the task. |
| `robot_type` | string | `so100` | The type of robot being controlled. |
| `use_dummy_input` | bool | `False` | Set to `True` to use generated dummy data instead of live sensor topics. |
| `publisher_rate` | int | `2` | The frequency (in Hz) at which the inference timer runs and publishes actions. |
| `image_subscription_qos` | int | `2` | The QoS history depth for image topic subscriptions. |
| `joint_state_subscription_qos` | int | `2` | The QoS history depth for the joint state topic subscription. |
| `enable_action_chunk_echo` | bool | `False` | If `True`, the node subscribes to its own output action chunk topic for debugging. |

## Available ROS Topics

The following topics are available in the system (from `ros2 topic list`):

**Camera Topics:**
- `/follower/cam_front/camera_info`
- `/follower/cam_front/image_raw`
- `/follower/cam_top1/image_raw`
- `/follower/cam_top2/image_raw`

**Joint State Topics:**
- `/follower/joint_states`
- `/follower/joint_states_raw` (if available)
- `/isaac/isaac_joint_states` - Used by this node
- `/isaac_joint_states`
- `/leader/dynamic_joint_states`
- `/leader/joint_states`
- `/leader/joint_states_raw`

**Command Topics:**
- `/follower/joint_commands`
- `/isaac/isaac_joint_command`
- `/isaac_joint_command`
- `/leader/joint_commands`

**SmolVLA Inference Topics (Published by this node):**
- `/smolvla_inference/action` - Single immediate action
- `/smolvla_inference/action_chunk` - Full action sequence (50 actions)

**Robot Description Topics:**
- `/follower/robot_description`
- `/leader/robot_description`

**Transform Topics:**
- `/tf`
- `/tf_static`

**Other Topics:**
- `/camera_info`
- `/clock`
- `/clicked_point`
- `/goal_pose`
- `/initialpose`
- `/parameter_events`
- `/rosout`
- `/follower/gripper_controller/transition_events`
- `/leader/joint_state_broadcaster/transition_event`

## Usage Example

To run the node with default parameters:
```bash
ros2 run so101_ros2_bridge smolvla_inference_node
```

To run with custom parameters:
```bash
ros2 run so101_ros2_bridge smolvla_inference_node \
  --ros-args \
  -p task:="Pick up the red cube and place it in the box" \
  -p publisher_rate:=5 \
  -p model_id:="lerobot/smolvla_base"
```

To monitor the output:
```bash
# Monitor single action output
ros2 topic echo /smolvla_inference/action

# Monitor action chunk output
ros2 topic echo /smolvla_inference/action_chunk

# Monitor with rate limiting
ros2 topic hz /smolvla_inference/action
```
