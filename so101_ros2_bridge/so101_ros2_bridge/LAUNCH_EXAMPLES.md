# SmolVLA Launch Examples

This document provides various examples of launching the SmolVLA inference and action execution system with different parameter configurations.

## Basic Launch (Default Parameters)

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py
```

This uses all default parameters:
- Model: `lerobot/smolvla_base`
- Task: "Pick up the white block and insert it on the green peg"
- Inference rate: 2 Hz
- Execution rate: 30 Hz
- Delay compensation: Enabled (0.5s inference delay)

---

## Example 1: Custom Task

Launch with a different task description:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    task:="Place the red cube into the box"
```

---

## Example 2: Faster Inference Rate

Increase inference frequency for more responsive behavior:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    inference_rate:=5 \
    inference_delay:=0.3
```

**Note:** Higher inference rates may require more powerful hardware. Adjust `inference_delay` based on actual measured inference time.

---

## Example 3: Different Execution Rate

Change the action execution frequency:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    publish_rate:=20.0
```

**Common execution rates:**
- 20 Hz (~50ms per action) - Slower, smoother
- 30 Hz (~33ms per action) - Default, balanced
- 50 Hz (~20ms per action) - Faster, more responsive

---

## Example 4: Disable Delay Compensation

For systems with very fast inference or debugging:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    use_delay_compensation:=false
```

This will execute actions starting from index 0 of each chunk instead of skipping initial actions.

---

## Example 5: Custom Topics

Launch with different topic names:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    camera1_topic:=/my_robot/camera/front \
    camera2_topic:=/my_robot/camera/top1 \
    camera3_topic:=/my_robot/camera/top2 \
    joint_state_topic:=/my_robot/joint_states \
    joint_command_topic:=/my_robot/joint_commands
```

---

## Example 6: Testing Mode with Dummy Data

Test the system without real camera/joint feeds:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    use_dummy_input:=true
```

---

## Example 7: High-Performance Configuration

Optimized for fast, responsive control:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    inference_rate:=5 \
    publish_rate:=50.0 \
    inference_delay:=0.2 \
    use_delay_compensation:=true
```

**Use case:** Systems with powerful GPUs that can achieve <200ms inference time.

---

## Example 8: Conservative Configuration

Lower rates for stability on slower hardware:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    inference_rate:=1 \
    publish_rate:=15.0 \
    inference_delay:=1.0
```

**Use case:** CPU-only inference or older hardware.

---

## Example 9: Production Setup with Full Configuration

Complete production configuration:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    model_id:="lerobot/smolvla_base" \
    task:="Pick up the blue cylinder and place it on the platform" \
    robot_type:="so100" \
    camera1_topic:="/follower/cam_front/image_raw" \
    camera2_topic:="/follower/cam_top1/image_raw" \
    camera3_topic:="/follower/cam_top2/image_raw" \
    joint_state_topic:="/isaac/isaac_joint_states" \
    action_topic:="/isaac/isaac_joint_command_test" \
    action_chunk_topic:="/smolvla_inference/action_chunk" \
    joint_command_topic:="/isaac/isaac_joint_command" \
    inference_rate:=3 \
    publish_rate:=30.0 \
    inference_delay:=0.4 \
    chunk_size:=50 \
    action_dim:=6 \
    use_delay_compensation:=true \
    image_qos:=2 \
    joint_state_qos:=2
```

---

## Example 10: 7-DOF Robot Configuration

For robots with 7 degrees of freedom:

```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    action_dim:=7 \
    robot_type:="custom_7dof"
```

---

## Understanding Key Parameters

### Inference Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_id` | `lerobot/smolvla_base` | HuggingFace model identifier |
| `task` | "Pick up..." | Natural language task description |
| `robot_type` | `so100` | Robot type identifier |
| `inference_rate` | `2` | How often to run inference (Hz) |
| `use_dummy_input` | `false` | Use synthetic data for testing |
| `image_qos` | `2` | QoS depth for camera subscriptions |
| `joint_state_qos` | `2` | QoS depth for joint state subscriptions |

### Executor Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_rate` | `30.0` | Action execution frequency (Hz) |
| `inference_delay` | `0.5` | Expected inference time (seconds) |
| `chunk_size` | `50` | Expected actions per chunk |
| `action_dim` | `6` | Number of robot joints |
| `use_delay_compensation` | `true` | Skip initial actions to compensate for inference delay |

---

## Calculating Delay Compensation

The executor automatically calculates how many actions to skip:

```
actions_to_skip = publish_rate × inference_delay
```

**Example:**
- Inference delay: 0.5 seconds
- Publish rate: 30 Hz
- Actions skipped: 30 × 0.5 = 15 actions

This means each new chunk starts execution from action 15 instead of action 0, compensating for the time that elapsed during inference.

---

## Measuring Your Inference Delay

To determine the optimal `inference_delay` parameter:

1. Launch with default settings and monitor logs:
   ```bash
   ros2 launch so101_ros2_bridge smolvla_nodes.launch.py
   ```

2. Look for inference timing in the logs:
   ```
   [smolvla_inference_node]: Total: 450.0ms (frame: 50ms, preproc: 80ms, model: 300ms, postproc: 20ms)
   ```

3. Set `inference_delay` to the average total time (in seconds):
   ```bash
   inference_delay:=0.45  # For 450ms inference time
   ```

---

## Troubleshooting

### Actions are delayed or sluggish
- Increase `inference_rate` (e.g., 3-5 Hz)
- Reduce `inference_delay` if your hardware is faster
- Ensure `use_delay_compensation:=true`

### Jerky or unstable motion
- Decrease `publish_rate` (e.g., 20 Hz)
- Increase `inference_delay` for more conservative compensation
- Check if inference time is consistent

### Running out of actions mid-chunk
- Your inference is too slow, increase `inference_delay`
- Or decrease `publish_rate` to make chunks last longer
- Verify `chunk_size` matches model output

### Robot doesn't respond to new commands quickly
- Enable delay compensation: `use_delay_compensation:=true`
- Tune `inference_delay` to match actual inference time
- Increase `inference_rate` if hardware allows

---

## Advanced: Launch File Composition

You can create custom launch files that import this one:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    smolvla_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('so101_ros2_bridge'),
                'launch',
                'smolvla_nodes.launch.py'
            )
        ]),
        launch_arguments={
            'task': 'Custom task here',
            'inference_rate': '4',
            'publish_rate': '40.0',
        }.items()
    )
    
    return LaunchDescription([
        smolvla_launch,
        # Add more nodes here
    ])
```

---

## Viewing Active Parameters

To see the current parameter values on running nodes:

```bash
# Inference node parameters
ros2 param list /smolvla_inference_node

# Executor node parameters  
ros2 param list /action_chunk_executor_node

# Get specific parameter value
ros2 param get /action_chunk_executor_node publish_rate
```

---

## Setting Parameters at Runtime

You can change certain parameters while nodes are running:

```bash
# Change execution rate
ros2 param set /action_chunk_executor_node publish_rate 25.0

# Enable/disable delay compensation
ros2 param set /action_chunk_executor_node use_delay_compensation true

# Note: Some parameters (like model_id) require node restart
```

---

## Best Practices

1. **Start with defaults** and measure actual performance
2. **Measure inference time** before tuning delay compensation
3. **Match execution rate** to your robot's control frequency
4. **Test with dummy data** before using real cameras
5. **Monitor logs** for warnings about chunk exhaustion
6. **Adjust gradually** - change one parameter at a time
7. **Document your setup** - save working configurations for different tasks

---

## Quick Reference Command Templates

**Standard operation:**
```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py
```

**With custom task:**
```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py task:="YOUR_TASK_HERE"
```

**Performance tuning:**
```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py \
    inference_rate:=X \
    publish_rate:=Y \
    inference_delay:=Z
```

**Testing mode:**
```bash
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py use_dummy_input:=true
```
