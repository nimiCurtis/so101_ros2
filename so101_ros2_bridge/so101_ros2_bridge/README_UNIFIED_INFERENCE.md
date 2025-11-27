# Unified Inference Node Documentation

## Overview

The unified inference node consolidates the previously separate `smolvla_inference_node.py` and `smolvla_inference_node_pi05.py` into a single `inference_node.py` that supports both SmolVLA and PI05 models through a `model_type` parameter.

## Key Changes

### 1. Single Node Implementation
- **Before**: Two separate nodes (`smolvla_inference_node.py` and `smolvla_inference_node_pi05.py`)
- **After**: One unified node (`inference_node.py`)

### 2. Model Type Parameter
- New parameter `model_type` with values: `'smolvla'` or `'pi05'`
- Automatically handles model-specific configurations internally

### 3. Model Loading Strategy
The node now includes two private methods:
- `_load_smolvla_model()`: Loads SmolVLA models with standard configuration
- `_load_pi05_model()`: Loads PI05 models with bfloat16 precision and dynamo disabled

## Model-Specific Differences

### SmolVLA Models
- Uses standard float32 precision
- Direct model loading: `SmolVLAPolicy.from_pretrained()`
- Standard preprocessor configuration
- No special torch compilation handling needed

### PI05 Models
- Requires bfloat16 precision for all parameters and buffers
- Torch dynamo must be disabled (`torch._dynamo.disable`)
- Gradient checkpointing disabled for inference
- Additional dtype verification and logging
- Environment variable: `TORCH_COMPILE_DISABLE=1`
- Preprocessor configured with: `float_dtype="bfloat16"`

## Usage

### Launch with SmolVLA Model

```bash
ros2 launch so101_ros2_bridge unified_nodes.launch.py \
    model_type:=smolvla \
    model_id:=/path/to/smolvla/model \
    task:="Pick the cube and place it in the bowl."
```

### Launch with PI05 Model

```bash
ros2 launch so101_ros2_bridge unified_nodes.launch.py \
    model_type:=pi05 \
    model_id:=/path/to/pi05/model \
    task:="Pick the cube and place it in the bowl."
```

### Additional Parameters

All previous parameters are still supported:

```bash
ros2 launch so101_ros2_bridge unified_nodes.launch.py \
    model_type:=pi05 \
    model_id:=/home/anton/outputs/train/pi05_model/pretrained_model \
    task:="Pick up the white block and insert it on the green peg" \
    camera1_topic:=/follower/cam_front/image_raw \
    camera2_topic:=/static_camera/cam_side/image_raw \
    joint_state_topic:=/follower/joint_states \
    action_topic:=/leader/joint_states \
    inference_rate:=1 \
    use_dummy_input:=false \
    teleop_mode:=isaac
```

## Implementation Details

### Node Initialization Flow

1. **Parameter Declaration**: Standard ROS2 parameters including new `model_type`
2. **Model Type Validation**: Ensures `model_type` is either 'smolvla' or 'pi05'
3. **Device Selection**: CUDA if available, otherwise CPU
4. **Model Loading**: Calls appropriate private method based on `model_type`
5. **Feature Extraction**: Extracts input/output features from model config
6. **Subscriber/Publisher Setup**: Standard ROS2 topic configuration
7. **Timer Setup**: Periodic inference at specified rate

### Inference Pipeline

The inference pipeline is identical for both model types:

1. **Frame Building**: Construct observation frame from sensor data
2. **Preprocessing**: Apply model-specific preprocessing
3. **Model Inference**: Call `model.predict_action_chunk()`
4. **Postprocessing**: Process action chunks
5. **Publishing**: Publish both single action (JointState) and action chunk (Float32MultiArray)

### Error Handling

- PI05 models include additional logging for inference failures
- Both models handle torch exceptions gracefully
- Inference progress flag prevents overlapping inferences

## Migration Guide

### From Separate Nodes to Unified Node

**Step 1**: Replace the old nodes in your package
```bash
# Remove old files
rm smolvla_inference_node.py
rm smolvla_inference_node_pi05.py

# Add new unified node
cp inference_node.py <your_ros2_package>/src/
```

**Step 2**: Update launch file
```bash
# Replace old launch file
rm smolvla_nodes.launch.py

# Add new unified launch file
cp unified_nodes.launch.py <your_ros2_package>/launch/
```

**Step 3**: Update CMakeLists.txt or setup.py
For Python packages (setup.py):
```python
entry_points={
    'console_scripts': [
        'inference_node = so101_ros2_bridge.inference_node:main',  # New unified node
        'action_chunk_executor_node = so101_ros2_bridge.action_chunk_executor_node:main',
    ],
}
```

**Step 4**: Update your launch commands
```bash
# Old command (SmolVLA)
ros2 launch so101_ros2_bridge smolvla_nodes.launch.py

# New command (SmolVLA)
ros2 launch so101_ros2_bridge unified_nodes.launch.py model_type:=smolvla

# New command (PI05)
ros2 launch so101_ros2_bridge unified_nodes.launch.py model_type:=pi05
```

## Benefits of Unified Approach

1. **Single Codebase**: Easier maintenance with one node to update
2. **Consistent Interface**: Same parameters and topics for both model types
3. **Reduced Duplication**: Shared code for subscribers, publishers, and inference loop
4. **Cleaner Launch Files**: One launch file with model type selection
5. **Easy Model Switching**: Change models by modifying one parameter
6. **Better Testing**: Single test suite covers both model types

## Troubleshooting

### PI05 Model Issues

If you encounter dtype-related errors with PI05:
```
Check the log output for:
"⚠️ Found X float32 parameters!"
"⚠️ Found X float32 buffers!"

If present, the model may not be properly converted to bfloat16.
```

Solution: Ensure the model checkpoint was saved with bfloat16 precision during training.

### Inference Timeout

If inference is too slow:
```bash
# Reduce inference rate
ros2 launch so101_ros2_bridge unified_nodes.launch.py \
    inference_rate:=0.5  # Try lower rates first
```

### Memory Issues

For large models on limited GPU memory:
- Consider reducing batch size in model config
- Use model quantization
- Check for memory leaks with `nvidia-smi`

## Performance Comparison

Based on the original implementations:

| Aspect | SmolVLA | PI05 |
|--------|---------|------|
| Precision | float32 | bfloat16 |
| Memory Usage | Higher | Lower |
| Speed | Baseline | Similar/Faster |
| Compilation | Standard | Disabled |
| Preprocessing | Standard | bfloat16-aware |

## Future Enhancements

Potential improvements to consider:

1. **Dynamic Model Loading**: Hot-swap models without restarting node
2. **Model Registry**: Configuration file with multiple model presets
3. **Performance Profiling**: Built-in timing and resource monitoring
4. **Model Ensembles**: Support for running multiple models simultaneously
5. **Automatic Model Detection**: Infer model type from checkpoint metadata

## Contact & Support

For issues or questions:
- Check ROS2 logs: `ros2 node info inference_node`
- Verify topics: `ros2 topic list`
- Monitor performance: `ros2 topic hz /smolvla_inference/action_chunk`
