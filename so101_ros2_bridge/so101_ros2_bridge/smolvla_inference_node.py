import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float64MultiArray
# from cv_bridge import CvBridge #  Uncomment if cv_bridge is available
import numpy as np
import torch
import sys
import os
import time

# Add the lerobot src directory to the Python path
lerobot_src_path = '/home/anton/lerobot/src'
if lerobot_src_path not in sys.path:
    sys.path.insert(0, lerobot_src_path)

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import build_inference_frame, make_robot_action

class SmolVLAInferenceNode(Node):
    def __init__(self):
        super().__init__('smolvla_inference_node')
        self.get_logger().info('SmolVLA Inference Node starting...')

        # Declare parameters
        self.declare_parameter('model_id', 'lerobot/smolvla_base')
        self.declare_parameter('image_topic', '/follower/cam_front/image_raw')
        self.declare_parameter('joint_state_topic', '/follower/robot_state_publisher')
        self.declare_parameter('action_topic', '/smolvla_inference/action')
        self.declare_parameter('task', 'Pick up the cube')
        self.declare_parameter('robot_type', 'so100')
        self.declare_parameter('use_dummy_input', True)

        model_id = self.get_parameter('model_id').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        action_topic = self.get_parameter('action_topic').get_parameter_value().string_value
        self.task = self.get_parameter('task').get_parameter_value().string_value
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.use_dummy_input = self.get_parameter('use_dummy_input').get_parameter_value().bool_value

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f'Using device: {self.device}')

        self.get_logger().info(f'Loading model: {model_id}')
        load_start_time = time.time()
        self.model = SmolVLAPolicy.from_pretrained(model_id).to(self.device)
        load_end_time = time.time()
        
        self.pre_processor, self.post_processor = make_pre_post_processors(
            self.model.config,
            model_id,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
        )
        
        # Calculate and log model size
        total_params = sum(p.numel() for p in self.model.parameters())
        trainable_params = sum(p.numel() for p in self.model.parameters() if p.requires_grad)
        model_size_mb = sum(p.numel() * p.element_size() for p in self.model.parameters()) / (1024 * 1024)
        
        self.get_logger().info(f'Model loaded successfully in {load_end_time - load_start_time:.2f} seconds')
        self.get_logger().info(f'Model size: {model_size_mb:.2f} MB')
        self.get_logger().info(f'Total parameters: {total_params:,}')
        self.get_logger().info(f'Trainable parameters: {trainable_params:,}')

        # self.bridge = CvBridge() # Uncomment if cv_bridge is available

        # Get the actual feature names from the model config
        # This ensures we match what the model expects
        self.input_features = self.model.config.input_features
        self.output_features = self.model.config.output_features
        
        # Log what features the model expects
        self.get_logger().info(f'Input features: {list(self.input_features.keys())}')
        self.get_logger().info(f'Output features: {list(self.output_features.keys())}')

        # Convert PolicyFeature objects to dictionary format for build_dataset_frame
        # build_dataset_frame expects dicts with "dtype", "shape", "names" keys
        self.dataset_features = {}
        
        for key, feature in self.input_features.items():
            if feature.type.value == "STATE":
                # State features need individual joint names
                state_dim = feature.shape[0]
                self.dataset_features[key] = {
                    "dtype": "float32",
                    "shape": feature.shape,
                    "names": [f"joint_{i}" for i in range(state_dim)]
                }
            elif feature.type.value == "VISUAL":
                # Image features
                self.dataset_features[key] = {
                    "dtype": "image",
                    "shape": feature.shape,
                    "names": None
                }
        
        for key, feature in self.output_features.items():
            # Action features
            action_dim = feature.shape[0]
            self.dataset_features[key] = {
                "dtype": "float32",
                "shape": feature.shape,
                "names": [f"action_{i}" for i in range(action_dim)]
            }
        
        # Log the converted dataset features
        self.get_logger().info(f'Converted dataset_features keys: {list(self.dataset_features.keys())}')
        self.get_logger().info(f'Sample feature: {self.dataset_features.get("observation.state", "N/A")}')

        self.latest_image_msg = None
        self.latest_joint_state_msg = None

        if self.use_dummy_input:
            self.get_logger().info('Using dummy input for image and joint states.')
        else:
            self.image_subscriber = self.create_subscription(
                Image,
                image_topic,
                self.image_subscriber_callback,
                10
            )
            self.get_logger().info(f'Subscribing to image topic: {image_topic}')

            self.joint_state_subscriber = self.create_subscription(
                Float64MultiArray,
                joint_state_topic,
                self.joint_state_subscriber_callback,
                10
            )
            self.get_logger().info(f'Subscribing to joint state topic: {joint_state_topic}')

        self.action_publisher = self.create_publisher(
            Float32MultiArray,
            action_topic,
            10
        )
        self.get_logger().info(f'Publishing to action topic: {action_topic}')

        # Create a timer for inference
        self.timer = self.create_timer(0.5, self.inference_timer_callback)  # 2 Hz inference

    def image_subscriber_callback(self, msg):
        self.latest_image_msg = msg

    def joint_state_subscriber_callback(self, msg):
        self.latest_joint_state_msg = msg

    def inference_timer_callback(self):
        inference_start_time = time.time()
        
        # Determine the correct observation keys from model config
        # Get all image and state keys
        image_keys = [k.replace("observation.images.", "") for k in self.input_features.keys() if "image" in k]
        state_keys = [k for k in self.input_features.keys() if "state" in k]
        
        if not image_keys or not state_keys:
            self.get_logger().error(f'Could not find image or state keys in model config. Available keys: {list(self.input_features.keys())}')
            return
        
        # self.get_logger().info(f'Found image keys: {image_keys}')
        # self.get_logger().info(f'Found state keys: {state_keys}')
        
        # Use the first state key
        state_key = state_keys[0].replace("observation.", "")  # Extract just "state"

        if self.use_dummy_input:
            # Get expected state dimension from model config
            state_feature = self.input_features[f"observation.{state_key}"]
            state_dim = state_feature.shape[0]
            
            # Generate dummy robot state as individual named values
            dummy_robot_state = {f"joint_{i}": 0.0 for i in range(state_dim)}
            
            # Create raw observation dict with ALL required cameras
            raw_obs = {**dummy_robot_state}  # Start with joint values
            
            # Add dummy images for each camera the model expects
            for img_key in image_keys:
                # Get expected image shape from model config
                # PolicyFeature.shape is (C, H, W) but we need (H, W, C) for numpy
                image_feature = self.input_features[f"observation.images.{img_key}"]
                c, h, w = image_feature.shape
                
                # Generate dummy image as numpy array (H, W, C)
                dummy_cv_image = np.zeros((h, w, c), dtype=np.uint8)
                raw_obs[img_key] = dummy_cv_image
        else:
            if self.latest_image_msg is None:
                self.get_logger().warn('No image message received yet. Skipping inference.')
                return
            if self.latest_joint_state_msg is None:
                self.get_logger().warn('No joint state message received yet. Skipping inference.')
                return

            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='rgb8')
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')
                return

            try:
                # Get expected state dimension from model config
                state_feature = self.input_features[f"observation.{state_key}"]
                state_dim = state_feature.shape[0]
                
                # Get joint states as numpy array
                robot_state_data = np.array(self.latest_joint_state_msg.data, dtype=np.float32)
                if robot_state_data.shape[0] != state_dim:
                    self.get_logger().error(
                        f"Joint state dimension mismatch. Expected {state_dim}, got {robot_state_data.shape[0]}"
                    )
                    return
                
                # Convert array to dict with individual named values like "joint_0", "joint_1", etc.
                robot_state_dict = {f"joint_{i}": float(robot_state_data[i]) for i in range(state_dim)}
            except Exception as e:
                self.get_logger().error(f'Error processing joint state: {e}')
                return

            # Create raw observation dict with state
            raw_obs = {**robot_state_dict}
            
            # Add images - use the real camera image for camera1, dummy for others
            for img_key in image_keys:
                if img_key == "camera1":  # Assuming camera1 is your real camera
                    raw_obs[img_key] = cv_image
                else:
                    # Use dummy images for other cameras
                    image_feature = self.input_features[f"observation.images.{img_key}"]
                    c, h, w = image_feature.shape
                    dummy_cv_image = np.zeros((h, w, c), dtype=np.uint8)
                    raw_obs[img_key] = dummy_cv_image
                    self.get_logger().warn(f'Using dummy image for {img_key}', throttle_duration_sec=5.0)

        try:
            # Build the inference frame - converts numpy to tensors
            frame_build_start = time.time()
            obs_frame = build_inference_frame(
                observation=raw_obs,
                ds_features=self.dataset_features,
                device=self.device,
                task=self.task,
                robot_type=self.robot_type
            )
            frame_build_time = (time.time() - frame_build_start) * 1000
            
            # Debug: Check what build_inference_frame returned
            # self.get_logger().info(f'obs_frame keys: {list(obs_frame.keys())}')
            
            # obs_frame has flat keys like "observation.state", "observation.images.camera1"
            # But the preprocessor expects a nested structure with "observation" as top-level key
            # We need to restructure: {"observation": {all obs keys}, "task": ..., "robot_type": ...}
            
            observation_dict = {}
            for key, value in obs_frame.items():
                if key.startswith("observation."):
                    # Remove "observation." prefix to get the nested key
                    nested_key = key.replace("observation.", "", 1)
                    observation_dict[nested_key] = value
            
            # Create the properly structured transition
            # For inference, we need a complete transition structure
            transition = {
                "observation": observation_dict,
                "action": None,  # No action during inference (we're predicting it)
                "complementary_data": {
                    "task": obs_frame.get("task", ""),
                    "robot_type": obs_frame.get("robot_type", "")
                },
                "reward": None,
                "done": None,
                "truncated": None,
                "info": None,
            }
            
            # Pre-process using _forward to skip to_transition conversion
            preprocess_start = time.time()
            processed_transition = self.pre_processor._forward(transition)
            preprocess_time = (time.time() - preprocess_start) * 1000
            
            # The model expects flat keys like "observation.images.camera1" at the top level
            # We need to flatten the nested observation structure back
            flattened_transition = {}
            
            if "observation" in processed_transition and isinstance(processed_transition["observation"], dict):
                for key, value in processed_transition["observation"].items():
                    # Check if key already starts with "observation." (e.g., from tokenizer)
                    if key.startswith("observation."):
                        # Use as-is, already has the prefix
                        flat_key = key
                    else:
                        # Add "observation." prefix
                        flat_key = f"observation.{key}"
                    flattened_transition[flat_key] = value
            
            # Copy over other top-level keys
            for key in ["action", "complementary_data", "reward", "done", "truncated", "info"]:
                if key in processed_transition:
                    flattened_transition[key] = processed_transition[key]
            
            # Run inference with flattened structure
            model_inference_start = time.time()
            with torch.no_grad():
                # predic 50 actions ahead 50 [actions]*0.033 [sec/action] = 1.65 [sec] ahead assuming 33ms per step
                action_tensor = self.model.predict_action_chunk(flattened_transition)

            model_inference_time = (time.time() - model_inference_start) * 1000

            # Post-process action
            postprocess_start = time.time()
            _, chunk_size, _ = action_tensor.shape
            processed_actions = []
            for i in range(chunk_size):
                # Extract action at timestep i: (B, action_dim)
                single_action = action_tensor[:, i, :]
                processed_action = self.post_processor(single_action)
                processed_actions.append(processed_action)

            # Stack back to (B, chunk_size, action_dim), then remove batch dim
            action_tensor = torch.stack(processed_actions, dim=1).squeeze(0)

            robot_action = make_robot_action(processed_action, self.dataset_features)
            postprocess_time = (time.time() - postprocess_start) * 1000
            
            # Debug: Check robot action
            # self.get_logger().info(f'Robot action: {robot_action}')

            # Publish action
            action_msg = Float32MultiArray()
            
            # robot_action is a dict with keys like 'action_0', 'action_1', etc.
            # Extract them in order
            if isinstance(robot_action, dict):
                # Get all action keys and sort them
                action_keys = sorted([k for k in robot_action.keys() if k.startswith('action_')])
                if action_keys:
                    action_msg.data = [float(robot_action[k]) for k in action_keys]
                else:
                    self.get_logger().error(f'No action keys found in robot_action: {list(robot_action.keys())}')
                    return
            else:
                # Fallback for other formats
                if hasattr(robot_action, 'squeeze'):
                    action_msg.data = robot_action.squeeze().tolist()
                elif hasattr(robot_action, 'tolist'):
                    action_msg.data = robot_action.tolist()
                else:
                    action_msg.data = list(robot_action)
                
            self.action_publisher.publish(action_msg)
            
            inference_end_time = time.time()
            total_inference_time = (inference_end_time - inference_start_time) * 1000
            
            # Log timing breakdown
            self.get_logger().info(
                f'Published action: {action_msg.data} | '
                f'Total: {total_inference_time:.1f}ms '
                f'(frame: {frame_build_time:.1f}ms, '
                f'preproc: {preprocess_time:.1f}ms, '
                f'model: {model_inference_time:.1f}ms, '
                f'postproc: {postprocess_time:.1f}ms)',
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Error during inference: {e}', throttle_duration_sec=1.0)
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = SmolVLAInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()