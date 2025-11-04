import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension
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

        # /camera_info
        # /follower/cam_front/camera_info
        # /follower/cam_front/image_raw
        # /follower/cam_top1/image_raw
        # /follower/cam_top2/image_raw
        # /isaac/isaac_joint_command
        # /isaac/isaac_joint_states

        # Declare parameters
        self.declare_parameter('model_id', 'lerobot/smolvla_base')  # Using base model (large doesn't exist)
        self.declare_parameter('camera1_topic', '/follower/cam_front/image_raw')
        self.declare_parameter('camera2_topic', '/follower/cam_top1/image_raw')
        self.declare_parameter('camera3_topic', '/follower/cam_top2/image_raw')
        self.declare_parameter('joint_state_topic', '/isaac/isaac_joint_states') # /isaac/isaac_joint_states 
        self.declare_parameter('action_topic', '/isaac/isaac_joint_command') # /smolvla_inference/action
        self.declare_parameter('action_chunk_topic', '/smolvla_inference/action_chunk')
        self.declare_parameter('task', 'Pick up the cube')
        self.declare_parameter('robot_type', 'so100')
        self.declare_parameter('use_dummy_input', False)  # Changed to False - use real topics by default
        self.declare_parameter('publisher_rate', 2)  # Hz for action publishing
        self.declare_parameter('image_subscription_qos', 2)  # QoS depth for image subscription
        self.declare_parameter('joint_state_subscription_qos', 2)  # QoS depth for joint state subscription

        model_id = self.get_parameter('model_id').get_parameter_value().string_value
        camera1_topic = self.get_parameter('camera1_topic').get_parameter_value().string_value
        camera2_topic = self.get_parameter('camera2_topic').get_parameter_value().string_value
        camera3_topic = self.get_parameter('camera3_topic').get_parameter_value().string_value
        joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        action_topic = self.get_parameter('action_topic').get_parameter_value().string_value
        action_chunk_topic = self.get_parameter('action_chunk_topic').get_parameter_value().string_value
        self.task = self.get_parameter('task').get_parameter_value().string_value
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.use_dummy_input = self.get_parameter('use_dummy_input').get_parameter_value().bool_value
        publisher_rate = self.get_parameter('publisher_rate').get_parameter_value().integer_value
        image_qos = self.get_parameter('image_subscription_qos').get_parameter_value().integer_value
        joint_state_qos = self.get_parameter('joint_state_subscription_qos').get_parameter_value().integer_value

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

        # Get the actual feature names from the model config
        self.input_features = self.model.config.input_features
        self.output_features = self.model.config.output_features
        
        # Log what features the model expects
        self.get_logger().info(f'Input features: {list(self.input_features.keys())}')
        self.get_logger().info(f'Output features: {list(self.output_features.keys())}')

        # Convert PolicyFeature objects to dictionary format for build_dataset_frame
        self.dataset_features = {}
        
        for key, feature in self.input_features.items():
            if feature.type.value == "STATE":
                state_dim = feature.shape[0]
                self.dataset_features[key] = {
                    "dtype": "float32",
                    "shape": feature.shape,
                    "names": [f"joint_{i}" for i in range(state_dim)]
                }
            elif feature.type.value == "VISUAL":
                self.dataset_features[key] = {
                    "dtype": "image",
                    "shape": feature.shape,
                    "names": None
                }
        
        for key, feature in self.output_features.items():
            action_dim = feature.shape[0]
            self.dataset_features[key] = {
                "dtype": "float32",
                "shape": feature.shape,
                "names": [f"action_{i}" for i in range(action_dim)]
            }

        self.latest_camera1_msg = None
        self.latest_camera2_msg = None
        self.latest_camera3_msg = None
        self.latest_joint_state_msg = None

        if self.use_dummy_input:
            self.get_logger().info('Using dummy input for image and joint states.')
        else:
            self.camera1_subscriber = self.create_subscription(
                Image,
                camera1_topic,
                self.camera1_callback,
                image_qos
            )
            self.get_logger().info(f'Subscribing to camera1: {camera1_topic} (QoS: {image_qos})')

            self.camera2_subscriber = self.create_subscription(
                Image,
                camera2_topic,
                self.camera2_callback,
                image_qos
            )
            self.get_logger().info(f'Subscribing to camera2: {camera2_topic} (QoS: {image_qos})')

            self.camera3_subscriber = self.create_subscription(
                Image,
                camera3_topic,
                self.camera3_callback,
                image_qos
            )
            self.get_logger().info(f'Subscribing to camera3: {camera3_topic} (QoS: {image_qos})')

            self.joint_state_subscriber = self.create_subscription(
                JointState,
                joint_state_topic,
                self.joint_state_subscriber_callback,
                joint_state_qos
            )
            self.get_logger().info(f'Subscribing to joint state topic: {joint_state_topic} (QoS: {joint_state_qos})')

        self.action_publisher = self.create_publisher(
            Float32MultiArray,
            action_topic,
            10
        )
        self.get_logger().info(f'Publishing to action topic: {action_topic}')

        self.action_chunk_publisher = self.create_publisher(
            Float32MultiArray,
            action_chunk_topic,
            10
        )
        self.get_logger().info(f'Publishing to action chunk topic: {action_chunk_topic}')

        # Optional: Subscribe to our own action chunk for demonstration/validation
        self.declare_parameter('enable_action_chunk_echo', False)
        enable_echo = self.get_parameter('enable_action_chunk_echo').get_parameter_value().bool_value
        
        if enable_echo:
            self.action_chunk_subscriber = self.create_subscription(
                Float32MultiArray,
                action_chunk_topic,
                self.action_chunk_callback,
                10
            )
            self.get_logger().info(f'Subscribing to action chunk topic for echo/validation')

        # Create a timer for inference
        timer_period = 1.0 / publisher_rate  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.inference_timer_callback)
        self.get_logger().info(f'Inference timer set to {publisher_rate} Hz ({timer_period*1000:.1f} ms period)')
        
        # Add a flag to prevent overlapping inference calls
        self.inference_in_progress = False

    def camera1_callback(self, msg):
        self.latest_camera1_msg = msg

    def camera2_callback(self, msg):
        self.latest_camera2_msg = msg

    def camera3_callback(self, msg):
        self.latest_camera3_msg = msg

    def joint_state_subscriber_callback(self, msg):
        # position, velocity, effort are all lists
        self.latest_joint_state_msg = msg.position # Store only the positions

    def action_chunk_callback(self, msg):
        """
        Callback to demonstrate how to read the structured action chunk data.
        This shows how subscribers can interpret the MultiArrayDimension layout.
        """
        try:
            # Extract dimensions from layout
            if len(msg.layout.dim) < 2:
                self.get_logger().warn('Action chunk message has invalid dimensions')
                return
            
            num_actions = msg.layout.dim[0].size
            action_dim = msg.layout.dim[1].size
            
            # Reshape the flattened data back to 2D array
            data = np.array(msg.data)
            
            if len(data) != num_actions * action_dim:
                self.get_logger().error(
                    f'Data size mismatch: expected {num_actions * action_dim}, got {len(data)}'
                )
                return
            
            actions = data.reshape(num_actions, action_dim)
            
            # Log structured information
            self.get_logger().info(
                f'Action Chunk: {num_actions} actions × {action_dim} dims | '
                f'First: {actions[0]} | '
                f'Mid: {actions[num_actions//2]} | '
                f'Last: {actions[-1]}',
                throttle_duration_sec=2.0
            )
            
            # Demonstrate accessing specific elements
            # Element at action[i][j] can be accessed as: msg.data[i * stride + j]
            # Or just use the reshaped numpy array: actions[i][j]
            
        except Exception as e:
            self.get_logger().error(f'Error parsing action chunk: {e}')

    def imgmsg_to_numpy(self, img_msg):
        """Convert ROS Image message to numpy array without cv_bridge."""
        # Get image dimensions
        height = img_msg.height
        width = img_msg.width
        
        # Convert bytes to numpy array
        dtype = np.uint8
        img_array = np.frombuffer(img_msg.data, dtype=dtype)
        
        # Reshape based on encoding
        if img_msg.encoding == 'rgb8':
            img_array = img_array.reshape((height, width, 3))
        elif img_msg.encoding == 'bgr8':
            img_array = img_array.reshape((height, width, 3))
            # Convert BGR to RGB
            img_array = img_array[:, :, ::-1]
        elif img_msg.encoding == 'mono8' or img_msg.encoding == 'gray8':
            img_array = img_array.reshape((height, width))
            # Convert grayscale to RGB by stacking
            img_array = np.stack([img_array, img_array, img_array], axis=2)
        else:
            raise ValueError(f"Unsupported encoding: {img_msg.encoding}")
        
        return img_array

    def inference_timer_callback(self):
        # Skip if previous inference is still running
        if self.inference_in_progress:
            self.get_logger().warn('Skipping inference - previous call still in progress', throttle_duration_sec=1.0)
            return
        
        self.inference_in_progress = True
        inference_start_time = time.time()
        
        # Determine the correct observation keys from model config
        image_keys = [k.replace("observation.images.", "") for k in self.input_features.keys() if "image" in k]
        state_keys = [k for k in self.input_features.keys() if "state" in k]
        
        if not image_keys or not state_keys:
            self.get_logger().error(f'Could not find image or state keys in model config.')
            return
        
        state_key = state_keys[0].replace("observation.", "")

        if self.use_dummy_input:
            # Get expected state dimension from model config
            state_feature = self.input_features[f"observation.{state_key}"]
            state_dim = state_feature.shape[0]
            
            # Generate dummy robot state as individual named values
            dummy_robot_state = {f"joint_{i}": 0.0 for i in range(state_dim)}
            
            # Create raw observation dict with ALL required cameras
            raw_obs = {**dummy_robot_state}
            
            # Add dummy images for each camera the model expects
            for img_key in image_keys:
                image_feature = self.input_features[f"observation.images.{img_key}"]
                c, h, w = image_feature.shape
                dummy_cv_image = np.zeros((h, w, c), dtype=np.uint8)
                raw_obs[img_key] = dummy_cv_image
        else:
            if self.latest_camera1_msg is None:
                self.get_logger().warn('No camera1 message received yet. Skipping inference.')
                return
            if self.latest_camera2_msg is None:
                self.get_logger().warn('No camera2 message received yet. Skipping inference.')
                return
            if self.latest_camera3_msg is None:
                self.get_logger().warn('No camera3 message received yet. Skipping inference.')
                return
            if self.latest_joint_state_msg is None:
                self.get_logger().warn('No joint state message received yet. Skipping inference.')
                return

            try:
                cv_image1 = self.imgmsg_to_numpy(self.latest_camera1_msg)
                cv_image2 = self.imgmsg_to_numpy(self.latest_camera2_msg)
                cv_image3 = self.imgmsg_to_numpy(self.latest_camera3_msg)
            except Exception as e:
                self.get_logger().error(f'Error converting images: {e}')
                return

            try:
                state_feature = self.input_features[f"observation.{state_key}"]
                state_dim = state_feature.shape[0]
                
                robot_state_data = np.array(self.latest_joint_state_msg, dtype=np.float32) # remove .data
                if robot_state_data.shape[0] != state_dim:
                    self.get_logger().error(
                        f"Joint state dimension mismatch. Expected {state_dim}, got {robot_state_data.shape[0]}"
                    )
                    return
                
                robot_state_dict = {f"joint_{i}": float(robot_state_data[i]) for i in range(state_dim)}
            except Exception as e:
                self.get_logger().error(f'Error processing joint state: {e}')
                return

            raw_obs = {**robot_state_dict}
            
            # Add all 3 real camera images
            raw_obs["camera1"] = cv_image1
            raw_obs["camera2"] = cv_image2
            raw_obs["camera3"] = cv_image3

        try:
            # Build the inference frame
            # self.get_logger().info('Starting frame build...', throttle_duration_sec=5.0)
            frame_build_start = time.time()
            obs_frame = build_inference_frame(
                observation=raw_obs,
                ds_features=self.dataset_features,
                device=self.device,
                task=self.task,
                robot_type=self.robot_type
            )
            frame_build_time = (time.time() - frame_build_start) * 1000
            # self.get_logger().info(f'Frame built in {frame_build_time:.1f}ms', throttle_duration_sec=5.0)
            
            # Restructure obs_frame
            observation_dict = {}
            for key, value in obs_frame.items():
                if key.startswith("observation."):
                    nested_key = key.replace("observation.", "", 1)
                    observation_dict[nested_key] = value
            
            # Create transition
            transition = {
                "observation": observation_dict,
                "action": None,
                "complementary_data": {
                    "task": obs_frame.get("task", ""),
                    "robot_type": obs_frame.get("robot_type", "")
                },
                "reward": None,
                "done": None,
                "truncated": None,
                "info": None,
            }
            
            # Preprocess
            # self.get_logger().info('Starting preprocessing...', throttle_duration_sec=5.0)
            preprocess_start = time.time()
            processed_transition = self.pre_processor._forward(transition)
            preprocess_time = (time.time() - preprocess_start) * 1000
            # self.get_logger().info(f'Preprocessed in {preprocess_time:.1f}ms', throttle_duration_sec=5.0)
            
            # Flatten transition
            flattened_transition = {}
            
            if "observation" in processed_transition and isinstance(processed_transition["observation"], dict):
                for key, value in processed_transition["observation"].items():
                    if key.startswith("observation."):
                        flat_key = key
                    else:
                        flat_key = f"observation.{key}"
                    flattened_transition[flat_key] = value
            
            for key in ["action", "complementary_data", "reward", "done", "truncated", "info"]:
                if key in processed_transition:
                    flattened_transition[key] = processed_transition[key]
            
            # Model inference
            model_inference_start = time.time()

            with torch.no_grad():
                # predic 50 actions ahead 50 [actions]*0.033 [sec/action] = 1.65 [sec] ahead assuming 33ms per step
                action_tensor = self.model.predict_action_chunk(flattened_transition)

            model_inference_time = (time.time() - model_inference_start) * 1000

            # Post-process action
            postprocess_start = time.time()
            _, chunk_size, _ = action_tensor.shape
            processed_actions = []
            robot_actions = []
            
            for i in range(chunk_size):
                # Extract action at timestep i: (B, action_dim)
                single_action = action_tensor[:, i, :]
                processed_action = self.post_processor(single_action)
                processed_actions.append(processed_action)
                
                # Convert each action to robot action format
                robot_action = make_robot_action(processed_action, self.dataset_features)
                robot_actions.append(robot_action)

            # Stack back to (B, chunk_size, action_dim), then remove batch dim
            action_tensor = torch.stack(processed_actions, dim=1).squeeze(0) 

            # Take the first robot action for immediate execution
            first_robot_action = robot_actions[0]
            postprocess_time = (time.time() - postprocess_start) * 1000

            # Publish single action (first in chunk)
            action_msg = Float32MultiArray()
            
            if isinstance(first_robot_action, dict):
                action_keys = sorted([k for k in first_robot_action.keys() if k.startswith('action_')])
                if action_keys:
                    action_msg.data = [float(first_robot_action[k]) for k in action_keys]
                else:
                    self.get_logger().error(f'No action keys found in robot_action')
                    return
            else:
                if hasattr(first_robot_action, 'squeeze'):
                    action_msg.data = first_robot_action.squeeze().tolist()
                elif hasattr(first_robot_action, 'tolist'):
                    action_msg.data = first_robot_action.tolist()
                else:
                    action_msg.data = list(first_robot_action)
                
            self.action_publisher.publish(action_msg)
            
            # Publish all 50 robot actions in the chunk
            action_chunk_msg = Float32MultiArray()
            all_actions = []
            
            # Collect all actions
            for robot_action in robot_actions:
                if isinstance(robot_action, dict):
                    action_keys = sorted([k for k in robot_action.keys() if k.startswith('action_')])
                    action_values = [float(robot_action[k]) for k in action_keys]
                    all_actions.extend(action_values)
                else:
                    if hasattr(robot_action, 'squeeze'):
                        all_actions.extend(robot_action.squeeze().tolist())
                    elif hasattr(robot_action, 'tolist'):
                        all_actions.extend(robot_action.tolist())
                    else:
                        all_actions.extend(list(robot_action))
            
            # Set up proper dimensions for 2D array [50, action_dim]
            action_dim = len(action_msg.data)  # Get dimension from first action
            
            dim0 = MultiArrayDimension()
            dim0.label = "actions"
            dim0.size = chunk_size  # Should be 50
            dim0.stride = chunk_size * action_dim
            
            dim1 = MultiArrayDimension()
            dim1.label = "action_dimensions"
            dim1.size = action_dim
            dim1.stride = action_dim
            
            action_chunk_msg.layout.dim = [dim0, dim1]
            action_chunk_msg.layout.data_offset = 0
            action_chunk_msg.data = all_actions
            
            self.action_chunk_publisher.publish(action_chunk_msg)

            
            total_inference_time = (time.time() - inference_start_time) * 1000
            
            self.get_logger().info(
                # f'Action: {action_msg.data} | '
                f'Chunk size: {len(action_chunk_msg.data)} | '
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
        finally:
            # Always reset the flag
            self.inference_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = SmolVLAInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()