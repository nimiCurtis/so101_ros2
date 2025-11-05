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
# lerobot_src_path = '/home/anton/lerobot/src'
# if lerobot_src_path not in sys.path:
#     sys.path.insert(0, lerobot_src_path)

# Ensure the conda site-packages directory is in the system path
from so101_ros2_bridge.utils import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import build_inference_frame, make_robot_action

class SmolVLAInferenceNode(Node):
    def __init__(self):
        super().__init__('smolvla_inference_node')
        self.get_logger().info('SmolVLA Inference Node starting...')

        # Declare parameters
        self.declare_parameter('model_id', 'lerobot/smolvla_base')  # Using base model by default
        self.declare_parameter('camera1_topic', '/follower/cam_front/image_raw') 
        self.declare_parameter('camera2_topic', '/follower/cam_top1/image_raw')
        self.declare_parameter('camera3_topic', '/follower/cam_top2/image_raw')
        self.declare_parameter('joint_state_topic', '/isaac/isaac_joint_states') # /isaac/isaac_joint_states 
        self.declare_parameter('action_topic', '/isaac/isaac_joint_command_test') # /smolvla_inference/action
        self.declare_parameter('action_chunk_topic', '/smolvla_inference/action_chunk')
        self.declare_parameter('task', 'Pick up the white block and insertit on the green peg')
        self.declare_parameter('robot_type', 'so101')
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

        # Define joint names based on your robot configuration
        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift', 
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'gripper'
        ]

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

        # MODIFIED: Changed publisher type from Float32MultiArray to JointState
        self.action_publisher = self.create_publisher(
            JointState,
            action_topic,
            10
        )
        self.get_logger().info(f'Publishing JointState to action topic: {action_topic}')

        self.action_chunk_publisher = self.create_publisher(
            Float32MultiArray,
            action_chunk_topic,
            10
        )
        self.get_logger().info(f'Publishing to action chunk topic: {action_chunk_topic}')

        # Optional: Subscribe to our own action chunk for demonstration/validation
        self.declare_parameter('enable_action_chunk_echo', False)
        enable_echo = self.get_parameter('enable_action_chunk_echo').get_parameter_value().bool_value
        
        if enable_echo: # TODO remove if not in use
            self.action_chunk_echo_subscriber = self.create_subscription(
                Float32MultiArray,
                action_chunk_topic,
                self.action_chunk_echo_callback,
                10
            )
            self.get_logger().info(f'Echoing action chunks from: {action_chunk_topic}')

        self.timer = self.create_timer(1.0/publisher_rate, self.inference_callback)
        self.get_logger().info(f'Publishing rate: {publisher_rate} Hz')
        
        self.inference_in_progress = False

    def camera1_callback(self, msg):
        self.latest_camera1_msg = msg
        # self.get_logger().debug('Camera1 image received', throttle_duration_sec=10.0)

    def camera2_callback(self, msg):
        self.latest_camera2_msg = msg
        # self.get_logger().debug('Camera2 image received', throttle_duration_sec=10.0)

    def camera3_callback(self, msg):
        self.latest_camera3_msg = msg
        # self.get_logger().debug('Camera3 image received', throttle_duration_sec=10.0)

    def joint_state_subscriber_callback(self, msg):
        self.latest_joint_state_msg = msg
        # self.get_logger().debug('Joint state received', throttle_duration_sec=10.0)

    def action_chunk_echo_callback(self, msg): #TODO remove if not in use
        # Extract dimensions from the layout (assuming 2D array [actions, action_dimensions])
        if len(msg.layout.dim) >= 2:
            num_actions = msg.layout.dim[0].size
            action_dim = msg.layout.dim[1].size
            
            self.get_logger().info(
                f'Action chunk received: {num_actions} actions x {action_dim} dimensions',
                throttle_duration_sec=1.0
            )
            
            # Optionally print first action
            if len(msg.data) >= action_dim:
                first_action = msg.data[:action_dim]
                self.get_logger().info(
                    f'First action: {first_action}',
                    throttle_duration_sec=1.0
                )

    def get_joint_states_from_dummy_input(self):
        # Return a numpy array with 6 joint values
        return np.array([0.0, 0.0139, 0.0044, 0.0005, 0.0, 0.0])

    def get_images_from_dummy_input(self):
        # Use the actual dimensions expected by the model for dummy images
        import cv2
        
        # Generate dummy images with expected dimensions
        # SmolVLA typically expects 224x224 RGB images
        dummy_image1 = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
        dummy_image2 = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
        dummy_image3 = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
        
        return dummy_image1, dummy_image2, dummy_image3

    def image_msg_to_numpy(self, img_msg):
        """Convert ROS Image message to numpy array."""
        import cv2
        import numpy as np
        
        # Get image dimensions
        height = img_msg.height
        width = img_msg.width
        
        # Convert based on encoding
        if img_msg.encoding == 'bgr8':
            # BGR8: 3 channels, 8 bits per channel
            img_array = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(height, width, 3)
            # Convert BGR to RGB
            img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)
        elif img_msg.encoding == 'rgb8':
            # RGB8: 3 channels, 8 bits per channel
            img_array = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(height, width, 3)
        elif img_msg.encoding == 'mono8':
            # Mono8: 1 channel, 8 bits
            img_array = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(height, width)
            # Convert grayscale to RGB
            img_array = cv2.cvtColor(img_array, cv2.COLOR_GRAY2RGB)
        else:
            self.get_logger().warn(f'Unsupported image encoding: {img_msg.encoding}')
            return None
            
        return img_array

    def inference_callback(self):
        """Main inference callback called by timer."""
        # Prevent concurrent inferences
        if self.inference_in_progress:
            self.get_logger().debug('Skipping inference - previous inference still in progress')
            return
        
        self.inference_in_progress = True
        
        try:
            # self.get_logger().info('Starting inference...', throttle_duration_sec=5.0)
            inference_start_time = time.time()
            
            # Get observation data
            if self.use_dummy_input:
                # Use dummy data for testing
                camera1, camera2, camera3 = self.get_images_from_dummy_input()
                joint_states = self.get_joint_states_from_dummy_input()
            else:
                # Check if we have all required data
                if (self.latest_camera1_msg is None or 
                    self.latest_camera2_msg is None or 
                    self.latest_camera3_msg is None or 
                    self.latest_joint_state_msg is None):
                    self.get_logger().info(
                        f'Waiting for data - Camera1: {self.latest_camera1_msg is not None}, '
                        f'Camera2: {self.latest_camera2_msg is not None}, '
                        f'Camera3: {self.latest_camera3_msg is not None}, '
                        f'Joint: {self.latest_joint_state_msg is not None}',
                        throttle_duration_sec=5.0
                    )
                    return
                
                # Convert images to numpy arrays
                camera1 = self.image_msg_to_numpy(self.latest_camera1_msg)
                camera2 = self.image_msg_to_numpy(self.latest_camera2_msg)
                camera3 = self.image_msg_to_numpy(self.latest_camera3_msg)
                
                # Get joint positions
                joint_states = np.array(self.latest_joint_state_msg.position[:6])
            
            # Build raw observation
            # The model expects joint states as individual keys like 'joint_0', 'joint_1', etc.
            raw_obs = {}
            
            # Add joint states with individual keys
            for i, joint_value in enumerate(joint_states):
                raw_obs[f'joint_{i}'] = joint_value
            
            # Add camera images
            raw_obs['camera1'] = camera1
            raw_obs['camera2'] = camera2
            raw_obs['camera3'] = camera3
            
            # Build inference frame
            # self.get_logger().info('Building inference frame...', throttle_duration_sec=5.0)
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
            # obs = self.pre_processor(obs_frame)

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

            
            
            # action = self.model.select_action(obs)
            # predic 50 actions ahead 50 [actions]*0.033 [sec/action] = 1.65 [sec] ahead assuming 33ms per step
            action_tensor = self.model.predict_action_chunk(flattened_transition) # TODO check if can use select action with observation_dict directly

            model_inference_time = (time.time() - model_inference_start) * 1000

            # Post-process action
            postprocess_start = time.time()
            _, chunk_size, _ = action_tensor.shape
            processed_actions = []
            robot_actions = []
            
            # in select mode
            # processed_action = self.post_processor(action)
            # robot_action = make_robot_action(action, self.dataset_features)
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

            # MODIFIED: Create and publish JointState message instead of Float32MultiArray
            action_msg = JointState()
            
            # Set header with timestamp
            action_msg.header.stamp = self.get_clock().now().to_msg() # TODO check if can get simtime
            action_msg.header.frame_id = ''
            
            # Set joint names
            action_msg.name = self.joint_names
            
            # Extract prediction values for position field
            if isinstance(first_robot_action, dict):
                action_keys = sorted([k for k in first_robot_action.keys() if k.startswith('action_')])
                if action_keys:
                    prediction_values = [float(first_robot_action[k]) for k in action_keys]
                else:
                    self.get_logger().error(f'No action keys found in robot_action')
                    return
            else:
                if hasattr(first_robot_action, 'squeeze'):
                    prediction_values = first_robot_action.squeeze().tolist()
                elif hasattr(first_robot_action, 'tolist'):
                    prediction_values = first_robot_action.tolist()
                else:
                    prediction_values = list(first_robot_action)
            
            # Ensure we have the right number of values
            if len(prediction_values) != len(self.joint_names):
                self.get_logger().warn(f'Prediction has {len(prediction_values)} values but expected {len(self.joint_names)}. Padding/truncating.')
                # Pad with zeros or truncate as needed
                if len(prediction_values) < len(self.joint_names):
                    prediction_values.extend([0.0] * (len(self.joint_names) - len(prediction_values)))
                else:
                    prediction_values = prediction_values[:len(self.joint_names)]
            
            # Set position to the prediction values
            action_msg.position = prediction_values
            
            # Set velocity and effort to zeros
            action_msg.velocity = [0.0] * len(self.joint_names)
            action_msg.effort = [0.0] * len(self.joint_names)
            
            # Publish the JointState message
            self.action_publisher.publish(action_msg)
            
            # Publish all 50 robot actions in the chunk (keeping this as Float32MultiArray)
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
            action_dim = len(prediction_values)  # Get dimension from first action
            
            dim0 = MultiArrayDimension()
            dim0.label = "actions"
            dim0.size = chunk_size  # Should be 50
            dim0.stride = chunk_size * action_dim
            
            dim1 = MultiArrayDimension()
            dim1.label = "action_dimensions"
            dim1.size = action_dim
            dim1.stride = action_dim
            
            action_chunk_msg.layout.dim = [dim0, dim1]
            
            # Add timestamp as data_offset (converting to nanoseconds as integer)
            # This is a creative use of data_offset field to store timestamp
            current_time = self.get_clock().now() # todo take from sim
            # Store seconds and nanoseconds in the data array at the beginning
            # First two values will be timestamp (seconds, nanoseconds), rest will be actions
            timestamp_and_actions = [
                float(current_time.seconds_nanoseconds()[0]),  # seconds
                float(current_time.seconds_nanoseconds()[1])   # nanoseconds
            ]
            timestamp_and_actions.extend(all_actions)
            
            action_chunk_msg.layout.data_offset = 2  # Indicate that actual data starts at index 2
            action_chunk_msg.data = timestamp_and_actions
            
            self.action_chunk_publisher.publish(action_chunk_msg)

            
            total_inference_time = (time.time() - inference_start_time) * 1000
            
            self.get_logger().info(
                # f'Published JointState with positions: {prediction_values[:3]}... | '  # Show first 3 values
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