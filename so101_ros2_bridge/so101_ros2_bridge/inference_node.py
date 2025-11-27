# Ensure the conda site-packages directory is in the system path
from so101_ros2_bridge.utils import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension
import numpy as np
import torch
import sys
import os
import time

# Disable torch.compile completely (needed for PI05)
import torch._dynamo
torch._dynamo.config.suppress_errors = True
os.environ['TORCH_COMPILE_DISABLE'] = '1'

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.pi05.modeling_pi05 import PI05Policy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import build_inference_frame, make_robot_action

class UnifiedInferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')
        self.get_logger().info('Unified Inference Node starting...')

        # Declare parameters
        self.declare_parameter('model_id', 'lerobot/smolvla_base')
        self.declare_parameter('model_type', 'smolvla')  # 'smolvla' or 'pi05'
        self.declare_parameter('camera1_topic', '/follower/cam_front/image_raw')
        self.declare_parameter('camera2_topic', '/follower/cam_top1/image_raw')
        self.declare_parameter('joint_state_topic', '/isaac/isaac_joint_states')
        self.declare_parameter('action_topic', '/isaac/isaac_joint_command_test')
        self.declare_parameter('action_chunk_topic', '/smolvla_inference/action_chunk')
        self.declare_parameter('task', 'Pick up the white block and insert it on the green peg')
        self.declare_parameter('robot_type', 'so101')
        self.declare_parameter('use_dummy_input', False)
        self.declare_parameter('publisher_rate', 1)  # Hz for action publishing
        self.declare_parameter('image_subscription_qos', 2)
        self.declare_parameter('joint_state_subscription_qos', 2)

        # Get parameters
        model_id = self.get_parameter('model_id').get_parameter_value().string_value
        self.model_type = self.get_parameter('model_type').get_parameter_value().string_value
        camera1_topic = self.get_parameter('camera1_topic').get_parameter_value().string_value
        camera2_topic = self.get_parameter('camera2_topic').get_parameter_value().string_value
        joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        action_topic = self.get_parameter('action_topic').get_parameter_value().string_value
        action_chunk_topic = self.get_parameter('action_chunk_topic').get_parameter_value().string_value
        self.task = self.get_parameter('task').get_parameter_value().string_value
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.use_dummy_input = self.get_parameter('use_dummy_input').get_parameter_value().bool_value
        publisher_rate = self.get_parameter('publisher_rate').get_parameter_value().integer_value
        image_qos = self.get_parameter('image_subscription_qos').get_parameter_value().integer_value
        joint_state_qos = self.get_parameter('joint_state_subscription_qos').get_parameter_value().integer_value

        # Validate model type (if not auto)
        if self.model_type not in ['smolvla', 'pi05', 'auto']:
            self.get_logger().error(f'Invalid model_type: {self.model_type}. Must be "smolvla", "pi05", or "auto"')
            raise ValueError(f'Invalid model_type: {self.model_type}')

        # Auto-detect model type if needed
        if self.model_type == 'auto':
            self.model_type = self._detect_model_type(model_id)
            self.get_logger().info(f'Auto-detected model type: {self.model_type}')
        else:
            self.get_logger().info(f'Using specified model type: {self.model_type}')

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

        # Load model based on type
        self.get_logger().info(f'Loading {self.model_type} model: {model_id}')
        load_start_time = time.time()
        
        if self.model_type == 'smolvla':
            self._load_smolvla_model(model_id)
        else:  # pi05
            self._load_pi05_model(model_id)
            
        load_end_time = time.time()
        
        # Log model information
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

            self.joint_state_subscriber = self.create_subscription(
                JointState,
                joint_state_topic,
                self.joint_state_subscriber_callback,
                joint_state_qos
            )
            self.get_logger().info(f'Subscribing to joint state topic: {joint_state_topic} (QoS: {joint_state_qos})')

        # Publishers
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
        self.get_logger().info(f'Publishing action chunks to: {action_chunk_topic}')

        # Timer for periodic inference
        self.timer = self.create_timer(1.0 / publisher_rate, self.timer_callback)
        self.get_logger().info(f'Timer set to {publisher_rate} Hz')
        self.inference_in_progress = False

    def _detect_model_type(self, model_id):
        """
        Auto-detect model type by checking the config file.
        Returns 'smolvla' or 'pi05' based on the model architecture.
        """
        import json
        import os
        
        try:
            # Try to find config.json in the model directory
            config_path = None
            if os.path.isdir(model_id):
                config_path = os.path.join(model_id, 'config.json')
            
            if config_path and os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = json.load(f)
                
                # Check the architecture or model_type field
                arch_type = config.get('architecture', None)
                model_type = config.get('model_type', None)
                
                self.get_logger().info(f'Config architecture: {arch_type}, model_type: {model_type}')
                
                # Check for PI0.5 indicators
                if model_type and 'pi0' in model_type.lower():
                    return 'pi05'
                if arch_type and 'pi0' in arch_type.lower():
                    return 'pi05'
                
                # Check for PaliGemma variant (PI0.5 specific)
                if 'paligemma_variant' in config:
                    return 'pi05'
                
                # Default to smolvla
                return 'smolvla'
            else:
                self.get_logger().warn(f'Config file not found at {config_path}, defaulting to smolvla')
                return 'smolvla'
                
        except Exception as e:
            self.get_logger().warn(f'Error detecting model type: {e}, defaulting to smolvla')
            return 'smolvla'

    def _load_smolvla_model(self, model_id):
        """Load SmolVLA model"""
        self.model = SmolVLAPolicy.from_pretrained(model_id).to(self.device)
        
        self.pre_processor, self.post_processor = make_pre_post_processors(
            self.model.config,
            model_id,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
        )

    def _load_pi05_model(self, model_id):
        """Load PI05 model with bfloat16 and dynamo disabled"""
        self.model = PI05Policy.from_pretrained(model_id).to(self.device)
        
        # Convert to bfloat16
        self.model = self.model.to(dtype=torch.bfloat16)

        # Convert buffers explicitly
        for name, buffer in self.model.named_buffers():
            if buffer.dtype == torch.float32:
                self.get_logger().warn(f'Converting buffer {name} to bfloat16')
                buffer.data = buffer.data.to(torch.bfloat16)

        # Move to device AFTER dtype conversion
        self.model = self.model.to(self.device)

        # Ensure model is in eval mode
        self.model.eval()
        self.get_logger().info('Model set to eval mode')

        # Disable gradient checkpointing for inference
        if hasattr(self.model.config, 'gradient_checkpointing'):
            self.model.config.gradient_checkpointing = False

        # Disable dynamo on predict_action_chunk
        self.model.predict_action_chunk = torch._dynamo.disable(
            self.model.predict_action_chunk
        )
        self.get_logger().info('✅ Disabled dynamo on predict_action_chunk')

        # Verify model dtype conversion
        self.get_logger().info('=== Model Dtype Verification ===')
        param_dtypes = {}
        buffer_dtypes = {}

        for name, param in self.model.named_parameters():
            dtype_str = str(param.dtype)
            param_dtypes[dtype_str] = param_dtypes.get(dtype_str, 0) + 1

        for name, buffer in self.model.named_buffers():
            dtype_str = str(buffer.dtype)
            buffer_dtypes[dtype_str] = buffer_dtypes.get(dtype_str, 0) + 1

        self.get_logger().info(f'Parameter dtypes: {param_dtypes}')
        self.get_logger().info(f'Buffer dtypes: {buffer_dtypes}')

        # Check for any float32
        float32_params = [n for n, p in self.model.named_parameters() if p.dtype == torch.float32]
        float32_buffers = [n for n, b in self.model.named_buffers() if b.dtype == torch.float32]

        if float32_params:
            self.get_logger().error(f'⚠️ Found {len(float32_params)} float32 parameters!')
            for name in float32_params[:5]:
                self.get_logger().error(f'  - {name}')
                
        if float32_buffers:
            self.get_logger().error(f'⚠️ Found {len(float32_buffers)} float32 buffers!')
            for name in float32_buffers[:5]:
                self.get_logger().error(f'  - {name}')

        if not float32_params and not float32_buffers:
            self.get_logger().info('✅ All model components are bfloat16!')

        # Create preprocessors with bfloat16 for PI05
        self.pre_processor, self.post_processor = make_pre_post_processors(
            self.model.config,
            model_id,
            preprocessor_overrides={
                "device_processor": {
                    "device": str(self.device),
                    "float_dtype": "bfloat16"
                }
            },
        )

    def camera1_callback(self, msg):
        self.latest_camera1_msg = msg

    def camera2_callback(self, msg):
        self.latest_camera2_msg = msg

    def joint_state_subscriber_callback(self, msg):
        self.latest_joint_state_msg = msg

    def _convert_to_bfloat16(self, data):
        """
        Recursively convert all float32 tensors to bfloat16 for PI05 models.
        Handles nested dictionaries and lists.
        Also ensures tensors are on the correct device.
        """
        if isinstance(data, torch.Tensor):
            # Convert dtype if needed
            if data.dtype == torch.float32:
                data = data.to(dtype=torch.bfloat16)
            # Ensure on correct device
            if data.device != self.device:
                data = data.to(device=self.device)
            return data
        elif isinstance(data, dict):
            return {key: self._convert_to_bfloat16(value) for key, value in data.items()}
        elif isinstance(data, list):
            return [self._convert_to_bfloat16(item) for item in data]
        elif isinstance(data, tuple):
            return tuple(self._convert_to_bfloat16(item) for item in data)
        else:
            return data

    def image_msg_to_numpy(self, img_msg):
        """Convert ROS Image message to numpy array."""
        import cv2
        
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

    def timer_callback(self):
        if self.inference_in_progress:
            self.get_logger().debug('Inference already in progress, skipping', throttle_duration_sec=1.0)
            return

        self.inference_in_progress = True

        try:
            inference_start_time = time.time()

            # Get observation data
            if self.use_dummy_input:
                # Use dummy data for testing (224x224 RGB images expected)
                camera1 = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
                camera2 = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
                joint_states = np.array([0.0, 0.0139, 0.0044, 0.0005, 0.0, 0.0])
            else:
                # Check if we have all required data
                if (self.latest_camera1_msg is None or
                    self.latest_camera2_msg is None or
                    self.latest_joint_state_msg is None):
                    self.get_logger().info(
                        f'Waiting for data - Camera1: {self.latest_camera1_msg is not None}, '
                        f'Camera2: {self.latest_camera2_msg is not None}, '
                        f'Joint: {self.latest_joint_state_msg is not None}',
                        throttle_duration_sec=5.0
                    )
                    self.inference_in_progress = False
                    return

                # Convert images to numpy arrays
                camera1 = self.image_msg_to_numpy(self.latest_camera1_msg)
                camera2 = self.image_msg_to_numpy(self.latest_camera2_msg)
                
                if camera1 is None or camera2 is None:
                    self.get_logger().error('Failed to convert image messages')
                    self.inference_in_progress = False
                    return
                
                # Get joint positions (first 6 joints)
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
            
            # Build inference frame
            frame_build_start = time.time()
            obs_frame = build_inference_frame(
                observation=raw_obs,
                ds_features=self.dataset_features,
                device=self.device,
                task=self.task,
                robot_type=self.robot_type
            )
            frame_build_time = (time.time() - frame_build_start) * 1000

            # Pre-process observation
            preprocess_start = time.time()
            obs = self.pre_processor(obs_frame)
            
            # For PI05, ensure all tensors are bfloat16 (recursively handle nested structures)
            if self.model_type == 'pi05':
                self.get_logger().info('🔄 Converting observation tensors to bfloat16...', throttle_duration_sec=5.0)
                
                # Log dtypes before conversion for debugging
                def log_dtypes(data, prefix="", depth=0):
                    if depth > 3:  # Limit recursion depth
                        return
                    if isinstance(data, torch.Tensor):
                        self.get_logger().info(f'  {prefix}: shape={data.shape}, dtype={data.dtype}, device={data.device}', throttle_duration_sec=5.0)
                    elif isinstance(data, dict):
                        for key, value in data.items():
                            log_dtypes(value, f'{prefix}.{key}' if prefix else key, depth+1)
                    elif isinstance(data, (list, tuple)) and len(data) > 0:
                        log_dtypes(data[0], f'{prefix}[0]', depth+1)
                
                self.get_logger().info('📊 Before conversion:', throttle_duration_sec=5.0)
                log_dtypes(obs, "obs")
                
                obs = self._convert_to_bfloat16(obs)
                
                self.get_logger().info('📊 After conversion:', throttle_duration_sec=5.0)
                log_dtypes(obs, "obs")
                
                # Verify conversion succeeded
                def check_all_bfloat16(data, path=""):
                    if isinstance(data, torch.Tensor):
                        if data.dtype == torch.float32:
                            self.get_logger().error(f'❌ Still float32 at {path}: {data.shape}, {data.dtype}')
                            return False
                    elif isinstance(data, dict):
                        return all(check_all_bfloat16(v, f'{path}.{k}' if path else k) for k, v in data.items())
                    elif isinstance(data, (list, tuple)):
                        return all(check_all_bfloat16(item, f'{path}[{i}]') for i, item in enumerate(data))
                    return True
                
                all_converted = check_all_bfloat16(obs, "obs")
                if all_converted:
                    self.get_logger().info('✅ All tensors converted to bfloat16', throttle_duration_sec=5.0)
                else:
                    self.get_logger().error('❌ Some tensors still in float32!')
            
            preprocess_time = (time.time() - preprocess_start) * 1000

            # Model inference
            model_inference_start = time.time()

            try:
                # Explicitly use torch.no_grad() and handle dtypes
                with torch.no_grad():
                    if self.model_type == 'pi05':
                        # For PI05, use autocast and detailed dtype verification
                        with torch.amp.autocast(device_type='cuda', dtype=torch.bfloat16):
                            # Verify input dtypes match model
                            model_dtype = next(self.model.parameters()).dtype
                            self.get_logger().info(f'Model dtype: {model_dtype}', throttle_duration_sec=10.0)
                            
                            # Helper function to recursively process nested dicts
                            def process_obs_dtypes(data, prefix=""):
                                if isinstance(data, dict):
                                    for key, value in data.items():
                                        full_key = f"{prefix}.{key}" if prefix else key
                                        if isinstance(value, dict):
                                            process_obs_dtypes(value, full_key)
                                        elif isinstance(value, torch.Tensor):
                                            original_dtype = value.dtype
                                            self.get_logger().info(
                                                f'Input key: {full_key}, dtype: {original_dtype}, shape: {value.shape}', 
                                                throttle_duration_sec=10.0
                                            )
                                            
                                            # Tokens must be long (int64)
                                            if 'tokens' in key:
                                                if value.dtype != torch.long:
                                                    data[key] = value.to(dtype=torch.long)
                                            # Attention masks MUST be bool
                                            elif 'attention_mask' in key:
                                                if value.dtype != torch.bool:
                                                    data[key] = value.to(dtype=torch.bool)
                                            # Everything else that's float should match model dtype
                                            elif value.dtype in [torch.float32, torch.float64, torch.float16]:
                                                if value.dtype != model_dtype:
                                                    data[key] = value.to(dtype=model_dtype)
                                            # Integer types (other than tokens) stay as-is
                                            # Bool types (other than attention_mask) stay as-is
                                            
                                            if original_dtype != data[key].dtype:
                                                self.get_logger().info(
                                                    f'Converted {full_key} from {original_dtype} to {data[key].dtype}',
                                                    throttle_duration_sec=10.0
                                                )
                            
                            # Process all observation tensors
                            process_obs_dtypes(obs)
                            
                            # Call model inference
                            self.get_logger().info('Calling predict_action_chunk...', throttle_duration_sec=5.0)
                            action_tensor = self.model.predict_action_chunk(obs)
                            self.get_logger().info(f'✅ Got action tensor: {action_tensor.shape}', throttle_duration_sec=2.0)
                    else:
                        # For SmolVLA, standard inference
                        action_tensor = self.model.predict_action_chunk(obs)
                
                model_inference_time = (time.time() - model_inference_start) * 1000
                if self.model_type == 'pi05':
                    self.get_logger().info(f'⏱️ Model inference time {model_inference_time:.1f}ms')
            except Exception as e:
                model_inference_time = (time.time() - model_inference_start) * 1000
                self.get_logger().error(f'❌ Model inference FAILED after {model_inference_time:.1f}ms: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                return

            # Post-process action
            postprocess_start = time.time()
            _, chunk_size, _ = action_tensor.shape
            processed_actions = []
            robot_actions = []
            
            for i in range(chunk_size):
                single_action = action_tensor[:, i, :]
                processed_action = self.post_processor(single_action)
                processed_actions.append(processed_action)
                
                robot_action = make_robot_action(processed_action, self.dataset_features)
                robot_actions.append(robot_action)

            action_tensor = torch.stack(processed_actions, dim=1).squeeze(0)
            first_robot_action = robot_actions[0]
            postprocess_time = (time.time() - postprocess_start) * 1000

            # Create and publish JointState message
            action_msg = JointState()
            action_msg.header.stamp = self.get_clock().now().to_msg()
            action_msg.header.frame_id = ''
            action_msg.name = self.joint_names
            
            # Extract prediction values
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
            
            # Ensure correct number of values
            if len(prediction_values) != len(self.joint_names):
                self.get_logger().warn(f'Prediction has {len(prediction_values)} values but expected {len(self.joint_names)}. Padding/truncating.')
                if len(prediction_values) < len(self.joint_names):
                    prediction_values.extend([0.0] * (len(self.joint_names) - len(prediction_values)))
                else:
                    prediction_values = prediction_values[:len(self.joint_names)]
            
            action_msg.position = prediction_values
            action_msg.velocity = [0.0] * len(self.joint_names)
            action_msg.effort = [0.0] * len(self.joint_names)
            
            self.action_publisher.publish(action_msg)
            
            # Publish action chunk
            action_chunk_msg = Float32MultiArray()
            all_actions = []
            
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
            
            action_dim = len(prediction_values)
            
            dim0 = MultiArrayDimension()
            dim0.label = "actions"
            dim0.size = chunk_size
            dim0.stride = chunk_size * action_dim
            
            dim1 = MultiArrayDimension()
            dim1.label = "action_dimensions"
            dim1.size = action_dim
            dim1.stride = action_dim
            
            action_chunk_msg.layout.dim = [dim0, dim1]
            
            current_time = self.get_clock().now()
            timestamp_and_actions = [
                float(current_time.seconds_nanoseconds()[0]),
                float(current_time.seconds_nanoseconds()[1])
            ]
            timestamp_and_actions.extend(all_actions)
            
            action_chunk_msg.layout.data_offset = 2
            action_chunk_msg.data = timestamp_and_actions
            
            self.action_chunk_publisher.publish(action_chunk_msg)

            total_inference_time = (time.time() - inference_start_time) * 1000
            
            self.get_logger().info(
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
            self.inference_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()