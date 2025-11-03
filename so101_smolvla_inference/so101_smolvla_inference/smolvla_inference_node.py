import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float64MultiArray # Assuming joint states might come as Float64MultiArray
# from cv_bridge import CvBridge #  Uncomment if cv_bridge is available
import numpy as np
import torch
import sys
import os
import time # For dummy data generation

# Add the lerobot src directory to the Python path
# This assumes the lerobot repository is cloned at /home/anton/lerobot
lerobot_src_path = '/home/anton/lerobot/src'
if lerobot_src_path not in sys.path:
    sys.path.insert(0, lerobot_src_path)

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.common.policies.utils import make_pre_post_processors
from lerobot.policies.utils import build_inference_frame, make_robot_action
from lerobot.datasets.utils import hw_to_dataset_features

class SmolVLAInferenceNode(Node):
    def __init__(self):
        super().__init__('smolvla_inference_node')
        self.get_logger().info('SmolVLA Inference Node starting...')

        # Declare parameters
        self.declare_parameter('model_id', 'lerobot/smolvla_base')
        self.declare_parameter('image_topic', '/follower/cam_front/image_raw')
        self.declare_parameter('joint_state_topic', '/follower/robot_state_publisher') # New parameter
        self.declare_parameter('action_topic', '/smolvla_inference/action')
        self.declare_parameter('task', '')
        self.declare_parameter('robot_type', '')
        self.declare_parameter('use_dummy_input', True) # New parameter

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
        self.model = SmolVLAPolicy.from_pretrained(model_id).to(self.device)
        self.pre_processor, self.post_processor = make_pre_post_processors(
            self.model.config,
            model_id,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
        )
        self.get_logger().info('Model loaded successfully.')

        # self.bridge = CvBridge() Uncomment if cv_bridge is available

        # Dummy dataset features for now.
        action_features = {
            "is_continuous": {"action": True},
            "shape": {"action": (7,)},
            "low": {"action": [-1.0]*7},
            "high": {"action": [1.0]*7},
        }
        obs_features = {
            "is_continuous": {"robot_state": True},
            "shape": {"robot_state": (7,)},
            "low": {"robot_state": [-1.0]*7},
            "high": {"robot_state": [1.0]*7},
        }
        self.dataset_features = {**action_features, **obs_features}

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
                Float64MultiArray, # Assuming joint states are Float64MultiArray
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
        self.timer = self.create_timer(0.1, self.inference_timer_callback) # 10 Hz inference

    def image_subscriber_callback(self, msg):
        self.latest_image_msg = msg

    def joint_state_subscriber_callback(self, msg):
        self.latest_joint_state_msg = msg

    def inference_timer_callback(self):
        image_tensor = None
        robot_state_tensor = None

        if self.use_dummy_input:
            # Generate dummy image
            dummy_cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
            image_tensor = torch.from_numpy(dummy_cv_image).permute(2, 0, 1).unsqueeze(0).float() / 255.0
            image_tensor = image_tensor.to(self.device)

            # Generate dummy robot state
            robot_state_tensor = torch.zeros(1, 7, dtype=torch.float32).to(self.device)
        else:
            if self.latest_image_msg is None:
                self.get_logger().warn('No image message received yet. Skipping inference.')
                return
            if self.latest_joint_state_msg is None:
                self.get_logger().warn('No joint state message received yet. Skipping inference.')
                return

            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='rgb8')
                image_tensor = torch.from_numpy(cv_image).permute(2, 0, 1).unsqueeze(0).float() / 255.0
                image_tensor = image_tensor.to(self.device)
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')
                return

            try:
                # Assuming joint states are a flat array of floats
                robot_state_data = np.array(self.latest_joint_state_msg.data, dtype=np.float32)
                robot_state_tensor = torch.from_numpy(robot_state_data).unsqueeze(0).to(self.device)
                if robot_state_tensor.shape[1] != 7: # Ensure it matches expected dimension
                    self.get_logger().error(f"Joint state dimension mismatch. Expected 7, got {robot_state_tensor.shape[1]}")
                    return
            except Exception as e:
                self.get_logger().error(f'Error processing joint state: {e}')
                return

        obs = {
            "image": {"camera1": image_tensor}, # Assuming 'camera1' as the key for the image
            "robot_state": robot_state_tensor
        }

        obs_frame = build_inference_frame(
            observation=obs,
            ds_features=self.dataset_features,
            device=self.device,
            task=self.task,
            robot_type=self.robot_type
        )

        processed_obs = self.pre_processor(obs_frame)

        # Run inference
        with torch.no_grad():
            action = self.model.select_action(processed_obs)

        # Post-process action
        processed_action = self.post_processor(action)
        robot_action = make_robot_action(processed_action, self.dataset_features)

        # Publish action
        action_msg = Float32MultiArray()
        action_msg.data = robot_action['action'].squeeze().tolist()
        self.action_publisher.publish(action_msg)
        self.get_logger().info(f'Published action: {action_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SmolVLAInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
