import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header
import numpy as np
import time

class SmolVLATestPublisher(Node):
    def __init__(self):
        super().__init__('smolvla_test_publisher')
        self.get_logger().info('SmolVLA Test Publisher Node starting...')

        # Declare parameters
        self.declare_parameter('camera1_topic', '/follower/cam_front/image_raw')
        self.declare_parameter('camera2_topic', '/follower/cam_top1/image_raw')
        self.declare_parameter('camera3_topic', '/follower/cam_top2/image_raw')
        # Changed default topic to match Isaac Sim
        self.declare_parameter('joint_state_topic', '/isaac/isaac_joint_states') 
        self.declare_parameter('publish_rate', 30)  # Hz
        self.declare_parameter('image_width', 128)
        self.declare_parameter('image_height', 128)
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('noise_type', 'static')  # 'random', 'sine', 'static', 'colored'
        self.declare_parameter('image_noise_level', 50)  # 0-255 for random noise amplitude
        self.declare_parameter('joint_noise_level', 0.5)  # radians for random noise amplitude
        self.declare_parameter('different_cameras', True)  # Make each camera different

        # Get parameters
        camera1_topic = self.get_parameter('camera1_topic').get_parameter_value().string_value
        camera2_topic = self.get_parameter('camera2_topic').get_parameter_value().string_value
        camera3_topic = self.get_parameter('camera3_topic').get_parameter_value().string_value
        joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.num_joints = self.get_parameter('num_joints').get_parameter_value().integer_value
        self.noise_type = self.get_parameter('noise_type').get_parameter_value().string_value
        self.image_noise_level = self.get_parameter('image_noise_level').get_parameter_value().integer_value
        self.joint_noise_level = self.get_parameter('joint_noise_level').get_parameter_value().double_value
        self.different_cameras = self.get_parameter('different_cameras').get_parameter_value().bool_value

        # Publishers for 3 cameras
        self.camera1_publisher = self.create_publisher(Image, camera1_topic, 10)
        self.camera2_publisher = self.create_publisher(Image, camera2_topic, 10)
        self.camera3_publisher = self.create_publisher(Image, camera3_topic, 10)
        self.get_logger().info(f'Publishing camera1 to: {camera1_topic}')
        self.get_logger().info(f'Publishing camera2 to: {camera2_topic}')
        self.get_logger().info(f'Publishing camera3 to: {camera3_topic}')

        # --- MODIFIED PUBLISHER ---
        # Changed message type from Float64MultiArray to JointState
        self.joint_state_publisher = self.create_publisher(
            JointState,
            joint_state_topic,
            10
        )
        self.get_logger().info(f'Publishing joint states to: {joint_state_topic}')
        
        # --- ADDED JOINT NAMES ---
        # Added joint names based on your 'ros2 topic echo' output
        self.joint_names = [
            'shoulder_pan', 'shoulder_lift', 'elbow_flex', 
            'wrist_flex', 'wrist_roll', 'gripper'
        ]
        # Check if num_joints parameter matches the list
        if self.num_joints != len(self.joint_names):
            self.get_logger().warning(
                f'num_joints parameter ({self.num_joints}) does not match '
                f'hardcoded joint_names list ({len(self.joint_names)}). '
                f'Using {len(self.joint_names)}.'
            )
            self.num_joints = len(self.joint_names)


        # Timer for publishing
        timer_period = 1.0 / publish_rate # seconds (30 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f'Publishing at {publish_rate} Hz ({timer_period*1000:.1f} ms period)')

        # State variables
        self.frame_count = 0
        self.start_time = time.time()

        self.get_logger().info(f'Noise type: {self.noise_type}')
        self.get_logger().info(f'Image size: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'Number of joints: {self.num_joints}')
        self.get_logger().info(f'Different camera views: {self.different_cameras}')

    def generate_image(self, camera_id=0):
        """Generate a test image based on noise type and camera ID."""
        # Add camera offset if different_cameras is True
        camera_offset = camera_id * 0.5 if self.different_cameras else 0
        
        if self.noise_type == 'random':
            # Random noise image with optional camera-specific shift
            img = np.random.randint(
                0, self.image_noise_level,
                (self.image_height, self.image_width, 3),
                dtype=np.uint8
            )
            # Add camera-specific color tint
            if self.different_cameras:
                if camera_id == 0:  # Camera 1: reddish tint
                    img[:, :, 0] = np.clip(img[:, :, 0] + 30, 0, 255)
                elif camera_id == 1:  # Camera 2: greenish tint
                    img[:, :, 1] = np.clip(img[:, :, 1] + 30, 0, 255)
                elif camera_id == 2:  # Camera 3: blueish tint
                    img[:, :, 2] = np.clip(img[:, :, 2] + 30, 0, 255)
                    
        elif self.noise_type == 'sine':
            # Sine wave pattern that changes over time
            t = self.frame_count * 0.1 + camera_offset
            x = np.linspace(0, 4 * np.pi, self.image_width)
            y = np.linspace(0, 4 * np.pi, self.image_height)
            xx, yy = np.meshgrid(x, y)
            
            # Create RGB channels with different sine patterns per camera
            r = ((np.sin(xx + t + camera_id * np.pi / 3) + 1) * 127.5).astype(np.uint8)
            g = ((np.sin(yy + t + camera_id * np.pi / 3) + 1) * 127.5).astype(np.uint8)
            b = ((np.sin(xx + yy + t + camera_id * np.pi / 3) + 1) * 127.5).astype(np.uint8)
            
            img = np.stack([r, g, b], axis=2)
            
        elif self.noise_type == 'colored':
            # Each camera shows a different solid color that changes over time
            t = self.frame_count * 0.05
            if camera_id == 0:  # Camera 1: Red channel wave
                r = int((np.sin(t) + 1) * 127.5)
                img = np.full((self.image_height, self.image_width, 3), [r, 50, 50], dtype=np.uint8)
            elif camera_id == 1:  # Camera 2: Green channel wave
                g = int((np.sin(t) + 1) * 127.5)
                img = np.full((self.image_height, self.image_width, 3), [50, g, 50], dtype=np.uint8)
            elif camera_id == 2:  # Camera 3: Blue channel wave
                b = int((np.sin(t) + 1) * 127.5)
                img = np.full((self.image_height, self.image_width, 3), [50, 50, b], dtype=np.uint8)
            else:
                img = np.full((self.image_height, self.image_width, 3), 128, dtype=np.uint8)
                
        elif self.noise_type == 'static':
            # Static gray image with camera-specific brightness
            brightness = 128 + (camera_id * 30 if self.different_cameras else 0)
            img = np.full(
                (self.image_height, self.image_width, 3),
                brightness,
                dtype=np.uint8
            )
        else:
            # Default to black
            img = np.zeros(
                (self.image_height, self.image_width, 3),
                dtype=np.uint8
            )
        
        return img

    # --- MODIFIED JOINT STATE GENERATION ---
    def generate_joint_states(self):
        """Generate test joint states (pos, vel, eff) based on noise type."""
        if self.noise_type == 'random':
            # Random joint positions, velocities, and efforts
            positions = np.random.uniform(
                -self.joint_noise_level,
                self.joint_noise_level,
                self.num_joints
            )
            velocities = np.random.uniform(-1.0, 1.0, self.num_joints)
            efforts = np.random.uniform(-0.5, 0.5, self.num_joints)

        elif self.noise_type in ['sine', 'colored']:
            # Sine wave joint positions, velocities (cosine), and efforts
            t = self.frame_count * 0.05
            positions = np.array([
                np.sin(t + i * np.pi / 3) * self.joint_noise_level
                for i in range(self.num_joints)
            ])
            velocities = np.array([
                np.cos(t + i * np.pi / 3) * 0.5 # Use cosine for variety
                for i in range(self.num_joints)
            ])
            efforts = np.array([
                np.sin(t * 2 + i * np.pi / 3) * 0.1 # Different freq/amplitude
                for i in range(self.num_joints)
            ])
        
        elif self.noise_type == 'static':
            # Static zero positions
            positions = np.zeros(self.num_joints)
            velocities = np.zeros(self.num_joints)
            efforts = np.zeros(self.num_joints)

        else:
            # Default to zeros
            positions = np.zeros(self.num_joints)
            velocities = np.zeros(self.num_joints)
            efforts = np.zeros(self.num_joints)
        
        return positions, velocities, efforts

    def create_image_msg(self, img, camera_id):
        """Create a ROS Image message from numpy array."""
        image_msg = Image()
        image_msg.header = Header()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = f'camera{camera_id+1}_frame'
        image_msg.height = self.image_height
        image_msg.width = self.image_width
        image_msg.encoding = 'rgb8'
        image_msg.is_bigendian = 0
        image_msg.step = self.image_width * 3
        image_msg.data = img.tobytes()
        return image_msg

    def timer_callback(self):
        """Publish test data at regular intervals."""
        current_stamp = self.get_clock().now().to_msg()

        # Generate and publish images for all 3 cameras
        img1 = self.generate_image(camera_id=0)
        img2 = self.generate_image(camera_id=1)
        img3 = self.generate_image(camera_id=2)
        
        # Create and publish image messages
        img_msg1 = self.create_image_msg(img1, 0)
        img_msg2 = self.create_image_msg(img2, 1)
        img_msg3 = self.create_image_msg(img3, 2)
        
        # Set all stamps to be the same for this cycle
        img_msg1.header.stamp = current_stamp
        img_msg2.header.stamp = current_stamp
        img_msg3.header.stamp = current_stamp

        self.camera1_publisher.publish(img_msg1)
        self.camera2_publisher.publish(img_msg2)
        self.camera3_publisher.publish(img_msg3)


        # --- MODIFIED JOINT STATE MESSAGE CREATION ---
        
        # Generate joint state data
        positions, velocities, efforts = self.generate_joint_states()
        
        # Create the JointState message
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = current_stamp # Use same stamp as images
        joint_msg.header.frame_id = '' # As per your example
        
        joint_msg.name = self.joint_names
        joint_msg.position = positions.tolist()
        joint_msg.velocity = velocities.tolist()
        joint_msg.effort = efforts.tolist()
        
        self.joint_state_publisher.publish(joint_msg)

        # Log statistics periodically
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            elapsed_time = time.time() - self.start_time
            actual_rate = self.frame_count / elapsed_time
            self.get_logger().info(
                f'Published {self.frame_count} frames (x3 cameras) | '
                f'Average rate: {actual_rate:.1f} Hz | '
                # Updated log to show position
                f'Sample position: [{positions[0]:.3f}, {positions[1]:.3f}, ...]'
            )

def main(args=None):
    rclpy.init(args=args)
    node = SmolVLATestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final statistics
        elapsed_time = time.time() - node.start_time
        actual_rate = node.frame_count / elapsed_time if elapsed_time > 0 else 0
        node.get_logger().info(
            f'Shutting down. Published {node.frame_count} frames (x3 cameras) in {elapsed_time:.1f}s '
            f'(average rate: {actual_rate:.1f} Hz)'
        )
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()