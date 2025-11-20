#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading
from collections import deque
import time


class  ActionChunkExecutor(Node):
    def __init__(self):
        super().__init__('action_chunk_executor')
        self.get_logger().info('Action Chunk Executor Node starting...')

        # Declare parameters
        self.declare_parameter('action_chunk_topic', '/smolvla_inference/action_chunk')
        self.declare_parameter('joint_command_topic', '/leader/isaac_joint_command')
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('inference_delay', 0.3)  # seconds - how long inference takes
        self.declare_parameter('chunk_size', 50)  # Expected number of actions in a chunk
        self.declare_parameter('action_dim', 6)  # Number of joints
        self.declare_parameter('use_delay_compensation', True)  # Whether to skip actions based on delay

        # Get parameters
        action_chunk_topic = self.get_parameter('action_chunk_topic').get_parameter_value().string_value
        joint_command_topic = self.get_parameter('joint_command_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.inference_delay = self.get_parameter('inference_delay').get_parameter_value().double_value
        self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
        self.action_dim = self.get_parameter('action_dim').get_parameter_value().integer_value
        self.use_delay_compensation = self.get_parameter('use_delay_compensation').get_parameter_value().bool_value

        # Calculate starting index based on inference delay
        # If inference takes 0.5 seconds and we publish at 30Hz, we've "missed" 15 actions
        if self.use_delay_compensation:
            self.skip_actions = int(self.publish_rate * self.inference_delay) 
            self.get_logger().info(
                f'Delay compensation ENABLED: Inference delay: {self.inference_delay}s, '
                f'Publishing rate: {self.publish_rate}Hz, '
                f'Will skip first {self.skip_actions} actions of each chunk'
            )
        else:
            self.skip_actions = 0
            self.get_logger().info(
                f'Delay compensation DISABLED: Starting from first action of each chunk'
            )

        # Define joint names based on robot configuration
        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'gripper'
        ]

        # Ensure we have the right number of joint names
        if len(self.joint_names) != self.action_dim:
            if len(self.joint_names) < self.action_dim:
                for i in range(len(self.joint_names), self.action_dim):
                    self.joint_names.append(f'joint_{i}')
            else:
                self.joint_names = self.joint_names[:self.action_dim]

        # Single chunk storage - simplified design
        self.current_chunk = []
        self.current_action_index = 0
        self.chunk_timestamp = None
        self.chunk_lock = threading.Lock()
        
        # Statistics
        self.chunks_received = 0
        self.actions_published = 0
        self.total_actions_skipped = 0

        # Subscribe to action chunks
        self.chunk_subscriber = self.create_subscription(
            Float32MultiArray,
            action_chunk_topic,
            self.action_chunk_callback,
            10
        )
        self.get_logger().info(f'Subscribing to action chunks: {action_chunk_topic}')

        # Publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            JointState,
            joint_command_topic,
            10
        )
        self.get_logger().info(f'Publishing joint commands to: {joint_command_topic}')

        # Create timer for publishing at specified rate
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_action_callback)
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')

        # Status reporting timer (every 2 seconds)
        self.status_timer = self.create_timer(2.0, self.status_callback)

    def action_chunk_callback(self, msg):
        """Handle incoming action chunks - immediately switch to new chunk."""
        try:
            # Check if we have timestamp in the data
            has_timestamp = msg.layout.data_offset == 2
            chunk_timestamp = None
            
            # extract action data and timestamp if present
            if has_timestamp and len(msg.data) >= 2:
                # Extract timestamp from first two values
                timestamp_sec = int(msg.data[0])
                timestamp_nsec = int(msg.data[1])
                chunk_timestamp = (timestamp_sec, timestamp_nsec)
                
                # Remove timestamp from data for processing
                action_data = msg.data[2:]
            else:
                # No timestamp, use all data as actions
                action_data = msg.data
            
            # Extract dimensions from the layout
            if len(msg.layout.dim) >= 2:
                num_actions = msg.layout.dim[0].size
                action_dim = msg.layout.dim[1].size
                
                # Validate dimensions
                if action_dim != self.action_dim:
                    self.get_logger().warn(
                        f'Received action dimension {action_dim} doesn\'t match expected {self.action_dim}'
                    )
                    return
                
                # Parse the flat array into individual actions
                new_chunk = []

                self.get_logger().info(f'num actions: {num_actions}')

                # parse actions of a new chunk to list new_chunk
                for i in range(num_actions):
                    start_idx = i * action_dim
                    end_idx = start_idx + action_dim
                    if end_idx <= len(action_data):
                        action = action_data[start_idx:end_idx]
                        new_chunk.append(action)
                
                # update idx of the current action to be published
                with self.chunk_lock:
                    # SIMPLIFIED: Immediately replace current chunk with new one
                    self.current_chunk = new_chunk
                    self.get_logger().info(f'old timestamp: {self.chunk_timestamp}, new timestamp: {chunk_timestamp}')
                    self.chunk_timestamp = chunk_timestamp

                    # Start from the action that corresponds to "now" 
                    # (skip actions that would have been executed during inference delay)
                    initial_skip = self.skip_actions
                    
                    # Calculate actual delay if timestamp available and compensation is enabled
                    delay_info = ""
                    actual_skip = initial_skip

                    # CRITICAL: Ensure the index is within valid bounds
                    # If delay is too large, we might need to skip the entire chunk
                
                    if actual_skip >= len(new_chunk):
                        self.get_logger().warn(
                            f'Delay too large! Would skip {actual_skip} actions but chunk only has {len(new_chunk)}. '
                            f'Starting from last action.'
                        )
                        self.current_action_index = 0
                    elif actual_skip < 0:
                        self.get_logger().warn(
                            f'Negative skip calculated ({actual_skip})! Starting from beginning.'
                        )
                        self.current_action_index = 0
                    else:
                        self.current_action_index = actual_skip


                    self.chunks_received += 1
                    self.total_actions_skipped += self.current_action_index
                    
                    self.get_logger().info(
                        f'New chunk received with {len(new_chunk)} actions, '
                        f'starting from action {self.current_action_index} '
                        f'(skipped {self.current_action_index} actions){delay_info}'
                    )
                    
            else:
                self.get_logger().warn('Received action chunk with invalid layout dimensions')
                
        except Exception as e:
            self.get_logger().error(f'Error processing action chunk: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def publish_action_callback(self):
        """Publish individual actions from the chunk at the specified rate."""
        try:
            with self.chunk_lock:
                # Check if we have a chunk and haven't exceeded it
                if not self.current_chunk or self.current_action_index >= len(self.current_chunk):
                    # No chunk or we've run out of actions - wait for next chunk
                    self.get_logger().debug(
                        'Waiting for new chunk or reached end of current chunk',
                        throttle_duration_sec=5.0
                    )
                    return

                # Get the current action
                current_action = self.current_chunk[self.current_action_index]
                
                # Create and publish JointState message
                joint_msg = JointState()
                
                # Set header
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.header.frame_id = ''
                
                # Set joint names
                joint_msg.name = self.joint_names
                
                # Set positions to the current action values
                joint_msg.position = list(current_action)
                
                # Ensure we have the right number of position values
                if len(joint_msg.position) != len(self.joint_names):
                    if len(joint_msg.position) < len(self.joint_names):
                        joint_msg.position.extend([0.0] * (len(self.joint_names) - len(joint_msg.position)))
                    else:
                        joint_msg.position = joint_msg.position[:len(self.joint_names)]
                
                # Set velocity and effort to zeros
                joint_msg.velocity = [0.0] * len(self.joint_names)
                joint_msg.effort = [0.0] * len(self.joint_names)
                
                # Publish the message
                self.joint_command_publisher.publish(joint_msg)
                
                # Only increment if chunk didn't change
                self.current_action_index += 1
                self.actions_published += 1
                
                # Log current action periodically

                remaining = len(self.current_chunk) - self.current_action_index
                self.get_logger().info(
                    f'Action {self.current_action_index}/{len(self.current_chunk)}, '
                    f'Remaining: {remaining}, '
                    f'Positions: [{joint_msg.position[0]:.3f}, {joint_msg.position[1]:.3f}, ...]'
                )
                    
        except Exception as e:
            self.get_logger().error(f'Error publishing action: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def status_callback(self):
        """Report status periodically."""
        with self.chunk_lock:
            if self.current_chunk:
                chunk_size = len(self.current_chunk)
                remaining = chunk_size - self.current_action_index
                progress = f'{self.current_action_index}/{chunk_size} (remaining: {remaining})'
                
                # Calculate chunk age if timestamp available
                chunk_age = "No timestamp"
                if self.chunk_timestamp:
                    current_time = self.get_clock().now()
                    age = (current_time.seconds_nanoseconds()[0] - self.chunk_timestamp[0]) + \
                          (current_time.seconds_nanoseconds()[1] - self.chunk_timestamp[1]) / 1e9
                    chunk_age = f"{age:.2f}s old"
            else:
                progress = 'No chunk'
                chunk_age = 'N/A'
        
        avg_skip = self.total_actions_skipped / self.chunks_received if self.chunks_received > 0 else 0
        
        self.get_logger().info(
            f'Status - Chunks: {self.chunks_received}, '
            f'Actions published: {self.actions_published}, '
            f'Current: {progress}, '
            f'Chunk age: {chunk_age}, '
            f'Avg actions skipped: {avg_skip:.1f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node =  ActionChunkExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Action Chunk Executor Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()