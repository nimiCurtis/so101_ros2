#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generates the launch description for the SmolVLA inference and action chunk executor nodes.
    
    This launch file configures both:
    1. SmolVLA Inference Node - Performs VLA model inference
    2. Action Chunk Executor Node - Executes predicted action chunks
    """

    # =============================================================================
    # Declare Launch Arguments - SmolVLA Inference Node
    # =============================================================================
    
    model_id_arg = DeclareLaunchArgument(
        'model_id',
        default_value='lerobot/smolvla_base',
        description='HuggingFace model ID for SmolVLA policy'
    )
    
    task_arg = DeclareLaunchArgument(
        'task',
        default_value='Pick up the white block and insert it on the green peg',
        description='Task description for the robot'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='so101',
        description='Robot type identifier'
    )
    
    camera1_topic_arg = DeclareLaunchArgument(
        'camera1_topic',
        default_value='/follower/cam_front/image_raw',
        description='Topic for front camera feed'
    )
    
    camera2_topic_arg = DeclareLaunchArgument(
        'camera2_topic',
        default_value='/follower/cam_top1/image_raw',
        description='Topic for top camera 1 feed'
    )
    
    camera3_topic_arg = DeclareLaunchArgument(
        'camera3_topic',
        default_value='/follower/cam_top2/image_raw',
        description='Topic for top camera 2 feed'
    )
    
    joint_state_topic_arg = DeclareLaunchArgument(
        'joint_state_topic',
        default_value='/isaac/isaac_joint_states',
        description='Topic for robot joint states'
    )
    
    action_topic_arg = DeclareLaunchArgument(
        'action_topic',
        default_value='/isaac/isaac_joint_command_test',
        description='Topic for publishing single actions (JointState)'
    )
    
    inference_rate_arg = DeclareLaunchArgument(
        'inference_rate',
        default_value='2',
        description='Publishing rate for inference node in Hz (typically 1-5 Hz)'
    )
    
    use_dummy_input_arg = DeclareLaunchArgument(
        'use_dummy_input',
        default_value='false',
        description='Use dummy inputs instead of real camera/joint data for testing'
    )
    
    image_qos_arg = DeclareLaunchArgument(
        'image_qos',
        default_value='2',
        description='QoS depth for image subscriptions'
    )
    
    joint_state_qos_arg = DeclareLaunchArgument(
        'joint_state_qos',
        default_value='2',
        description='QoS depth for joint state subscriptions'
    )
    
    # =============================================================================
    # Declare Launch Arguments - Action Chunk Executor Node
    # =============================================================================
    
    action_chunk_topic_arg = DeclareLaunchArgument(
        'action_chunk_topic',
        default_value='/smolvla_inference/action_chunk',
        description='Topic for receiving action chunks from inference node'
    )
    
    joint_command_topic_arg = DeclareLaunchArgument(
        'joint_command_topic',
        default_value='/isaac/isaac_joint_command',
        description='Topic for publishing joint commands to robot controller'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Action execution rate in Hz (30Hz = ~33ms per action)'
    )
    
    inference_delay_arg = DeclareLaunchArgument(
        'inference_delay',
        default_value='0.5',
        description='Expected inference delay in seconds for delay compensation'
    )
    
    chunk_size_arg = DeclareLaunchArgument(
        'chunk_size',
        default_value='50',
        description='Expected number of actions in each chunk'
    )
    
    action_dim_arg = DeclareLaunchArgument(
        'action_dim',
        default_value='6',
        description='Number of joints/action dimensions'
    )
    
    use_delay_compensation_arg = DeclareLaunchArgument(
        'use_delay_compensation',
        default_value='true',
        description='Enable/disable delay compensation (skip initial actions)'
    )

    # =============================================================================
    # Configure Nodes
    # =============================================================================

    # SmolVLA Inference Node
    smolvla_inference_node = Node(
        package='so101_ros2_bridge',
        executable='smolvla_inference_node.py',
        name='smolvla_inference_node',
        output='screen',
        parameters=[{
            'model_id': LaunchConfiguration('model_id'),
            'task': LaunchConfiguration('task'),
            'robot_type': LaunchConfiguration('robot_type'),
            'camera1_topic': LaunchConfiguration('camera1_topic'),
            'camera2_topic': LaunchConfiguration('camera2_topic'),
            'camera3_topic': LaunchConfiguration('camera3_topic'),
            'joint_state_topic': LaunchConfiguration('joint_state_topic'),
            'action_topic': LaunchConfiguration('action_topic'),
            'action_chunk_topic': LaunchConfiguration('action_chunk_topic'),
            'publisher_rate': LaunchConfiguration('inference_rate'),
            'use_dummy_input': LaunchConfiguration('use_dummy_input'),
            'image_subscription_qos': LaunchConfiguration('image_qos'),
            'joint_state_subscription_qos': LaunchConfiguration('joint_state_qos'),
        }],
        # Uncomment to add remappings if needed
        # remappings=[
        #     ('/some/topic', '/remapped/topic'),
        # ]
    )

    # Action Chunk Executor Node
    action_chunk_executor_node = Node(
        package='so101_ros2_bridge',
        executable='action_chunk_executor_node.py',
        name='action_chunk_executor_node',
        output='screen',
        parameters=[{
            'action_chunk_topic': LaunchConfiguration('action_chunk_topic'),
            'joint_command_topic': LaunchConfiguration('joint_command_topic'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'inference_delay': LaunchConfiguration('inference_delay'),
            'chunk_size': LaunchConfiguration('chunk_size'),
            'action_dim': LaunchConfiguration('action_dim'),
            'use_delay_compensation': LaunchConfiguration('use_delay_compensation'),
        }]
    )

    # =============================================================================
    # Build Launch Description
    # =============================================================================
    
    return LaunchDescription([
        # Inference node arguments
        model_id_arg,
        task_arg,
        robot_type_arg,
        camera1_topic_arg,
        camera2_topic_arg,
        camera3_topic_arg,
        joint_state_topic_arg,
        action_topic_arg,
        inference_rate_arg,
        use_dummy_input_arg,
        image_qos_arg,
        joint_state_qos_arg,
        
        # Executor node arguments
        action_chunk_topic_arg,
        joint_command_topic_arg,
        publish_rate_arg,
        inference_delay_arg,
        chunk_size_arg,
        action_dim_arg,
        use_delay_compensation_arg,
        
        # Nodes
        smolvla_inference_node,
        action_chunk_executor_node,
    ])