from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='so101_smolvla_inference',
            executable='smolvla_inference_node',
            name='smolvla_inference_node',
            output='screen',
            parameters=[
                {'model_id': 'lerobot/smolvla_base'},
                {'image_topic': '/follower/cam_front/image_raw'},
                {'joint_state_topic': '/follower/robot_state_publisher'},
                {'action_topic': '/smolvla_inference/action'},
                {'task': ''},
                {'robot_type': ''},
                {'use_dummy_input': True},
            ]
        )
    ])
