from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generates the launch description for the smolvla and executor nodes."""

    # Node for smolvla_inference_node.py
    # This assumes 'smolvla_inference_node.py' is registered as an
    # executable entry-point in your setup.py or installed as a script.
    smolvla_inference_node = Node(
        package='so101_ros2_bridge',
        executable='smolvla_inference_node.py',
        name='smolvla_inference_node',
        output='screen',  # Show node output directly in the console
    )

    # Node for action_chunk_executor_node.py
    # This assumes 'action_chunk_executor_node.py' is also registered
    # as an executable entry-point in your setup.py or installed as a script.
    action_chunk_executor_node = Node(
        package='so101_ros2_bridge',
        executable='action_chunk_executor_node.py',
        name='action_chunk_executor_node',
        output='screen',  # Show node output directly in the console
    )

    # Create the launch description and add the nodes
    ld = LaunchDescription()
    ld.add_action(smolvla_inference_node)
    ld.add_action(action_chunk_executor_node)

    return ld
