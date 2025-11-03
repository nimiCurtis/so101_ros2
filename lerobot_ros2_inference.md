# Plan for creating a ROS2 node for smolvla inference

This document outlines the steps to create a ROS2 node that uses the `smolvla` model from the `lerobot` library for inference.

## 1. Project Setup

*   Create a new ROS2 package (e.g., `lerobot_ros2`).
*   Add dependencies to `package.xml`, including `rclpy`, `sensor_msgs`, and any other necessary packages.
*   Install `lerobot` and its dependencies in the ROS2 workspace.

## 2. Node Implementation (`inference_node.py`)

### 2.1. Initialization

*   Import necessary modules: `rclpy`, `SmolVLAPolicy`, `make_pre_post_processors`, `build_inference_frame`, etc.
*   Create a class for the ROS2 node (e.g., `SmolVLAInferenceNode`).
*   In the `__init__` method:
    *   Initialize the node.
    *   Load the `SmolVLAPolicy` model using `SmolVLAPolicy.from_pretrained("lerobot/smolvla_base")`.
    *   Create the pre- and post-processors using `make_pre_post_processors`.
    *   Create a subscriber to an image topic (e.g., `/image_raw` of type `sensor_msgs/Image`).
    *   Create a publisher for the output action (e.g., `/robot_action` of a custom message type or a standard type like `std_msgs/Float32MultiArray`).

### 2.2. Image Callback

*   This method will be called whenever a new image is received.
*   Inside the callback:
    *   Convert the ROS image message to a format that `lerobot` can use (e.g., a NumPy array or a PyTorch tensor).
    *   Get the observation from the robot (e.g., joint states). This might come from another ROS topic.
    *   Use `build_inference_frame` to prepare the observation data.
    *   Apply the pre-processor to the observation.
    *   Run inference using `model.select_action(observation)`.
    *   Apply the post-processor to the action.
    *   Use `make_robot_action` to format the action.
    *   Publish the action using the publisher.

## 3. Launch File

*   Create a launch file to start the inference node.
*   This will allow for easy configuration of parameters like the model ID, image topic, etc.

## 4. Custom Messages

*   If necessary, define custom message types for actions or observations.

## 5. Running the Node

*   Source the ROS2 workspace.
*   Run the launch file.
*   Publish images to the subscribed topic to test the node.
