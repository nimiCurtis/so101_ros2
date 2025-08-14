// Copyright 2025 Your Name
// SPDX-License-Identifier: MIT

#include "so101_teleop/leader_teleop_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>


namespace so101_teleop
{

  LeaderTeleopComponent::LeaderTeleopComponent(const rclcpp::NodeOptions &options)
      : rclcpp::Node("leader_teleop_component", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing LeaderTeleopComponent...");

    // Declare and retrieve parameters
    auto leader_topic = this->declare_parameter<std::string>("leader_joint_states_topic", "/leader/joint_states");
    auto follower_topic = this->declare_parameter<std::string>("follower_trajectory_topic", "/follower/arm_controller/joint_trajectory");

    RCLCPP_INFO(this->get_logger(), "Subscribing to leader joint states on: '%s'", leader_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to follower trajectory on: '%s'", follower_topic.c_str());

    // Create publisher and subscriber
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(follower_topic, 10);

    // The callback is a class method, so we use std::bind.
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        leader_topic, 10, std::bind(&LeaderTeleopComponent::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Component successfully initialized.");
  }

  void LeaderTeleopComponent::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // On the first message, store the joint names in their received order.
    if (!is_initialized_)
    {
      if (msg->name.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Received JointState message with no joint names. Ignoring.");
        return;
      }
      ordered_joint_names_ = msg->name;
      is_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initialized with joint order: [%s, ...]", ordered_joint_names_[0].c_str());
    }

    // Construct the JointTrajectory message
    auto trajectory_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
    trajectory_msg->header.stamp = this->get_clock()->now();
    trajectory_msg->joint_names = ordered_joint_names_;

    // Create a single trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = msg->position;
    // Set a small duration to create a smooth streaming effect
    point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(100));

    trajectory_msg->points.push_back(point);

    // Publish the message
    trajectory_pub_->publish(std::move(trajectory_msg));
  }

} // namespace so101_teleop

// Register the component with class_loader, making it available to the ROS 2 component manager
RCLCPP_COMPONENTS_REGISTER_NODE(so101_teleop::LeaderTeleopComponent)
