// Copyright 2025 Your Name
// SPDX-License-Identifier: MIT

#include "so101_teleop/leader_teleop_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm> // For std::find

namespace so101_teleop
{

  LeaderTeleopComponent::LeaderTeleopComponent(const rclcpp::NodeOptions &options)
      : rclcpp::Node("leader_teleop_component", options), is_initialized_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing LeaderTeleopComponent...");

    // === Declare and retrieve parameters for arm and gripper ===
    auto leader_topic = this->declare_parameter<std::string>(
        "leader_joint_states_topic",
        "/leader/joint_states");
    auto follower_arm_topic = this->declare_parameter<std::string>(
        "follower_trajectory_topic",
        "/follower/arm_controller/joint_trajectory");
    auto follower_gripper_action = this->declare_parameter<std::string>(
        "follower_gripper_action_name",
        "/follower/gripper_controller/gripper_cmd");
    leader_gripper_joint_name_ = this->declare_parameter<std::string>(
        "leader_gripper_joint_name",
        "gripper");

    RCLCPP_INFO(
        this->get_logger(), "Subscribing to leader joint states on: '%s'",
        leader_topic.c_str());
    RCLCPP_INFO(
        this->get_logger(), "Publishing to follower arm trajectory on: '%s'",
        follower_arm_topic.c_str());
    RCLCPP_INFO(
        this->get_logger(), "Creating gripper action client for: '%s'",
        follower_gripper_action.c_str());

    // === Create publishers and subscribers ===

    // Publisher for the arm trajectory
    trajectory_pub_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(follower_arm_topic, 10);

    // Action client for the gripper
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
        this,
        follower_gripper_action);

    // Subscriber for the leader's joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        leader_topic, 10,
        std::bind(&LeaderTeleopComponent::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Component successfully initialized.");
  }

  void LeaderTeleopComponent::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // On the first message, store the joint names and find the gripper index.
    if (!is_initialized_)
    {
      if (msg->name.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Received JointState message with no joint names. Ignoring.");
        return;
      }

      const auto &names = msg->name;
      // Find the index of the leader's gripper joint
      auto it = std::find(names.begin(), names.end(), leader_gripper_joint_name_);
      if (it != names.end())
      {
        leader_gripper_joint_index_ = std::distance(names.begin(), it);
        RCLCPP_INFO(
            this->get_logger(), "Found leader gripper joint '%s' at index %zu",
            leader_gripper_joint_name_.c_str(), *leader_gripper_joint_index_);
      }
      else
      {
        RCLCPP_WARN(
            this->get_logger(),
            "Leader gripper joint '%s' not found in JointState message. Gripper teleop disabled.",
            leader_gripper_joint_name_.c_str());
      }

      // Store the names of the ARM joints only (exclude the gripper)
      for (const auto &joint_name : names)
      {
        if (joint_name != leader_gripper_joint_name_)
        {
          ordered_arm_joint_names_.push_back(joint_name);
        }
      }
      is_initialized_ = true;
      RCLCPP_INFO(
          this->get_logger(), "Initialized with arm joints: [%s, ...]",
          ordered_arm_joint_names_[0].c_str());
    }

    // === Handle Arm Teleop ===
    auto trajectory_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
    trajectory_msg->header.stamp = this->get_clock()->now();
    trajectory_msg->joint_names = ordered_arm_joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    // Populate arm positions, skipping the gripper position
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (leader_gripper_joint_index_.has_value() && i == *leader_gripper_joint_index_)
      {
        continue; // Skip gripper joint
      }
      point.positions.push_back(msg->position[i]);
    }

    point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(5));
    trajectory_msg->points.push_back(point);
    trajectory_pub_->publish(std::move(trajectory_msg));

    // === Handle Gripper Teleop ===
    if (leader_gripper_joint_index_.has_value())
    {
      // Use wait_for_action_server with a zero timeout for a non-blocking check
      if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(0)))
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Gripper action server is not available yet.");
        return;
      }

      auto goal_msg = control_msgs::action::GripperCommand::Goal();
      goal_msg.command.position = msg->position[*leader_gripper_joint_index_];
      goal_msg.command.max_effort = 10.0;

      auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
      gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
    }
  }

} // namespace so101_teleop

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(so101_teleop::LeaderTeleopComponent)