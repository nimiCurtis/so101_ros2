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

    // --- QoS profiles ---

    // 1) Leader JointState subscriber: low latency, drop rather than block
    rclcpp::QoS js_sub_qos{rclcpp::KeepLast(1)};
    js_sub_qos.best_effort()
        .durability_volatile();

    // 2) Follower arm JointTrajectory publisher: reliable delivery to controller
    rclcpp::QoS traj_pub_qos{rclcpp::KeepLast(2)};
    traj_pub_qos.reliable()
        .durability_volatile();

    // === Create publishers and subscribers ===
    // Publisher for the arm trajectory
    trajectory_pub_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(follower_arm_topic, traj_pub_qos);

    // Create action client (note: rclcpp_action, not clcpp_action)
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
        this,
        follower_gripper_action);

    // === NEW: Declare and get the gripper deadband parameter ===
    gripper_deadband_ = this->declare_parameter<double>("gripper_deadband", 0.01);
    RCLCPP_INFO(this->get_logger(), "Using gripper deadband of: %.4f radians", gripper_deadband_);

    // Subscriber for the leader's joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        leader_topic, js_sub_qos,
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
    // Clear and reuse the pre-allocated message
    trajectory_msg_.points.clear();
    trajectory_msg_.header.stamp = this->get_clock()->now();
    trajectory_msg_.joint_names = ordered_arm_joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(ordered_arm_joint_names_.size()); // Avoid reallocations
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (leader_gripper_joint_index_.has_value() && i == *leader_gripper_joint_index_)
      {
        continue;
      }
      point.positions.push_back(msg->position[i]);
    }

    point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(3));
    trajectory_msg_.points.push_back(point);
    trajectory_pub_->publish(trajectory_msg_); // Publish by const reference

    // === Handle Gripper Teleop with Deadband Logic ===
    if (leader_gripper_joint_index_.has_value())
    {
      double current_gripper_pos = msg->position[*leader_gripper_joint_index_];

      // ONLY send a new goal if the position has changed by more than the deadband
      if (std::abs(current_gripper_pos - last_gripper_goal_position_) > gripper_deadband_)
      {
        if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(0)))
        {
          RCLCPP_WARN_ONCE(this->get_logger(), "Gripper action server is not available yet.");
          return;
        }

        auto goal_msg = control_msgs::action::GripperCommand::Goal();
        goal_msg.command.position = current_gripper_pos;
        goal_msg.command.max_effort = 10.0;

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
        gripper_action_client_->async_send_goal(goal_msg, send_goal_options);

        // IMPORTANT: Update the last sent position
        last_gripper_goal_position_ = current_gripper_pos;
      }
    }
  }
} // namespace so101_teleop

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(so101_teleop::LeaderTeleopComponent)