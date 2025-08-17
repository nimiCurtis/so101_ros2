// Copyright 2025 nimiCurtis
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "so101_hardware_interface/so101_hardware_bridge.hpp"

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace so101_hardware_interface
{

hardware_interface::CallbackReturn So101HardwareBridge::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create a ROS 2 node to handle topic communication
  node_ = rclcpp::Node::make_shared("so101_hardware_bridge_node");

  // Initialize storage vectors based on URDF info
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_names_.resize(info_.joints.size());

  // Store joint names from the URDF
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_names_[i] = info_.joints[i].name;
  }

  RCLCPP_INFO(node_->get_logger(), "So101HardwareBridge initialized successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> So101HardwareBridge::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> So101HardwareBridge::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // Only position command interface is supported
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn So101HardwareBridge::on_activate(const rclcpp_lifecycle::State &)
{
  // Create the publisher for sending commands to the Python node
  cmd_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("joint_commands", 10);

  // Create the subscriber to receive states from the Python node
  state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states_raw", 10,
    std::bind(&So101HardwareBridge::state_subscriber_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "So101HardwareBridge activated. Subscribing to /joint_states and publishing to /joint_commands.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn So101HardwareBridge::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  cmd_publisher_.reset();
  state_subscriber_.reset();
  RCLCPP_INFO(node_->get_logger(), "So101HardwareBridge deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type So101HardwareBridge::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Process any incoming messages from the Python node's /joint_states topic
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type So101HardwareBridge::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Create and publish the command message to the Python node
  auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();

  // Ensure commands are not NaN before sending
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    if (std::isnan(hw_commands_[i])) {
      // If a command is NaN, use the last known position instead.
      // This prevents sending invalid commands on startup before the first
      // command is received from a controller.
      std::lock_guard<std::mutex> guard(lock_);
      msg->data.push_back(hw_positions_[i]);
    } else {
      msg->data.push_back(hw_commands_[i]);
    }
  }

  cmd_publisher_->publish(std::move(msg));

  return hardware_interface::return_type::OK;
}

void So101HardwareBridge::state_subscriber_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(lock_);
  // Match received joint names with the ones from the URDF
  for (size_t i = 0; i < msg->name.size(); ++i) {
    for (size_t j = 0; j < joint_names_.size(); ++j) {
      if (msg->name[i] == joint_names_[j]) {
        hw_positions_[j] = msg->position[i];
        if (msg->velocity.size() > i) {
          hw_velocities_[j] = msg->velocity[i];
        }
        break;
      }
    }
  }
}

} // namespace so101_hardware_interface

// Export the class as a plugin for ros2_control
PLUGINLIB_EXPORT_CLASS(
  so101_hardware_interface::So101HardwareBridge,
  hardware_interface::SystemInterface)
