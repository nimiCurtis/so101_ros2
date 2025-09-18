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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <optional> // For std::optional
#include <vector>
#include <string>

namespace so101_teleop
{

  /**
   * @class LeaderTeleopComponent
   * @brief A ROS 2 component for joint-space teleoperation.
   *
   * Subscribes to JointState messages from a leader arm and publishes
   * JointTrajectory messages for a follower arm, assuming identical joint names.
   */
  class LeaderTeleopComponent : public rclcpp::Node
  {
  public:
    /**
     * @brief Construct a new Leader Teleop Component object
     * @param options The node options for this component.
     */
    explicit LeaderTeleopComponent(const rclcpp::NodeOptions &options);

  private:
    /**
     * @brief Callback function for the JointState subscriber.
     *
     * Processes the leader's joint states and constructs/publishes a
     * trajectory for the follower.
     * @param msg The received JointState message.
     */
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ROS 2 Communications
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;

    // Member Variables
    bool is_initialized_;
    std::vector<std::string> ordered_arm_joint_names_;
    std::string leader_gripper_joint_name_;
    std::optional<size_t> leader_gripper_joint_index_;
  };

} // namespace so101_teleop
