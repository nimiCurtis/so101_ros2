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


#ifndef SO101_HARDWARE_INTERFACE__SO101_HARDWARE_BRIDGE_HPP_
#define SO101_HARDWARE_INTERFACE__SO101_HARDWARE_BRIDGE_HPP_

#include <vector>
#include <string>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace so101_hardware_interface
{
class So101HardwareBridge : public hardware_interface::SystemInterface
{
public:
  // Macro for creating shared pointers for this class
  RCLCPP_SHARED_PTR_DEFINITIONS(So101HardwareBridge)

  // Standard ros2_control lifecycle methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;

  // Methods for exporting command and state interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Methods for reading from and writing to the hardware
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ROS 2 node for handling topic communication
  std::shared_ptr<rclcpp::Node> node_;

  // Publisher for sending commands to the Python hardware node
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_publisher_;

  // Subscriber for receiving joint states from the Python hardware node
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;

  // Mutex to ensure thread-safe access to hardware state data
  std::mutex lock_;

  // Storage for hardware commands and states
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Names of the joints as defined in the URDF
  std::vector<std::string> joint_names_;

  // Callback function for the joint state subscriber
  void state_subscriber_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};
} // namespace so101_hardware_interface

#endif // SO101_HARDWARE_INTERFACE__SO101_HARDWARE_BRIDGE_HPP_
