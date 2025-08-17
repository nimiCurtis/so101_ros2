// Copyright 2025 Your Name
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

#include "so101_teleop/leader_teleop_component.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

/**
 * @brief Main function to run the LeaderTeleopComponent as a standalone node.
 */
int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Set node options
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true); // Recommended for components

  // Create an instance of the component
  auto teleop_component = std::make_shared<so101_teleop::LeaderTeleopComponent>(options);

  // Create an executor and add the node to it
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(teleop_component);

  // Spin the executor to process callbacks
  exec.spin();

  // Shut down ROS 2
  rclcpp::shutdown();

  return 0;
}
