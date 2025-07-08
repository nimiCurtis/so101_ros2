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

#include "so101_teleop/keyboard_servo_component.hpp"

#include <moveit_msgs/msg/planning_scene.hpp>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <stdexcept>

namespace moveit_servo
{

  class KeyboardReader
  {
  public:
    KeyboardReader()
        : file_descriptor_(0)
    {
      tcgetattr(file_descriptor_, &cooked_);
      struct termios raw;
      memcpy(&raw, &cooked_, sizeof(struct termios));
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VEOL] = 1;
      raw.c_cc[VEOF] = 2;
      tcsetattr(file_descriptor_, TCSANOW, &raw);
    }

    void readOne(char *c)
    {
      int rc = read(file_descriptor_, c, 1);
      if (rc < 0)
      {
        throw std::runtime_error("read failed");
      }
    }

    void shutdown()
    {
      tcsetattr(file_descriptor_, TCSANOW, &cooked_);
    }

  private:
    int file_descriptor_;
    struct termios cooked_;
  };

  KeyboardServoComponent::KeyboardServoComponent(const rclcpp::NodeOptions &options)
      : Node("so101_keyboard_teleop", options),
        frame_to_publish_("base_link"),
        joint_vel_cmd_(1.0),
        running_(true)
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
        "/servo_node/delta_joint_cmds",
        10);

    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    reader_thread_ = std::thread(&KeyboardServoComponent::keyboardLoop, this);
  }

  KeyboardServoComponent::~KeyboardServoComponent()
  {
    running_ = false;
    if (reader_thread_.joinable())
    {
      reader_thread_.join();
    }
  }

  void KeyboardServoComponent::keyboardLoop()
  {
    KeyboardReader input;

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
    puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
    puts("Use 1|2|3|4|5 keys to joint jog. 'R' to reverse the direction of jogging.");
    puts("'Q' to quit.");

    char c;
    while (running_ && rclcpp::ok())
    {
      try
      {
        input.readOne(&c);
      }
      catch (const std::runtime_error &)
      {
        perror("read():");
        break;
      }

      auto twist = std::make_unique<geometry_msgs::msg::TwistStamped>();
      auto joint = std::make_unique<control_msgs::msg::JointJog>();
      bool send_twist = false, send_joint = false;

      switch (c)
      {
      case 0x41:
        twist->twist.linear.x = 1.0;
        send_twist = true;
        break;
      case 0x42:
        twist->twist.linear.x = -1.0;
        send_twist = true;
        break;
      case 0x43:
        twist->twist.linear.y = 1.0;
        send_twist = true;
        break;
      case 0x44:
        twist->twist.linear.y = -1.0;
        send_twist = true;
        break;
      case 0x2E:
        twist->twist.linear.z = -1.0;
        send_twist = true;
        break;
      case 0x3B:
        twist->twist.linear.z = 1.0;
        send_twist = true;
        break;
      case 0x77:
        frame_to_publish_ = "base_link";
        break;
      case 0x65:
        frame_to_publish_ = "gripper_link";
        break;
      case 0x72:
        joint_vel_cmd_ *= -1.0;
        break;
      case 0x31:
        joint->joint_names.push_back("shoulder_pan");
        send_joint = true;
        break;
      case 0x32:
        joint->joint_names.push_back("shoulder_lift");
        send_joint = true;
        break;
      case 0x33:
        joint->joint_names.push_back("elbow_flex");
        send_joint = true;
        break;
      case 0x34:
        joint->joint_names.push_back("wrist_flex");
        send_joint = true;
        break;
      case 0x35:
        joint->joint_names.push_back("wrist_roll");
        send_joint = true;
        break;
      case 0x71:
        running_ = false;
        break;
      }

      if (send_twist)
      {
        twist->header.stamp = this->now();
        twist->header.frame_id = frame_to_publish_;
        twist_pub_->publish(std::move(twist));
      }
      else if (send_joint)
      {
        joint->velocities.push_back(joint_vel_cmd_);
        joint->header.stamp = this->now();
        joint->header.frame_id = frame_to_publish_;
        joint_pub_->publish(std::move(joint));
      }
    }

    rclcpp::shutdown();
  }

} // namespace moveit_servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::KeyboardServoComponent)
