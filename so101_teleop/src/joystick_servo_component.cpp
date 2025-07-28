// Copyright 2025 nimiCurtis
// SPDX-License-Identifier: MIT

#include "so101_teleop/joystick_servo_component.hpp"

#include <signal.h>
#include <termios.h>
#include <unistd.h>

#include <moveit_msgs/msg/planning_scene.hpp>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "gripper_link";
const std::string BASE_FRAME_ID = "base_link";

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = {{LEFT_TRIGGER, 1.0}, {RIGHT_TRIGGER, 1.0}};
std::map<Button, double> BUTTON_DEFAULTS;

// To change controls or setup a new controller, all you should to do is change the above enums and
// the follow 2 functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(
  const std::vector<float> & axes, const std::vector<int> & buttons,
  std::unique_ptr<geometry_msgs::msg::TwistStamped> & twist,
  std::unique_ptr<control_msgs::msg::JointJog> & joint)
{
    // Give joint jogging priority because it is only buttons
    // If any joint jog command is requested, we are only publishing joint commands
    if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
    {
        // Map the D_PAD to the proximal joints
        joint->joint_names.push_back("Rotation");
        joint->velocities.push_back(axes[D_PAD_X]);
        joint->joint_names.push_back("Pitch");
        joint->velocities.push_back(axes[D_PAD_Y]);

        // Map the diamond to the distal joints
        joint->joint_names.push_back("Wrist_Pitch");
        joint->velocities.push_back(buttons[B] - buttons[X]);
        joint->joint_names.push_back("Wrist_Roll");
        joint->velocities.push_back(buttons[Y] - buttons[A]);
        return false;
    }
    twist->twist.linear.z = axes[LEFT_STICK_Y];
    twist->twist.linear.x = axes[LEFT_STICK_X];

  double lin_y_right = 0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_y_left = -0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist->twist.linear.y = lin_y_right + lin_y_left;

  twist->twist.angular.y = axes[RIGHT_STICK_X];
  twist->twist.angular.x = axes[RIGHT_STICK_Y];

  double roll_positive = -1 * buttons[RIGHT_BUMPER];
  double roll_negative = (buttons[LEFT_BUMPER]);
  twist->twist.angular.z = roll_positive + roll_negative;

    return true;
}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via
 * controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string & frame_name, const std::vector<int> & buttons)
{
    if (buttons.size() > CHANGE_VIEW && buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
    {
        frame_name = BASE_FRAME_ID;
        RCLCPP_INFO(rclcpp::get_logger("joy_teleop"), "Changed frame to BASE_FRAME_ID");
    }
    else if (buttons.size() > MENU && buttons[MENU] && frame_name == BASE_FRAME_ID)
    {
        frame_name = EEF_FRAME_ID;
        RCLCPP_INFO(rclcpp::get_logger("joy_teleop"), "Changed frame to EEF_FRAME_ID");
    }
}

namespace moveit_servo
{

    JoyToServoComponent::JoyToServoComponent(const rclcpp::NodeOptions &options)
        : Node("so101_joy_teleop", options),
          frame_to_publish_(BASE_FRAME_ID),
          joint_vel_cmd_(0.0),
          running_(true)
    {
        RCLCPP_INFO(this->get_logger(), "Starting JoyToServoComponent...");

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
            { return JoyCBLoop(msg); });

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());

        servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
        if (!servo_start_client_->wait_for_service(std::chrono::seconds(3)))
        {
            RCLCPP_WARN(this->get_logger(), "Servo start service not available after 3 second.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Calling start_servo service...");
            servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        }
    }

/**
   * @brief Destructor for JoyToServoComponent.
   * Sets the running flag to false and joins the joystick reading thread to ensure proper shutdown.
   */
JoyToServoComponent::~JoyToServoComponent() {}

    void JoyToServoComponent::JoyCBLoop(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received joystick input.");

        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

  // This call updates the frame for twist commands
  updateCmdFrame(frame_to_publish_, msg->buttons);

        if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
        {
            RCLCPP_INFO(this->get_logger(), "Publishing twist command: lin[x,y,z]=[%.2f, %.2f, %.2f], ang[x,y,z]=[%.2f, %.2f, %.2f]",
                        twist_msg->twist.linear.x, twist_msg->twist.linear.y, twist_msg->twist.linear.z,
                        twist_msg->twist.angular.x, twist_msg->twist.angular.y, twist_msg->twist.angular.z);

            twist_msg->header.frame_id = frame_to_publish_;
            twist_msg->header.stamp = this->now();
            twist_pub_->publish(std::move(twist_msg));
            RCLCPP_DEBUG(this->get_logger(), "Published TwistStamped command to %s", TWIST_TOPIC.c_str());
        }
        else
        {
            joint_msg->header.stamp = this->now();
            joint_msg->header.frame_id = frame_to_publish_;
            joint_pub_->publish(std::move(joint_msg));
            RCLCPP_DEBUG(this->get_logger(), "Published JointJog command to %s", JOINT_TOPIC.c_str());
        }

    } // namespace moveit_servo
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoComponent)
