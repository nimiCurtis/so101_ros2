/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 */

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "gripper";
const std::string BASE_FRAME_ID = "base";

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

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float> &axes, const std::vector<int> &buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped> &twist,
                     std::unique_ptr<control_msgs::msg::JointJog> &joint)
{
    // The bread and butter: map buttons to twist commands
    /*
    twist->twist.linear.z = axes[LEFT_STICK_Y];
    twist->twist.linear.y = axes[LEFT_STICK_X];

    double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
    twist->twist.linear.x = lin_x_right + lin_x_left;

    twist->twist.angular.y = axes[RIGHT_STICK_Y];
    twist->twist.angular.x = axes[RIGHT_STICK_X];

    double roll_positive = buttons[RIGHT_BUMPER];
    double roll_negative = -1 * (buttons[LEFT_BUMPER]);
    twist->twist.angular.z = roll_positive + roll_negative;
    */

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
    //}
}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string &frame_name, const std::vector<int> &buttons)
{
    if (buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
        frame_name = BASE_FRAME_ID;
    else if (buttons[MENU] && frame_name == BASE_FRAME_ID)
        frame_name = EEF_FRAME_ID;
}

namespace moveit_servo
{
    class JoyToServoPub : public rclcpp::Node
    {
    public:
        JoyToServoPub(const rclcpp::NodeOptions &options)
            : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
        {
            // Setup pub/sub
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
                [this](const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
                { return joyCB(msg); });

            twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
            joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
            collision_pub_ =
                this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

            // Create a service client to start the ServoNode
            servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
            servo_start_client_->wait_for_service(std::chrono::seconds(1));
            servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        }

        ~JoyToServoPub() override
        {
            if (collision_pub_thread_.joinable())
                collision_pub_thread_.join();
        }

        void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
        {
            // Create the messages we might publish
            auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

            // This call updates the frame for twist commands
            updateCmdFrame(frame_to_publish_, msg->buttons);

            // Convert the joystick message to Twist or JointJog and publish
            if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
            {
                // publish the TwistStamped
                twist_msg->header.frame_id = frame_to_publish_;
                twist_msg->header.stamp = this->now();
                twist_pub_->publish(std::move(twist_msg));
            }
            else
            {
                // publish the JointJog
                joint_msg->header.stamp = this->now();
                joint_msg->header.frame_id = frame_to_publish_;
                joint_pub_->publish(std::move(joint_msg));
            }
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

        std::string frame_to_publish_;

        std::thread collision_pub_thread_;
    }; // class JoyToServoPub

} // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)
