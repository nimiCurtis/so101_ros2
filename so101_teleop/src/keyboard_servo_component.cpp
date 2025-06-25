#include "so101_teleop/keyboard_servo_component.hpp"

#include <moveit_msgs/msg/planning_scene.hpp>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

namespace moveit_servo
{

    KeyboardServoComponent::KeyboardServoComponent(const rclcpp::NodeOptions &options)
        : Node("so101_keyboard_teleop", options),
          frame_to_publish_("base"),
          joint_vel_cmd_(1.0),
          running_(true)
    {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);

        servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
        servo_start_client_->wait_for_service(std::chrono::seconds(1));
        servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

        reader_thread_ = std::thread(&KeyboardServoComponent::keyboardLoop, this);
    }

    KeyboardServoComponent::~KeyboardServoComponent()
    {
        running_ = false;
        if (reader_thread_.joinable())
            reader_thread_.join();
    }

    void KeyboardServoComponent::keyboardLoop()
    {
        termios cooked, raw;
        tcgetattr(0, &cooked);
        raw = cooked;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(0, TCSANOW, &raw);

        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
        puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
        puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
        puts("'Q' to quit.");
        char c;
        while (running_ && read(0, &c, 1) > 0 && rclcpp::ok())
        {
            auto twist = std::make_unique<geometry_msgs::msg::TwistStamped>();
            auto joint = std::make_unique<control_msgs::msg::JointJog>();

            bool send_twist = false, send_joint = false;

            switch (c)
            {
            case 0x41:
                twist->twist.linear.x = 1.0;
                send_twist = true;
                break; // UP
            case 0x42:
                twist->twist.linear.x = -1.0;
                send_twist = true;
                break; // DOWN
            case 0x43:
                twist->twist.linear.y = 1.0;
                send_twist = true;
                break; // RIGHT
            case 0x44:
                twist->twist.linear.y = -1.0;
                send_twist = true;
                break; // LEFT
            case 0x2E:
                twist->twist.linear.z = -1.0;
                send_twist = true;
                break; // .
            case 0x3B:
                twist->twist.linear.z = 1.0;
                send_twist = true;
                break; // ;
            case 0x77:
                frame_to_publish_ = "world";
                break; // w
            case 0x65:
                frame_to_publish_ = "gripper";
                break; // e
            case 0x72:
                joint_vel_cmd_ *= -1.0;
                break; // r
            case 0x31:
                joint->joint_names.push_back("Rotation");
                send_joint = true;
                break;
            case 0x32:
                joint->joint_names.push_back("Pitch");
                send_joint = true;
                break;
            case 0x33:
                joint->joint_names.push_back("Elbow");
                send_joint = true;
                break;
            case 0x34:
                joint->joint_names.push_back("Wrist_Pitch");
                send_joint = true;
                break;
            case 0x35:
                joint->joint_names.push_back("Wrist_Roll");
                send_joint = true;
                break;
            // case 0x36:
            //     joint->joint_names.push_back("Jaw");
            //     send_joint = true;
            //     break;
            case 0x71:
                running_ = false;
                break; // q
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

        tcsetattr(0, TCSANOW, &cooked);
        rclcpp::shutdown();
    }

} // namespace moveit_servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::KeyboardServoComponent)
