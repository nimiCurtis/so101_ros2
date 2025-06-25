#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <termios.h>
#include <thread>

namespace moveit_servo
{

    class KeyboardServoComponent : public rclcpp::Node
    {
    public:
        explicit KeyboardServoComponent(const rclcpp::NodeOptions &options);

        ~KeyboardServoComponent() override;

    private:
        void keyboardLoop();

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

        std::string frame_to_publish_;
        double joint_vel_cmd_;
        bool running_;
        std::thread reader_thread_;
    };

} // namespace moveit_servo
