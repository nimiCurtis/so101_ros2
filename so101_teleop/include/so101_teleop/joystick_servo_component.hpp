#pragma once

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

namespace moveit_servo
{

    class JoyToServoComponent : public rclcpp::Node
    {
    public:
        explicit JoyToServoComponent(const rclcpp::NodeOptions &options);

        ~JoyToServoComponent() override;

    private:
        void JoyCBLoop(const sensor_msgs::msg::Joy::ConstSharedPtr &msg);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

        std::string frame_to_publish_;
        double joint_vel_cmd_;
        bool running_;
        std::thread reader_thread_;
    };

} // namespace moveit_servo
