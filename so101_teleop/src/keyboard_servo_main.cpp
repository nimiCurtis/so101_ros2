#include <rclcpp/rclcpp.hpp>
#include "so101_teleop/keyboard_servo_component.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<moveit_servo::KeyboardServoComponent>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
