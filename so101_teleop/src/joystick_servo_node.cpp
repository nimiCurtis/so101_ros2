#include <rclcpp/rclcpp.hpp>
#include "so101_teleop/joystick_servo_component.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<moveit_servo::JoyToServoComponent>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
