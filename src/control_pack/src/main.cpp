#include "robotic_task.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("control_pack_node");
    auto arm_handle = std::make_shared<RoboticTask>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}