#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot.hpp"
#include <iostream>

class Listener : public rclcpp::Node
{
public:
    Listener() : Node("listener")
    {
        subscription_ = this->create_subscription<robot_interfaces::msg::Robot>(
            "/myjoints_state", 10,
            [this](const robot_interfaces::msg::Robot::SharedPtr msg) {
                std::cout << "=== /myjoints_state 话题内容 ===" << std::endl;
                for (int i = 0; i < 6; i++) {
                    std::cout << "关节 " << i << ": "
                              << "rad=" << msg->joints[i].rad << ", "
                              << "omega=" << msg->joints[i].omega << ", "
                              << "torque=" << msg->joints[i].torque << ", "
                              << "alpha=" << msg->joints[i].alpha << std::endl;
                }
                std::cout << "========================" << std::endl;
                rclcpp::shutdown(); // 只接收一次就退出
            });
    }

private:
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Listener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
