#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <robot_interfaces/action/catch.hpp>
#include <robot_interfaces/action/detail/catch__struct.hpp>

class ActionTestNode : public rclcpp::Node{
    private:
        rclcpp_action::
    public:
        using Catch = robot_interfaces::action::Catch;
        using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;

        ActionTestNode() : Node("action_test_node"){
            RCLCPP_INFO(this->get_logger(), "ActionTestNode 启动，准备连接 Action Server...");


        }
};
