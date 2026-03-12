#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <robot_interfaces/action/catch.hpp>
#include <robot_interfaces/action/detail/catch__struct.hpp>
#include <robot_interfaces/msg/visual_image.hpp>

using Catch = robot_interfaces::action::Catch;
using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;
using VisualImage = robot_interfaces::msg::VisualImage;

class ActionTestNode : public rclcpp::Node{
    private:
        rclcpp_action::Client<Catch>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<VisualImage>::SharedPtr visual_image_;

        bool goal_sent_ = false;

    public:
        ActionTestNode() : Node("action_test_node"){
            RCLCPP_INFO(this->get_logger(), "ActionTestNode 启动，准备连接 Action Server...");

            timer_ = this->create_wall_timer(
                std::chrono::seconds(0.1), 
                std::bind()
            );
        }
    
    private:
        void send_goal(){
            if (goal_sent_) return;

            goal_sent_ = true;

            
        }
};
