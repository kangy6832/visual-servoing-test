/**
 * @file visual_servoing_cnotroller.cpp
 * @brief 简单控制器
 * @date 2026-1-31
 */

 #include "visual_servoing_controller.hpp"
 #include "robot_interfaces/msg/robot.hpp"
 #include <controller_interface/controller_interface.hpp>

 /**
  * @brief 提供插件注册宏
  * 
  * PLUGINLIB_EXPORT_CLASS
  */
 #include <pluginlib/class_list_macros.hpp>

 using robot_interfaces::msg::Robot;

 namespace VisualServoingController{

    /**
     * @brief 构造函数
     */
    Controller::Controller() = default;

    /**
     * @brief 初始化发布者、订阅者、关节名称
     */
    controller_interface::CallbackReturn Controller::on_init(){
        state_publisher_ = get_node()->create_publisher<Robot>("robot_status", 10);
        target_subscriber_ = get_node()->create_subscription<Robot>(
            "robot_target", 10, [this](const Robot& msg){joints_target_positons_ = msg;});
        joints_name_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    }

    controller_interface::CallbackReturn Controller::on_configure(const rclcpp_lifecycle::State& previous_state){
        (void)previous_state;
        return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Controller::on_activate(const rclcpp_lifecycle::State& previous_state){
        (void)previous_state;
        return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Controller::on_deactivate(const rclcpp_lifecycle::State& previous_state){
        (void)previous_state;
        return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type Controller::update(const rclcpp::Times& time, const rclcpp::Duration& period){
        auto joints_num = joints_name_.size();
        robot_interfaces::msg::Robot state_msg;
    }



 /**
 * @brief ROS插件库中的核心库
 * 
 * 将C++类注册为ROS插件,使controller_manager能动态啊加载此控制器
 * PLUGINLIB_EXPORT_CLASS(ClassName, BaseClassName)
 */
PLUGINLIB_EXPORT_CLASS(VisualServoingController::Controller, controller_interface::ControllerInterface)

} // namespace VisualServoingController