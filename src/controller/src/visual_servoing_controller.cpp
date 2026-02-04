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

        RCLCPP_INFO(this->get_node()->get_logger(),"控制器初始化");

        trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
            get_node(), "robotic_arm_controller/arm_command", std::bind(&Controller::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Controller::handle_cancel, this, std::placeholders::_1), std::bind(&Controller::handle_accepted, this, std::placeholders::_1)
        );

        result_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        feedback_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
        joints_name_ = {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};

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

    controller_interface::return_type Controller::update(const rclcpp::Time& time, const rclcpp::Duration& period){
        auto joints_num = joints_name_.size();
        robot_interfaces::msg::Robot state_msg;

        for(size_t i = 0 ; i < joints_num ; ++i){
            state_msg.joints[i].rad = state_interfaces_[i * 3 + 0].get_value();
            state_msg.joints[i].omega = state_interfaces_[i * 3 + 1].get_value();
            state_msg.joints[i].torque = state_interfaces_[i * 3 + 2].get_value();
            state_msg.joints[i].alpha = state_interfaces_[i * 3 + 3].get_value();
        }

        state_publisher_->publish(state_msg);

        for (size_t i = 0 ; i < joints_num ; ++i){
            command_interfaces_[i * 3 + 0].set_value((double)joints_target.joints[i].rad);
            command_interfaces_[i * 3 + 1].set_value((double)joints_target.joints[i].omega);
            command_interfaces_[i * 3 + 2].set_value((double)joints_target.joints[i].torque);
        }

    }

    controller_interface::InterfaceConfiguration Controller::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration cfg;
        cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto& name : joints_name_){
            cfg.names.push_back(name + "/position");
            cfg.names.push_back(name + "/velocity");
            cfg.names.push_back(name + "/effort");
        }
        return cfg;
    }

    controller_interface::InterfaceConfiguration Controller::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration cfg;
        cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto& name : joints_name_) {
            cfg.names.push_back(name + "/position");
            cfg.names.push_back(name + "/velocity");
            cfg.names.push_back(name + "/effort");
        }
        return cfg;
    }


 /**
 * @brief ROS插件库中的核心库
 * 
 * 将C++类注册为ROS插件,使controller_manager能动态啊加载此控制器
 * PLUGINLIB_EXPORT_CLASS(ClassName, BaseClassName)
 */
PLUGINLIB_EXPORT_CLASS(VisualServoingController::Controller, controller_interface::ControllerInterface)

} // namespace VisualServoingController