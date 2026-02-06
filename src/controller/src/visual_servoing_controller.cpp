/**
 * @file visual_servoing_controller.cpp
 * @brief 简单控制器
 * @date 2026-1-31
 */

 #include "controller/visual_servoing_controller.hpp"
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

        RCLCPP_INFO(this->get_node()->get_logger(),"视觉伺服控制器初始化开始");

        trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
            get_node(), "robotic_arm_controller/arm_command", std::bind(&Controller::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Controller::handle_cancel, this, std::placeholders::_1), std::bind(&Controller::handle_accepted, this, std::placeholders::_1)
        );

        result_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        feedback_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
        joints_name_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        // 初始化joints_target（固定6个关节）
        for (size_t i = 0; i < 6; ++i) {
            joints_target.joints[i].rad = 0.0;
            joints_target.joints[i].omega = 0.0;
            joints_target.joints[i].torque = 0.0;
            // joints_target.joints[i].alpha = 0.0;
        }

        RCLCPP_INFO(this->get_node()->get_logger(),"视觉伺服控制器初始化完成");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Controller::on_configure(const rclcpp_lifecycle::State& previous_state){
        (void)previous_state;

        auto node = this->get_node();
        // 初始化状态发布者
        state_publisher_ = node->create_publisher<robot_interfaces::msg::Robot>("robot_state", 10);

        // 初始化目标命令订阅者
        target_subscriber_ = node->create_subscription<robot_interfaces::msg::Robot>(
            "robot_target", 10, 
            [this](const robot_interfaces::msg::Robot::SharedPtr msg){
                // 复制目标值
                for (size_t i = 0; i < 6; ++i) {
                    this->joints_target.joints[i] = msg->joints[i];
                }
            }
        );

        // 初始化action server 
        trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
            node, "robotic_arm_controller/arm_command", 
            std::bind(&Controller::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Controller::handle_cancel, this, std::placeholders::_1),
            std::bind(&Controller::handle_accepted, this, std::placeholders::_1)
        );

        result_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        feedback_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
        
        RCLCPP_INFO(node->get_logger(), "控制器配置成功");

        return controller_interface::CallbackReturn::SUCCESS;
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
        (void)time;
        (void)period;
        
        const size_t joints_num = 6;

        // 安全检查
        if(state_interfaces_.size() < joints_num * 3 || command_interfaces_.size() < joints_num * 3){
            RCLCPP_ERROR(get_node()->get_logger(), 
                    "接口数量不足。状态接口: %zu, 命令接口: %zu, 需要各 %zu",
                    state_interfaces_.size(), command_interfaces_.size(), joints_num * 3);

            return controller_interface::return_type::ERROR;
        }

        // 发布状态
        if(state_publisher_){
            robot_interfaces::msg::Robot state_msg;
            

            for(size_t i = 0 ; i < joints_num ; ++i){
                state_msg.joints[i].rad = state_interfaces_[i * 3 + 0].get_value();
                state_msg.joints[i].omega = state_interfaces_[i * 3 + 1].get_value();
                state_msg.joints[i].torque = state_interfaces_[i * 3 + 2].get_value();
                
            }

            state_publisher_->publish(state_msg);
        }

        // 发送控制命令
        for (size_t i = 0 ; i < joints_num ; ++i){
            command_interfaces_[i * 3 + 0].set_value((double)joints_target.joints[i].rad);
            command_interfaces_[i * 3 + 1].set_value((double)joints_target.joints[i].omega);
            command_interfaces_[i * 3 + 2].set_value((double)joints_target.joints[i].torque);
        }

        return controller_interface::return_type::OK;
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


    rclcpp_action::GoalResponse Controller::handle_goal(
        const rclcpp_action::GoalUUID& uuid, 
        const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
    ) {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(get_node()->get_logger(), "Received trajectory goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse Controller::handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
    ) {
        (void)goal_handle;
        RCLCPP_INFO(get_node()->get_logger(), "Received trajectory cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Controller::handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
    ) {
        RCLCPP_INFO(get_node()->get_logger(), "Trajectory goal accepted");
        
        // Execute trajectory in a separate thread to avoid blocking
        std::thread{[this, goal_handle]() {
            rclcpp::Rate rate(100); // 100Hz control rate
            auto start_time = this->get_node()->now();
            
            for (size_t i = 0; i < goal_handle->get_goal()->trajectory.points.size(); ++i) {
                if (goal_handle->is_canceling()) {
                    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
                    result->error_code = control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL;
                    goal_handle->canceled(result);
                    return;
                }
                
                auto& point = goal_handle->get_goal()->trajectory.points[i];
                
                // Set joint targets
                for (size_t j = 0; j < point.positions.size() && j < 6; ++j) {
                    joints_target.joints[j].rad = point.positions[j];
                    if (j < point.velocities.size()) {
                        joints_target.joints[j].omega = point.velocities[j];
                    }
                    if (j < point.effort.size()) {
                        joints_target.joints[j].torque = point.effort[j];
                    }
                }
                
                // Send feedback
                feedback_msg->joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
                feedback_msg->actual.positions.resize(6);
                feedback_msg->actual.velocities.resize(6);
                for (int k = 0; k < 6; ++k) {
                    feedback_msg->actual.positions[k] = joints_target.joints[k].rad;
                    feedback_msg->actual.velocities[k] = joints_target.joints[k].omega;
                }
                goal_handle->publish_feedback(feedback_msg);
                
                // Wait until trajectory point time
                auto target_time = start_time + point.time_from_start;
                while (this->get_node()->now() < target_time && !goal_handle->is_canceling()) {
                    rate.sleep();
                }
            }
            
            // Mark goal as succeeded
            auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
            result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_node()->get_logger(), "Trajectory execution completed successfully");
        }}.detach();
    }

 /**
 * @brief ROS插件库中的核心库
 * 
 * 将C++类注册为ROS插件,使controller_manager能动态啊加载此控制器
 * PLUGINLIB_EXPORT_CLASS(ClassName, BaseClassName)
 */
PLUGINLIB_EXPORT_CLASS(VisualServoingController::Controller, controller_interface::ControllerInterface)

} // namespace VisualServoingController