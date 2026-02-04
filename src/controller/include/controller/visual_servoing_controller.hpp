#ifndef VISUAL_SERVOING_CONTROLLER_HPP
#define VISUAL_SERVOING_CONTROLLER_HPP

/**
 * @brief Eigen 核心模块
 */
#include <Eigen/Dense>

/**
 * @brief 机器人关节轨迹跟随的Ation接口
 * 
 * FollowJointTrajectory_Goal - 目标轨迹（关节位置、速度、加速度序列）
 * FollowJointTrajectory_Result - 执行结果（成功/失败状态）
 * FollowJointTrajectory_Feedback - 执行反馈（当前状态、剩余时间等）
 */
#include <control_msgs/action/follow_joint_trajectory.hpp> 

/**
 * @brief 控制器接口基类
 * 
 * 定义 ControllerInterface 基类
 * 提供控制器生命周期管理方法
 */
#include <controller_interface/controller_interface.hpp>

/**
 * @brief 控制器基础接口
 */
#include <controller_interface/controller_interface_base.hpp>

/**
 * @brief 访问硬件控制接口
 */
#include <hardware_interface/loaned_command_interface.hpp>

/**
 * @brief 硬件状态接受接口
 */
#include <hardware_interface/loaned_state_interface.hpp>

/**
 * @brief 提供关节轨迹插值算法
 */
#include <joint_trajectory_controller/interpolation_methods.hpp>

#include <kdl/chain.hpp>

/**
 * @brief 机器人链的动力学计算
 * 
 * 质量矩阵、科里奥利力、离心力、重力矩阵
 */
#include <kdl/chaindynparam.hpp>

/**
 * @brief 正向运动学位置求解器
 */
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/frames.hpp>

/**
 * @brief 关节空间数组定义
 */
#include <kdl/jntarray.hpp>

#include <kdl/tree.hpp>

/**
 * @brief URDF->KDL结构
 */
#include <kdl_parser/kdl_parser.hpp>
#include <memory>

/**
 * @brief 插件类注册宏
 * 
 * 将控制器注册为ROS2插件
 */
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

/**
 * @brief 关节轨迹消息
 */
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

#include <robot_interfaces/msg/robot.hpp>

using robot_interfaces::msg::Robot;

namespace VisualServoingController{
    class Controller : public controller_interface::ControllerInterface{
        public:
            /**
             * @brief 构造函数 
             */
            Controller();

            /** 
             * @brief 初始化控制器
             * @return controller_interface::CallbackReturn ROS2控制框架中控制器生命周期的回调返回类型
             */
            controller_interface::CallbackReturn on_init() override;

            /**
             * @brief Unconfigured->Inactive
             * @param ROS2生命周期状态对象
             * @return 配置成功
             */
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

            /**
             * @brief Inactive->Active
             * @param 生命周期状态
             * @return 配置成功
             */
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

            /**
             * @brief Active->Inactive
             * @param 生命周期状态
             * @return successful
             */
            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            /**
             * @brief 实时更新函数，Active状态时定期调用
             * @param time 当前控制周期开始的时间戳
             * @param period 当前控制周期的持续时间
             * @return 更新操作的结果状态
             */
            controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

            /**
             * @brief 命令接口配置声明函数，声明需从硬件获取的命令接口
             * @return 需要的接口
             */
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            /**
             * @brief 状态接口配置声明函数
             * @return 控制器需要读取的状态接口
             */
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            
        private:
            rclcpp::Publisher<Robot>::SharedPtr state_publisher_;
            rclcpp::Subscription<Robot>::SharedPtr target_subscriber_;
            std::vector<std::string> joints_name_;

            robot_interfaces::msg::Robot joints_target;

            rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_server_;

            control_msgs::action::FollowJointTrajectory::Result::SharedPtr result_msg;
            control_msgs::action::FollowJointTrajectory::Feedback::SharedPtr feedback_msg;
            KDL::JntArray q_kdl, dq_kdl, ddq_kdl;
            KDL::JntSpaceInertiaMatrix M_kdl;
            



            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID& uuid, 
                const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
            );
            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
            );
            void handle_accepted(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
            );
    };


    

} // namespace VisualServoingController


#endif //  !VISUAL_SERVOING_CONTROLLER_HPP