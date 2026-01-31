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
 * 质量矩阵、科里奥利力、离心力、重力
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

            robot_interfaces::msg::Robot joints_target_positons_;
            robot_interfaces::msg::Robot joints_target_velocity_;
            robot_interfaces::msg::Robot joints_target_acceleration_;

    };

    /**
     * @class QuinticParam
     * @brief 五次多项式轨迹插值器
     * @details 用于生成平滑的关节空间轨迹，保证位置、速度、加速度的连续性
     */
    class QuinticParam{
        public:
            /**
             * @brief 构造函数
             */
            QuinticParam();

            /**
             * @brief 设置五次多项式的参数
             * @param t0 起始时间
             * @param t1 终止时间
             * @param p0 起始位置
             * @param v0 起始速度
             * @param a0 起始加速度
             * @param pt 终止位置
             * @param v1 终止速度
             * @param at 终止加速度
             */
            void set_param(
                const double t0, const double t1, const double p0, const double v0, 
                const double a0, const double pt, const double v1, const double at
            );

            /**
             * @brief 计算在给定时间t的位置值
             * @param t 当前时间
             * @return 时间t对应的位置值
             */
            double get_position(const double t);

            /**
             * @brief 计算在给定时间t的速度值
             * @param t 当前时间
             * @return 时间t对应的速度值
             */
            double get_velocity(const double t);

            /**
             * @brief 计算在给定时间t的加速度值
             * @param t 当前时间
             * @return 时间t对应的加速度值
             */
            double get_acceleration(const double t);

        private:
            double a{0.0};
            double b{0.0};
            double c{0.0};
            double d{0.0};
            double e{0.0};
            double f{0.0};
            double t0{0.0};
            double t1{0.0};

    };

    /**
     * @class ContinuousTrajectory
     * @brief 管理多段连续轨迹的执行
     * @details 用于管理和执行多段连续的五次多项式轨迹。
     *          该类接受一个由多个轨迹点组成的轨迹，在每两个相邻轨迹点之间使用五次多项式
     *          进行平滑插值，实现整个轨迹的连续执行。
     */
    class ContinuousTrajectory{
        public:
            /**
             * @brief 构造函数
             * @details 初始化轨迹管理器，重置所有内部状态
             */
            explicit ContinuousTrajectory();

            /**
             * @brief 获取指定时间的期望轨迹点
             * @param time 当前时间
             * @param[out] output 输出的轨迹点，位置、速度、加速度
             * @return bool
             */
            bool get_target(const rclcpp::Time& time, trajectory_msgs::msg::JointTrajectoryPoint& output);

            /**
             * @brief 开始轨迹跟踪
             * @param now 轨迹开始执行的基准时间
             */
            void start_track(rclcpp::Time now);

            /**
             * @brief 设置要执行的轨迹
             * @param trajectory 要执行的关节轨迹
             */
            void set_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
            
        private:
            QuinticParam line[6];
            size_t cur_index;
            rclcpp::Time start_time;
            trajectory_msgs::msg::JointTrajectory trajectory;
    };

} // namespace VisualServoingController


#endif //  !VISUAL_SERVOING_CONTROLLER_HPP