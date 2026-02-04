/**
 * @file robot_pose_polynomial.hpp
 * @brief 五次多项式
 */


#ifndef ROBOT_POSE_POLYOMIAL_HPP
#define ROBOT_POSE_POLYOMIAL_HPP

#include <Eigen/Dense>
#include <rclcpp/time.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace RobotPosePolynomial{
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
             * @param[in] time 当前时间
             * @param[out] output 输出的轨迹点，位置、速度、加速度
             * @return 
             *  - true : 成功获取目标轨迹点
             *  - false : 时间超出轨迹范围或轨迹未初始化
             */
            bool get_target(const rclcpp::Time& time, trajectory_msgs::msg::JointTrajectoryPoint& output);

            /**
             * @brief 开始轨迹跟踪
             * @param[in] now 轨迹开始执行的基准时间
             */
            void start_track(rclcpp::Time now);

            /**
             * @brief 设置要执行的轨迹
             * @param[in] trajectory 要执行的关节轨迹
             */
            void set_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
            
        private:
            QuinticParam line[6];
            size_t cur_index;
            rclcpp::Time start_time;
            trajectory_msgs::msg::JointTrajectory trajectory;
            bool initialized_{false};

            void init_segment(std::size_t index);
    };




} // namespace RobotPosePolynomial

#endif // !ROBOT_POSE_POLYOMIAL_HPP