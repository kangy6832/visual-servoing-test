#pragma once

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/logging.hpp>

namespace trajectory_executor {
    /**
     * @class TrajectoryExecutor
     * @brief 轨迹执行器（整合动力学、KDL、Action）
     * @details 负责轨迹执行、动力学计算、Action 服务
     */
    class TrajectoryExecutor {
        public:
            explicit TrajectoryExecutor(const rclcpp::Node::SharedPtr node);
            ~TrajectoryExecutor() = default;

            // 禁止拷贝
            TrajectoryExecutor(const TrajectoryExecutor&) = delete;
            TrajectoryExecutor& operator=(const TrajectoryExecutor&) = delete;

            /**
             * @brief 初始化 KDL 与动力学求解器
             * @param urdf_xml URDF 字符串
             * @return true 初始化成功
             */
            bool initFromURDF(const std::string& urdf_xml);

            /**
             * @brief 动力学计算
             * @return 6 维动力学向量
             */
            Eigen::Vector<double, 6> dynamicCalc();

            // Action 回调
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

            // 执行状态
            bool is_execut_trajectory{false};
            bool cancle_execut{false};
            bool finished_execut{false};

        private:
            // KDL 与动力学
            KDL::Tree tree;
            KDL::Chain chain;
            std::string urdf_xml;
            rclcpp::Node::SharedPtr node;
            rclcpp::SyncParametersClient::SharedPtr robot_description_param_;

            KDL::Vector gravity;
            std::shared_ptr<KDL::ChainDynParam> dyn;
            KDL::JntSpaceInertiaMatrix M_kdl;
            KDL::JntArray C_kdl, G_kdl;
            KDL::JntArray q_kdl, dq_kdl, ddq_kdl;
    };
} // namespace trajectory_executor
