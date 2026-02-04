#include "control_pack/trajectory_executor.hpp"

namespace trajectory_executor {

    TrajectoryExecutor::TrajectoryExecutor(const rclcpp::Node::SharedPtr node) 
        : node(node), gravity(0.0, 0.0, -9.81) {
        // 初始化参数客户端（可选）
        robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node, "robot_description");
    }

    bool TrajectoryExecutor::initFromURDF(const std::string& urdf_xml) {
        this->urdf_xml = urdf_xml;

        // 从 URDF 解析 KDL Tree
        if (!kdl_parser::treeFromString(urdf_xml, tree)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to construct KDL tree from URDF");
            return false;
        }

        // 获取指定链（示例：base_link 到 tool0）
        if (!tree.getChain("base_link", "tool0", chain)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to get KDL chain from tree");
            return false;
        }

        // 初始化动力学求解器
        dyn = std::make_shared<KDL::ChainDynParam>(chain, gravity);

        // 预分配数组
        q_kdl.resize(chain.getNrOfJoints());
        dq_kdl.resize(chain.getNrOfJoints());
        ddq_kdl.resize(chain.getNrOfJoints());
        M_kdl.resize(chain.getNrOfJoints());
        C_kdl.resize(chain.getNrOfJoints());
        G_kdl.resize(chain.getNrOfJoints());

        return true;
    }

    Eigen::Vector<double, 6> TrajectoryExecutor::dynamicCalc() {
        Eigen::Vector<double, 6> result = Eigen::Vector<double, 6>::Zero();

        if (!dyn) {
            RCLCPP_WARN(node->get_logger(), "Dynamics solver not initialized");
            return result;
        }

        // 计算动力学（假设 q_kdl, dq_kdl, ddq_kdl 已被外部填充）
        dyn->JntToMass(q_kdl, M_kdl);        // 惯性矩阵
        dyn->JntToCoriolis(q_kdl, dq_kdl, C_kdl);  // 科里奥利力
        dyn->JntToGravity(q_kdl, G_kdl);     // 重力

        // 简单示例：返回关节空间的总力矩（可按需扩展）
        for (int i = 0; i < q_kdl.rows(); ++i) {
            result(i) = M_kdl(i, i) * ddq_kdl(i) + C_kdl(i) + G_kdl(i);
        }

        return result;
    }

    // Action 回调实现（占位）
    rclcpp_action::GoalResponse TrajectoryExecutor::handle_goal(
        const rclcpp_action::GoalUUID& uuid, 
        const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(node->get_logger(), "TrajectoryExecutor: handle_goal called");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse TrajectoryExecutor::handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(node->get_logger(), "TrajectoryExecutor: handle_cancel called");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void TrajectoryExecutor::handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(node->get_logger(), "TrajectoryExecutor: handle_accepted called");
    }

} // namespace trajectory_executor
