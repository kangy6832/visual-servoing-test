#include "robotic_arm/planning/moveit_planner.hpp"

#include <stdexcept>

namespace robotic_arm::planning {

MoveItPlanner::MoveItPlanner(rclcpp::Node::SharedPtr node, std::string planning_group)
    : node_(std::move(node)), planning_group_(std::move(planning_group)) {
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
}

void MoveItPlanner::setPlannerId(const std::string &planner_id) {
  move_group_->setPlannerId(planner_id);
}

void MoveItPlanner::setPlanningTime(double seconds) {
  move_group_->setPlanningTime(seconds);
}

moveit_msgs::msg::RobotTrajectory MoveItPlanner::planToPose(const geometry_msgs::msg::Pose &target_pose) {
  move_group_->setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_->plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("MoveIt planning to pose failed.");
  }
  return plan.trajectory_;
}

moveit_msgs::msg::RobotTrajectory MoveItPlanner::planToJointGoal(const std::vector<double> &joint_goal) {
  move_group_->setJointValueTarget(joint_goal);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_->plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("MoveIt planning to joint goal failed.");
  }
  return plan.trajectory_;
}

const std::string &MoveItPlanner::planningGroup() const { return planning_group_; }

}  // namespace robotic_arm::planning
