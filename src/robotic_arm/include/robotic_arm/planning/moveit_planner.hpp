#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace robotic_arm::planning {

class MoveItPlanner {
public:
  MoveItPlanner(rclcpp::Node::SharedPtr node, std::string planning_group);

  void setPlannerId(const std::string &planner_id);
  void setPlanningTime(double seconds);

  moveit_msgs::msg::RobotTrajectory planToPose(const geometry_msgs::msg::Pose &target_pose);
  moveit_msgs::msg::RobotTrajectory planToJointGoal(const std::vector<double> &joint_goal);

  const std::string &planningGroup() const;

private:
  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
};

}  // namespace robotic_arm::planning
