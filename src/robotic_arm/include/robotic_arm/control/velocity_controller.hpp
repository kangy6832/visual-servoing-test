#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace robotic_arm::control {

class VelocityController {
public:
  explicit VelocityController(rclcpp::Node &node);

  void setJointNames(const std::vector<std::string> &joint_names);
  void setCommandTopic(const std::string &topic);

  void sendVelocityCommand(const std::vector<double> &velocities, double duration_sec);

private:
  rclcpp::Node &node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  std::vector<std::string> joint_names_;
};

}  // namespace robotic_arm::control
