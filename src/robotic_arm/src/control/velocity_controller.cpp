#include "robotic_arm/control/velocity_controller.hpp"

#include <chrono>
#include <stdexcept>

namespace robotic_arm::control {

VelocityController::VelocityController(rclcpp::Node &node) : node_(node) {
  setCommandTopic("/joint_velocity_controller/joint_trajectory");
}

void VelocityController::setJointNames(const std::vector<std::string> &joint_names) {
  joint_names_ = joint_names;
}

void VelocityController::setCommandTopic(const std::string &topic) {
  publisher_ = node_.create_publisher<trajectory_msgs::msg::JointTrajectory>(topic, 10);
}

void VelocityController::sendVelocityCommand(const std::vector<double> &velocities, double duration_sec) {
  if (joint_names_.empty()) {
    throw std::runtime_error("Joint names must be set before sending commands.");
  }
  if (velocities.size() != joint_names_.size()) {
    throw std::runtime_error("Velocity command size does not match joint count.");
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.velocities = velocities;
  const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(duration_sec));
  point.time_from_start.sec = static_cast<int32_t>(duration.count() / 1000000000LL);
  point.time_from_start.nanosec = static_cast<uint32_t>(duration.count() % 1000000000LL);

  trajectory.points.push_back(point);
  publisher_->publish(trajectory);
}

}  // namespace robotic_arm::control
