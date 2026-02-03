#pragma once

#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

namespace robotic_arm::kinematics {

struct KinematicsResult {
  std::vector<double> joint_positions;
  double manipulability{0.0};
};

class KinematicsSolver {
public:
  explicit KinematicsSolver(std::string base_frame, std::string tip_frame);

  void setJointLimits(const std::vector<double> &min_limits,
                      const std::vector<double> &max_limits);

  std::optional<KinematicsResult> solveIK(const geometry_msgs::msg::Pose &target_pose,
                                         const std::vector<double> &seed) const;

  geometry_msgs::msg::Pose solveFK(const std::vector<double> &joint_positions) const;

  const std::string &baseFrame() const;
  const std::string &tipFrame() const;

private:
  std::string base_frame_;
  std::string tip_frame_;
  std::vector<double> min_limits_;
  std::vector<double> max_limits_;

  bool withinLimits(const std::vector<double> &joint_positions) const;
};

}  // namespace robotic_arm::kinematics
