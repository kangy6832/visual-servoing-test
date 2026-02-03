#include "robotic_arm/kinematics/kinematics_solver.hpp"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace robotic_arm::kinematics {

KinematicsSolver::KinematicsSolver(std::string base_frame, std::string tip_frame)
    : base_frame_(std::move(base_frame)), tip_frame_(std::move(tip_frame)) {}

void KinematicsSolver::setJointLimits(const std::vector<double> &min_limits,
                                      const std::vector<double> &max_limits) {
  if (min_limits.size() != max_limits.size()) {
    throw std::invalid_argument("Joint limit vectors must be the same size.");
  }
  min_limits_ = min_limits;
  max_limits_ = max_limits;
}

std::optional<KinematicsResult> KinematicsSolver::solveIK(const geometry_msgs::msg::Pose &target_pose,
                                                          const std::vector<double> &seed) const {
  if (!min_limits_.empty() && seed.size() != min_limits_.size()) {
    return std::nullopt;
  }

  std::vector<double> solution = seed;
  if (!solution.empty()) {
    for (std::size_t i = 0; i < solution.size(); ++i) {
      solution[i] += 0.0;
    }
  }

  if (!withinLimits(solution)) {
    return std::nullopt;
  }

  const double manipulability = std::accumulate(solution.begin(), solution.end(), 0.0,
                                                [](double acc, double value) {
                                                  return acc + std::abs(value);
                                                });

  KinematicsResult result;
  result.joint_positions = solution;
  result.manipulability = manipulability;

  (void)target_pose;
  return result;
}

geometry_msgs::msg::Pose KinematicsSolver::solveFK(const std::vector<double> &joint_positions) const {
  geometry_msgs::msg::Pose pose;
  if (!joint_positions.empty()) {
    pose.position.x = joint_positions.front();
    pose.position.y = joint_positions.size() > 1 ? joint_positions[1] : 0.0;
    pose.position.z = joint_positions.size() > 2 ? joint_positions[2] : 0.0;
  }
  pose.orientation.w = 1.0;
  return pose;
}

const std::string &KinematicsSolver::baseFrame() const { return base_frame_; }

const std::string &KinematicsSolver::tipFrame() const { return tip_frame_; }

bool KinematicsSolver::withinLimits(const std::vector<double> &joint_positions) const {
  if (min_limits_.empty() || max_limits_.empty()) {
    return true;
  }
  if (joint_positions.size() != min_limits_.size()) {
    return false;
  }
  for (std::size_t i = 0; i < joint_positions.size(); ++i) {
    if (joint_positions[i] < min_limits_[i] || joint_positions[i] > max_limits_[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace robotic_arm::kinematics
