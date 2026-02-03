#include "robotic_arm/control/pbvs_controller.hpp"

#include <cmath>

namespace robotic_arm::control {

PBVSController::PBVSController() = default;

void PBVSController::setGains(double linear_gain, double angular_gain) {
  linear_gain_ = linear_gain;
  angular_gain_ = angular_gain;
}

void PBVSController::setMaxVelocity(double linear_limit, double angular_limit) {
  linear_limit_ = linear_limit;
  angular_limit_ = angular_limit;
}

geometry_msgs::msg::Twist PBVSController::computeTwistCommand(const geometry_msgs::msg::Pose &current,
                                                              const geometry_msgs::msg::Pose &target) const {
  geometry_msgs::msg::Twist twist;
  const double dx = target.position.x - current.position.x;
  const double dy = target.position.y - current.position.y;
  const double dz = target.position.z - current.position.z;

  twist.linear.x = clamp(linear_gain_ * dx, linear_limit_);
  twist.linear.y = clamp(linear_gain_ * dy, linear_limit_);
  twist.linear.z = clamp(linear_gain_ * dz, linear_limit_);

  const double qw = current.orientation.w;
  const double qx = current.orientation.x;
  const double qy = current.orientation.y;
  const double qz = current.orientation.z;
  const double tqw = target.orientation.w;
  const double tqx = target.orientation.x;
  const double tqy = target.orientation.y;
  const double tqz = target.orientation.z;

  const double ex = tqw * qx - tqx * qw - tqy * qz + tqz * qy;
  const double ey = tqw * qy + tqx * qz - tqy * qw - tqz * qx;
  const double ez = tqw * qz - tqx * qy + tqy * qx - tqz * qw;

  twist.angular.x = clamp(angular_gain_ * ex, angular_limit_);
  twist.angular.y = clamp(angular_gain_ * ey, angular_limit_);
  twist.angular.z = clamp(angular_gain_ * ez, angular_limit_);

  return twist;
}

double PBVSController::clamp(double value, double limit) {
  if (limit <= 0.0) {
    return value;
  }
  return std::max(-limit, std::min(value, limit));
}

}  // namespace robotic_arm::control
