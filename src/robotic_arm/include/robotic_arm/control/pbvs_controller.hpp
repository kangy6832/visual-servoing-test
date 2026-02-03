#pragma once

#include <array>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace robotic_arm::control {

class PBVSController {
public:
  PBVSController();

  void setGains(double linear_gain, double angular_gain);
  void setMaxVelocity(double linear_limit, double angular_limit);

  geometry_msgs::msg::Twist computeTwistCommand(const geometry_msgs::msg::Pose &current,
                                                const geometry_msgs::msg::Pose &target) const;

private:
  double linear_gain_{0.5};
  double angular_gain_{0.8};
  double linear_limit_{0.2};
  double angular_limit_{0.5};

  static double clamp(double value, double limit);
};

}  // namespace robotic_arm::control
