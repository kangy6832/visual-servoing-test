#pragma once

#include <vector>

namespace robotic_arm::trajectory {

struct QuinticSegment {
  double t0{0.0};
  double t1{0.0};
  std::vector<double> coefficients;
};

class QuinticTrajectory {
public:
  QuinticTrajectory() = default;

  void setWaypoints(const std::vector<double> &times,
                    const std::vector<std::vector<double>> &positions,
                    const std::vector<std::vector<double>> &velocities,
                    const std::vector<std::vector<double>> &accelerations);

  std::vector<double> sample(double time) const;
  std::vector<double> sampleVelocity(double time) const;
  std::vector<double> sampleAcceleration(double time) const;

  std::size_t dof() const;
  const std::vector<QuinticSegment> &segments() const;

private:
  std::vector<double> times_;
  std::vector<std::vector<double>> positions_;
  std::vector<std::vector<double>> velocities_;
  std::vector<std::vector<double>> accelerations_;
  std::vector<QuinticSegment> segments_;

  std::vector<double> solveSegment(std::size_t index) const;
  std::vector<double> evaluateSegment(const std::vector<double> &coeffs, double dt) const;
  std::vector<double> evaluateSegmentVelocity(const std::vector<double> &coeffs, double dt) const;
  std::vector<double> evaluateSegmentAcceleration(const std::vector<double> &coeffs, double dt) const;
};

}  // namespace robotic_arm::trajectory
