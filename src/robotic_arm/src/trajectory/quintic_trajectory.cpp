#include "robotic_arm/trajectory/quintic_trajectory.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotic_arm::trajectory {

void QuinticTrajectory::setWaypoints(const std::vector<double> &times,
                                     const std::vector<std::vector<double>> &positions,
                                     const std::vector<std::vector<double>> &velocities,
                                     const std::vector<std::vector<double>> &accelerations) {
  if (times.size() < 2) {
    throw std::invalid_argument("At least two time points are required.");
  }
  if (positions.size() != times.size() || velocities.size() != times.size() ||
      accelerations.size() != times.size()) {
    throw std::invalid_argument("Waypoint vectors must align with time points.");
  }
  const auto dof_count = positions.front().size();
  if (dof_count == 0) {
    throw std::invalid_argument("Waypoint positions cannot be empty.");
  }
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (positions[i].size() != dof_count || velocities[i].size() != dof_count ||
        accelerations[i].size() != dof_count) {
      throw std::invalid_argument("All waypoint vectors must share the same DOF.");
    }
    if (i > 0 && times[i] <= times[i - 1]) {
      throw std::invalid_argument("Time points must be strictly increasing.");
    }
  }

  times_ = times;
  positions_ = positions;
  velocities_ = velocities;
  accelerations_ = accelerations;

  segments_.clear();
  segments_.reserve(times_.size() - 1);
  for (std::size_t i = 0; i + 1 < times_.size(); ++i) {
    QuinticSegment segment;
    segment.t0 = times_[i];
    segment.t1 = times_[i + 1];
    segment.coefficients = solveSegment(i);
    segments_.push_back(segment);
  }
}

std::vector<double> QuinticTrajectory::sample(double time) const {
  if (segments_.empty()) {
    return {};
  }
  const auto clamped_time = std::clamp(time, segments_.front().t0, segments_.back().t1);
  const auto it = std::find_if(segments_.begin(), segments_.end(),
                               [clamped_time](const QuinticSegment &seg) {
                                 return clamped_time >= seg.t0 && clamped_time <= seg.t1;
                               });
  const auto &segment = (it == segments_.end()) ? segments_.back() : *it;
  return evaluateSegment(segment.coefficients, clamped_time - segment.t0);
}

std::vector<double> QuinticTrajectory::sampleVelocity(double time) const {
  if (segments_.empty()) {
    return {};
  }
  const auto clamped_time = std::clamp(time, segments_.front().t0, segments_.back().t1);
  const auto it = std::find_if(segments_.begin(), segments_.end(),
                               [clamped_time](const QuinticSegment &seg) {
                                 return clamped_time >= seg.t0 && clamped_time <= seg.t1;
                               });
  const auto &segment = (it == segments_.end()) ? segments_.back() : *it;
  return evaluateSegmentVelocity(segment.coefficients, clamped_time - segment.t0);
}

std::vector<double> QuinticTrajectory::sampleAcceleration(double time) const {
  if (segments_.empty()) {
    return {};
  }
  const auto clamped_time = std::clamp(time, segments_.front().t0, segments_.back().t1);
  const auto it = std::find_if(segments_.begin(), segments_.end(),
                               [clamped_time](const QuinticSegment &seg) {
                                 return clamped_time >= seg.t0 && clamped_time <= seg.t1;
                               });
  const auto &segment = (it == segments_.end()) ? segments_.back() : *it;
  return evaluateSegmentAcceleration(segment.coefficients, clamped_time - segment.t0);
}

std::size_t QuinticTrajectory::dof() const {
  return positions_.empty() ? 0 : positions_.front().size();
}

const std::vector<QuinticSegment> &QuinticTrajectory::segments() const {
  return segments_;
}

std::vector<double> QuinticTrajectory::solveSegment(std::size_t index) const {
  const double t0 = times_[index];
  const double t1 = times_[index + 1];
  const double dt = t1 - t0;
  const auto dof_count = positions_[index].size();
  std::vector<double> coefficients;
  coefficients.reserve(dof_count * 6);

  for (std::size_t i = 0; i < dof_count; ++i) {
    const double p0 = positions_[index][i];
    const double v0 = velocities_[index][i];
    const double a0 = accelerations_[index][i];
    const double p1 = positions_[index + 1][i];
    const double v1 = velocities_[index + 1][i];
    const double a1 = accelerations_[index + 1][i];

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;

    const double c0 = p0;
    const double c1 = v0;
    const double c2 = a0 / 2.0;
    const double c3 = (20.0 * (p1 - p0) - (8.0 * v1 + 12.0 * v0) * dt - (3.0 * a0 - a1) * dt2) /
                      (2.0 * dt3);
    const double c4 = (30.0 * (p0 - p1) + (14.0 * v1 + 16.0 * v0) * dt + (3.0 * a0 - 2.0 * a1) * dt2) /
                      (2.0 * dt4);
    const double c5 = (12.0 * (p1 - p0) - (6.0 * v1 + 6.0 * v0) * dt - (a0 - a1) * dt2) /
                      (2.0 * dt5);

    coefficients.insert(coefficients.end(), {c0, c1, c2, c3, c4, c5});
  }

  return coefficients;
}

std::vector<double> QuinticTrajectory::evaluateSegment(const std::vector<double> &coeffs, double dt) const {
  const std::size_t dof_count = coeffs.size() / 6;
  std::vector<double> output(dof_count, 0.0);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double dt5 = dt4 * dt;
  for (std::size_t i = 0; i < dof_count; ++i) {
    const std::size_t base = i * 6;
    output[i] = coeffs[base] + coeffs[base + 1] * dt + coeffs[base + 2] * dt2 +
                coeffs[base + 3] * dt3 + coeffs[base + 4] * dt4 + coeffs[base + 5] * dt5;
  }
  return output;
}

std::vector<double> QuinticTrajectory::evaluateSegmentVelocity(const std::vector<double> &coeffs, double dt) const {
  const std::size_t dof_count = coeffs.size() / 6;
  std::vector<double> output(dof_count, 0.0);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  for (std::size_t i = 0; i < dof_count; ++i) {
    const std::size_t base = i * 6;
    output[i] = coeffs[base + 1] + 2.0 * coeffs[base + 2] * dt + 3.0 * coeffs[base + 3] * dt2 +
                4.0 * coeffs[base + 4] * dt3 + 5.0 * coeffs[base + 5] * dt4;
  }
  return output;
}

std::vector<double> QuinticTrajectory::evaluateSegmentAcceleration(const std::vector<double> &coeffs, double dt) const {
  const std::size_t dof_count = coeffs.size() / 6;
  std::vector<double> output(dof_count, 0.0);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  for (std::size_t i = 0; i < dof_count; ++i) {
    const std::size_t base = i * 6;
    output[i] = 2.0 * coeffs[base + 2] + 6.0 * coeffs[base + 3] * dt +
                12.0 * coeffs[base + 4] * dt2 + 20.0 * coeffs[base + 5] * dt3;
  }
  return output;
}

}  // namespace robotic_arm::trajectory
