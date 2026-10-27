#include "haul/vcu/estimator.hpp"

#include <cmath>

#include "haul/core/units.hpp"

namespace haul::vcu {

void SpeedEstimator::reset(double speed_mps) {
  x_ = {speed_mps, 0.0};
  p_ = {{{0.5, 0.0}, {0.0, 0.01}}};
  initialised_ = true;
}

void SpeedEstimator::predict(double dt, double specific_force, double pitch_rad, bool imu_valid) {
  if (!initialised_ || dt <= 0.0) return;
  double q_v = noise_.speed_process;
  if (imu_valid) {
    const double accel = specific_force - kGravity * std::sin(pitch_rad) - x_[1];
    x_[0] += accel * dt;
  } else {
    q_v = noise_.no_imu_process;
  }
  // P = F P F^T + Q with F = [[1, -dt], [0, 1]] (the bias enters the speed with a minus sign).
  const double p00 = p_[0][0], p01 = p_[0][1], p11 = p_[1][1];
  const double n00 = p00 - 2.0 * dt * p01 + dt * dt * p11 + q_v * dt;
  const double n01 = p01 - dt * p11;
  const double n11 = p11 + noise_.bias_process * dt;
  p_ = {{{n00, n01}, {n01, n11}}};
}

double SpeedEstimator::update(double measured_speed, double variance) {
  if (!initialised_) {
    reset(measured_speed);
    return 0.0;
  }
  const double innovation = measured_speed - x_[0];
  const double s = p_[0][0] + variance;
  const double k0 = p_[0][0] / s;
  const double k1 = p_[1][0] / s;
  x_[0] += k0 * innovation;
  x_[1] += k1 * innovation;
  const double p00 = p_[0][0], p01 = p_[0][1], p11 = p_[1][1];
  p_ = {{{(1.0 - k0) * p00, (1.0 - k0) * p01}, {(1.0 - k0) * p01, p11 - k1 * p01}}};
  return innovation;
}

}  // namespace haul::vcu
