#pragma once

#include <array>

namespace haul::vcu {

// Speed estimator: a two-state Kalman filter over [speed, accelerometer bias].
//
// Predict with the IMU (longitudinal specific force minus the gravity component from pitch),
// correct with whatever speed measurements arrived this cycle: undriven-axle wheel speed,
// driven-axle wheel speed (only while it isn't slipping) and GNSS speed. If one source drops
// out the others carry on; with only the IMU left it dead-reckons, and the variance grows so
// downstream logic can see the estimate getting worse.
class SpeedEstimator {
 public:
  struct Noise {
    double speed_process = 0.05;  // (m/s)^2 per s
    double bias_process = 1e-4;   // (m/s^2)^2 per s
    double no_imu_process = 1.0;  // used instead when there is no IMU: speed is a random walk
  };

  SpeedEstimator() = default;
  explicit SpeedEstimator(Noise n) : noise_(n) {}

  void reset(double speed_mps);
  // `specific_force` is what the accelerometer reads; `pitch_rad` is nose-up positive.
  void predict(double dt, double specific_force, double pitch_rad, bool imu_valid);
  // Returns the innovation (measurement minus prediction) for diagnostics.
  double update(double measured_speed, double variance);

  double speed() const { return x_[0] < 0.0 ? 0.0 : x_[0]; }
  double accel_bias() const { return x_[1]; }
  double speed_variance() const { return p_[0][0]; }
  bool initialised() const { return initialised_; }

 private:
  Noise noise_;
  std::array<double, 2> x_{0.0, 0.0};
  std::array<std::array<double, 2>, 2> p_{{{1.0, 0.0}, {0.0, 0.01}}};
  bool initialised_ = false;
};

}  // namespace haul::vcu
