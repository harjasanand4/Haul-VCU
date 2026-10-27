#include "haul/sim/truck_model.hpp"

#include <algorithm>
#include <cmath>

#include "haul/core/units.hpp"

namespace haul::sim {

void TruckModel::reset(double speed_mps, double position_m) {
  s_ = TruckState{};
  s_.speed_mps = std::max(0.0, speed_mps);
  s_.position_m = position_m;
}

double TruckModel::traction_limit(double v) const {
  return std::min(p_.max_traction_n, p_.max_traction_power_w / std::max(v, 0.5));
}

double TruckModel::retard_limit(double v) const {
  return std::min(p_.max_retard_n, p_.max_retard_power_w / std::max(v, 0.5)) * clamp(v / p_.retard_fade_mps, 0.0, 1.0);
}

void TruckModel::step(double dt, double drive_request_n, double brake_request_pct, double grade_rad, double road_mu) {
  const double m = mass();
  const double v = s_.speed_mps;
  s_.grade_rad = grade_rad;
  s_.road_mu = road_mu;

  // Actuators: clip to what the hardware can do, then first-order lag.
  double drive_target = clamp(drive_request_n, -retard_limit(v), traction_limit(v));
  if (drive_failed_) drive_target = 0.0;
  s_.drive_force_n += (drive_target - s_.drive_force_n) * (1.0 - std::exp(-dt / p_.traction_tau_s));
  const double brake_target = clamp(brake_request_pct, 0.0, 100.0) / 100.0 * p_.max_brake_n;
  s_.brake_force_n += (brake_target - s_.brake_force_n) * (1.0 - std::exp(-dt / p_.brake_tau_s));

  // Grip on the driven axle. Ask for more than the road can take and the wheels spin.
  const double grip = road_mu * m * p_.rear_axle_share * kGravity * std::cos(grade_rad);
  if (std::abs(s_.drive_force_n) > grip) {
    s_.transmitted_force_n = std::copysign(grip, s_.drive_force_n);
    if (s_.drive_force_n > 0.0) s_.slip = std::min(0.6, s_.slip + dt * 4.0 * (s_.drive_force_n - grip) / grip);
  } else {
    s_.transmitted_force_n = s_.drive_force_n;
    s_.slip = std::max(0.0, s_.slip - dt * 1.5);
  }

  const double f_grade = m * kGravity * std::sin(grade_rad);
  const double f_roll = p_.rolling_resistance * m * kGravity * std::cos(grade_rad);
  const double f_aero = 0.5 * p_.air_density * p_.drag_area_m2 * v * v;

  double net = 0.0;
  if (v > 1e-3) {
    net = s_.transmitted_force_n - s_.brake_force_n - f_grade - f_roll - f_aero;
  } else {
    // Static friction: brakes and rolling resistance hold the truck until the drive (or
    // gravity) overcomes them. No rolling backwards: the truck sticks instead.
    const double push = s_.transmitted_force_n - f_grade;
    const double hold = s_.brake_force_n + f_roll;
    net = push > hold ? push - hold : 0.0;
  }
  double v_next = v + net / m * dt;
  if (v_next < 0.0) v_next = 0.0;
  s_.accel_mps2 = (v_next - v) / dt;
  s_.speed_mps = v_next;
  s_.position_m += 0.5 * (v + v_next) * dt;
}

}  // namespace haul::sim
