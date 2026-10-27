#include "haul/vcu/speed_controller.hpp"

#include <algorithm>
#include <cmath>

#include "haul/core/units.hpp"

namespace haul::vcu {

void SetpointProfile::step(double goal, double accel_up, double accel_down, double jerk, double dt) {
  const double err = goal - speed_;
  const double accel_before = accel_;
  if (std::abs(err) < 1e-4 && std::abs(accel_) < 1e-3) {
    speed_ = goal;
    accel_ = 0.0;
    jerk_ = dt > 0.0 ? -accel_before / dt : 0.0;
    return;
  }
  // The largest acceleration from which we can still bring the acceleration back to zero, at
  // the jerk limit, exactly as we arrive at the goal. In continuous time that is
  // sqrt(2 * jerk * |err|); this is the exact discrete-time version: ramping down from
  // a = n * J (J = jerk * dt per step) covers J * dt * n (n + 1) / 2, so solve that for n.
  // (Using the continuous formula leaves a little acceleration at arrival, which then has
  // to be dropped in one step: a jerk spike that the feedforward turns into a force spike.)
  const double cap = err > 0.0 ? accel_up : accel_down;
  const double step_accel = jerk * dt;
  const double n = 0.5 * (std::sqrt(1.0 + 8.0 * std::abs(err) / (step_accel * dt)) - 1.0);
  const double wanted = std::copysign(std::min(cap, n * step_accel), err);
  accel_ = rate_limit(accel_, wanted, step_accel, step_accel);
  const double next = speed_ + accel_ * dt;
  if ((goal - next) * err <= 0.0) {
    // Arrived. On a normal approach the acceleration is down to less than one jerk step by
    // now and winds down to zero on the next step. If the goal moved and we would shoot
    // straight through it, stop dead instead: a rare discontinuity beats overshooting.
    speed_ = goal;
    if (std::abs(accel_) > step_accel) accel_ = 0.0;
  } else {
    speed_ = next;
  }
  jerk_ = dt > 0.0 ? (accel_ - accel_before) / dt : 0.0;
}

double SpeedController::max_traction(double v) const {
  return std::min(cal_->max_traction_n, cal_->max_traction_power_w / std::max(v, 0.5));
}

double SpeedController::max_retard(double v) const {
  const double power_limited = std::min(cal_->max_retard_n, cal_->max_retard_power_w / std::max(v, 0.5));
  const double fade = clamp(v / cal_->retard_fade_mps, 0.0, 1.0);
  return power_limited * fade;
}

ControlOutput SpeedController::step(const ControlInput& in) {
  ControlOutput out;
  const double m = in.mass_kg;
  const double v = in.speed_mps;

  // Holding at standstill: brakes on, nothing integrating.
  if (in.setpoint_mps <= 0.0 && v < cal_->standstill_mps) {
    reset();
    last_traction_ = 0.0;
    out.brake_pct = cal_->hold_brake_pct;
    return out;
  }

  const double sin_p = std::sin(in.pitch_rad);
  const double cos_p = std::cos(in.pitch_rad);
  const double grade_term = cal_->grade_feedforward ? sin_p : 0.0;
  out.feedforward_n = m * kGravity * (grade_term + cal_->rolling_resistance * cos_p) +
                      0.5 * cal_->air_density * cal_->drag_area_m2 * v * v;

  const double error = in.setpoint_mps - v;
  // Lead the acceleration feedforward by the drive's lag, so the force is already falling
  // as the profile eases off, instead of 0.3 s later.
  const double accel_ff = in.setpoint_accel + cal_->drive_lag_s * in.setpoint_jerk;
  const double accel_cmd = accel_ff + cal_->kp * error + cal_->ki * integral_;
  const double wanted = m * accel_cmd + out.feedforward_n;
  out.requested_n = wanted;

  // Allocate: drive motors first (they don't wear), then the friction brakes.
  double traction_limit = max_traction(v);
  if (in.slip) {
    if (tc_limit_ < 0.0) tc_limit_ = std::max(0.0, last_traction_) * cal_->slip_traction_scale;  // slip onset
    tc_limit_ *= std::max(0.0, 1.0 - cal_->slip_decay_per_s * in.dt);
  } else if (tc_limit_ >= 0.0) {
    tc_limit_ += cal_->slip_recovery_n_per_s * in.dt;
    if (tc_limit_ >= traction_limit) tc_limit_ = -1.0;  // fully recovered
  }
  if (tc_limit_ >= 0.0) traction_limit = std::min(traction_limit, tc_limit_);
  const double retard_limit = max_retard(v);
  double applied = 0.0;
  if (wanted >= 0.0) {
    out.traction_n = std::min(wanted, traction_limit);
    applied = out.traction_n;
  } else {
    out.traction_n = std::max(wanted, -retard_limit);
    const double brake_n = std::min(out.traction_n - wanted, cal_->max_brake_n);
    out.brake_pct = 100.0 * brake_n / cal_->max_brake_n;
    applied = out.traction_n - brake_n;
  }
  out.saturated = std::abs(applied - wanted) > 1.0;
  last_traction_ = out.traction_n;

  // Back-calculation anti-windup: bleed the integrator by however much the output was clipped.
  const double clipped_accel = (applied - wanted) / m;
  const double windup = cal_->ki > 0.0 ? cal_->kaw * clipped_accel / cal_->ki : 0.0;
  integral_ += in.dt * (error + windup);
  return out;
}

}  // namespace haul::vcu
