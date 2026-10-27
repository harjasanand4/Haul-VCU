#pragma once

#include "haul/vcu/calibration.hpp"

namespace haul::vcu {

// Longitudinal speed controller.
//
//   force = mass * (a_ref + kp * error + ki * integral) + grade + rolling resistance + drag
//
// The PI terms are in acceleration units and multiplied by the current mass, so the same
// gains behave the same whether the truck is empty (165 t) or loaded (390 t): gain scheduling
// by mass. The feedforward cancels the known road load, which keeps the PI part small and
// makes ramp entries smooth. The total force is split across the drive motors (traction or
// electric retarding, both power-limited) and the service brakes, and the integrator uses
// back-calculation anti-windup so it doesn't wind up while the drive is at its power limit.
//
// Traction control: when the VCU reports wheel slip, the drive force is capped below what was
// being applied when the wheels broke loose, and the cap keeps shrinking until they grip again.
// Afterwards force comes back at a limited rate, so the truck doesn't immediately spin again.
struct ControlInput {
  double dt = 0.01;
  double setpoint_mps = 0.0;
  double setpoint_accel = 0.0;  // feedforward acceleration from the setpoint profile
  double setpoint_jerk = 0.0;   // its rate of change, used to lead the drive's lag
  double speed_mps = 0.0;
  double mass_kg = 165'000.0;
  double pitch_rad = 0.0;
  bool slip = false;
};

struct ControlOutput {
  double traction_n = 0.0;   // signed: + drives, - electric retarding
  double brake_pct = 0.0;    // service brake request 0..100
  double requested_n = 0.0;  // total force before allocation
  double feedforward_n = 0.0;
  bool saturated = false;
};

// Speed setpoint profile with limited acceleration and jerk (an S-curve). The truck is told to
// change speed smoothly, and the profile's acceleration doubles as the controller's
// feedforward, which is most of why it doesn't overshoot.
class SetpointProfile {
 public:
  void reset(double speed_mps) {
    speed_ = speed_mps;
    accel_ = 0.0;
    jerk_ = 0.0;
  }
  void step(double goal, double accel_up, double accel_down, double jerk, double dt);
  double speed() const { return speed_; }
  double accel() const { return accel_; }
  double jerk() const { return jerk_; }

 private:
  double speed_ = 0.0;
  double accel_ = 0.0;
  double jerk_ = 0.0;
};

class SpeedController {
 public:
  explicit SpeedController(const Calibration& cal) : cal_(&cal) {}

  ControlOutput step(const ControlInput& in);
  void reset() {
    integral_ = 0.0;
    tc_limit_ = -1.0;
  }
  double integral() const { return integral_; }
  bool traction_control_active() const { return tc_limit_ >= 0.0; }

  // What the drive can do at a given speed (also used to judge actuator faults).
  double max_traction(double speed_mps) const;
  double max_retard(double speed_mps) const;

 private:
  const Calibration* cal_;
  double integral_ = 0.0;
  double tc_limit_ = -1.0;  // traction-control force cap, < 0 when inactive
  double last_traction_ = 0.0;
};

}  // namespace haul::vcu
