#pragma once

#include <string>
#include <vector>

#include "haul/core/time.hpp"

namespace haul::vcu {

// Everything the VCU knows about the truck and every tuning knob, in SI units.
// The numbers are in the range of a 220 t-class diesel-electric haul truck.
struct Calibration {
  // Vehicle (used for feedforward and to schedule gains by mass)
  double empty_mass_kg = 165'000.0;
  double rolling_resistance = 0.025;  // haul road, dimensionless
  double drag_area_m2 = 42.0;         // Cd * A
  double air_density = 1.2;

  // Drive and retarder limits (what the VCU expects the drive to deliver)
  double max_traction_n = 900'000.0;
  double max_traction_power_w = 2'600'000.0;
  double max_retard_n = 900'000.0;
  double max_retard_power_w = 3'400'000.0;
  double retard_fade_mps = 0.8;      // electric retarding is ineffective near standstill
  double max_brake_n = 1'400'000.0;  // service brakes at 100 %
  double drive_lag_s = 0.30;         // expected response time of the drive

  // Speed controller
  double kp = 0.45;              // (m/s^2) per (m/s) of speed error
  double ki = 0.06;              // (m/s^2) per (m s) of integrated error
  double kaw = 1.5;              // anti-windup back-calculation gain
  double accel_limit = 0.45;     // m/s^2, setpoint profile when speeding up
  double decel_limit = 0.70;     // m/s^2, setpoint profile when slowing down
  double jerk_limit = 0.35;      // m/s^3, so the profile is an S-curve, not a ramp with corners
  double safe_stop_decel = 1.0;  // m/s^2, controlled stop on a critical fault
  double safe_stop_jerk = 2.0;   // m/s^3
  double hold_brake_pct = 55.0;  // brake held while stopped (holds a loaded truck on a 12 % grade)
  double standstill_mps = 0.15;
  bool grade_feedforward = true;  // off only for the ablation run in the README

  // Degraded operation
  double degraded_speed_limit_mps = 20.0 / 3.6;
  double slip_traction_scale = 0.80;         // traction control: cut to 80 % of the force at slip onset...
  double slip_decay_per_s = 0.6;             // ...and keep cutting 60 %/s while the wheels still spin
  double slip_recovery_n_per_s = 120'000.0;  // then give force back gradually

  // Input supervision
  int timeout_cycles = 3;           // a message is missing after 3 cycle times...
  Micros min_timeout = ms(50);      // ...but never less than 50 ms
  int e2e_errors_per_second = 5;    // more bad frames than this in 1 s raises E2EError
  double slip_threshold_mps = 0.6;  // rear minus front axle speed
  double slip_ratio = 0.10;         //   or more than 10 %
  Micros slip_debounce = ms(150);
  double plausibility_threshold_mps = 1.2;  // front axle vs GNSS
  Micros plausibility_debounce = ms(1500);
  double actuator_tolerance_n = 160'000.0;
  Micros actuator_debounce = ms(1200);

  // State machine
  Micros init_settle = ms(500);       // all inputs healthy this long before READY
  Micros stopped_confirm = ms(1000);  // speed below standstill this long counts as stopped
  Micros fault_clear = ms(1000);      // operator holds STOP this long to clear a latched fault
};

// Override one tuning value by name ("kp=0.6", "grade_feedforward=0"). Returns false for an
// unknown key. Used by scenario files and `haul-sim --cal`.
bool set_calibration(Calibration& cal, const std::string& key, double value);
std::vector<std::string> calibration_keys();

}  // namespace haul::vcu
