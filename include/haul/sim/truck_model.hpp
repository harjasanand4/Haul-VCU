#pragma once

namespace haul::sim {

// Physical parameters of the simulated truck. These are the "true" values; the VCU's
// Calibration holds its own estimates, deliberately not identical (rolling resistance and
// drive power differ a little), so the tests show the controller coping with model error.
struct TruckParams {
  double empty_mass_kg = 165'000.0;
  double payload_kg = 0.0;
  double rolling_resistance = 0.028;
  double drag_area_m2 = 45.0;
  double air_density = 1.2;
  double max_traction_n = 900'000.0;
  double max_traction_power_w = 2'500'000.0;
  double max_retard_n = 900'000.0;
  double max_retard_power_w = 3'400'000.0;
  double retard_fade_mps = 0.8;
  double max_brake_n = 1'400'000.0;
  double traction_tau_s = 0.30;   // electric drive response
  double brake_tau_s = 0.25;      // brake pressure build-up
  double rear_axle_share = 0.67;  // share of weight on the driven rear axle (loaded)
};

struct TruckState {
  double position_m = 0.0;
  double speed_mps = 0.0;
  double accel_mps2 = 0.0;
  double drive_force_n = 0.0;        // what the motors produce (signed)
  double transmitted_force_n = 0.0;  // what reaches the road (limited by grip)
  double brake_force_n = 0.0;
  double slip = 0.0;  // driven-wheel slip ratio, 0 = rolling, 0.2 = spinning 20 % fast
  double grade_rad = 0.0;
  double road_mu = 0.55;
};

// Longitudinal dynamics of a diesel-electric haul truck:
//
//   m dv/dt = F_road - F_brake - m g sin(theta) - Crr m g cos(theta) - 0.5 rho CdA v^2
//
// with first-order actuator lags, power-limited traction and electric retarding, grip-limited
// force on the driven axle (wheel slip on a wet road) and static friction at standstill.
class TruckModel {
 public:
  explicit TruckModel(TruckParams p) : p_(p) {}

  void reset(double speed_mps, double position_m = 0.0);
  // Advance by dt seconds with the given requests, on a road of the given grade and grip.
  void step(double dt, double drive_request_n, double brake_request_pct, double grade_rad, double road_mu);

  double mass() const { return p_.empty_mass_kg + p_.payload_kg; }
  double traction_limit(double v) const;
  double retard_limit(double v) const;
  const TruckState& state() const { return s_; }
  const TruckParams& params() const { return p_; }
  void set_drive_failed(bool failed) { drive_failed_ = failed; }

 private:
  TruckParams p_;
  TruckState s_;
  bool drive_failed_ = false;
};

}  // namespace haul::sim
