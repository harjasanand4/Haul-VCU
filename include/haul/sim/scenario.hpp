#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "haul/core/time.hpp"
#include "haul/sim/truck_sim.hpp"
#include "haul/vcu/faults.hpp"
#include "haul/vcu/state_machine.hpp"

namespace haul::sim {

// One row of a run, recorded every 10 ms control cycle.
struct Sample {
  double t = 0.0;
  double speed_kph = 0.0;     // truth
  double estimate_kph = 0.0;  // VCU's estimate
  double setpoint_kph = 0.0;
  double target_kph = 0.0;
  double limit_kph = 0.0;
  double drive_request_kn = 0.0;
  double drive_actual_kn = 0.0;
  double brake_request_pct = 0.0;
  double brake_force_kn = 0.0;
  double accel_mps2 = 0.0;
  double grade_pct = 0.0;
  double position_m = 0.0;
  double slip = 0.0;
  vcu::VcuState state = vcu::VcuState::Init;
  vcu::FaultSet faults;
};

struct Transition {
  double t = 0.0;
  vcu::VcuState from = vcu::VcuState::Init;
  vcu::VcuState to = vcu::VcuState::Init;
  std::string reason;
};

struct Run {
  std::vector<Sample> samples;
  std::vector<Transition> transitions;
  long frames_to_vcu = 0;
  long frames_to_truck = 0;
  long frames_dropped = 0;
  long frames_corrupted = 0;
  bool truck_watchdog_tripped = false;
};

// A pass/fail criterion evaluated over a recorded run. Times are seconds.
struct Check {
  std::string kind;  // see evaluate() for the list
  double from_s = 0.0;
  double to_s = 1e9;
  double target_kph = 0.0;
  double tolerance_kph = 0.5;
  double min_kph = -1e9;
  double max_kph = 1e9;
  double limit = 0.0;     // m/s^2 for max_decel, % for max_brake, kph for estimate_error / max_overshoot
  double within_s = 1e9;  // for *_reached / fault_raised: allowed latency after from_s
  std::optional<vcu::VcuState> state;
  std::optional<vcu::Fault> fault;
  bool any_critical = false;
};

struct CheckResult {
  std::string name;  // "speed_settles(40 kph from 30 s)"
  bool passed = false;
  std::string detail;  // what was measured
};

struct Scenario {
  std::string name;
  std::string description;
  double duration_s = 60.0;
  TruckSetup setup;
  std::map<std::string, double> calibration;  // VCU tuning overrides for this scenario
  std::vector<Check> checks;

  static Scenario from_json(const std::string& text);
  static Scenario load(const std::filesystem::path& path);
};

std::vector<CheckResult> evaluate(const Scenario& sc, const Run& run);

}  // namespace haul::sim
