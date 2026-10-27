#include "haul/vcu/calibration.hpp"

#include <functional>
#include <map>

namespace haul::vcu {

namespace {

using Setter = std::function<void(Calibration&, double)>;

const std::map<std::string, Setter>& setters() {
  static const std::map<std::string, Setter> s{
      {"kp", [](Calibration& c, double v) { c.kp = v; }},
      {"ki", [](Calibration& c, double v) { c.ki = v; }},
      {"kaw", [](Calibration& c, double v) { c.kaw = v; }},
      {"accel_limit", [](Calibration& c, double v) { c.accel_limit = v; }},
      {"decel_limit", [](Calibration& c, double v) { c.decel_limit = v; }},
      {"safe_stop_decel", [](Calibration& c, double v) { c.safe_stop_decel = v; }},
      {"jerk_limit", [](Calibration& c, double v) { c.jerk_limit = v; }},
      {"drive_lag_s", [](Calibration& c, double v) { c.drive_lag_s = v; }},
      {"grade_feedforward", [](Calibration& c, double v) { c.grade_feedforward = v != 0.0; }},
      {"rolling_resistance", [](Calibration& c, double v) { c.rolling_resistance = v; }},
      {"degraded_speed_limit_kph", [](Calibration& c, double v) { c.degraded_speed_limit_mps = v / 3.6; }},
      {"slip_traction_scale", [](Calibration& c, double v) { c.slip_traction_scale = v; }},
      {"timeout_cycles", [](Calibration& c, double v) { c.timeout_cycles = static_cast<int>(v); }},
      {"hold_brake_pct", [](Calibration& c, double v) { c.hold_brake_pct = v; }},
  };
  return s;
}

}  // namespace

bool set_calibration(Calibration& cal, const std::string& key, double value) {
  const auto it = setters().find(key);
  if (it == setters().end()) return false;
  it->second(cal, value);
  return true;
}

std::vector<std::string> calibration_keys() {
  std::vector<std::string> keys;
  for (const auto& [k, _] : setters()) keys.push_back(k);
  return keys;
}

}  // namespace haul::vcu
