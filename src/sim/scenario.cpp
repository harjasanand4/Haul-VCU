#include "haul/sim/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "haul/vcu/calibration.hpp"

namespace haul::sim {

using nlohmann::json;

namespace {

std::string fmt(const char* f, double a, double b = 0.0) {
  char buf[160];
  std::snprintf(buf, sizeof buf, f, a, b);
  return buf;
}

Check parse_check(const json& j) {
  Check c;
  c.kind = j.at("check").get<std::string>();
  c.from_s = j.value("from_s", 0.0);
  c.to_s = j.value("to_s", 1e9);
  if (j.contains("by_s")) c.from_s = j.at("by_s").get<double>();
  c.target_kph = j.value("target_kph", 0.0);
  c.tolerance_kph = j.value("tolerance_kph", 0.5);
  c.min_kph = j.value("min_kph", -1e9);
  c.max_kph = j.value("max_kph", 1e9);
  c.within_s = j.value("within_s", 1e9);
  for (const char* key : {"limit_kph", "limit_mps2", "limit_pct"}) {
    if (j.contains(key)) c.limit = j.at(key).get<double>();
  }
  if (j.contains("state")) {
    vcu::VcuState s;
    if (!vcu::parse_state(j.at("state").get<std::string>(), s)) throw std::invalid_argument("unknown state in check");
    c.state = s;
  }
  if (j.contains("fault")) {
    const auto name = j.at("fault").get<std::string>();
    if (name == "any_critical") {
      c.any_critical = true;
    } else {
      vcu::Fault f;
      if (!vcu::parse_fault(name, f)) throw std::invalid_argument("unknown fault '" + name + "' in check");
      c.fault = f;
    }
  }
  static const std::vector<std::string> kinds{"speed_settles", "max_overshoot",  "speed_between", "state_reached",
                                              "state_never",   "fault_raised",   "fault_never",   "stopped_by",
                                              "max_decel",     "estimate_error", "max_brake"};
  if (std::find(kinds.begin(), kinds.end(), c.kind) == kinds.end()) {
    throw std::invalid_argument("unknown check '" + c.kind + "'");
  }
  return c;
}

}  // namespace

Scenario Scenario::from_json(const std::string& text) {
  const json j = json::parse(text);
  Scenario sc;
  sc.name = j.at("name").get<std::string>();
  sc.description = j.value("description", "");
  sc.duration_s = j.value("duration_s", 60.0);
  sc.setup.seed = j.value("seed", 42ull);

  const json truck = j.value("truck", json::object());
  auto& p = sc.setup.params;
  p.payload_kg = truck.value("payload_t", 0.0) * 1000.0;
  p.rolling_resistance = truck.value("rolling_resistance", p.rolling_resistance);
  p.max_traction_power_w = truck.value("traction_power_kw", p.max_traction_power_w / 1000.0) * 1000.0;
  p.rear_axle_share = truck.value("rear_axle_share", p.payload_kg > 0.0 ? 0.67 : 0.50);

  if (j.contains("road")) {
    sc.setup.road.points.clear();
    for (const auto& pt : j.at("road"))
      sc.setup.road.points.emplace_back(pt.at(0).get<double>(), pt.at(1).get<double>());
  }
  for (const auto& w : j.value("wet_patches", json::array())) {
    sc.setup.wet_patches.push_back({w.at("from_m").get<double>(), w.at("to_m").get<double>(), w.value("mu", 0.1)});
  }
  sc.setup.mission.clear();
  for (const auto& m : j.at("mission")) {
    MissionStep step;
    step.t_s = m.value("t", 0.0);
    if (m.contains("at_m")) step.at_m = m.at("at_m").get<double>();
    if (!m.contains("t") && !step.at_m) throw std::invalid_argument("mission step needs \"t\" or \"at_m\"");
    step.target_kph = m.value("target_kph", 0.0);
    step.autonomous = m.value("mode", std::string("AUTO")) == "AUTO";
    step.estop = m.value("estop", false);
    sc.setup.mission.push_back(step);
  }
  // Before the first step the fleet management system says STOP.
  sc.setup.mission.insert(sc.setup.mission.begin(), MissionStep{0.0, 0.0, false, false, {}});
  for (const auto& f : j.value("faults", json::array())) {
    FaultInjection fi;
    fi.type = f.at("type").get<std::string>();
    fi.message = f.value("message", "");
    fi.t_s = f.at("t").get<double>();
    fi.duration_s = f.value("duration_s", 1e9);
    fi.amount = f.value("amount", 1.0);
    sc.setup.faults.push_back(fi);
  }
  // Bind the object first: j.value() returns a temporary, and in C++20 a range-for only keeps
  // the outermost temporary (the items() proxy) alive, not the json it points into.
  const json overrides = j.value("vcu", json::object());
  for (const auto& [key, value] : overrides.items()) {
    const double v = value.is_boolean() ? (value.get<bool>() ? 1.0 : 0.0) : value.get<double>();
    vcu::Calibration probe;
    if (!vcu::set_calibration(probe, key, v)) throw std::invalid_argument("unknown VCU calibration key '" + key + "'");
    sc.calibration[key] = v;
  }
  for (const auto& c : j.at("expect")) sc.checks.push_back(parse_check(c));
  return sc;
}

Scenario Scenario::load(const std::filesystem::path& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("can't open " + path.string());
  std::stringstream ss;
  ss << f.rdbuf();
  try {
    return from_json(ss.str());
  } catch (const std::exception& e) {
    throw std::runtime_error(path.filename().string() + ": " + e.what());
  }
}

std::vector<CheckResult> evaluate(const Scenario& sc, const Run& run) {
  std::vector<CheckResult> out;
  const auto& s = run.samples;
  for (const auto& c : sc.checks) {
    CheckResult r;
    const double to = std::min(c.to_s, sc.duration_s);
    auto in_window = [&](const Sample& x) { return x.t >= c.from_s - 1e-9 && x.t <= to + 1e-9; };

    if (c.kind == "speed_settles") {
      double worst = 0.0;
      for (const auto& x : s) {
        if (in_window(x)) worst = std::max(worst, std::abs(x.speed_kph - c.target_kph));
      }
      r.name = fmt("speed_settles at %.1f km/h from %.0f s", c.target_kph, c.from_s);
      r.passed = worst <= c.tolerance_kph;
      r.detail = fmt("max error %.2f km/h (allowed %.2f)", worst, c.tolerance_kph);
    } else if (c.kind == "max_overshoot") {
      double peak = -1e9;
      for (const auto& x : s) {
        if (in_window(x)) peak = std::max(peak, x.speed_kph - c.target_kph);
      }
      r.name = fmt("overshoot above %.1f km/h", c.target_kph);
      r.passed = peak <= c.limit;
      r.detail = fmt("peak %+.2f km/h (allowed +%.2f)", peak, c.limit);
    } else if (c.kind == "speed_between") {
      double lo = 1e9, hi = -1e9;
      for (const auto& x : s) {
        if (!in_window(x)) continue;
        lo = std::min(lo, x.speed_kph);
        hi = std::max(hi, x.speed_kph);
      }
      if (c.min_kph <= -1e8) {
        r.name = fmt("speed at most %.1f km/h", c.max_kph);
      } else if (c.max_kph >= 1e8) {
        r.name = fmt("speed at least %.1f km/h", c.min_kph);
      } else {
        r.name = fmt("speed within %.1f..%.1f km/h", c.min_kph, c.max_kph);
      }
      r.passed = lo >= c.min_kph && hi <= c.max_kph;
      r.detail = fmt("min %.2f, max %.2f km/h", lo, hi);
    } else if (c.kind == "state_reached" || c.kind == "fault_raised") {
      const bool want_state = c.kind == "state_reached";
      std::optional<double> when;
      for (const auto& x : s) {
        if (x.t < c.from_s - 1e-9) continue;
        const bool hit = want_state ? (c.state && x.state == *c.state)
                                    : (c.any_critical ? x.faults.critical() : (c.fault && x.faults.has(*c.fault)));
        if (hit) {
          when = x.t;
          break;
        }
      }
      const std::string what =
          want_state ? vcu::to_string(*c.state) : (c.any_critical ? "a critical fault" : vcu::fault_name(*c.fault));
      r.name = what + fmt(" within %.2f s of %.1f s", c.within_s, c.from_s);
      r.passed = when && *when - c.from_s <= c.within_s + 1e-9;
      r.detail = when ? fmt("after %.3f s", *when - c.from_s) : "never happened";
    } else if (c.kind == "state_never" || c.kind == "fault_never") {
      const bool want_state = c.kind == "state_never";
      std::optional<double> when;
      for (const auto& x : s) {
        if (!in_window(x)) continue;
        const bool hit = want_state ? (c.state && x.state == *c.state)
                                    : (c.any_critical ? x.faults.critical() : (c.fault && x.faults.has(*c.fault)));
        if (hit) {
          when = x.t;
          break;
        }
      }
      const std::string what =
          want_state ? vcu::to_string(*c.state) : (c.any_critical ? "any critical fault" : vcu::fault_name(*c.fault));
      r.name = "never " + what;
      r.passed = !when;
      r.detail = when ? fmt("happened at %.2f s", *when) : "never happened";
    } else if (c.kind == "stopped_by") {
      bool ok = true;
      std::optional<double> moving_until;
      for (const auto& x : s) {
        if (x.speed_kph > 0.1) moving_until = x.t;
        if (in_window(x) && x.speed_kph > 0.1) ok = false;
      }
      r.name = fmt("stopped by %.1f s", c.from_s);
      r.passed = ok;
      r.detail = moving_until ? fmt("last moving at %.2f s", *moving_until) : "never moved";
    } else if (c.kind == "max_decel") {
      // Deceleration from the true speed over 100 ms windows (10 samples).
      double worst = 0.0;
      for (std::size_t i = 0; i + 10 < s.size(); ++i) {
        if (!in_window(s[i]) || !in_window(s[i + 10])) continue;
        const double a = (s[i + 10].speed_kph - s[i].speed_kph) / 3.6 / (s[i + 10].t - s[i].t);
        worst = std::max(worst, -a);
      }
      r.name = fmt("deceleration at most %.2f m/s2", c.limit);
      r.passed = worst <= c.limit;
      r.detail = fmt("peak %.2f m/s2", worst);
    } else if (c.kind == "estimate_error") {
      double worst = 0.0;
      for (const auto& x : s) {
        if (in_window(x)) worst = std::max(worst, std::abs(x.estimate_kph - x.speed_kph));
      }
      r.name = fmt("speed estimate within %.2f km/h", c.limit);
      r.passed = worst <= c.limit;
      r.detail = fmt("max error %.3f km/h", worst);
    } else if (c.kind == "max_brake") {
      double worst = 0.0;
      for (const auto& x : s) {
        if (in_window(x)) worst = std::max(worst, x.brake_request_pct);
      }
      r.name = fmt("service brake at most %.0f %%", c.limit);
      r.passed = worst <= c.limit;
      r.detail = fmt("peak %.1f %%", worst);
    }
    out.push_back(r);
  }
  return out;
}

}  // namespace haul::sim
