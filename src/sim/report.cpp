#include "haul/sim/report.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

namespace haul::sim {

bool ScenarioResult::passed() const {
  for (const auto& c : checks) {
    if (!c.passed) return false;
  }
  return true;
}

void write_csv(const Run& run, const std::string& path) {
  std::ofstream out(path);
  out << "t,speed_kph,estimate_kph,setpoint_kph,target_kph,limit_kph,drive_request_kn,drive_actual_kn,"
         "brake_request_pct,brake_force_kn,accel_mps2,grade_pct,position_m,slip,state,faults\n";
  char buf[512];
  for (const auto& s : run.samples) {
    std::snprintf(buf, sizeof buf, "%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.3f,%.2f,%.1f,%.3f,%s,%s\n", s.t,
                  s.speed_kph, s.estimate_kph, s.setpoint_kph, s.target_kph, s.limit_kph, s.drive_request_kn,
                  s.drive_actual_kn, s.brake_request_pct, s.brake_force_kn, s.accel_mps2, s.grade_pct, s.position_m,
                  s.slip, vcu::to_string(s.state), vcu::describe(s.faults).c_str());
    out << buf;
  }
}

namespace {

std::string xml_escape(const std::string& s) {
  std::string out;
  for (char c : s) {
    switch (c) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '"':
        out += "&quot;";
        break;
      default:
        out += c;
    }
  }
  return out;
}

}  // namespace

void write_junit(const std::vector<ScenarioResult>& results, const std::string& path) {
  int tests = 0, failures = 0;
  for (const auto& r : results) {
    tests += static_cast<int>(r.checks.size());
    for (const auto& c : r.checks) failures += c.passed ? 0 : 1;
  }
  std::ofstream out(path);
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  out << "<testsuites name=\"haul-vcu scenarios\" tests=\"" << tests << "\" failures=\"" << failures << "\">\n";
  for (const auto& r : results) {
    int f = 0;
    for (const auto& c : r.checks) f += c.passed ? 0 : 1;
    out << "  <testsuite name=\"" << xml_escape(r.scenario.name) << "\" tests=\"" << r.checks.size() << "\" failures=\""
        << f << "\" time=\"" << r.wall_ms / 1000.0 << "\">\n";
    for (const auto& c : r.checks) {
      out << "    <testcase classname=\"" << xml_escape(r.scenario.name) << "\" name=\"" << xml_escape(c.name) << "\">";
      if (!c.passed) out << "<failure message=\"" << xml_escape(c.detail) << "\"/>";
      out << "<system-out>" << xml_escape(c.detail) << "</system-out></testcase>\n";
    }
    out << "  </testsuite>\n";
  }
  out << "</testsuites>\n";
}

std::string markdown_summary(const std::vector<ScenarioResult>& results) {
  std::ostringstream out;
  out << "| Scenario | Result | Checks | What was measured |\n|---|---|---|---|\n";
  for (const auto& r : results) {
    int passed = 0;
    std::string details;
    for (const auto& c : r.checks) {
      passed += c.passed ? 1 : 0;
      if (!details.empty()) details += "; ";
      details += (c.passed ? "" : "**FAILED** ") + c.name + ": " + c.detail;
    }
    out << "| `" << r.scenario.name << "` | " << (r.passed() ? "pass" : "**FAIL**") << " | " << passed << "/"
        << r.checks.size() << " | " << details << " |\n";
  }
  return out.str();
}

}  // namespace haul::sim
