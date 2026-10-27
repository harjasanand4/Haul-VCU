#pragma once

#include <string>
#include <vector>

#include "haul/sim/scenario.hpp"

namespace haul::sim {

struct ScenarioResult {
  Scenario scenario;
  Run run;
  std::vector<CheckResult> checks;
  double wall_ms = 0.0;
  bool passed() const;
};

void write_csv(const Run& run, const std::string& path);
void write_junit(const std::vector<ScenarioResult>& results, const std::string& path);
std::string markdown_summary(const std::vector<ScenarioResult>& results);

}  // namespace haul::sim
