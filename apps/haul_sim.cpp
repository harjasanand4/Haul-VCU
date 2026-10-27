// haul-sim: run scenarios in software-in-the-loop and check them.
//
//   haul-sim scenarios/*.json                        run, print a summary, exit 1 on any failure
//   haul-sim --out results scenarios/*.json          also write per-scenario CSV, JUnit XML, summary.md
//   haul-sim --candump run.log scenarios/x.json      record every frame on the bus (candump format)

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "haul/can/dbc.hpp"
#include "haul/sim/report.hpp"
#include "haul/sim/sil_runner.hpp"

namespace fs = std::filesystem;
using namespace haul;

static void usage() {
  std::fprintf(
      stderr,
      "usage: haul-sim [--dbc FILE] [--out DIR] [--candump FILE] [--cal KEY=VALUE]... [--quiet] SCENARIO.json...\n"
      "  --dbc      CAN database (default: dbc/haul_truck.dbc)\n"
      "  --out      write <name>.csv per scenario, junit.xml and summary.md into DIR\n"
      "  --candump  record bus traffic of the (single) scenario in candump format\n"
      "  --cal      override a VCU calibration value, e.g. --cal grade_feedforward=0 --cal kp=0.6\n");
}

int main(int argc, char** argv) {
  std::string dbc_path = std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc";
  std::string out_dir, candump;
  std::vector<std::pair<std::string, double>> cal_overrides;
  bool quiet = false;
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if ((a == "--dbc" || a == "--out" || a == "--candump") && i + 1 < argc) {
      (a == "--dbc" ? dbc_path : a == "--out" ? out_dir : candump) = argv[++i];
    } else if (a == "--cal" && i + 1 < argc) {
      const std::string kv = argv[++i];
      const auto eq = kv.find('=');
      if (eq == std::string::npos) {
        std::fprintf(stderr, "--cal expects key=value\n");
        return 2;
      }
      cal_overrides.emplace_back(kv.substr(0, eq), std::stod(kv.substr(eq + 1)));
    } else if (a == "--quiet") {
      quiet = true;
    } else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else {
      files.push_back(a);
    }
  }
  if (files.empty()) {
    usage();
    return 2;
  }

  try {
    const auto db = can::Database::load(dbc_path);
    if (!out_dir.empty()) fs::create_directories(out_dir);
    std::vector<sim::ScenarioResult> results;
    for (const auto& file : files) {
      sim::ScenarioResult r;
      r.scenario = sim::Scenario::load(file);
      sim::SilOptions opt;
      auto apply = [&](const std::string& key, double value) {
        if (!vcu::set_calibration(opt.calibration, key, value))
          throw std::invalid_argument("unknown calibration key " + key);
      };
      for (const auto& [k, v] : r.scenario.calibration) apply(k, v);
      for (const auto& [k, v] : cal_overrides) apply(k, v);
      if (!candump.empty() && files.size() == 1) opt.candump_path = candump;
      const auto t0 = std::chrono::steady_clock::now();
      r.run = sim::run_sil(db, r.scenario, opt);
      r.wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      r.checks = sim::evaluate(r.scenario, r.run);

      std::printf("%s %-32s %6.0f s simulated in %5.0f ms\n", r.passed() ? "PASS" : "FAIL", r.scenario.name.c_str(),
                  r.scenario.duration_s, r.wall_ms);
      if (!quiet) {
        for (const auto& tr : r.run.transitions) {
          std::printf("       %7.2f s  %s -> %s (%s)\n", tr.t, vcu::to_string(tr.from), vcu::to_string(tr.to),
                      tr.reason.c_str());
        }
        for (const auto& c : r.checks) {
          std::printf("     %s %s: %s\n", c.passed ? "ok  " : "FAIL", c.name.c_str(), c.detail.c_str());
        }
      }
      if (!out_dir.empty()) sim::write_csv(r.run, out_dir + "/" + r.scenario.name + ".csv");
      results.push_back(std::move(r));
    }

    int failed = 0;
    for (const auto& r : results) failed += r.passed() ? 0 : 1;
    std::printf("\n%zu scenarios, %d failed\n", results.size(), failed);
    if (!out_dir.empty()) {
      sim::write_junit(results, out_dir + "/junit.xml");
      std::ofstream(out_dir + "/summary.md") << sim::markdown_summary(results);
    }
    return failed ? 1 : 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "haul-sim: %s\n", e.what());
    return 2;
  }
}
