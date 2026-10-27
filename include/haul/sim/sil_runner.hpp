#pragma once

#include <optional>
#include <string>

#include "haul/can/dbc.hpp"
#include "haul/sim/scenario.hpp"
#include "haul/vcu/calibration.hpp"

namespace haul::sim {

struct SilOptions {
  Micros plant_step = ms(1);                // plant integration step
  Micros bus_latency = 300;                 // per frame, about one 8-byte frame at 500 kbit/s
  std::optional<std::string> candump_path;  // record all bus traffic
  vcu::Calibration calibration;
};

// Software-in-the-loop: the real VCU code against the simulated truck, on a simulated CAN
// bus, in simulated time. Deterministic: same scenario and seed, same result, every time.
Run run_sil(const can::Database& db, const Scenario& scenario, const SilOptions& options = {});

}  // namespace haul::sim
