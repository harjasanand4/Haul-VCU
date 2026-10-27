#pragma once

#include <cstdint>

namespace haul {

// All timestamps are integer microseconds. The SIL simulation advances a simulated clock in
// these units; the real-time nodes take them from std::chrono::steady_clock. Integer time keeps
// a 10 ms control period exact over hours of simulation (a double accumulates drift).
using Micros = std::int64_t;

constexpr Micros kMicrosPerMs = 1'000;
constexpr Micros kMicrosPerSecond = 1'000'000;

constexpr Micros ms(std::int64_t v) {
  return v * kMicrosPerMs;
}
constexpr Micros seconds(double v) {
  return static_cast<Micros>(v * static_cast<double>(kMicrosPerSecond));
}
constexpr double to_seconds(Micros t) {
  return static_cast<double>(t) / static_cast<double>(kMicrosPerSecond);
}

}  // namespace haul
