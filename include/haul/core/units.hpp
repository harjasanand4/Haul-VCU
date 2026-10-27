#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace haul {

constexpr double kGravity = 9.80665;  // m/s^2

constexpr double kph_to_mps(double kph) {
  return kph / 3.6;
}
constexpr double mps_to_kph(double mps) {
  return mps * 3.6;
}
constexpr double deg_to_rad(double deg) {
  return deg * std::numbers::pi / 180.0;
}
constexpr double rad_to_deg(double rad) {
  return rad * 180.0 / std::numbers::pi;
}

// Road grade in percent (rise over run * 100) to the slope angle.
inline double grade_pct_to_rad(double pct) {
  return std::atan(pct / 100.0);
}

template <typename T>
constexpr T clamp(T v, T lo, T hi) {
  return std::min(std::max(v, lo), hi);
}

// Move `current` towards `target` by at most `max_up` (when rising) or `max_down` (when falling).
constexpr double rate_limit(double current, double target, double max_up, double max_down) {
  const double delta = target - current;
  if (delta > max_up) return current + max_up;
  if (delta < -max_down) return current - max_down;
  return target;
}

}  // namespace haul
