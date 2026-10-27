#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace haul {

// Collects samples (microseconds) and reports percentiles. Not thread-safe: one owner.
class LatencyStats {
 public:
  // Keeps up to `max_samples` samples for percentiles. `reserve` up front for a caller that must
  // not allocate while recording (the real-time control loop reserves all of it).
  explicit LatencyStats(std::size_t max_samples = 2'000'000, std::size_t reserve = 4096) : max_(max_samples) {
    samples_.reserve(std::min(reserve, max_samples));
  }

  void add(double us) {
    ++count_;
    worst_ = std::max(worst_, us);
    if (samples_.size() < max_) samples_.push_back(us);
  }

  std::size_t count() const { return count_; }
  double max() const { return worst_; }
  double percentile(double p) const {
    if (samples_.empty()) return 0.0;
    std::vector<double> s = samples_;
    const auto k = static_cast<std::size_t>(std::clamp(p, 0.0, 1.0) * static_cast<double>(s.size() - 1));
    std::nth_element(s.begin(), s.begin() + static_cast<std::ptrdiff_t>(k), s.end());
    return s[k];
  }

 private:
  std::size_t max_;
  std::size_t count_ = 0;
  double worst_ = 0.0;
  std::vector<double> samples_;
};

}  // namespace haul
