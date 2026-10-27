#pragma once

#include <array>
#include <cstddef>
#include <string_view>
#include <vector>

#include "haul/can/dbc.hpp"
#include "haul/can/e2e.hpp"
#include "haul/core/time.hpp"
#include "haul/vcu/calibration.hpp"

namespace haul::vcu {

// A condition must hold for `on` before it counts, and be gone for `off` before it clears.
// Every fault goes through one of these so a single noisy sample can't stop a truck.
class Debouncer {
 public:
  explicit Debouncer(Micros on = 0, Micros off = 0) : on_(on), off_(off) {}
  bool update(bool condition, Micros now);
  bool active() const { return active_; }
  void reset() {
    active_ = false;
    pending_ = false;
  }

 private:
  Micros on_, off_;
  bool active_ = false;
  bool pending_ = false;
  Micros since_ = 0;
};

// Supervision of one received message: E2E check, decode, timeout, error rate.
class InputChannel {
 public:
  InputChannel(const can::Message& msg, const Calibration& cal);

  // Returns true if the frame passed E2E and was decoded.
  bool accept(const can::Frame& f, Micros now);

  bool received() const { return received_; }
  bool timed_out(Micros now) const { return !received_ || now - last_valid_ > timeout_; }
  // True once per new valid frame (cleared by take_fresh()).
  bool take_fresh() {
    const bool f = fresh_;
    fresh_ = false;
    return f;
  }
  int errors_in_last_second(Micros now) const;

  double value(std::size_t signal_index) const { return values_[signal_index]; }
  std::size_t index(std::string_view signal) const { return msg_->index_of(signal); }
  const can::Message& message() const { return *msg_; }
  Micros timeout() const { return timeout_; }
  Micros last_valid() const { return last_valid_; }

  struct Stats {
    long accepted = 0;
    long crc_errors = 0;
    long sequence_errors = 0;  // repeated or out-of-sequence counters
  };
  const Stats& stats() const { return stats_; }

 private:
  const can::Message* msg_;
  can::E2EProfile profile_;
  can::E2EReceiver receiver_;
  std::vector<double> values_;
  Micros timeout_;
  Micros last_valid_ = 0;
  bool received_ = false;
  bool fresh_ = false;
  void note_error(Micros now);

  // Times of the most recent bad frames, in a fixed ring: enough to count the error rate
  // against the threshold without allocating on the error path.
  std::array<Micros, 32> errors_{};
  std::size_t error_head_ = 0;
  std::size_t error_count_ = 0;
  Stats stats_;
};

}  // namespace haul::vcu
