#include "haul/vcu/monitors.hpp"

#include <algorithm>

namespace haul::vcu {

bool Debouncer::update(bool condition, Micros now) {
  if (condition == active_) {
    pending_ = false;
    return active_;
  }
  if (!pending_) {
    pending_ = true;
    since_ = now;
  }
  if (now - since_ >= (condition ? on_ : off_)) {
    active_ = condition;
    pending_ = false;
  }
  return active_;
}

InputChannel::InputChannel(const can::Message& msg, const Calibration& cal)
    : msg_(&msg),
      profile_(msg),
      receiver_(profile_),
      values_(msg.signals.size(), 0.0),
      timeout_(std::max<Micros>(cal.min_timeout, ms(msg.cycle_time_ms.value_or(100)) * cal.timeout_cycles)) {}

bool InputChannel::accept(const can::Frame& f, Micros now) {
  if (f.dlc != msg_->dlc) {
    note_error(now);
    ++stats_.crc_errors;
    return false;
  }
  const auto status = receiver_.check(f);
  if (!can::usable(status)) {
    note_error(now);
    if (status == can::E2EStatus::CrcError) {
      ++stats_.crc_errors;
    } else {
      ++stats_.sequence_errors;
    }
    return false;
  }
  msg_->decode(f, values_);
  last_valid_ = now;
  received_ = true;
  fresh_ = true;
  ++stats_.accepted;
  return true;
}

void InputChannel::note_error(Micros now) {
  errors_[error_head_] = now;
  error_head_ = (error_head_ + 1) % errors_.size();
  error_count_ = std::min(error_count_ + 1, errors_.size());
}

int InputChannel::errors_in_last_second(Micros now) const {
  int n = 0;
  for (std::size_t i = 0; i < error_count_; ++i) {
    if (now - errors_[i] <= kMicrosPerSecond) ++n;
  }
  return n;
}

}  // namespace haul::vcu
