#pragma once

// End-to-end protection for safety-relevant CAN messages, in the style of AUTOSAR E2E
// Profile 1: a 4-bit alive counter and an 8-bit CRC (SAE J1850) per frame. The CRC also
// covers the message's "data ID" (here its CAN ID), so a frame that turns up on the wrong
// identifier fails the check even if its payload is intact.

#include <cstdint>
#include <span>
#include <vector>

#include "haul/can/dbc.hpp"
#include "haul/can/frame.hpp"

namespace haul::can {

// CRC-8/SAE-J1850: poly 0x1D, init 0xFF, final XOR 0xFF, no reflection. Check("123456789") = 0x4B.
std::uint8_t crc8_sae_j1850(std::span<const std::uint8_t> data, std::uint8_t start = 0xFF);

enum class E2EStatus {
  Ok,             // CRC good, counter advanced by 1
  OkSomeLost,     // CRC good, counter advanced by 2..max_delta (frames were lost, data is fresh)
  Initial,        // first frame seen (no counter history yet)
  Repeated,       // same counter as last time: stale or replayed data
  WrongSequence,  // counter jumped by more than max_delta
  CrcError,
};

constexpr bool usable(E2EStatus s) {
  return s == E2EStatus::Ok || s == E2EStatus::OkSomeLost || s == E2EStatus::Initial;
}

const char* to_string(E2EStatus s);

// Where the counter and CRC live in a message (looked up by naming convention:
// <Message>_Counter and <Message>_CRC).
class E2EProfile {
 public:
  explicit E2EProfile(const Message& msg);

  // Write the counter, then the CRC over everything else.
  void protect(Frame& f, std::uint8_t counter) const;
  std::uint8_t compute_crc(const Frame& f) const;
  bool crc_ok(const Frame& f) const;
  std::uint8_t counter(const Frame& f) const;
  std::uint8_t counter_modulo() const { return modulo_; }

 private:
  const Signal* counter_ = nullptr;
  const Signal* crc_ = nullptr;
  std::uint32_t data_id_ = 0;
  std::vector<bool> crc_bytes_;  // which data bytes hold CRC bits (excluded from the CRC)
  std::uint8_t modulo_ = 16;
};

// Receiver-side state machine for one message.
class E2EReceiver {
 public:
  explicit E2EReceiver(const E2EProfile& profile, std::uint8_t max_delta = 3)
      : profile_(&profile), max_delta_(max_delta) {}

  E2EStatus check(const Frame& f);
  void reset() { has_last_ = false; }

 private:
  const E2EProfile* profile_;
  std::uint8_t max_delta_;
  bool has_last_ = false;
  std::uint8_t last_ = 0;
};

}  // namespace haul::can
