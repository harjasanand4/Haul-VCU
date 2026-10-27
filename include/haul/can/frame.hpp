#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>

namespace haul::can {

// A classic CAN 2.0 frame (up to 8 data bytes), mirroring Linux's struct can_frame.
struct Frame {
  std::uint32_t id = 0;  // 11-bit standard or 29-bit extended identifier
  bool extended = false;
  std::uint8_t dlc = 8;  // data length code: number of valid bytes in `data`
  std::array<std::uint8_t, 8> data{};

  std::span<std::uint8_t> bytes() { return {data.data(), dlc}; }
  std::span<const std::uint8_t> bytes() const { return {data.data(), dlc}; }

  friend bool operator==(const Frame&, const Frame&) = default;
};

// "1A0#DEADBEEF" (candump / cansend notation).
std::string to_string(const Frame& f);

}  // namespace haul::can
