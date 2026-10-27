#pragma once

// A small DBC (CAN database) parser and signal codec.
//
// Supports what real vehicle networks use day to day: Intel (little-endian) and Motorola
// (big-endian) byte order, signed and unsigned signals, scale/offset, min/max, units,
// GenMsgCycleTime attributes and VAL_ value tables. Multiplexed signals are parsed but
// treated as ordinary signals. The bit layout follows the same convention as Vector tools
// and Python's cantools; tools/crosscheck_dbc.py checks that against cantools directly.

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "haul/can/frame.hpp"

namespace haul::can {

struct DbcError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

enum class ByteOrder { Intel, Motorola };

class Signal {
 public:
  std::string name;
  std::uint32_t start_bit = 0;  // DBC start bit: LSB for Intel, MSB for Motorola
  std::uint32_t length = 1;
  ByteOrder order = ByteOrder::Intel;
  bool is_signed = false;
  double scale = 1.0;
  double offset = 0.0;
  double minimum = 0.0;
  double maximum = 0.0;
  std::string unit;
  std::vector<std::string> receivers;
  std::map<std::int64_t, std::string> value_names;

  // Must be called once the layout fields are set (the parser does this).
  void compute_layout(std::uint8_t dlc);

  std::int64_t raw_min() const;
  std::int64_t raw_max() const;

  std::int64_t decode_raw(std::span<const std::uint8_t> data) const;
  double decode(std::span<const std::uint8_t> data) const;
  void encode_raw(std::span<std::uint8_t> data, std::int64_t raw) const;
  // Physical -> raw with rounding, clamped to what the signal can represent.
  void encode(std::span<std::uint8_t> data, double physical) const;

  // Absolute bit index (byte * 8 + bit, bit 0 = LSB) of raw bit k, k = 0 is the raw LSB.
  const std::vector<std::uint16_t>& bit_positions() const { return positions_; }

 private:
  std::vector<std::uint16_t> positions_;
};

class Message {
 public:
  std::uint32_t id = 0;
  bool extended = false;
  std::string name;
  std::uint8_t dlc = 8;
  std::string transmitter;
  std::vector<Signal> signals;
  std::optional<int> cycle_time_ms;

  const Signal* find(std::string_view signal) const;
  std::size_t index_of(std::string_view signal) const;  // throws DbcError

  Frame encode(std::span<const double> values) const;               // one value per signal, in order
  void decode(const Frame& f, std::span<double> out) const;         // one value per signal, in order
  Frame encode(const std::map<std::string, double>& values) const;  // missing signals encode as 0
  std::map<std::string, double> decode(const Frame& f) const;
};

class Database {
 public:
  static Database parse(std::string_view text);
  static Database load(const std::filesystem::path& path);

  const std::vector<Message>& messages() const { return messages_; }
  const Message& message(std::string_view name) const;  // throws DbcError
  const Message* by_id(std::uint32_t id, bool extended = false) const;
  const std::vector<std::string>& nodes() const { return nodes_; }

 private:
  std::vector<Message> messages_;
  std::vector<std::string> nodes_;
};

}  // namespace haul::can
