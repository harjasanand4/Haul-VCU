#include "haul/can/e2e.hpp"

#include <array>

namespace haul::can {

namespace {

constexpr std::array<std::uint8_t, 256> make_table() {
  std::array<std::uint8_t, 256> t{};
  for (int i = 0; i < 256; ++i) {
    auto c = static_cast<std::uint8_t>(i);
    for (int b = 0; b < 8; ++b) {
      c = static_cast<std::uint8_t>((c & 0x80u) ? ((c << 1) ^ 0x1Du) : (c << 1));
    }
    t[static_cast<std::size_t>(i)] = c;
  }
  return t;
}

constexpr auto kTable = make_table();

}  // namespace

std::uint8_t crc8_sae_j1850(std::span<const std::uint8_t> data, std::uint8_t start) {
  std::uint8_t crc = start;
  for (auto b : data) crc = kTable[static_cast<std::uint8_t>(crc ^ b)];
  return static_cast<std::uint8_t>(crc ^ 0xFFu);
}

const char* to_string(E2EStatus s) {
  switch (s) {
    case E2EStatus::Ok:
      return "ok";
    case E2EStatus::OkSomeLost:
      return "ok_some_lost";
    case E2EStatus::Initial:
      return "initial";
    case E2EStatus::Repeated:
      return "repeated";
    case E2EStatus::WrongSequence:
      return "wrong_sequence";
    case E2EStatus::CrcError:
      return "crc_error";
  }
  return "?";
}

E2EProfile::E2EProfile(const Message& msg) : data_id_(msg.id), crc_bytes_(msg.dlc, false) {
  counter_ = msg.find(msg.name + "_Counter");
  crc_ = msg.find(msg.name + "_CRC");
  if (!counter_ || !crc_) throw DbcError("message " + msg.name + " has no _Counter/_CRC signals for E2E");
  if (crc_->length != 8) throw DbcError("message " + msg.name + ": E2E CRC must be 8 bits");
  if (counter_->length > 8) throw DbcError("message " + msg.name + ": E2E counter must be at most 8 bits");
  modulo_ = static_cast<std::uint8_t>(counter_->length >= 8 ? 0 : (1u << counter_->length));
  for (auto pos : crc_->bit_positions()) crc_bytes_[pos / 8u] = true;
}

std::uint8_t E2EProfile::compute_crc(const Frame& f) const {
  std::array<std::uint8_t, 10> buf{};
  std::size_t n = 0;
  buf[n++] = static_cast<std::uint8_t>(data_id_ & 0xFFu);
  buf[n++] = static_cast<std::uint8_t>((data_id_ >> 8) & 0xFFu);
  for (std::size_t i = 0; i < f.dlc && i < crc_bytes_.size(); ++i) {
    if (!crc_bytes_[i]) buf[n++] = f.data[i];
  }
  return crc8_sae_j1850({buf.data(), n});
}

void E2EProfile::protect(Frame& f, std::uint8_t counter) const {
  counter_->encode_raw(f.bytes(), modulo_ ? counter % modulo_ : counter);
  crc_->encode_raw(f.bytes(), compute_crc(f));
}

bool E2EProfile::crc_ok(const Frame& f) const {
  return static_cast<std::uint8_t>(crc_->decode_raw(f.bytes())) == compute_crc(f);
}

std::uint8_t E2EProfile::counter(const Frame& f) const {
  return static_cast<std::uint8_t>(counter_->decode_raw(f.bytes()));
}

E2EStatus E2EReceiver::check(const Frame& f) {
  if (!profile_->crc_ok(f)) return E2EStatus::CrcError;
  const std::uint8_t c = profile_->counter(f);
  if (!has_last_) {
    has_last_ = true;
    last_ = c;
    return E2EStatus::Initial;
  }
  const unsigned mod = profile_->counter_modulo() ? profile_->counter_modulo() : 256u;
  const unsigned delta = (c + mod - last_) % mod;
  if (delta == 0) return E2EStatus::Repeated;
  last_ = c;  // resynchronise in every case below
  if (delta == 1) return E2EStatus::Ok;
  if (delta <= max_delta_) return E2EStatus::OkSomeLost;
  return E2EStatus::WrongSequence;
}

}  // namespace haul::can
