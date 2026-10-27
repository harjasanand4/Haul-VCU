#include "haul/can/dbc.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>

namespace haul::can {

std::string to_string(const Frame& f) {
  char buf[48];
  int n = std::snprintf(buf, sizeof buf, f.extended ? "%08X#" : "%03X#", f.id);
  std::string out(buf, static_cast<std::size_t>(n));
  for (std::size_t i = 0; i < f.dlc; ++i) {
    std::snprintf(buf, sizeof buf, "%02X", f.data[i]);
    out += buf;
  }
  return out;
}

// ---- Signal ------------------------------------------------------------------------------

void Signal::compute_layout(std::uint8_t dlc) {
  if (length == 0 || length > 64) throw DbcError("signal " + name + ": length must be 1..64");
  const int limit = static_cast<int>(dlc) * 8;
  positions_.assign(length, 0);
  if (order == ByteOrder::Intel) {
    for (std::uint32_t k = 0; k < length; ++k) {
      const int pos = static_cast<int>(start_bit + k);
      if (pos >= limit) throw DbcError("signal " + name + " does not fit in the message");
      positions_[k] = static_cast<std::uint16_t>(pos);
    }
  } else {
    // Motorola: the start bit is the MSB. Walk towards the LSB through the "sawtooth"
    // numbering: down within a byte, then to bit 7 of the next byte.
    int pos = static_cast<int>(start_bit);
    for (std::uint32_t i = 0; i < length; ++i) {
      if (pos < 0 || pos >= limit) throw DbcError("signal " + name + " does not fit in the message");
      positions_[length - 1 - i] = static_cast<std::uint16_t>(pos);
      pos = (pos % 8 == 0) ? pos + 15 : pos - 1;
    }
  }
}

std::int64_t Signal::raw_min() const {
  if (!is_signed) return 0;
  return length >= 64 ? std::numeric_limits<std::int64_t>::min() : -(std::int64_t{1} << (length - 1));
}

std::int64_t Signal::raw_max() const {
  if (is_signed) {
    return length >= 64 ? std::numeric_limits<std::int64_t>::max() : (std::int64_t{1} << (length - 1)) - 1;
  }
  return length >= 63 ? std::numeric_limits<std::int64_t>::max() : (std::int64_t{1} << length) - 1;
}

std::int64_t Signal::decode_raw(std::span<const std::uint8_t> data) const {
  std::uint64_t raw = 0;
  for (std::size_t k = 0; k < positions_.size(); ++k) {
    const auto pos = positions_[k];
    if (pos / 8u >= data.size()) continue;  // short frame: missing bits read as 0
    if ((data[pos / 8u] >> (pos % 8u)) & 1u) raw |= std::uint64_t{1} << k;
  }
  if (is_signed && length < 64 && (raw >> (length - 1)) & 1u) {
    raw |= ~((std::uint64_t{1} << length) - 1);  // sign-extend
  }
  return static_cast<std::int64_t>(raw);
}

double Signal::decode(std::span<const std::uint8_t> data) const {
  return static_cast<double>(decode_raw(data)) * scale + offset;
}

void Signal::encode_raw(std::span<std::uint8_t> data, std::int64_t raw) const {
  const auto bits = static_cast<std::uint64_t>(raw);
  for (std::size_t k = 0; k < positions_.size(); ++k) {
    const auto pos = positions_[k];
    if (pos / 8u >= data.size()) continue;
    const auto mask = static_cast<std::uint8_t>(1u << (pos % 8u));
    if ((bits >> k) & 1u) {
      data[pos / 8u] = static_cast<std::uint8_t>(data[pos / 8u] | mask);
    } else {
      data[pos / 8u] = static_cast<std::uint8_t>(data[pos / 8u] & ~mask);
    }
  }
}

void Signal::encode(std::span<std::uint8_t> data, double physical) const {
  if (!std::isfinite(physical)) physical = 0.0;
  const double raw_d = std::round((physical - offset) / scale);
  const auto lo = static_cast<double>(raw_min());
  const auto hi = static_cast<double>(raw_max());
  const double clamped = raw_d < lo ? lo : (raw_d > hi ? hi : raw_d);
  encode_raw(data, static_cast<std::int64_t>(clamped));
}

// ---- Message -----------------------------------------------------------------------------

const Signal* Message::find(std::string_view signal) const {
  for (const auto& s : signals) {
    if (s.name == signal) return &s;
  }
  return nullptr;
}

std::size_t Message::index_of(std::string_view signal) const {
  for (std::size_t i = 0; i < signals.size(); ++i) {
    if (signals[i].name == signal) return i;
  }
  throw DbcError("message " + name + " has no signal " + std::string(signal));
}

Frame Message::encode(std::span<const double> values) const {
  if (values.size() != signals.size()) throw DbcError("message " + name + ": wrong number of values");
  Frame f;
  f.id = id;
  f.extended = extended;
  f.dlc = dlc;
  for (std::size_t i = 0; i < signals.size(); ++i) signals[i].encode(f.bytes(), values[i]);
  return f;
}

void Message::decode(const Frame& f, std::span<double> out) const {
  if (out.size() != signals.size()) throw DbcError("message " + name + ": wrong output size");
  for (std::size_t i = 0; i < signals.size(); ++i) out[i] = signals[i].decode(f.bytes());
}

Frame Message::encode(const std::map<std::string, double>& values) const {
  std::vector<double> v(signals.size(), 0.0);
  for (const auto& [k, val] : values) v[index_of(k)] = val;
  return encode(v);
}

std::map<std::string, double> Message::decode(const Frame& f) const {
  std::map<std::string, double> out;
  for (const auto& s : signals) out[s.name] = s.decode(f.bytes());
  return out;
}

// ---- Database ----------------------------------------------------------------------------

namespace {

constexpr std::uint32_t kExtendedFlag = 0x80000000u;

bool has_unquoted_semicolon(const std::string& s) {
  bool quoted = false;
  for (char c : s) {
    if (c == '"') quoted = !quoted;
    if (c == ';' && !quoted) return true;
  }
  return false;
}

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return {};
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::vector<std::string> split_receivers(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',' || c == ' ' || c == '\t') {
      if (!cur.empty()) out.push_back(cur);
      cur.clear();
    } else {
      cur += c;
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

double to_double(const std::string& s, int line) {
  try {
    std::size_t used = 0;
    double v = std::stod(s, &used);
    if (used != s.size()) throw std::invalid_argument(s);
    return v;
  } catch (const std::exception&) {
    throw DbcError("line " + std::to_string(line) + ": bad number '" + s + "'");
  }
}

}  // namespace

Database Database::parse(std::string_view text) {
  static const std::regex bo_re(R"(^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+))");
  static const std::regex sg_re(
      R"(^SG_\s+(\w+)\s*(?:[Mm]\d*M?)?\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(\s*([^,\s]+)\s*,\s*([^)\s]+)\s*\)\s*\[\s*([^|\s]+)\s*\|\s*([^\]\s]+)\s*\]\s*\"([^\"]*)\"\s*(.*)$)");
  static const std::regex cycle_re(R"(^BA_\s+\"GenMsgCycleTime\"\s+BO_\s+(\d+)\s+(\d+)\s*;)");
  static const std::regex val_re(R"(^VAL_\s+(\d+)\s+(\w+)\s+(.*);)");
  static const std::regex val_pair_re(R"((-?\d+)\s+\"([^\"]*)\")");

  Database db;
  std::istringstream in{std::string(text)};
  std::string line;
  int line_no = 0;
  Message* current = nullptr;
  std::vector<std::pair<std::uint32_t, int>> cycle_times;
  std::vector<std::tuple<std::uint32_t, std::string, std::string>> value_tables;

  while (std::getline(in, line)) {
    ++line_no;
    std::string stmt = trim(line);
    if (stmt.empty()) continue;

    // Statements that may span several lines end at the first unquoted semicolon.
    // (Bare "CM_" / "BA_" words inside the NS_ block are not statements.)
    const bool multi = stmt.rfind("CM_ ", 0) == 0 || stmt.rfind("VAL_ ", 0) == 0 || stmt.rfind("BA_ ", 0) == 0;
    if (multi) {
      std::string more;
      while (!has_unquoted_semicolon(stmt) && std::getline(in, more)) {
        ++line_no;
        stmt += " " + trim(more);
      }
    }

    std::smatch m;
    if (stmt.rfind("BU_", 0) == 0) {
      const auto colon = stmt.find(':');
      if (colon != std::string::npos) db.nodes_ = split_receivers(stmt.substr(colon + 1));
      current = nullptr;
    } else if (std::regex_search(stmt, m, bo_re)) {
      Message msg;
      const auto raw_id = static_cast<std::uint32_t>(std::stoul(m[1].str()));
      msg.extended = (raw_id & kExtendedFlag) != 0;
      msg.id = raw_id & ~kExtendedFlag;
      msg.name = m[2].str();
      const int dlc = std::stoi(m[3].str());
      if (dlc < 0 || dlc > 8)
        throw DbcError("line " + std::to_string(line_no) + ": DLC must be 0..8 (CAN FD is not supported)");
      msg.dlc = static_cast<std::uint8_t>(dlc);
      msg.transmitter = m[4].str();
      db.messages_.push_back(std::move(msg));
      current = &db.messages_.back();
    } else if (stmt.rfind("SG_", 0) == 0) {
      if (!current) throw DbcError("line " + std::to_string(line_no) + ": SG_ outside of a BO_ block");
      if (!std::regex_search(stmt, m, sg_re))
        throw DbcError("line " + std::to_string(line_no) + ": can't parse signal");
      Signal s;
      s.name = m[1].str();
      s.start_bit = static_cast<std::uint32_t>(std::stoul(m[2].str()));
      s.length = static_cast<std::uint32_t>(std::stoul(m[3].str()));
      s.order = m[4].str() == "1" ? ByteOrder::Intel : ByteOrder::Motorola;
      s.is_signed = m[5].str() == "-";
      s.scale = to_double(m[6].str(), line_no);
      s.offset = to_double(m[7].str(), line_no);
      s.minimum = to_double(m[8].str(), line_no);
      s.maximum = to_double(m[9].str(), line_no);
      s.unit = m[10].str();
      s.receivers = split_receivers(m[11].str());
      if (s.scale == 0.0) throw DbcError("line " + std::to_string(line_no) + ": scale of 0");
      s.compute_layout(current->dlc);
      current->signals.push_back(std::move(s));
    } else if (std::regex_search(stmt, m, cycle_re)) {
      cycle_times.emplace_back(static_cast<std::uint32_t>(std::stoul(m[1].str())), std::stoi(m[2].str()));
    } else if (std::regex_search(stmt, m, val_re)) {
      value_tables.emplace_back(static_cast<std::uint32_t>(std::stoul(m[1].str())), m[2].str(), m[3].str());
    } else if (stmt.rfind("BO_", 0) == 0) {
      throw DbcError("line " + std::to_string(line_no) + ": can't parse message definition");
    } else {
      current = nullptr;  // anything else ends the BO_ block
    }
  }

  auto find_raw = [&](std::uint32_t raw_id) -> Message* {
    for (auto& msg : db.messages_) {
      if ((msg.id | (msg.extended ? kExtendedFlag : 0u)) == raw_id) return &msg;
    }
    return nullptr;
  };
  for (const auto& [raw_id, ms] : cycle_times) {
    if (auto* msg = find_raw(raw_id)) msg->cycle_time_ms = ms;
  }
  for (const auto& [raw_id, sig, pairs] : value_tables) {
    Message* msg = find_raw(raw_id);
    if (!msg) continue;
    for (auto& s : msg->signals) {
      if (s.name != sig) continue;
      for (auto it = std::sregex_iterator(pairs.begin(), pairs.end(), val_pair_re); it != std::sregex_iterator();
           ++it) {
        s.value_names[std::stoll((*it)[1].str())] = (*it)[2].str();
      }
    }
  }
  return db;
}

Database Database::load(const std::filesystem::path& path) {
  std::ifstream f(path);
  if (!f) throw DbcError("can't open " + path.string());
  std::stringstream ss;
  ss << f.rdbuf();
  return parse(ss.str());
}

const Message& Database::message(std::string_view name) const {
  for (const auto& m : messages_) {
    if (m.name == name) return m;
  }
  throw DbcError("no message named " + std::string(name));
}

const Message* Database::by_id(std::uint32_t id, bool extended) const {
  for (const auto& m : messages_) {
    if (m.id == id && m.extended == extended) return &m;
  }
  return nullptr;
}

}  // namespace haul::can
