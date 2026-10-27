#include "haul/can/candump.hpp"

#include <cstdio>
#include <cstdlib>

namespace haul::can {

std::string format_candump(const LoggedFrame& lf) {
  char ts[40];
  std::snprintf(ts, sizeof ts, "(%lld.%06lld)", static_cast<long long>(lf.time / kMicrosPerSecond),
                static_cast<long long>(lf.time % kMicrosPerSecond));
  return std::string(ts) + " " + lf.interface + " " + to_string(lf.frame);
}

namespace {

int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

}  // namespace

std::optional<LoggedFrame> parse_candump(const std::string& line) {
  // (seconds.micros) iface ID#DATA
  if (line.size() < 10 || line[0] != '(') return std::nullopt;
  const auto close = line.find(')');
  const auto dot = line.find('.');
  if (close == std::string::npos || dot == std::string::npos || dot > close) return std::nullopt;
  LoggedFrame lf;
  const long long secs = std::strtoll(line.c_str() + 1, nullptr, 10);
  std::string frac = line.substr(dot + 1, close - dot - 1);
  frac.resize(6, '0');
  lf.time = secs * kMicrosPerSecond + std::strtoll(frac.c_str(), nullptr, 10);

  const auto if_start = line.find_first_not_of(' ', close + 1);
  if (if_start == std::string::npos) return std::nullopt;
  const auto if_end = line.find(' ', if_start);
  if (if_end == std::string::npos) return std::nullopt;
  lf.interface = line.substr(if_start, if_end - if_start);

  const auto frame_start = line.find_first_not_of(' ', if_end);
  if (frame_start == std::string::npos) return std::nullopt;
  const auto hash = line.find('#', frame_start);
  if (hash == std::string::npos) return std::nullopt;
  const std::string id = line.substr(frame_start, hash - frame_start);
  if (id.empty() || id.size() > 8) return std::nullopt;
  for (char c : id) {
    if (hex_value(c) < 0) return std::nullopt;
  }
  lf.frame.id = static_cast<std::uint32_t>(std::strtoul(id.c_str(), nullptr, 16));
  lf.frame.extended = id.size() > 3;

  std::size_t i = hash + 1;
  std::uint8_t n = 0;
  while (i + 1 < line.size() && n < 8) {
    const int hi = hex_value(line[i]);
    const int lo = hex_value(line[i + 1]);
    if (hi < 0 || lo < 0) break;
    lf.frame.data[n++] = static_cast<std::uint8_t>(hi * 16 + lo);
    i += 2;
  }
  lf.frame.dlc = n;
  return lf;
}

std::vector<LoggedFrame> read_candump(const std::string& path) {
  std::ifstream in(path);
  std::vector<LoggedFrame> out;
  std::string line;
  while (std::getline(in, line)) {
    if (auto lf = parse_candump(line)) out.push_back(*lf);
  }
  return out;
}

CandumpWriter::CandumpWriter(const std::string& path, std::string interface)
    : out_(path), interface_(std::move(interface)) {}

void CandumpWriter::write(Micros time, const Frame& f) {
  out_ << format_candump({time, interface_, f}) << '\n';
}

}  // namespace haul::can
