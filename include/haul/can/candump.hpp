#pragma once

// Read and write candump log files: "(1700000000.123456) vcan0 100#0A0B0C0D00000000".
// The SIL runner records every frame on the simulated bus in this format, so a run can be
// replayed onto a real interface with can-utils' canplayer, or decoded with cantools.

#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include "haul/can/frame.hpp"
#include "haul/core/time.hpp"

namespace haul::can {

struct LoggedFrame {
  Micros time = 0;
  std::string interface;
  Frame frame;
};

std::string format_candump(const LoggedFrame& lf);
std::optional<LoggedFrame> parse_candump(const std::string& line);  // nullopt for malformed lines
std::vector<LoggedFrame> read_candump(const std::string& path);

class CandumpWriter {
 public:
  explicit CandumpWriter(const std::string& path, std::string interface = "vcan0");
  void write(Micros time, const Frame& f);
  bool ok() const { return static_cast<bool>(out_); }

 private:
  std::ofstream out_;
  std::string interface_;
};

}  // namespace haul::can
