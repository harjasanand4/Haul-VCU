// haul-can: small CAN database tool.
//
//   haul-can info                                   messages and signals in the DBC
//   haul-can decode run.log                         decode a candump log, with E2E status per frame
//   haul-can encode VcuCommand TractionForceRequest=-120.5 BrakeRequest=10 [--counter 3]
//   haul-can vectors < cases.jsonl                  encode/decode JSON test vectors (used by tools/crosscheck_dbc.py)

#include <cstdio>
#include <iostream>
#include <map>
#include <string>

#include <nlohmann/json.hpp>

#include "haul/can/candump.hpp"
#include "haul/can/dbc.hpp"
#include "haul/can/e2e.hpp"

using namespace haul;

namespace {

void info(const can::Database& db) {
  for (const auto& m : db.messages()) {
    std::printf("0x%03X %-15s dlc %u  from %-6s every %s\n", m.id, m.name.c_str(), m.dlc, m.transmitter.c_str(),
                m.cycle_time_ms ? (std::to_string(*m.cycle_time_ms) + " ms").c_str() : "-");
    for (const auto& s : m.signals) {
      std::printf("      %-24s %2u|%-2u %s %s  x%-6g %+g  [%g, %g] %s\n", s.name.c_str(), s.start_bit, s.length,
                  s.order == can::ByteOrder::Intel ? "intel   " : "motorola", s.is_signed ? "signed  " : "unsigned",
                  s.scale, s.offset, s.minimum, s.maximum, s.unit.c_str());
    }
  }
}

int decode(const can::Database& db, const std::string& path) {
  std::map<std::uint32_t, can::E2EProfile> profiles;
  std::map<std::uint32_t, can::E2EReceiver> receivers;
  for (const auto& m : db.messages()) profiles.emplace(m.id, can::E2EProfile(m));
  for (const auto& [id, p] : profiles) receivers.emplace(id, can::E2EReceiver(p));

  const auto frames = can::read_candump(path);
  if (frames.empty()) {
    std::fprintf(stderr, "no frames in %s\n", path.c_str());
    return 1;
  }
  for (const auto& lf : frames) {
    const can::Message* m = db.by_id(lf.frame.id, lf.frame.extended);
    std::printf("%12.6f %-14s", to_seconds(lf.time), m ? m->name.c_str() : can::to_string(lf.frame).c_str());
    if (m) {
      std::printf(" %-14s", can::to_string(receivers.at(m->id).check(lf.frame)));
      for (const auto& s : m->signals) {
        if (s.name.ends_with("_Counter") || s.name.ends_with("_CRC")) continue;
        const double v = s.decode(lf.frame.bytes());
        const auto named = s.value_names.find(static_cast<std::int64_t>(s.decode_raw(lf.frame.bytes())));
        if (named != s.value_names.end()) {
          std::printf(" %s=%s", s.name.c_str(), named->second.c_str());
        } else {
          std::printf(" %s=%g", s.name.c_str(), v);
        }
      }
    }
    std::printf("\n");
  }
  return 0;
}

int encode(const can::Database& db, int argc, char** argv) {
  if (argc < 3) return 2;
  const auto& m = db.message(argv[2]);
  std::map<std::string, double> values;
  int counter = -1;
  for (int i = 3; i < argc; ++i) {
    const std::string kv = argv[i];
    if (kv == "--counter" && i + 1 < argc) {
      counter = std::stoi(argv[++i]);
      continue;
    }
    const auto eq = kv.find('=');
    if (eq == std::string::npos) return 2;
    values[kv.substr(0, eq)] = std::stod(kv.substr(eq + 1));
  }
  auto f = m.encode(values);
  if (counter >= 0) can::E2EProfile(m).protect(f, static_cast<std::uint8_t>(counter));
  std::printf("%s\n", can::to_string(f).c_str());
  return 0;
}

// Test vectors for tools/crosscheck_dbc.py, one JSON object per line on stdin:
//   {"message": "Imu", "signals": {"LongAccel": 1.2, ...}}   encode, then decode the result
//   {"message": "Imu", "hex": "2efb3b0200000f3c"}             decode these bytes
// Output per line: {"hex": "...", "decoded": {...}, "crc_ok": true|null}
int vectors(const can::Database& db) {
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    const auto in = nlohmann::json::parse(line);
    const auto& m = db.message(in.at("message").get<std::string>());
    can::Frame f;
    if (in.contains("hex")) {
      const auto hex = in.at("hex").get<std::string>();
      f.id = m.id;
      f.extended = m.extended;
      f.dlc = static_cast<std::uint8_t>(hex.size() / 2);
      for (std::size_t i = 0; i < f.dlc && i < 8; ++i) {
        f.data[i] = static_cast<std::uint8_t>(std::stoul(hex.substr(2 * i, 2), nullptr, 16));
      }
    } else {
      std::map<std::string, double> values;
      const auto& signals = in.at("signals");
      for (const auto& [k, v] : signals.items()) values[k] = v.get<double>();
      f = m.encode(values);
    }
    std::string hex;
    char buf[3];
    for (std::size_t i = 0; i < f.dlc; ++i) {
      std::snprintf(buf, sizeof buf, "%02x", f.data[i]);
      hex += buf;
    }
    nlohmann::json out;
    out["hex"] = hex;
    out["decoded"] = m.decode(f);
    const bool protected_msg = m.find(m.name + "_Counter") != nullptr && m.find(m.name + "_CRC") != nullptr;
    out["crc_ok"] = protected_msg ? nlohmann::json(can::E2EProfile(m).crc_ok(f)) : nlohmann::json(nullptr);
    std::cout << out.dump() << '\n';
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string dbc_path = std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc";
  if (argc >= 3 && std::string(argv[1]) == "--dbc") {
    dbc_path = argv[2];
    argv += 2;
    argc -= 2;
  }
  const std::string cmd = argc > 1 ? argv[1] : "";
  try {
    const auto db = can::Database::load(dbc_path);
    if (cmd == "info") {
      info(db);
      return 0;
    }
    if (cmd == "decode" && argc > 2) return decode(db, argv[2]);
    if (cmd == "encode") {
      if (const int rc = encode(db, argc, argv); rc != 2) return rc;
    }
    if (cmd == "vectors") return vectors(db);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "haul-can: %s\n", e.what());
    return 1;
  }
  std::fprintf(
      stderr, "usage: haul-can [--dbc FILE] info | decode LOG | encode MESSAGE SIG=VALUE... [--counter N] | vectors\n");
  return 2;
}
