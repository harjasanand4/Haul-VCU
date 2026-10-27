// haul-vcu: the VCU as a real-time process on a real (or virtual) CAN bus.
//
//   haul-vcu --bus socketcan:vcan0
//   haul-vcu --bus udp:9002:127.0.0.1:9001          # macOS / no SocketCAN: pair with haul-truck
//   haul-vcu --bus socketcan:can0 --realtime --telemetry vcu.csv
//
// Ctrl-C stops it and prints timing statistics.

#include <atomic>
#include <csignal>
#include <cstdio>
#include <string>

#include "haul/can/bus.hpp"
#include "haul/can/dbc.hpp"
#include "haul/rt/vcu_node.hpp"

using namespace haul;

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) {
  g_stop.store(true);
}
}  // namespace

int main(int argc, char** argv) {
  std::string bus_spec = "udp:9002:127.0.0.1:9001";
  std::string dbc_path = std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc";
  rt::NodeOptions opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--bus" && i + 1 < argc) {
      bus_spec = argv[++i];
    } else if (a == "--dbc" && i + 1 < argc) {
      dbc_path = argv[++i];
    } else if (a == "--telemetry" && i + 1 < argc) {
      opt.telemetry_csv = argv[++i];
    } else if (a == "--realtime") {
      opt.realtime = true;
    } else if (a == "--quiet") {
      opt.print_status = false;
    } else {
      std::fprintf(stderr,
                   "usage: haul-vcu [--bus SPEC] [--dbc FILE] [--telemetry FILE.csv] [--realtime] [--quiet]\n"
                   "  SPEC: socketcan:<ifname> | udp:<listen_port>:<peer_host>:<peer_port>\n");
      return a == "-h" || a == "--help" ? 0 : 2;
    }
  }

  try {
    const auto db = can::Database::load(dbc_path);
    auto bus = can::open_bus(bus_spec);
    vcu::Vcu v(db);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::printf("haul-vcu on %s, 10 ms cycle. Ctrl-C to stop.\n", bus->describe().c_str());
    const auto r = rt::run_vcu_node(v, *bus, opt, g_stop);

    std::printf(
        "\n%ld cycles, %ld deadline misses, %ld frames received (%ld overruns), %ld send errors (%ld overruns)%s\n",
        r.cycles, r.deadline_misses, r.rx_frames, r.rx_overruns, r.tx_errors, r.tx_overruns,
        r.realtime_granted ? ", real-time scheduling" : ", best-effort scheduling");
    std::printf("wake-up jitter  p50 %7.1f us  p99 %7.1f us  max %7.1f us\n", r.wake_jitter.percentile(0.5),
                r.wake_jitter.percentile(0.99), r.wake_jitter.max());
    std::printf("cycle compute   p50 %7.1f us  p99 %7.1f us  max %7.1f us\n", r.compute.percentile(0.5),
                r.compute.percentile(0.99), r.compute.max());
    std::printf("bus write (tx)  p50 %7.1f us  p99 %7.1f us  max %7.1f us\n", r.send_time.percentile(0.5),
                r.send_time.percentile(0.99), r.send_time.max());
    std::printf("step -> on bus  p50 %7.1f us  p99 %7.1f us  max %7.1f us\n", r.tx_latency.percentile(0.5),
                r.tx_latency.percentile(0.99), r.tx_latency.max());
    for (const char* name : {"WheelSpeeds", "Imu", "Powertrain", "Gnss", "MissionCommand"}) {
      const auto& s = v.channel(name).stats();
      std::printf("  %-15s accepted %7ld  crc errors %5ld  sequence errors %5ld\n", name, s.accepted, s.crc_errors,
                  s.sequence_errors);
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "haul-vcu: %s\n", e.what());
    return 1;
  }
}
