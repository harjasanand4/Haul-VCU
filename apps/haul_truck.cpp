// haul-truck: the simulated truck (plant, sensors, fleet management, fault injection) as a
// real-time process, for running against haul-vcu over a real bus.
//
//   haul-truck --bus socketcan:vcan0 --scenario scenarios/06_fms_link_loss_safe_stop.json
//   haul-truck --bus udp:9001:127.0.0.1:9002 --target-kph 30 --payload-t 227 --duration 60

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

#include "haul/can/bus.hpp"
#include "haul/can/dbc.hpp"
#include "haul/core/rt_thread.hpp"
#include "haul/core/units.hpp"
#include "haul/sim/scenario.hpp"
#include "haul/sim/truck_sim.hpp"

using namespace haul;

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) {
  g_stop.store(true);
}
}  // namespace

int main(int argc, char** argv) {
  std::string bus_spec = "udp:9001:127.0.0.1:9002";
  std::string dbc_path = std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc";
  std::string scenario_path, truth_csv;
  double target_kph = 30.0, payload_t = 227.0, duration = 60.0;
  bool realtime = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--bus") {
      bus_spec = next();
    } else if (a == "--dbc") {
      dbc_path = next();
    } else if (a == "--scenario") {
      scenario_path = next();
    } else if (a == "--target-kph") {
      target_kph = std::stod(next());
    } else if (a == "--payload-t") {
      payload_t = std::stod(next());
    } else if (a == "--duration") {
      duration = std::stod(next());
    } else if (a == "--truth") {
      truth_csv = next();
    } else if (a == "--realtime") {
      realtime = true;
    } else {
      std::fprintf(stderr,
                   "usage: haul-truck [--bus SPEC] [--scenario FILE.json | --target-kph K --payload-t T --duration S]"
                   " [--truth FILE.csv] [--realtime]\n");
      return a == "-h" || a == "--help" ? 0 : 2;
    }
  }

  try {
    const auto db = can::Database::load(dbc_path);
    sim::TruckSetup setup;
    if (!scenario_path.empty()) {
      const auto sc = sim::Scenario::load(scenario_path);
      setup = sc.setup;
      duration = sc.duration_s;
      std::printf("scenario %s: %s\n", sc.name.c_str(), sc.description.c_str());
    } else {
      setup.params.payload_kg = payload_t * 1000.0;
      setup.mission = {sim::MissionStep{0.0, 0.0, false, false, {}},
                       sim::MissionStep{1.0, target_kph, true, false, {}}};
    }
    auto bus = can::open_bus(bus_spec);
    sim::TruckSim truck(db, setup);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::ofstream truth;
    if (!truth_csv.empty()) {
      truth.open(truth_csv);
      truth << "t,speed_kph,drive_kn,brake_kn,position_m\n";
    }
    std::printf("haul-truck on %s for %.0f s. Start haul-vcu on the other end.\n", bus->describe().c_str(), duration);

    // The truck stands in for real hardware, so it has to keep time too: a 1 ms loop that
    // stalls for tens of milliseconds looks to the VCU exactly like a dead ECU.
    prefer_interactive_scheduling();
    if (realtime && !make_thread_realtime(ms(1), 200, 800, 70))
      std::fprintf(stderr, "haul-truck: real-time scheduling not granted, running best-effort\n");

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    auto next = start;
    for (Micros now = 0; now <= seconds(duration) && !g_stop.load(); now += ms(1)) {
      next += std::chrono::milliseconds(1);
      std::this_thread::sleep_until(next);
      while (auto f = bus->receive(std::chrono::milliseconds(0))) truck.on_frame(*f, now);
      for (const auto& f : truck.step(now, 0.001)) bus->send(f);

      const auto& s = truck.truth();
      if (now % ms(100) == 0 && truth.is_open()) {
        truth << to_seconds(now) << ',' << mps_to_kph(s.speed_mps) << ',' << s.drive_force_n / 1000.0 << ','
              << s.brake_force_n / 1000.0 << ',' << s.position_m << '\n';
      }
      if (now % kMicrosPerSecond == 0) {
        std::printf("%6.0f s  truck %6.2f km/h  at %7.1f m  drive %7.1f kN  brake %6.1f kN%s\n", to_seconds(now),
                    mps_to_kph(s.speed_mps), s.position_m, s.drive_force_n / 1000.0, s.brake_force_n / 1000.0,
                    truck.watchdog_tripped() ? "  [drive watchdog: no VCU commands]" : "");
        std::fflush(stdout);
      }
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "haul-truck: %s\n", e.what());
    return 1;
  }
}
