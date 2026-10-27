// haul-bench: how fast are the hot paths? Run a Release build.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "haul/can/dbc.hpp"
#include "haul/can/e2e.hpp"
#include "haul/core/spsc_ring.hpp"
#include "haul/sim/sil_runner.hpp"
#include "haul/sim/truck_sim.hpp"
#include "haul/vcu/vcu.hpp"

using namespace haul;
using Clock = std::chrono::steady_clock;

namespace {

volatile double g_sink = 0.0;  // keeps the optimiser from deleting the work

template <typename F>
double ns_per_op(long iterations, F&& f) {
  const auto t0 = Clock::now();
  for (long i = 0; i < iterations; ++i) f(i);
  return std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / static_cast<double>(iterations);
}

}  // namespace

int main() {
  const auto db = can::Database::load(std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc");
  const auto& cmd = db.message("VcuCommand");
  const auto& wheels = db.message("WheelSpeeds");

  std::vector<double> values(cmd.signals.size(), 0.0);
  values[0] = -123.4;
  values[1] = 37.5;
  values[2] = 2;
  const double enc = ns_per_op(2'000'000, [&](long i) {
    values[0] = static_cast<double>(i % 1000);
    g_sink = g_sink + cmd.encode(values).data[0];
  });
  const auto frame = cmd.encode(values);
  std::vector<double> out(cmd.signals.size());
  const double dec = ns_per_op(2'000'000, [&](long) {
    cmd.decode(frame, out);
    g_sink = g_sink + out[0];
  });
  const can::E2EProfile prof(wheels);
  auto wf = wheels.encode(std::vector<double>(wheels.signals.size(), 1.0));
  const double crc = ns_per_op(5'000'000, [&](long i) {
    wf.data[0] = static_cast<std::uint8_t>(i);
    g_sink = g_sink + prof.compute_crc(wf);
  });

  // VCU control cycle: feed one cycle's worth of frames, then step.
  sim::TruckSetup setup;
  setup.params.payload_kg = 227'000.0;
  setup.mission = {sim::MissionStep{0.0, 0.0, false, false, {}}, sim::MissionStep{0.5, 40.0, true, false, {}}};
  sim::TruckSim truck(db, setup);
  vcu::Vcu v(db);
  double step_ns = 0.0;
  long steps = 0;
  for (Micros now = 0; now < seconds(120); now += ms(1)) {
    for (const auto& f : truck.step(now, 0.001)) v.on_frame(f, now);
    if (now % vcu::Vcu::kPeriod == 0) {
      const auto t0 = Clock::now();
      const auto frames = v.step(now);
      step_ns += std::chrono::duration<double, std::nano>(Clock::now() - t0).count();
      ++steps;
      for (const auto& f : frames) truck.on_frame(f, now);
    }
  }

  // SPSC ring across two threads.
  constexpr long kItems = 20'000'000;
  auto ring = std::make_unique<SpscRing<long, 1024>>();
  std::atomic<long> sum{0};
  const auto r0 = Clock::now();
  std::thread consumer([&] {
    long got = 0, s = 0;
    while (got < kItems) {
      if (auto x = ring->try_pop()) {
        s += *x;
        ++got;
      }
    }
    sum = s;
  });
  for (long i = 0; i < kItems; ++i) {
    while (!ring->try_push(i)) {
    }
  }
  consumer.join();
  const double ring_s = std::chrono::duration<double>(Clock::now() - r0).count();
  const bool ring_ok = sum.load() == kItems * (kItems - 1) / 2;

  // Whole software-in-the-loop run.
  sim::Scenario sc;
  sc.name = "bench";
  sc.duration_s = 600.0;
  sc.setup = setup;
  const auto s0 = Clock::now();
  sim::run_sil(db, sc);
  const double sil_s = std::chrono::duration<double>(Clock::now() - s0).count();

  std::printf("DBC encode (VcuCommand, Motorola)   %8.1f ns/frame\n", enc);
  std::printf("DBC decode (VcuCommand)             %8.1f ns/frame\n", dec);
  std::printf("E2E CRC-8 (SAE J1850)               %8.1f ns/frame\n", crc);
  std::printf("VCU control cycle (Vcu::step)       %8.2f us/cycle  (budget 10 000 us)\n",
              step_ns / static_cast<double>(steps) / 1000.0);
  std::printf("SPSC ring, 2 threads                %8.1f M items/s %s\n", static_cast<double>(kItems) / ring_s / 1e6,
              ring_ok ? "(checksum ok)" : "(CHECKSUM MISMATCH)");
  std::printf("SIL simulation, 600 s scenario      %8.0f x real time\n", sc.duration_s / sil_s);
  return ring_ok ? 0 : 1;
}
