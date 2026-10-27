// The real-time node end to end: rx, control, tx and telemetry threads against a simulated truck
// running in its own thread on the other end of a UDP "bus". The timing assertions are loose
// on purpose (CI runners are noisy); the point is that the threads cooperate correctly, which
// the ThreadSanitizer job checks on every push.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "haul/can/bus.hpp"
#include "haul/can/dbc.hpp"
#include "haul/rt/vcu_node.hpp"
#include "haul/sim/truck_sim.hpp"
#include "haul/vcu/vcu.hpp"

using namespace haul;
using Clock = std::chrono::steady_clock;

TEST(VcuNode, RunsTheControlLoopAgainstATruckOverUdp) {
  const auto db = can::Database::load(std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc");
  auto vcu_bus = can::open_bus("udp:47202:127.0.0.1:47201");
  auto truck_bus = can::open_bus("udp:47201:127.0.0.1:47202");

  sim::TruckSetup setup;
  setup.mission = {sim::MissionStep{0.0, 0.0, false, false, {}}, sim::MissionStep{0.2, 10.0, true, false, {}}};
  std::atomic<bool> stop{false};
  std::atomic<bool> truck_got_commands{false};

  std::thread truck([&] {
    sim::TruckSim sim(db, setup);
    const auto start = Clock::now();
    auto next = start;
    for (Micros now = 0; !stop.load(); now += ms(1)) {
      next += std::chrono::milliseconds(1);
      std::this_thread::sleep_until(next);
      while (auto f = truck_bus->receive(std::chrono::milliseconds(0))) {
        sim.on_frame(*f, now);
        truck_got_commands = true;
      }
      for (const auto& f : sim.step(now, 0.001)) truck_bus->send(f);
    }
  });

  vcu::Vcu v(db);
  rt::NodeOptions opt;
  opt.print_status = false;
  std::thread stopper([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    stop = true;
  });
  const auto report = rt::run_vcu_node(v, *vcu_bus, opt, stop);
  stopper.join();
  truck.join();

  EXPECT_GT(report.cycles, 150);  // ~200 at 10 ms
  EXPECT_LT(report.cycles, 220);
  EXPECT_GT(report.rx_frames, 400);
  EXPECT_EQ(report.rx_overruns, 0);
  EXPECT_EQ(report.tx_errors, 0);
  EXPECT_EQ(report.wake_jitter.count(), static_cast<std::size_t>(report.cycles));
  EXPECT_TRUE(truck_got_commands.load());
  EXPECT_EQ(v.debug().state, vcu::VcuState::Driving) << vcu::to_string(v.debug().state);
  EXPECT_GT(v.channel("WheelSpeeds").stats().accepted, 100);
  EXPECT_EQ(v.channel("WheelSpeeds").stats().crc_errors, 0);
}
