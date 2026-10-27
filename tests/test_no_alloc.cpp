// The control cycle must not touch the heap: a malloc can take a lock or page-fault, and on a
// real-time target either one can blow the 10 ms deadline. This binary replaces global
// operator new with a counting one and runs the VCU through a drive with faults injected,
// counting allocations made inside Vcu::on_frame() and Vcu::step() only.
//
// (Built without sanitizers: they bring their own allocator.)

#include <gtest/gtest.h>

#include <cstdlib>
#include <new>
#include <string>

#include "haul/can/dbc.hpp"
#include "haul/sim/truck_sim.hpp"
#include "haul/vcu/vcu.hpp"

namespace {
long g_allocations = 0;
bool g_counting = false;
}  // namespace

void* operator new(std::size_t n) {
  if (g_counting) ++g_allocations;
  if (void* p = std::malloc(n == 0 ? 1 : n)) return p;
  throw std::bad_alloc();
}
void* operator new[](std::size_t n) {
  return operator new(n);
}
void operator delete(void* p) noexcept {
  std::free(p);
}
void operator delete[](void* p) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}

using namespace haul;

TEST(NoAlloc, TheCounterSeesAllocations) {  // check the instrument before trusting a zero from it
  const long before = g_allocations;
  g_counting = true;
  auto* p = new std::string(100, 'x');  // too long for the small-string buffer: two allocations
  g_counting = false;
  delete p;
  EXPECT_EQ(g_allocations - before, 2);
}

TEST(NoAlloc, ControlCycleNeverAllocates) {
  const auto db = can::Database::load(std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc");
  sim::TruckSetup setup;
  setup.params.payload_kg = 227'000.0;
  setup.road.points = {{0.0, 0.0}, {200.0, 0.0}, {240.0, 8.0}, {5000.0, 8.0}};
  setup.wet_patches = {{20.0, 80.0, 0.08}};
  setup.mission = {sim::MissionStep{0.0, 0.0, false, false, {}}, sim::MissionStep{1.0, 30.0, true, false, {}},
                   sim::MissionStep{100.0, 30.0, true, true, {}}};  // e-stop at the end
  // Exercise the error paths too: corrupted frames, a stale IMU, a lost GNSS fix.
  setup.faults = {{"corrupt_crc", "WheelSpeeds", 30.0, 5.0, 0.3},
                  {"freeze_counter", "Imu", 45.0, 5.0, 1.0},
                  {"gnss_invalid", "", 60.0, 10.0, 1.0}};
  sim::TruckSim truck(db, setup);
  vcu::Vcu v(db);

  const long before = g_allocations;
  long cycles = 0;
  for (Micros now = 0; now < seconds(110); now += ms(1)) {
    const auto frames = truck.step(now, 0.001);  // the simulator may allocate; it's not on the truck
    g_counting = true;
    for (const auto& f : frames) v.on_frame(f, now);
    if (now % vcu::Vcu::kPeriod == 0) {
      const auto out = v.step(now);
      g_counting = false;
      for (const auto& f : out) truck.on_frame(f, now);
      ++cycles;
    }
    g_counting = false;
  }
  EXPECT_EQ(g_allocations - before, 0) << "heap allocations inside the control path over " << cycles << " cycles";
  EXPECT_EQ(v.debug().state, vcu::VcuState::EmergencyStop);  // the run really went somewhere
  EXPECT_GT(v.channel("WheelSpeeds").stats().crc_errors, 0);
  EXPECT_GT(v.channel("Imu").stats().sequence_errors, 0);
}
