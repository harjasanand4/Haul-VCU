#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "haul/can/dbc.hpp"
#include "haul/can/e2e.hpp"
#include "haul/core/time.hpp"
#include "haul/sim/truck_model.hpp"

namespace haul::sim {

// Haul road: grade (percent) at points along the route, linearly interpolated.
struct Road {
  std::vector<std::pair<double, double>> points{{0.0, 0.0}};  // (distance m, grade %)
  double grade_pct_at(double x) const;
};

struct WetPatch {
  double from_m = 0.0;
  double to_m = 0.0;
  double mu = 0.1;
};

// A mission update from fleet management. It takes effect at time `t_s`, or, if `at_m` is
// set, when the truck reaches that point on the road (speed zones: "15 km/h on the ramp").
struct MissionStep {
  double t_s = 0.0;
  double target_kph = 0.0;
  bool autonomous = true;
  bool estop = false;
  std::optional<double> at_m;
};

// Something to break during a run.
//   drop            message stops being sent (any message name, including VcuCommand)
//   corrupt_crc     a share (`amount`, 0..1) of the message's frames get a bad CRC
//   freeze_counter  message keeps being sent with the same alive counter (stale data)
//   gnss_invalid    receiver loses its fix (GnssValid = 0)
//   wheel_bias      front-axle sensor reads `amount` m/s high
//   drive_failure   the drive motors stop producing force
struct FaultInjection {
  std::string type;
  std::string message;
  double t_s = 0.0;
  double duration_s = 1e9;
  double amount = 1.0;
  bool active(double t) const { return t >= t_s && t < t_s + duration_s; }
};

struct TruckSetup {
  TruckParams params;
  Road road;
  std::vector<WetPatch> wet_patches;
  std::vector<MissionStep> mission{MissionStep{0.0, 0.0, false, false, {}}};
  std::vector<FaultInjection> faults;
  std::uint64_t seed = 42;
};

// Deterministic normal noise. std::normal_distribution's output is implementation-defined
// (libstdc++ and libc++ differ), so this uses Box-Muller on mt19937_64, which is specified
// bit-for-bit by the standard: a scenario gives the same numbers on Linux and macOS.
class Noise {
 public:
  explicit Noise(std::uint64_t seed) : eng_(seed) {}
  double uniform();
  double gaussian(double sigma);

 private:
  std::mt19937_64 eng_;
  std::optional<double> spare_;
};

// Everything on the truck side of the network: the plant, its sensors and ECUs, the fleet
// management system sending missions, and the fault injector. It produces the frames the
// VCU receives and consumes the VCU's commands, so the SIL runner and the real-time truck
// node can share it.
class TruckSim {
 public:
  TruckSim(const can::Database& db, TruckSetup setup);

  void on_frame(const can::Frame& f, Micros now);
  // Advance the plant to `now` (by `dt` seconds) and return the frames due at `now`.
  std::vector<can::Frame> step(Micros now, double dt);

  const TruckState& truth() const { return model_.state(); }
  const TruckModel& model() const { return model_; }
  const MissionStep& mission() const { return setup_.mission[mission_index_]; }
  double drive_request_n() const { return drive_req_; }
  double brake_request_pct() const { return brake_req_; }
  bool watchdog_tripped() const { return watchdog_; }
  long frames_dropped() const { return dropped_; }
  long frames_corrupted() const { return corrupted_; }

 private:
  struct Tx {
    const can::Message* msg = nullptr;
    can::E2EProfile e2e;
    Micros period = 0;
    Micros phase = 0;
    std::uint8_t counter = 0;
  };

  bool fault(const std::string& type, const std::string& message, double t) const;
  const FaultInjection* find_fault(const std::string& type, const std::string& message, double t) const;
  std::optional<can::Frame> emit(Tx& tx, Micros now);

  TruckSetup setup_;
  TruckModel model_;
  Noise noise_;
  std::vector<Tx> tx_;
  std::size_t mission_index_ = 0;
  const can::Message* command_msg_;
  can::E2EProfile command_e2e_;
  can::E2EReceiver command_rx_;
  std::size_t i_force_, i_brake_;
  std::deque<std::pair<Micros, double>> gnss_history_;  // for the receiver's ~100 ms latency
  double drive_req_ = 0.0;
  double brake_req_ = 100.0;  // parked until the VCU says otherwise
  std::optional<Micros> last_command_;
  bool watchdog_ = false;
  long dropped_ = 0;
  long corrupted_ = 0;
};

}  // namespace haul::sim
