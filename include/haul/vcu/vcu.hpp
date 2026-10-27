#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "haul/can/dbc.hpp"
#include "haul/can/e2e.hpp"
#include "haul/core/time.hpp"
#include "haul/vcu/calibration.hpp"
#include "haul/vcu/estimator.hpp"
#include "haul/vcu/faults.hpp"
#include "haul/vcu/monitors.hpp"
#include "haul/vcu/speed_controller.hpp"
#include "haul/vcu/state_machine.hpp"

namespace haul::vcu {

// What the VCU is doing, for telemetry, tests and the scenario checks.
struct VcuDebug {
  Micros time = 0;
  VcuState state = VcuState::Init;
  const char* state_reason = "power on";  // why the last transition happened (a literal)
  FaultSet faults;
  double speed_estimate_mps = 0.0;
  double speed_variance = 0.0;
  double target_mps = 0.0;
  double setpoint_mps = 0.0;
  double speed_limit_mps = 0.0;
  double traction_request_n = 0.0;
  double brake_request_pct = 0.0;
  double feedforward_n = 0.0;
  double mass_kg = 0.0;
  double pitch_rad = 0.0;
  double step_compute_us = 0.0;  // filled in by the real-time runtime
};

// The frames one control cycle transmits. Fixed capacity, so a cycle never allocates.
class FrameBatch {
 public:
  void push(const can::Frame& f) { frames_.at(size_++) = f; }
  const can::Frame* begin() const { return frames_.data(); }
  const can::Frame* end() const { return frames_.data() + size_; }
  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  const can::Frame& operator[](std::size_t i) const { return frames_.at(i); }

 private:
  std::array<can::Frame, 4> frames_{};
  std::size_t size_ = 0;
};

// The vehicle control unit. No I/O and no clock of its own: frames come in through
// on_frame(), and step() runs one 10 ms control cycle and returns the frames to transmit.
// That keeps every decision deterministic and lets the same object run inside the
// software-in-the-loop simulator and inside the real-time node. Everything is allocated in the
// constructor: on_frame() and step() never touch the heap (tests/test_no_alloc.cpp checks).
class Vcu {
 public:
  static constexpr Micros kPeriod = ms(10);

  explicit Vcu(const can::Database& db, Calibration cal = {});

  void on_frame(const can::Frame& f, Micros now);
  FrameBatch step(Micros now);

  const VcuDebug& debug() const { return debug_; }
  const Calibration& calibration() const { return cal_; }
  const StateMachine& state_machine() const { return sm_; }
  const InputChannel& channel(const char* message) const;

 private:
  void update_estimator(double dt, Micros now);
  void update_faults(Micros now);
  void update_setpoint(double dt);
  can::Frame encode_command(Micros now);
  can::Frame encode_status();

  Calibration cal_;
  SpeedEstimator estimator_;
  SpeedController controller_;
  StateMachine sm_;

  // Inputs
  InputChannel wheels_, imu_, powertrain_, gnss_, mission_;
  std::unordered_map<std::uint32_t, InputChannel*> by_id_;
  std::size_t i_rear_left_, i_rear_right_, i_front_, i_accel_, i_pitch_, i_force_actual_, i_payload_, i_gnss_speed_,
      i_gnss_valid_, i_target_, i_mode_, i_estop_;

  // Outputs
  static constexpr std::size_t kMaxSignals = 16;
  std::array<double, kMaxSignals> scratch_{};  // signal values for encoding, reused every cycle
  const can::Message* command_msg_;
  const can::Message* status_msg_;
  can::E2EProfile command_e2e_, status_e2e_;
  std::uint8_t command_counter_ = 0, status_counter_ = 0;
  std::size_t i_cmd_force_, i_cmd_brake_, i_cmd_state_, i_st_speed_, i_st_limit_, i_st_faults_;

  // Fault detection
  Debouncer slip_, implausible_, gnss_invalid_, actuator_, stopped_;
  FaultSet faults_;
  double expected_force_n_ = 0.0;  // command passed through the drive's expected lag

  // Control
  SetpointProfile profile_;
  double pitch_rad_ = 0.0;
  double last_front_mps_ = 0.0;
  double last_rear_mps_ = 0.0;
  double last_gnss_mps_ = 0.0;
  ControlOutput last_out_;
  std::optional<Micros> last_step_;
  Micros last_status_ = -kMicrosPerSecond;
  VcuDebug debug_;
};

}  // namespace haul::vcu
