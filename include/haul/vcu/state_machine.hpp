#pragma once

#include <cstdint>
#include <string>

#include "haul/core/time.hpp"
#include "haul/vcu/calibration.hpp"
#include "haul/vcu/monitors.hpp"

namespace haul::vcu {

enum class VcuState : std::uint8_t { Init = 0, Ready = 1, Driving = 2, SafeStop = 3, EmergencyStop = 4, Fault = 5 };

const char* to_string(VcuState s);
bool parse_state(const std::string& name, VcuState& out);

struct StateInputs {
  Micros now = 0;
  bool inputs_ready = false;  // every required message received and fresh
  bool critical = false;      // a fault that forbids driving
  bool estop = false;
  bool mission_auto = false;
  double target_mps = 0.0;
  bool stopped = false;  // confirmed standstill
};

//            estop (from any state)
//   INIT ───────────────────────────────────────────▶ EMERGENCY_STOP ──stopped & released──┐
//    │ inputs ready, no critical fault for 0.5 s                                            ▼
//    ▼                                   critical fault                                  FAULT
//   READY ──AUTO & target > 0──▶ DRIVING ──────────────▶ SAFE_STOP ──stopped──────────────▲
//    ▲  │                           │                                                      │
//    │  └───────critical──────────────────────────────────────────────────────────────────┤
//    └──────────── STOP & stopped ──┘        FAULT clears to READY after STOP is held 1 s ─┘
//                                                         with no critical fault
class StateMachine {
 public:
  explicit StateMachine(const Calibration& cal) : ready_(cal.init_settle, 0), clear_(cal.fault_clear, 0) {}

  VcuState step(const StateInputs& in);
  VcuState state() const { return state_; }
  Micros entered_at() const { return entered_; }
  const char* reason() const { return reason_; }  // why the last transition happened

 private:
  void go(VcuState next, Micros now, const char* why);

  VcuState state_ = VcuState::Init;
  Micros entered_ = 0;
  const char* reason_ = "power on";  // always a string literal: no allocation on a transition
  Debouncer ready_;
  Debouncer clear_;
};

}  // namespace haul::vcu
