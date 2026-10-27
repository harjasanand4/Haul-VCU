#include "haul/vcu/state_machine.hpp"

#include <array>

namespace haul::vcu {

namespace {
constexpr std::array<const char*, 6> kNames{"INIT", "READY", "DRIVING", "SAFE_STOP", "EMERGENCY_STOP", "FAULT"};
}

const char* to_string(VcuState s) {
  return kNames[static_cast<std::size_t>(s)];
}

bool parse_state(const std::string& name, VcuState& out) {
  for (std::size_t i = 0; i < kNames.size(); ++i) {
    if (name == kNames[i]) {
      out = static_cast<VcuState>(i);
      return true;
    }
  }
  return false;
}

void StateMachine::go(VcuState next, Micros now, const char* why) {
  state_ = next;
  entered_ = now;
  reason_ = why;
  ready_.reset();
  clear_.reset();
}

VcuState StateMachine::step(const StateInputs& in) {
  if (in.estop && state_ != VcuState::EmergencyStop) {
    go(VcuState::EmergencyStop, in.now, "e-stop from fleet management");
    return state_;
  }
  switch (state_) {
    case VcuState::Init:
      if (ready_.update(in.inputs_ready && !in.critical, in.now)) go(VcuState::Ready, in.now, "inputs healthy");
      break;
    case VcuState::Ready:
      if (in.critical) {
        go(VcuState::Fault, in.now, "critical fault at standstill");
      } else if (in.mission_auto && in.target_mps > 0.05) {
        go(VcuState::Driving, in.now, "mission AUTO");
      }
      break;
    case VcuState::Driving:
      if (in.critical) {
        go(VcuState::SafeStop, in.now, "critical fault while driving");
      } else if ((!in.mission_auto || in.target_mps <= 0.05) && in.stopped) {
        go(VcuState::Ready, in.now, "mission complete");
      }
      break;
    case VcuState::SafeStop:
      if (in.stopped) go(VcuState::Fault, in.now, "safe stop complete");
      break;
    case VcuState::EmergencyStop:
      if (in.stopped && !in.estop) go(VcuState::Fault, in.now, "e-stop released at standstill");
      break;
    case VcuState::Fault:
      if (clear_.update(!in.critical && !in.mission_auto && !in.estop, in.now)) {
        go(VcuState::Ready, in.now, "fault cleared by operator (STOP held)");
      }
      break;
  }
  return state_;
}

}  // namespace haul::vcu
