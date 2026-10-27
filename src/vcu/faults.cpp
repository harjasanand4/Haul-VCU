#include "haul/vcu/faults.hpp"

#include <array>
#include <utility>

namespace haul::vcu {

namespace {
constexpr std::array<std::pair<Fault, const char*>, 10> kFaults{{
    {Fault::WheelSpeedTimeout, "WheelSpeedTimeout"},
    {Fault::ImuTimeout, "ImuTimeout"},
    {Fault::PowertrainTimeout, "PowertrainTimeout"},
    {Fault::GnssTimeout, "GnssTimeout"},
    {Fault::MissionTimeout, "MissionTimeout"},
    {Fault::E2EError, "E2EError"},
    {Fault::WheelSlip, "WheelSlip"},
    {Fault::SpeedImplausible, "SpeedImplausible"},
    {Fault::GnssInvalid, "GnssInvalid"},
    {Fault::ActuatorMismatch, "ActuatorMismatch"},
}};
}  // namespace

const char* fault_name(Fault f) {
  for (const auto& [fault, name] : kFaults) {
    if (fault == f) return name;
  }
  return "?";
}

std::string describe(FaultSet faults) {
  std::string out;
  for (const auto& [fault, name] : kFaults) {
    if (!faults.has(fault)) continue;
    if (!out.empty()) out += '|';
    out += name;
  }
  return out.empty() ? "none" : out;
}

bool parse_fault(const std::string& name, Fault& out) {
  for (const auto& [fault, n] : kFaults) {
    if (name == n) {
      out = fault;
      return true;
    }
  }
  return false;
}

}  // namespace haul::vcu
