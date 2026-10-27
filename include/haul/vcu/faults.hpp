#pragma once

#include <cstdint>
#include <string>

namespace haul::vcu {

// Bit flags reported in VcuStatus.FaultFlags.
enum class Fault : std::uint16_t {
  WheelSpeedTimeout = 1u << 0,
  ImuTimeout = 1u << 1,
  PowertrainTimeout = 1u << 2,
  GnssTimeout = 1u << 3,
  MissionTimeout = 1u << 4,
  E2EError = 1u << 5,          // repeated CRC / counter failures on an input
  WheelSlip = 1u << 6,         // driven axle spinning faster than the undriven axle
  SpeedImplausible = 1u << 7,  // wheel speed and GNSS speed disagree
  GnssInvalid = 1u << 8,       // receiver reports no fix
  ActuatorMismatch = 1u << 9,  // drive not delivering the force it was asked for
};

class FaultSet {
 public:
  constexpr FaultSet() = default;
  constexpr explicit FaultSet(std::uint16_t bits) : bits_(bits) {}

  constexpr bool has(Fault f) const { return (bits_ & static_cast<std::uint16_t>(f)) != 0; }
  constexpr void set(Fault f, bool on) {
    bits_ = on ? static_cast<std::uint16_t>(bits_ | static_cast<std::uint16_t>(f))
               : static_cast<std::uint16_t>(bits_ & ~static_cast<std::uint16_t>(f));
  }
  constexpr std::uint16_t bits() const { return bits_; }
  constexpr bool any() const { return bits_ != 0; }

  // Faults that make it unsafe to keep driving: stop the truck in a controlled way.
  constexpr bool critical() const {
    const bool no_speed_source = has(Fault::WheelSpeedTimeout) && (has(Fault::GnssTimeout) || has(Fault::GnssInvalid));
    return has(Fault::MissionTimeout) || has(Fault::PowertrainTimeout) || has(Fault::ActuatorMismatch) ||
           has(Fault::SpeedImplausible) || no_speed_source;
  }
  // Faults the truck can drive through at reduced speed. (E2EError on its own is only a
  // diagnostic: bad frames are already discarded, and if too many go missing the message
  // times out, which is handled above.)
  constexpr bool degraded() const {
    return has(Fault::WheelSpeedTimeout) || has(Fault::ImuTimeout) || has(Fault::GnssTimeout) ||
           has(Fault::GnssInvalid);
  }

  friend constexpr bool operator==(FaultSet, FaultSet) = default;

 private:
  std::uint16_t bits_ = 0;
};

const char* fault_name(Fault f);
std::string describe(FaultSet faults);  // "GnssInvalid|WheelSlip" or "none"
bool parse_fault(const std::string& name, Fault& out);

}  // namespace haul::vcu
