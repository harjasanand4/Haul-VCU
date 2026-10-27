#include "haul/sim/sil_runner.hpp"

#include <deque>
#include <memory>

#include "haul/can/candump.hpp"
#include "haul/core/units.hpp"
#include "haul/sim/truck_sim.hpp"
#include "haul/vcu/vcu.hpp"

namespace haul::sim {

namespace {

// A FIFO with a fixed delivery delay. CAN delivers frames in order, so a queue is enough.
struct InFlight {
  Micros deliver_at;
  can::Frame frame;
};

}  // namespace

Run run_sil(const can::Database& db, const Scenario& sc, const SilOptions& opt) {
  TruckSim truck(db, sc.setup);
  vcu::Vcu vcu(db, opt.calibration);
  std::unique_ptr<can::CandumpWriter> log;
  if (opt.candump_path) log = std::make_unique<can::CandumpWriter>(*opt.candump_path);

  std::deque<InFlight> to_vcu, to_truck;
  Run run;
  const Micros end = seconds(sc.duration_s);
  const double dt = to_seconds(opt.plant_step);
  vcu::VcuState last_state = vcu::VcuState::Init;

  for (Micros now = 0; now <= end; now += opt.plant_step) {
    for (const auto& f : truck.step(now, dt)) {
      to_vcu.push_back({now + opt.bus_latency, f});
      if (log) log->write(now, f);
      ++run.frames_to_vcu;
    }
    while (!to_vcu.empty() && to_vcu.front().deliver_at <= now) {
      vcu.on_frame(to_vcu.front().frame, to_vcu.front().deliver_at);
      to_vcu.pop_front();
    }

    if (now % vcu::Vcu::kPeriod == 0) {
      for (const auto& f : vcu.step(now)) {
        to_truck.push_back({now + opt.bus_latency, f});
        if (log) log->write(now, f);
        ++run.frames_to_truck;
      }
      const auto& d = vcu.debug();
      const auto& s = truck.truth();
      Sample x;
      x.t = to_seconds(now);
      x.speed_kph = mps_to_kph(s.speed_mps);
      x.estimate_kph = mps_to_kph(d.speed_estimate_mps);
      x.setpoint_kph = mps_to_kph(d.setpoint_mps);
      x.target_kph = mps_to_kph(d.target_mps);
      x.limit_kph = mps_to_kph(d.speed_limit_mps);
      x.drive_request_kn = d.traction_request_n / 1000.0;
      x.drive_actual_kn = s.drive_force_n / 1000.0;
      x.brake_request_pct = d.brake_request_pct;
      x.brake_force_kn = s.brake_force_n / 1000.0;
      x.accel_mps2 = s.accel_mps2;
      x.grade_pct = std::tan(s.grade_rad) * 100.0;
      x.position_m = s.position_m;
      x.slip = s.slip;
      x.state = d.state;
      x.faults = d.faults;
      run.samples.push_back(x);
      if (d.state != last_state) {
        run.transitions.push_back({x.t, last_state, d.state, vcu.state_machine().reason()});
        last_state = d.state;
      }
    }
    while (!to_truck.empty() && to_truck.front().deliver_at <= now) {
      truck.on_frame(to_truck.front().frame, to_truck.front().deliver_at);
      to_truck.pop_front();
    }
    run.truck_watchdog_tripped = run.truck_watchdog_tripped || truck.watchdog_tripped();
  }
  run.frames_dropped = truck.frames_dropped();
  run.frames_corrupted = truck.frames_corrupted();
  return run;
}

}  // namespace haul::sim
