#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>

#include "haul/can/dbc.hpp"
#include "haul/can/e2e.hpp"
#include "haul/core/units.hpp"
#include "haul/sim/truck_sim.hpp"
#include "haul/vcu/estimator.hpp"
#include "haul/vcu/faults.hpp"
#include "haul/vcu/monitors.hpp"
#include "haul/vcu/speed_controller.hpp"
#include "haul/vcu/state_machine.hpp"
#include "haul/vcu/vcu.hpp"

using namespace haul;
using namespace haul::vcu;

namespace {

const can::Database& db() {
  static const can::Database d = can::Database::load(std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc");
  return d;
}

constexpr double kLoadedMass = 165'000.0 + 227'000.0;

}  // namespace

// ---------------------------------------------------------------- setpoint profile

TEST(SetpointProfile, RespectsAccelAndJerkLimitsAndNeverOvershoots) {
  const Calibration cal;
  SetpointProfile p;
  p.reset(0.0);
  const double goal = kph_to_mps(40.0), dt = 0.01;
  double peak = 0.0;
  int t_arrive = -1;
  for (int k = 0; k < 10'000; ++k) {
    p.step(goal, cal.accel_limit, cal.decel_limit, cal.jerk_limit, dt);
    EXPECT_LE(p.accel(), cal.accel_limit + 1e-9);
    EXPECT_LE(std::abs(p.jerk()), cal.jerk_limit + 1e-6) << "at step " << k;
    peak = std::max(peak, p.speed());
    if (t_arrive < 0 && p.speed() == goal) t_arrive = k;
  }
  EXPECT_DOUBLE_EQ(peak, goal);
  EXPECT_DOUBLE_EQ(p.speed(), goal);
  EXPECT_DOUBLE_EQ(p.accel(), 0.0);
  // 11.1 m/s at 0.45 m/s^2 is ~24.7 s, plus ~1.3 s per jerk-limited corner.
  EXPECT_GT(t_arrive * dt, 24.7);
  EXPECT_LT(t_arrive * dt, 28.5);
}

TEST(SetpointProfile, SlowsDownWithTheDecelLimit) {
  const Calibration cal;
  SetpointProfile p;
  p.reset(10.0);
  double lowest_accel = 0.0;
  for (int k = 0; k < 5'000; ++k) {
    p.step(0.0, cal.accel_limit, cal.decel_limit, cal.jerk_limit, 0.01);
    lowest_accel = std::min(lowest_accel, p.accel());
    EXPECT_GE(p.speed(), 0.0);
  }
  EXPECT_NEAR(lowest_accel, -cal.decel_limit, 1e-9);
  EXPECT_DOUBLE_EQ(p.speed(), 0.0);
}

// ---------------------------------------------------------------- speed controller

TEST(SpeedController, HoldsTheBrakeAtStandstill) {
  const Calibration cal;
  SpeedController c(cal);
  ControlInput in;
  in.mass_kg = kLoadedMass;
  const auto out = c.step(in);
  EXPECT_DOUBLE_EQ(out.traction_n, 0.0);
  EXPECT_DOUBLE_EQ(out.brake_pct, cal.hold_brake_pct);
}

TEST(SpeedController, FeedforwardCancelsTheRoadLoadOnAClimb) {
  const Calibration cal;
  SpeedController c(cal);
  ControlInput in;
  in.setpoint_mps = in.speed_mps = 5.0;
  in.mass_kg = kLoadedMass;
  in.pitch_rad = grade_pct_to_rad(10.0);
  const auto out = c.step(in);
  const double expected =
      kLoadedMass * kGravity * (std::sin(in.pitch_rad) + cal.rolling_resistance * std::cos(in.pitch_rad)) +
      0.5 * cal.air_density * cal.drag_area_m2 * 25.0;
  EXPECT_NEAR(out.feedforward_n, expected, 1.0);
  EXPECT_NEAR(out.traction_n, expected, 1.0);  // zero error: the PI part contributes nothing
  EXPECT_DOUBLE_EQ(out.brake_pct, 0.0);
  EXPECT_FALSE(out.saturated);
}

TEST(SpeedController, RetardsElectricallyDownhillBeforeUsingFrictionBrakes) {
  const Calibration cal;
  SpeedController c(cal);
  ControlInput in;
  in.setpoint_mps = in.speed_mps = 10.0;
  in.mass_kg = kLoadedMass;
  in.pitch_rad = grade_pct_to_rad(-8.0);
  auto out = c.step(in);
  EXPECT_LT(out.traction_n, 0.0);
  EXPECT_DOUBLE_EQ(out.brake_pct, 0.0);

  // Steeper than the retarder can hold at this speed (it is power limited): the brakes make
  // up the difference.
  in.pitch_rad = grade_pct_to_rad(-14.0);
  c.reset();
  out = c.step(in);
  EXPECT_NEAR(out.traction_n, -c.max_retard(10.0), 1.0);
  EXPECT_GT(out.brake_pct, 0.0);
  EXPECT_NEAR(out.traction_n - out.brake_pct / 100.0 * cal.max_brake_n, out.requested_n, 1.0);
}

TEST(SpeedController, AntiWindupKeepsTheIntegratorBoundedWhileSaturated) {
  // Asked for 15 m/s while stuck at 3 m/s on a 10 % ramp (the drive is at its power limit).
  const auto run = [](const Calibration& cal, int steps) {
    SpeedController c(cal);
    ControlInput in;
    in.setpoint_mps = 15.0;
    in.speed_mps = 3.0;
    in.mass_kg = kLoadedMass;
    in.pitch_rad = grade_pct_to_rad(10.0);
    for (int k = 0; k < steps; ++k) EXPECT_TRUE(c.step(in).saturated);
    return c.integral();
  };
  Calibration naive;
  naive.kaw = 0.0;
  EXPECT_NEAR(run(naive, 6'000), 12.0 * 60.0, 1e-6);  // without anti-windup: 12 m/s for 60 s
  const Calibration cal;
  const double after_30s = run(cal, 3'000), after_60s = run(cal, 6'000);
  EXPECT_LT(std::abs(after_60s), 0.1 * 720.0);
  EXPECT_NEAR(after_30s, after_60s, 0.5) << "the integrator settles instead of growing";
}

TEST(SpeedController, TractionControlCutsForceWhileTheWheelsSpin) {
  const Calibration cal;
  SpeedController c(cal);
  ControlInput in;
  in.setpoint_mps = 8.0;
  in.speed_mps = 2.0;
  in.mass_kg = kLoadedMass;
  const double before = c.step(in).traction_n;
  in.slip = true;
  double last = before;
  for (int k = 0; k < 50; ++k) {
    const double f = c.step(in).traction_n;
    EXPECT_LE(f, cal.slip_traction_scale * before + 1e-6);
    EXPECT_LE(f, last + 1e-6);  // keeps shrinking while slipping
    last = f;
  }
  EXPECT_TRUE(c.traction_control_active());
  in.slip = false;
  for (int k = 0; k < 20; ++k) {
    const double f = c.step(in).traction_n;
    EXPECT_LE(f - last, cal.slip_recovery_n_per_s * in.dt + 1e-6);  // gives force back gradually
    last = f;
  }
}

// ---------------------------------------------------------------- estimator

TEST(SpeedEstimator, LearnsTheAccelerometerBias) {
  SpeedEstimator est;
  sim::Noise noise(7);
  const double v_true = 10.0, bias = 0.2;
  est.reset(v_true);
  for (int k = 0; k < 6'000; ++k) {  // 60 s at 100 Hz, truck cruising
    est.predict(0.01, bias, 0.0, true);
    est.update(v_true + noise.gaussian(0.03), 0.03 * 0.03);
  }
  EXPECT_NEAR(est.accel_bias(), bias, 0.02);
  EXPECT_NEAR(est.speed(), v_true, 0.05);
}

TEST(SpeedEstimator, RemovesGravityUsingPitch) {
  SpeedEstimator est;
  est.reset(5.0);
  const double pitch = grade_pct_to_rad(10.0);
  // Holding a steady speed up a 10 % ramp, the accelerometer reads g*sin(pitch).
  for (int k = 0; k < 500; ++k) est.predict(0.01, kGravity * std::sin(pitch), pitch, true);
  EXPECT_NEAR(est.speed(), 5.0, 1e-9);
}

TEST(SpeedEstimator, UncertaintyGrowsWhenDeadReckoning) {
  SpeedEstimator est;
  est.reset(8.0);
  for (int k = 0; k < 200; ++k) {
    est.predict(0.01, 0.0, 0.0, true);
    est.update(8.0, 0.03 * 0.03);
  }
  const double tracked = est.speed_variance();
  for (int k = 0; k < 200; ++k) est.predict(0.01, 0.0, 0.0, true);
  const double imu_only = est.speed_variance();
  for (int k = 0; k < 200; ++k) est.predict(0.01, 0.0, 0.0, false);
  EXPECT_GT(imu_only, 10.0 * tracked);
  EXPECT_GT(est.speed_variance(), 10.0 * imu_only);
}

// ---------------------------------------------------------------- faults

TEST(FaultSet, ClassifiesCriticalAndDegradedFaults) {
  FaultSet f;
  EXPECT_FALSE(f.critical());
  EXPECT_FALSE(f.degraded());
  f.set(Fault::GnssTimeout, true);
  EXPECT_TRUE(f.degraded());
  EXPECT_FALSE(f.critical());
  f.set(Fault::WheelSpeedTimeout, true);  // and now no speed source is left
  EXPECT_TRUE(f.critical());
  f = FaultSet{};
  f.set(Fault::E2EError, true);
  EXPECT_FALSE(f.critical());
  EXPECT_FALSE(f.degraded());
  for (Fault c : {Fault::MissionTimeout, Fault::PowertrainTimeout, Fault::ActuatorMismatch, Fault::SpeedImplausible}) {
    FaultSet one;
    one.set(c, true);
    EXPECT_TRUE(one.critical()) << fault_name(c);
  }
  f.set(Fault::WheelSlip, true);
  EXPECT_EQ(describe(f), "E2EError|WheelSlip");
  EXPECT_EQ(describe(FaultSet{}), "none");
  Fault parsed{};
  EXPECT_TRUE(parse_fault("GnssInvalid", parsed));
  EXPECT_EQ(parsed, Fault::GnssInvalid);
  EXPECT_FALSE(parse_fault("Gremlins", parsed));
}

TEST(InputChannel, TimeoutsFollowTheMessageCycleTime) {
  const Calibration cal;
  InputChannel wheels(db().message("WheelSpeeds"), cal);
  InputChannel gnss(db().message("Gnss"), cal);
  EXPECT_EQ(wheels.timeout(), ms(50));  // 3 x 10 ms is below the 50 ms floor
  EXPECT_EQ(gnss.timeout(), ms(300));   // 3 x 100 ms
  EXPECT_TRUE(wheels.timed_out(0));     // nothing received yet
}

TEST(InputChannel, AcceptsProtectedFramesAndRejectsBadOnes) {
  const Calibration cal;
  const auto& msg = db().message("WheelSpeeds");
  InputChannel ch(msg, cal);
  const can::E2EProfile e2e(msg);
  auto frame = [&](std::uint8_t counter, double front_kph) {
    auto f = msg.encode(std::map<std::string, double>{{"FrontAxleSpeed", front_kph}});
    e2e.protect(f, counter);
    return f;
  };
  EXPECT_TRUE(ch.accept(frame(0, 12.5), ms(10)));
  EXPECT_DOUBLE_EQ(ch.value(ch.index("FrontAxleSpeed")), 12.5);
  EXPECT_TRUE(ch.take_fresh());
  EXPECT_FALSE(ch.take_fresh());

  auto corrupt = frame(1, 99.0);
  corrupt.data[0] ^= 0x10;
  EXPECT_FALSE(ch.accept(corrupt, ms(20)));
  EXPECT_FALSE(ch.accept(frame(0, 99.0), ms(30)));  // replayed counter
  EXPECT_DOUBLE_EQ(ch.value(ch.index("FrontAxleSpeed")), 12.5) << "rejected frames must not change the data";
  EXPECT_EQ(ch.stats().crc_errors, 1);
  EXPECT_EQ(ch.stats().sequence_errors, 1);
  EXPECT_EQ(ch.errors_in_last_second(ms(40)), 2);
  EXPECT_EQ(ch.errors_in_last_second(ms(1100)), 0);

  EXPECT_FALSE(ch.timed_out(ms(55)));
  EXPECT_TRUE(ch.timed_out(ms(61)));
}

// ---------------------------------------------------------------- state machine

namespace {
StateInputs healthy(Micros now) {
  StateInputs in;
  in.now = now;
  in.inputs_ready = true;
  in.stopped = true;
  return in;
}
}  // namespace

TEST(StateMachine, NormalMissionCycle) {
  const Calibration cal;
  StateMachine sm(cal);
  EXPECT_EQ(sm.step(healthy(0)), VcuState::Init);
  EXPECT_EQ(sm.step(healthy(ms(400))), VcuState::Init);  // must stay healthy for init_settle
  EXPECT_EQ(sm.step(healthy(ms(500))), VcuState::Ready);

  auto in = healthy(ms(600));
  in.mission_auto = true;
  in.target_mps = 8.0;
  EXPECT_EQ(sm.step(in), VcuState::Driving);
  in.stopped = false;
  EXPECT_EQ(sm.step(in), VcuState::Driving);
  in.target_mps = 0.0;
  EXPECT_EQ(sm.step(in), VcuState::Driving);  // still rolling
  in.stopped = true;
  EXPECT_EQ(sm.step(in), VcuState::Ready);
  EXPECT_STREQ(sm.reason(), "mission complete");
}

TEST(StateMachine, CriticalFaultWhileDrivingStopsThenLatches) {
  const Calibration cal;
  StateMachine sm(cal);
  sm.step(healthy(0));
  sm.step(healthy(ms(500)));
  auto in = healthy(ms(510));
  in.mission_auto = true;
  in.target_mps = 8.0;
  in.stopped = false;
  ASSERT_EQ(sm.step(in), VcuState::Driving);

  in.critical = true;
  in.now = ms(520);
  EXPECT_EQ(sm.step(in), VcuState::SafeStop);
  in.critical = false;  // fault went away, but a safe stop is always completed
  in.now = ms(530);
  EXPECT_EQ(sm.step(in), VcuState::SafeStop);
  in.stopped = true;
  EXPECT_EQ(sm.step(in), VcuState::Fault);

  // Latched: AUTO is ignored; the operator has to hold STOP for fault_clear.
  in.now = ms(600);
  EXPECT_EQ(sm.step(in), VcuState::Fault);
  in.mission_auto = false;
  in.now = ms(700);
  EXPECT_EQ(sm.step(in), VcuState::Fault);
  in.now = ms(1700);
  EXPECT_EQ(sm.step(in), VcuState::Ready);
}

TEST(StateMachine, EmergencyStopOverridesEverything) {
  const Calibration cal;
  for (bool get_to_driving : {false, true}) {
    StateMachine sm(cal);
    sm.step(healthy(0));
    sm.step(healthy(ms(500)));
    auto in = healthy(ms(510));
    if (get_to_driving) {
      in.mission_auto = true;
      in.target_mps = 5.0;
      in.stopped = false;
      ASSERT_EQ(sm.step(in), VcuState::Driving);
    }
    in.estop = true;
    EXPECT_EQ(sm.step(in), VcuState::EmergencyStop);
    in.stopped = true;
    EXPECT_EQ(sm.step(in), VcuState::EmergencyStop) << "stays stopped until the e-stop is released";
    in.estop = false;
    EXPECT_EQ(sm.step(in), VcuState::Fault);
  }
}

TEST(StateMachine, NeverLeavesInitWithACriticalFault) {
  const Calibration cal;
  StateMachine sm(cal);
  auto in = healthy(0);
  in.critical = true;
  for (Micros t = 0; t < seconds(5); t += ms(10)) {
    in.now = t;
    EXPECT_EQ(sm.step(in), VcuState::Init);
  }
}

// ---------------------------------------------------------------- VCU

TEST(Vcu, WithNoInputsItStaysInInitWithTheBrakesOn) {
  Vcu v(db());
  const auto& cmd = db().message("VcuCommand");
  can::E2EReceiver truck_side{can::E2EProfile(cmd)};
  const can::E2EProfile profile(cmd);
  can::E2EReceiver rx(profile);
  int statuses = 0;
  for (Micros t = 0; t < seconds(2); t += Vcu::kPeriod) {
    for (const auto& f : v.step(t)) {
      if (f.id == cmd.id) {
        EXPECT_TRUE(can::usable(rx.check(f)));  // every command is E2E protected, counter advancing
        const auto values = cmd.decode(f);
        EXPECT_DOUBLE_EQ(values.at("TractionForceRequest"), 0.0);
        EXPECT_DOUBLE_EQ(values.at("BrakeRequest"), Calibration{}.hold_brake_pct);
        EXPECT_DOUBLE_EQ(values.at("VcuState"), 0.0);
      } else {
        EXPECT_EQ(f.id, db().message("VcuStatus").id);
        ++statuses;
      }
    }
  }
  EXPECT_EQ(v.debug().state, VcuState::Init);
  EXPECT_EQ(statuses, 20);  // 10 Hz
  EXPECT_TRUE(v.debug().faults.has(Fault::MissionTimeout));
}

TEST(Vcu, DrivesTheSimulatedTruckToTheTargetSpeed) {
  sim::TruckSetup setup;
  setup.params.payload_kg = 227'000.0;
  setup.mission = {sim::MissionStep{0.0, 0.0, false, false, {}}, sim::MissionStep{1.0, 25.0, true, false, {}}};
  sim::TruckSim truck(db(), setup);
  Vcu v(db());
  for (Micros now = 0; now < seconds(60); now += ms(1)) {
    for (const auto& f : truck.step(now, 0.001)) v.on_frame(f, now);
    if (now % Vcu::kPeriod == 0) {
      for (const auto& f : v.step(now)) truck.on_frame(f, now);
    }
  }
  EXPECT_EQ(v.debug().state, VcuState::Driving);
  EXPECT_NEAR(mps_to_kph(truck.truth().speed_mps), 25.0, 0.3);
  EXPECT_NEAR(mps_to_kph(v.debug().speed_estimate_mps), 25.0, 0.3);
  EXPECT_NEAR(v.debug().mass_kg, 392'000.0, 1.0);  // payload read from the powertrain ECU
  EXPECT_FALSE(v.debug().faults.any()) << describe(v.debug().faults);
  EXPECT_FALSE(truck.watchdog_tripped());
}

TEST(Calibration, OverridesByName) {
  Calibration cal;
  EXPECT_TRUE(set_calibration(cal, "kp", 0.6));
  EXPECT_DOUBLE_EQ(cal.kp, 0.6);
  EXPECT_TRUE(set_calibration(cal, "grade_feedforward", 0));
  EXPECT_FALSE(cal.grade_feedforward);
  EXPECT_TRUE(set_calibration(cal, "degraded_speed_limit_kph", 18));
  EXPECT_NEAR(cal.degraded_speed_limit_mps, 5.0, 1e-12);
  EXPECT_FALSE(set_calibration(cal, "warp_drive", 1));
  EXPECT_FALSE(calibration_keys().empty());
}
