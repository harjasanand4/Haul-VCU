#include "haul/vcu/vcu.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "haul/core/units.hpp"

namespace haul::vcu {

namespace {

constexpr double kFrontVariance = 0.03 * 0.03;  // (m/s)^2: undriven axle, the best speed source
constexpr double kRearVariance = 0.06 * 0.06;   // driven axle: fine unless it slips
constexpr double kGnssVariance = 0.15 * 0.15;   // GNSS speed: noisier and ~100 ms late
constexpr double kPitchFilterTau = 0.2;         // s
constexpr double kNoLimitMps = 80.0 / 3.6;

}  // namespace

Vcu::Vcu(const can::Database& db, Calibration cal)
    : cal_(cal),
      controller_(cal_),
      sm_(cal_),
      wheels_(db.message("WheelSpeeds"), cal_),
      imu_(db.message("Imu"), cal_),
      powertrain_(db.message("Powertrain"), cal_),
      gnss_(db.message("Gnss"), cal_),
      mission_(db.message("MissionCommand"), cal_),
      command_msg_(&db.message("VcuCommand")),
      status_msg_(&db.message("VcuStatus")),
      command_e2e_(*command_msg_),
      status_e2e_(*status_msg_),
      slip_(cal_.slip_debounce, ms(300)),
      implausible_(cal_.plausibility_debounce, ms(500)),
      gnss_invalid_(ms(300), ms(300)),
      actuator_(cal_.actuator_debounce, ms(500)),
      stopped_(cal_.stopped_confirm, 0) {
  for (InputChannel* ch : {&wheels_, &imu_, &powertrain_, &gnss_, &mission_}) by_id_[ch->message().id] = ch;
  i_rear_left_ = wheels_.index("RearLeftSpeed");
  i_rear_right_ = wheels_.index("RearRightSpeed");
  i_front_ = wheels_.index("FrontAxleSpeed");
  i_accel_ = imu_.index("LongAccel");
  i_pitch_ = imu_.index("Pitch");
  i_force_actual_ = powertrain_.index("TractionForceActual");
  i_payload_ = powertrain_.index("Payload");
  i_gnss_speed_ = gnss_.index("GnssSpeed");
  i_gnss_valid_ = gnss_.index("GnssValid");
  i_target_ = mission_.index("TargetSpeed");
  i_mode_ = mission_.index("MissionMode");
  i_estop_ = mission_.index("EStop");
  i_cmd_force_ = command_msg_->index_of("TractionForceRequest");
  i_cmd_brake_ = command_msg_->index_of("BrakeRequest");
  i_cmd_state_ = command_msg_->index_of("VcuState");
  i_st_speed_ = status_msg_->index_of("SpeedEstimate");
  i_st_limit_ = status_msg_->index_of("SpeedLimit");
  i_st_faults_ = status_msg_->index_of("FaultFlags");
  if (command_msg_->signals.size() > kMaxSignals || status_msg_->signals.size() > kMaxSignals) {
    throw std::invalid_argument("VcuCommand/VcuStatus have more signals than the encode buffer holds");
  }
}

const InputChannel& Vcu::channel(const char* message) const {
  for (const InputChannel* ch : {&wheels_, &imu_, &powertrain_, &gnss_, &mission_}) {
    if (ch->message().name == message) return *ch;
  }
  throw std::invalid_argument(std::string("no input channel ") + message);
}

void Vcu::on_frame(const can::Frame& f, Micros now) {
  if (f.extended) return;
  const auto it = by_id_.find(f.id);
  if (it != by_id_.end()) it->second->accept(f, now);
}

void Vcu::update_estimator(double dt, Micros now) {
  const bool wheel_fresh = wheels_.take_fresh();
  if (wheel_fresh) {
    last_front_mps_ = kph_to_mps(wheels_.value(i_front_));
    last_rear_mps_ = kph_to_mps(0.5 * (wheels_.value(i_rear_left_) + wheels_.value(i_rear_right_)));
  }
  const bool imu_valid = imu_.received() && !imu_.timed_out(now);
  if (imu_.take_fresh()) {
    const double pitch = deg_to_rad(imu_.value(i_pitch_));
    pitch_rad_ += (pitch - pitch_rad_) * std::min(1.0, dt / kPitchFilterTau);
  }

  if (!estimator_.initialised() && wheel_fresh) estimator_.reset(last_front_mps_);
  estimator_.predict(dt, imu_valid ? imu_.value(i_accel_) : 0.0, pitch_rad_, imu_valid);
  if (wheel_fresh) {
    estimator_.update(last_front_mps_, kFrontVariance);
    // Gate the driven axle: skip it the moment it disagrees with the undriven one, without
    // waiting for the debounced WheelSlip fault, so a spinning wheel never drags the estimate up.
    const bool rear_agrees = std::abs(last_rear_mps_ - last_front_mps_) < 0.5 * cal_.slip_threshold_mps;
    if (!slip_.active() && rear_agrees) estimator_.update(last_rear_mps_, kRearVariance);
  }
  if (gnss_.take_fresh() && gnss_.value(i_gnss_valid_) > 0.5) {
    last_gnss_mps_ = kph_to_mps(gnss_.value(i_gnss_speed_));
    if (!estimator_.initialised()) estimator_.reset(last_gnss_mps_);
    estimator_.update(last_gnss_mps_, kGnssVariance);
  }
}

void Vcu::update_faults(Micros now) {
  FaultSet f;
  const bool wheels_ok = !wheels_.timed_out(now);
  const bool gnss_ok = !gnss_.timed_out(now);
  f.set(Fault::WheelSpeedTimeout, !wheels_ok);
  f.set(Fault::ImuTimeout, imu_.timed_out(now));
  f.set(Fault::PowertrainTimeout, powertrain_.timed_out(now));
  f.set(Fault::GnssTimeout, !gnss_ok);
  f.set(Fault::MissionTimeout, mission_.timed_out(now));

  bool e2e = false;
  for (InputChannel* ch : {&wheels_, &imu_, &powertrain_, &gnss_, &mission_}) {
    e2e = e2e || ch->errors_in_last_second(now) > cal_.e2e_errors_per_second;
  }
  f.set(Fault::E2EError, e2e);

  // Driven wheels turning faster than the undriven axle: wheel spin.
  const double slip_margin = std::max(cal_.slip_threshold_mps, cal_.slip_ratio * last_front_mps_);
  f.set(Fault::WheelSlip, slip_.update(wheels_ok && last_rear_mps_ - last_front_mps_ > slip_margin, now));

  const bool gnss_valid = gnss_ok && gnss_.value(i_gnss_valid_) > 0.5;
  f.set(Fault::GnssInvalid, gnss_invalid_.update(gnss_.received() && gnss_ok && !gnss_valid, now));
  const bool disagree = std::abs(last_front_mps_ - last_gnss_mps_) > cal_.plausibility_threshold_mps;
  f.set(Fault::SpeedImplausible, implausible_.update(wheels_ok && gnss_valid && disagree, now));

  // Is the drive doing what it was told? Compare its reported force with the request passed
  // through the lag we expect from the drive, clipped to what the drive can deliver at this speed.
  const double v = estimator_.speed();
  const double achievable = clamp(last_out_.traction_n, -controller_.max_retard(v), controller_.max_traction(v));
  expected_force_n_ += (achievable - expected_force_n_) * std::min(1.0, to_seconds(Vcu::kPeriod) / cal_.drive_lag_s);
  const bool driving = sm_.state() == VcuState::Driving || sm_.state() == VcuState::SafeStop;
  const double actual = powertrain_.value(i_force_actual_) * 1000.0;
  const bool mismatch =
      driving && !powertrain_.timed_out(now) && std::abs(actual - expected_force_n_) > cal_.actuator_tolerance_n;
  f.set(Fault::ActuatorMismatch, actuator_.update(mismatch, now));

  faults_ = f;
}

void Vcu::update_setpoint(double dt) {
  const VcuState s = sm_.state();
  const bool entered = sm_.entered_at() == debug_.time;
  if (s == VcuState::Driving) {
    if (entered) profile_.reset(estimator_.speed());  // bumpless start from wherever we are
    const double goal = std::min(debug_.target_mps, debug_.speed_limit_mps);
    profile_.step(goal, cal_.accel_limit, cal_.decel_limit, cal_.jerk_limit, dt);
  } else if (s == VcuState::SafeStop) {
    if (entered) profile_.reset(std::min(profile_.speed(), estimator_.speed()));
    profile_.step(0.0, cal_.accel_limit, cal_.safe_stop_decel, cal_.safe_stop_jerk, dt);
  } else {
    profile_.reset(0.0);
  }
  debug_.setpoint_mps = profile_.speed();
}

FrameBatch Vcu::step(Micros now) {
  const double dt = last_step_ ? clamp(to_seconds(now - *last_step_), 0.001, 0.1) : to_seconds(kPeriod);
  last_step_ = now;
  debug_.time = now;

  update_estimator(dt, now);
  update_faults(now);
  const double v = estimator_.speed();

  const bool mission_ok = !mission_.timed_out(now);
  StateInputs in;
  in.now = now;
  in.inputs_ready = !wheels_.timed_out(now) && !powertrain_.timed_out(now) && mission_ok;
  in.critical = faults_.critical();
  in.estop = mission_.received() && mission_.value(i_estop_) > 0.5;
  in.mission_auto = mission_ok && std::lround(mission_.value(i_mode_)) == 1;
  in.target_mps = in.mission_auto ? kph_to_mps(mission_.value(i_target_)) : 0.0;
  in.stopped = stopped_.update(v < cal_.standstill_mps && last_front_mps_ < cal_.standstill_mps, now);
  const VcuState state = sm_.step(in);

  debug_.target_mps = in.target_mps;
  debug_.speed_limit_mps = faults_.degraded() ? cal_.degraded_speed_limit_mps : kNoLimitMps;
  update_setpoint(dt);

  const double payload_kg = powertrain_.received() ? powertrain_.value(i_payload_) * 1000.0 : 0.0;
  const double mass = cal_.empty_mass_kg + clamp(payload_kg, 0.0, 300'000.0);

  ControlOutput out;
  switch (state) {
    case VcuState::Driving:
    case VcuState::SafeStop: {
      ControlInput ci;
      ci.dt = dt;
      ci.setpoint_mps = profile_.speed();
      ci.setpoint_accel = profile_.accel();
      ci.setpoint_jerk = profile_.jerk();
      ci.speed_mps = v;
      ci.mass_kg = mass;
      ci.pitch_rad = pitch_rad_;
      ci.slip = faults_.has(Fault::WheelSlip);
      out = controller_.step(ci);
      break;
    }
    case VcuState::EmergencyStop:
      controller_.reset();
      out.brake_pct = 100.0;
      break;
    default:
      controller_.reset();
      out.brake_pct = cal_.hold_brake_pct;
      break;
  }
  last_out_ = out;

  debug_.state = state;
  debug_.state_reason = sm_.reason();
  debug_.faults = faults_;
  debug_.speed_estimate_mps = v;
  debug_.speed_variance = estimator_.speed_variance();
  debug_.traction_request_n = out.traction_n;
  debug_.brake_request_pct = out.brake_pct;
  debug_.feedforward_n = out.feedforward_n;
  debug_.mass_kg = mass;
  debug_.pitch_rad = pitch_rad_;

  FrameBatch frames;
  frames.push(encode_command(now));
  if (now - last_status_ >= ms(100)) {
    last_status_ = now;
    frames.push(encode_status());
  }
  return frames;
}

can::Frame Vcu::encode_command(Micros) {
  const std::span<double> values(scratch_.data(), command_msg_->signals.size());
  std::fill(values.begin(), values.end(), 0.0);
  values[i_cmd_force_] = last_out_.traction_n / 1000.0;
  values[i_cmd_brake_] = last_out_.brake_pct;
  values[i_cmd_state_] = static_cast<double>(debug_.state);
  can::Frame f = command_msg_->encode(values);
  command_e2e_.protect(f, command_counter_++);
  return f;
}

can::Frame Vcu::encode_status() {
  const std::span<double> values(scratch_.data(), status_msg_->signals.size());
  std::fill(values.begin(), values.end(), 0.0);
  values[i_st_speed_] = mps_to_kph(debug_.speed_estimate_mps);
  values[i_st_limit_] = mps_to_kph(debug_.speed_limit_mps);
  values[i_st_faults_] = static_cast<double>(faults_.bits());
  can::Frame f = status_msg_->encode(values);
  status_e2e_.protect(f, status_counter_++);
  return f;
}

}  // namespace haul::vcu
