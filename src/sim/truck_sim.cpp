#include "haul/sim/truck_sim.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "haul/core/units.hpp"

namespace haul::sim {

double Road::grade_pct_at(double x) const {
  if (points.empty()) return 0.0;
  if (x <= points.front().first) return points.front().second;
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (x <= points[i].first) {
      const auto [x0, g0] = points[i - 1];
      const auto [x1, g1] = points[i];
      return x1 > x0 ? g0 + (g1 - g0) * (x - x0) / (x1 - x0) : g1;
    }
  }
  return points.back().second;
}

double Noise::uniform() {
  // 53 random bits -> [0, 1)
  return static_cast<double>(eng_() >> 11) * (1.0 / 9007199254740992.0);
}

double Noise::gaussian(double sigma) {
  if (spare_) {
    const double s = *spare_;
    spare_.reset();
    return s * sigma;
  }
  double u1 = uniform();
  while (u1 <= 0.0) u1 = uniform();
  const double u2 = uniform();
  const double r = std::sqrt(-2.0 * std::log(u1));
  spare_ = r * std::sin(2.0 * std::numbers::pi * u2);
  return r * std::cos(2.0 * std::numbers::pi * u2) * sigma;
}

namespace {

// Spread the messages over the cycle so they don't all hit the bus in the same millisecond.
Micros phase_for(const std::string& name) {
  if (name == "WheelSpeeds") return ms(0);
  if (name == "Imu") return ms(2);
  if (name == "Powertrain") return ms(4);
  if (name == "Gnss") return ms(6);
  return ms(8);  // MissionCommand
}

constexpr Micros kGnssLatency = ms(100);
constexpr Micros kCommandWatchdog = ms(100);

}  // namespace

TruckSim::TruckSim(const can::Database& db, TruckSetup setup)
    : setup_(std::move(setup)),
      model_(setup_.params),
      noise_(setup_.seed),
      command_msg_(&db.message("VcuCommand")),
      command_e2e_(*command_msg_),
      command_rx_(command_e2e_),
      i_force_(command_msg_->index_of("TractionForceRequest")),
      i_brake_(command_msg_->index_of("BrakeRequest")) {
  for (const char* name : {"WheelSpeeds", "Imu", "Powertrain", "Gnss", "MissionCommand"}) {
    const auto& msg = db.message(name);
    tx_.push_back(Tx{&msg, can::E2EProfile(msg), ms(msg.cycle_time_ms.value_or(100)), phase_for(name)});
  }
  if (setup_.mission.empty()) setup_.mission.push_back(MissionStep{});
  model_.reset(0.0);
}

const FaultInjection* TruckSim::find_fault(const std::string& type, const std::string& message, double t) const {
  for (const auto& f : setup_.faults) {
    if (f.type == type && (message.empty() || f.message == message) && f.active(t)) return &f;
  }
  return nullptr;
}

bool TruckSim::fault(const std::string& type, const std::string& message, double t) const {
  return find_fault(type, message, t) != nullptr;
}

void TruckSim::on_frame(const can::Frame& f, Micros now) {
  if (f.id != command_msg_->id || f.extended) return;
  if (fault("drop", "VcuCommand", to_seconds(now))) return;
  if (!can::usable(command_rx_.check(f))) return;  // the drive ignores corrupted or stale commands
  const auto values = command_msg_->decode(f);
  drive_req_ = values.at("TractionForceRequest") * 1000.0;
  brake_req_ = values.at("BrakeRequest");
  last_command_ = now;
}

std::optional<can::Frame> TruckSim::emit(Tx& tx, Micros now) {
  const double t = to_seconds(now);
  const std::string& name = tx.msg->name;
  if (fault("drop", name, t)) {
    ++dropped_;
    return std::nullopt;
  }
  const TruckState& s = model_.state();
  std::map<std::string, double> v;
  if (name == "WheelSpeeds") {
    const double bias = find_fault("wheel_bias", "", t) ? find_fault("wheel_bias", "", t)->amount : 0.0;
    const double rear = s.speed_mps * (1.0 + s.slip);
    v["FrontAxleSpeed"] = mps_to_kph(std::max(0.0, s.speed_mps + bias + noise_.gaussian(0.02)));
    v["RearLeftSpeed"] = mps_to_kph(std::max(0.0, rear + noise_.gaussian(0.02)));
    v["RearRightSpeed"] = mps_to_kph(std::max(0.0, rear + noise_.gaussian(0.02)));
  } else if (name == "Imu") {
    // An accelerometer measures specific force: real acceleration plus the gravity component
    // along the truck's axis. It also has a bias, which the VCU's Kalman filter estimates.
    v["LongAccel"] = s.accel_mps2 + kGravity * std::sin(s.grade_rad) + 0.03 + noise_.gaussian(0.05);
    v["Pitch"] = rad_to_deg(s.grade_rad) + noise_.gaussian(0.1);
  } else if (name == "Powertrain") {
    v["TractionForceActual"] = s.drive_force_n / 1000.0;
    v["BrakePressure"] = s.brake_force_n / model_.params().max_brake_n * 120.0;
    v["Payload"] = model_.params().payload_kg / 1000.0;
  } else if (name == "Gnss") {
    double delayed = s.speed_mps;
    for (const auto& [time, speed] : gnss_history_) {
      if (time <= now - kGnssLatency) delayed = speed;
    }
    v["GnssSpeed"] = mps_to_kph(std::max(0.0, delayed + noise_.gaussian(0.05)));
    v["GnssValid"] = fault("gnss_invalid", "", t) ? 0.0 : 1.0;
  } else {  // MissionCommand, from the fleet management system
    const MissionStep& m = mission();
    v["TargetSpeed"] = m.target_kph;
    v["MissionMode"] = m.autonomous ? 1.0 : 0.0;
    v["EStop"] = m.estop ? 1.0 : 0.0;
  }

  can::Frame f = tx.msg->encode(v);
  if (!fault("freeze_counter", name, t)) ++tx.counter;
  tx.e2e.protect(f, tx.counter);
  if (const FaultInjection* c = find_fault("corrupt_crc", name, t); c && noise_.uniform() < c->amount) {
    f.data[7] = static_cast<std::uint8_t>(f.data[7] ^ 0x5Au);
    ++corrupted_;
  }
  return f;
}

std::vector<can::Frame> TruckSim::step(Micros now, double dt) {
  const double t = to_seconds(now);

  // The drive's own watchdog: no valid command for 100 ms means the VCU is gone.
  // Independently of the VCU, cut traction and apply the brakes.
  watchdog_ = last_command_ && now - *last_command_ > kCommandWatchdog;
  const double drive = watchdog_ ? 0.0 : drive_req_;
  const double brake = watchdog_ ? 100.0 : brake_req_;

  const double x = model_.state().position_m;
  double mu = 0.55;
  for (const auto& w : setup_.wet_patches) {
    if (x >= w.from_m && x < w.to_m) mu = w.mu;
  }
  model_.set_drive_failed(fault("drive_failure", "", t));
  model_.step(dt, drive, brake, grade_pct_to_rad(setup_.road.grade_pct_at(x)), mu);

  // Mission steps fire in order, each on its time or its position on the road.
  while (mission_index_ + 1 < setup_.mission.size()) {
    const MissionStep& next = setup_.mission[mission_index_ + 1];
    const bool fired = t >= next.t_s && (!next.at_m || model_.state().position_m >= *next.at_m);
    if (!fired) break;
    ++mission_index_;
  }

  gnss_history_.emplace_back(now, model_.state().speed_mps);
  while (gnss_history_.size() > 2 && gnss_history_[1].first <= now - kGnssLatency) gnss_history_.pop_front();

  std::vector<can::Frame> out;
  for (auto& tx : tx_) {
    if ((now - tx.phase) % tx.period != 0 || now < tx.phase) continue;
    if (auto f = emit(tx, now)) out.push_back(*f);
  }
  return out;
}

}  // namespace haul::sim
