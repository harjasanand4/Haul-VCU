#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "haul/can/candump.hpp"
#include "haul/can/dbc.hpp"
#include "haul/core/units.hpp"
#include "haul/sim/scenario.hpp"
#include "haul/sim/sil_runner.hpp"
#include "haul/sim/truck_model.hpp"
#include "haul/sim/truck_sim.hpp"

namespace fs = std::filesystem;
using namespace haul;
using namespace haul::sim;

namespace {

const can::Database& db() {
  static const can::Database d = can::Database::load(std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc");
  return d;
}

std::vector<fs::path> scenario_files() {
  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(fs::path(HAUL_SOURCE_DIR) / "scenarios")) {
    if (e.path().extension() == ".json") files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());
  return files;
}

}  // namespace

// ---------------------------------------------------------------- plant model vs closed-form physics

TEST(TruckModel, CoastDownMatchesRollingResistanceAndDrag) {
  TruckParams p;
  p.payload_kg = 227'000.0;
  TruckModel m(p);
  const double v0 = 10.0;
  m.reset(v0);
  m.step(1e-4, 0.0, 0.0, 0.0, 0.55);
  const double expected =
      -(p.rolling_resistance * m.mass() * kGravity + 0.5 * p.air_density * p.drag_area_m2 * v0 * v0) / m.mass();
  EXPECT_NEAR(m.state().accel_mps2, expected, 1e-6);
}

TEST(TruckModel, ConstantForceReachesTheSteadyStateSpeedTheEquationPredicts) {
  TruckParams p;
  TruckModel m(p);
  m.reset(0.0);
  const double force = 60'000.0;  // ~1.4 MW at the resulting speed: below the power limit
  for (int k = 0; k < 300'000; ++k) m.step(0.01, force, 0.0, 0.0, 0.55);  // 3000 s: drag settles slowly
  // force = Crr m g + 0.5 rho CdA v^2  ->  v = sqrt((F - Crr m g) / (0.5 rho CdA))
  const double v_ss =
      std::sqrt((force - p.rolling_resistance * m.mass() * kGravity) / (0.5 * p.air_density * p.drag_area_m2));
  EXPECT_NEAR(m.state().speed_mps, v_ss, 0.05);
}

TEST(TruckModel, DriveIsPowerLimitedAtSpeed) {
  TruckParams p;
  TruckModel m(p);
  m.reset(15.0);
  for (int k = 0; k < 5'000; ++k) m.step(0.001, 2e6, 0.0, 0.0, 0.55);
  EXPECT_NEAR(m.state().drive_force_n * m.state().speed_mps, p.max_traction_power_w, 0.02 * p.max_traction_power_w);
}

TEST(TruckModel, ActuatorFollowsAFirstOrderLag) {
  TruckParams p;
  TruckModel m(p);
  m.reset(5.0);
  const double request = 300'000.0;
  const int steps = static_cast<int>(p.traction_tau_s / 0.001);
  for (int k = 0; k < steps; ++k) m.step(0.001, request, 0.0, 0.0, 0.55);
  EXPECT_NEAR(m.state().drive_force_n / request, 1.0 - std::exp(-1.0), 0.01);  // 63 % after one time constant
}

TEST(TruckModel, BrakesHoldALoadedTruckOnARampWithoutRollingBack) {
  TruckParams p;
  p.payload_kg = 227'000.0;
  TruckModel m(p);
  m.reset(0.0);
  for (int k = 0; k < 10'000; ++k) m.step(0.001, 0.0, 55.0, grade_pct_to_rad(10.0), 0.55);
  EXPECT_DOUBLE_EQ(m.state().speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(m.state().position_m, 0.0);
}

TEST(TruckModel, WheelsSpinWhenTheDriveExceedsTheGrip) {
  TruckParams p;
  p.payload_kg = 227'000.0;
  TruckModel m(p);
  m.reset(2.0);
  for (int k = 0; k < 2'000; ++k) m.step(0.001, 600'000.0, 0.0, 0.0, 0.08);
  const double grip = 0.08 * m.mass() * p.rear_axle_share * kGravity;
  EXPECT_NEAR(m.state().transmitted_force_n, grip, 1.0);
  EXPECT_GT(m.state().slip, 0.2);
}

TEST(Road, InterpolatesGradeBetweenPoints) {
  Road r;
  r.points = {{0.0, 0.0}, {100.0, 10.0}, {200.0, 10.0}};
  EXPECT_DOUBLE_EQ(r.grade_pct_at(-5.0), 0.0);
  EXPECT_DOUBLE_EQ(r.grade_pct_at(50.0), 5.0);
  EXPECT_DOUBLE_EQ(r.grade_pct_at(150.0), 10.0);
  EXPECT_DOUBLE_EQ(r.grade_pct_at(1e6), 10.0);
}

TEST(Noise, IsDeterministicAndRoughlyNormal) {
  Noise a(123), b(123);
  double sum = 0.0, sq = 0.0;
  constexpr int n = 200'000;
  for (int i = 0; i < n; ++i) {
    const double x = a.gaussian(2.0);
    ASSERT_EQ(x, b.gaussian(2.0));
    sum += x;
    sq += x * x;
  }
  EXPECT_NEAR(sum / n, 0.0, 0.02);
  EXPECT_NEAR(std::sqrt(sq / n), 2.0, 0.02);
}

// ---------------------------------------------------------------- scenario files

TEST(Scenario, RejectsBadFiles) {
  EXPECT_THROW(Scenario::from_json(R"({"name": "x", "mission": [], "expect": [{"check": "vibes"}]})"), std::exception);
  EXPECT_THROW(Scenario::from_json(R"({"name": "x", "mission": [{"target_kph": 3}], "expect": []})"), std::exception);
  EXPECT_THROW(
      Scenario::from_json(
          R"({"name": "x", "mission": [{"t": 0}], "expect": [{"check": "fault_never", "fault": "Gremlins"}]})"),
      std::exception);
  EXPECT_THROW(Scenario::from_json(R"({"name": "x", "mission": [{"t": 0}], "vcu": {"warp": 1}, "expect": []})"),
               std::exception);
  EXPECT_THROW(Scenario::from_json("{not json"), std::exception);
}

TEST(Scenario, ChecksCatchARegression) {
  // Same physics, but with the controller deliberately detuned: the checks must notice.
  auto sc = Scenario::load(fs::path(HAUL_SOURCE_DIR) / "scenarios" / "01_flat_cruise_loaded.json");
  SilOptions opt;
  opt.calibration.kp = 0.0;
  opt.calibration.ki = 0.0;
  opt.calibration.rolling_resistance = 0.0;  // and a wrong road-load model
  const auto run = run_sil(db(), sc, opt);
  const auto results = evaluate(sc, run);
  EXPECT_TRUE(std::any_of(results.begin(), results.end(), [](const CheckResult& r) { return !r.passed; }));
}

class ScenarioSuite : public ::testing::TestWithParam<fs::path> {};

TEST_P(ScenarioSuite, Passes) {
  const auto sc = Scenario::load(GetParam());
  SilOptions opt;
  for (const auto& [k, v] : sc.calibration) ASSERT_TRUE(vcu::set_calibration(opt.calibration, k, v)) << k;
  const auto run = run_sil(db(), sc, opt);
  const auto results = evaluate(sc, run);
  ASSERT_FALSE(results.empty());
  for (const auto& r : results) EXPECT_TRUE(r.passed) << r.name << ": " << r.detail;
}

INSTANTIATE_TEST_SUITE_P(AllScenarios, ScenarioSuite, ::testing::ValuesIn(scenario_files()),
                         [](const ::testing::TestParamInfo<fs::path>& p) {
                           std::string name = p.param.stem().string();
                           for (auto& c : name) {
                             if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
                           }
                           return name;
                         });

TEST(Sil, IsDeterministic) {
  const auto sc = Scenario::load(fs::path(HAUL_SOURCE_DIR) / "scenarios" / "04_wheel_slip_wet_road.json");
  const auto a = run_sil(db(), sc);
  const auto b = run_sil(db(), sc);
  ASSERT_EQ(a.samples.size(), b.samples.size());
  for (std::size_t i = 0; i < a.samples.size(); ++i) {
    ASSERT_EQ(a.samples[i].speed_kph, b.samples[i].speed_kph) << "diverged at sample " << i;
    ASSERT_EQ(a.samples[i].drive_request_kn, b.samples[i].drive_request_kn);
    ASSERT_EQ(a.samples[i].faults, b.samples[i].faults);
  }
  EXPECT_EQ(a.frames_to_vcu, b.frames_to_vcu);
  EXPECT_EQ(a.transitions.size(), b.transitions.size());
}

TEST(Sil, RecordsAReplayableCandumpLog) {
  auto sc = Scenario::load(fs::path(HAUL_SOURCE_DIR) / "scenarios" / "13_speed_steps_empty.json");
  sc.duration_s = 5.0;
  SilOptions opt;
  const auto path = fs::temp_directory_path() / "haul_sil_test.log";
  opt.candump_path = path.string();
  const auto run = run_sil(db(), sc, opt);
  const auto frames = can::read_candump(path.string());
  fs::remove(path);
  EXPECT_EQ(static_cast<long>(frames.size()), run.frames_to_vcu + run.frames_to_truck);
  ASSERT_FALSE(frames.empty());
  EXPECT_TRUE(std::is_sorted(frames.begin(), frames.end(),
                             [](const can::LoggedFrame& x, const can::LoggedFrame& y) { return x.time < y.time; }));
  for (const auto& lf : frames) EXPECT_NE(db().by_id(lf.frame.id), nullptr);
}
