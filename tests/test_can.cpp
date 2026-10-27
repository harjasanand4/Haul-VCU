#include <gtest/gtest.h>

#include <map>
#include <string>

#include "haul/can/candump.hpp"
#include "haul/can/dbc.hpp"
#include "haul/can/e2e.hpp"

using namespace haul::can;

namespace {

const Database& db() {
  static const Database d = Database::load(std::string(HAUL_SOURCE_DIR) + "/dbc/haul_truck.dbc");
  return d;
}

std::string hex(const Frame& f) {
  std::string out;
  char buf[3];
  for (std::size_t i = 0; i < f.dlc; ++i) {
    std::snprintf(buf, sizeof buf, "%02x", f.data[i]);
    out += buf;
  }
  return out;
}

}  // namespace

TEST(Dbc, ParsesMessagesCycleTimesAndValueTables) {
  ASSERT_EQ(db().messages().size(), 7u);
  const auto& wheels = db().message("WheelSpeeds");
  EXPECT_EQ(wheels.id, 0x100u);
  EXPECT_EQ(wheels.cycle_time_ms, 10);
  EXPECT_EQ(db().message("Gnss").cycle_time_ms, 100);
  const auto* state = db().message("VcuCommand").find("VcuState");
  ASSERT_NE(state, nullptr);
  EXPECT_EQ(state->value_names.at(3), "SAFE_STOP");
  EXPECT_EQ(db().nodes(), (std::vector<std::string>{"TRUCK", "VCU", "FMS"}));
}

// Reference bytes produced by Python cantools for the same DBC (tools/crosscheck_dbc.py
// checks thousands of random vectors in CI; these pin a few in the C++ tests too).
TEST(Dbc, EncodingMatchesCantools) {
  const std::map<std::string, std::pair<std::map<std::string, double>, std::string>> cases{
      {"VcuCommand",
       {{{"TractionForceRequest", -1234.5},
         {"BrakeRequest", 42.5},
         {"VcuState", 3},
         {"VcuCommand_Counter", 9},
         {"VcuCommand_CRC", 0xA5}},
        "cfc75530000090a5"}},
      {"Imu", {{{"LongAccel", -1.234}, {"Pitch", 5.71}, {"Imu_Counter", 15}, {"Imu_CRC", 0x3C}}, "2efb3b0200000f3c"}},
      {"Powertrain",
       {{{"TractionForceActual", 812.3}, {"BrakePressure", 60.5}, {"Payload", 227.0}, {"Powertrain_Counter", 1}},
        "bb1f79de08000100"}},
      {"MissionCommand",
       {{{"TargetSpeed", 37.25},
         {"MissionMode", 1},
         {"EStop", 1},
         {"MissionCommand_Counter", 4},
         {"MissionCommand_CRC", 0x11}},
        "8d0e050000000411"}},
  };
  for (const auto& [name, c] : cases) {
    const auto f = db().message(name).encode(c.first);
    EXPECT_EQ(hex(f), c.second) << name;
    const auto back = db().message(name).decode(f);
    for (const auto& [sig, v] : c.first) EXPECT_NEAR(back.at(sig), v, 1e-9) << name << "." << sig;
  }
}

TEST(Dbc, SignedValuesAndClampingToTheRawRange) {
  const auto& imu = db().message("Imu");
  auto f = imu.encode(std::map<std::string, double>{{"LongAccel", -32.768}});
  EXPECT_DOUBLE_EQ(imu.decode(f).at("LongAccel"), -32.768);
  f = imu.encode(std::map<std::string, double>{{"LongAccel", 1000.0}});  // way out of range
  EXPECT_DOUBLE_EQ(imu.decode(f).at("LongAccel"), 32.767);
  f = imu.encode(std::map<std::string, double>{{"LongAccel", -1000.0}});
  EXPECT_DOUBLE_EQ(imu.decode(f).at("LongAccel"), -32.768);
}

TEST(Dbc, MotorolaBitLayoutWalksTheSawtooth) {
  const auto* force = db().message("VcuCommand").find("TractionForceRequest");  // 7|16@0
  ASSERT_NE(force, nullptr);
  const auto& pos = force->bit_positions();
  ASSERT_EQ(pos.size(), 16u);
  EXPECT_EQ(pos[15], 7);  // MSB is the DBC start bit
  EXPECT_EQ(pos[8], 0);   // ...down through byte 0
  EXPECT_EQ(pos[7], 15);  // then the top of byte 1
  EXPECT_EQ(pos[0], 8);   // LSB
}

TEST(Dbc, RejectsBrokenDefinitions) {
  EXPECT_THROW(Database::parse("BO_ 1 M: 8 N\n SG_ S : 60|8@1+ (1,0) [0|1] \"\" X\n"), DbcError);  // past byte 7
  EXPECT_THROW(Database::parse("BO_ 1 M: 9 N\n"), DbcError);                                       // DLC > 8
  EXPECT_THROW(Database::parse(" SG_ S : 0|8@1+ (1,0) [0|1] \"\" X\n"), DbcError);                 // no message
  EXPECT_THROW(Database::parse("BO_ 1 M: 8 N\n SG_ S : 0|8@1+ (0,0) [0|1] \"\" X\n"), DbcError);   // zero scale
}

TEST(Dbc, HandlesExtendedIdsAndMultiLineComments) {
  const auto d = Database::parse(
      "BO_ 2566844693 J1939Msg: 8 N\n"
      " SG_ Speed : 8|16@1+ (0.00390625,0) [0|250.996] \"km/h\" X\n\n"
      "CM_ BO_ 2566844693 \"A comment\nthat spans; lines\";\n"
      "BA_ \"GenMsgCycleTime\" BO_ 2566844693 100;\n");
  const auto& m = d.message("J1939Msg");
  EXPECT_TRUE(m.extended);
  EXPECT_EQ(m.id, 0x18FEF115u);
  EXPECT_EQ(m.cycle_time_ms, 100);
  EXPECT_EQ(d.by_id(0x18FEF115u, true), &m);
}

TEST(E2E, Crc8SaeJ1850CheckValues) {
  const auto crc = [](std::initializer_list<std::uint8_t> bytes) {
    return crc8_sae_j1850(std::span<const std::uint8_t>(bytes.begin(), bytes.size()));
  };
  const std::string check = "123456789";
  EXPECT_EQ(crc8_sae_j1850({reinterpret_cast<const std::uint8_t*>(check.data()), check.size()}), 0x4B);
  // Test vectors from the AUTOSAR CRC library specification.
  EXPECT_EQ(crc({0x00, 0x00, 0x00, 0x00}), 0x59);
  EXPECT_EQ(crc({0xF2, 0x01, 0x83}), 0x37);
  EXPECT_EQ(crc({0x0F, 0xAA, 0x00, 0x55}), 0x79);
  EXPECT_EQ(crc({0xFF, 0xFF, 0xFF, 0xFF}), 0x74);
  EXPECT_EQ(crc({0x92, 0x6B, 0x55}), 0x8C);
}

TEST(E2E, ProtectedFramesPassAndTamperedOnesFail) {
  const auto& msg = db().message("WheelSpeeds");
  const E2EProfile p(msg);
  auto f = msg.encode(std::map<std::string, double>{{"FrontAxleSpeed", 31.5}});
  p.protect(f, 5);
  EXPECT_TRUE(p.crc_ok(f));
  EXPECT_EQ(p.counter(f), 5);

  auto flipped = f;
  flipped.data[4] ^= 0x01;  // one bit of speed
  EXPECT_FALSE(p.crc_ok(flipped));

  // Same layout, different data ID: a WheelSpeeds frame that turns up where an Imu frame is
  // expected (a masquerading or mis-routed frame) fails the Imu check even though it is intact.
  const E2EProfile imu(db().message("Imu"));
  EXPECT_FALSE(imu.crc_ok(f));
}

TEST(E2E, ReceiverTracksTheAliveCounter) {
  const auto& msg = db().message("Imu");
  const E2EProfile p(msg);
  E2EReceiver rx(p, 3);
  auto frame = [&](std::uint8_t c) {
    auto f = msg.encode(std::map<std::string, double>{});
    p.protect(f, c);
    return f;
  };
  EXPECT_EQ(rx.check(frame(14)), E2EStatus::Initial);
  EXPECT_EQ(rx.check(frame(15)), E2EStatus::Ok);
  EXPECT_EQ(rx.check(frame(0)), E2EStatus::Ok);  // wraps at 16
  EXPECT_EQ(rx.check(frame(0)), E2EStatus::Repeated);
  EXPECT_EQ(rx.check(frame(3)), E2EStatus::OkSomeLost);
  EXPECT_EQ(rx.check(frame(10)), E2EStatus::WrongSequence);
  EXPECT_EQ(rx.check(frame(11)), E2EStatus::Ok);  // resynchronised
  auto bad = frame(12);
  bad.data[7] ^= 0xFF;
  EXPECT_EQ(rx.check(bad), E2EStatus::CrcError);
}

TEST(Candump, RoundTripsAndRejectsGarbage) {
  Frame f;
  f.id = 0x18FEF115;
  f.extended = true;
  f.dlc = 3;
  f.data = {0xDE, 0xAD, 0xBE};
  const LoggedFrame lf{12'345'678'901, "can0", f};
  const auto line = format_candump(lf);
  EXPECT_EQ(line, "(12345.678901) can0 18FEF115#DEADBE");
  const auto back = parse_candump(line);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->frame, f);
  EXPECT_EQ(back->time, lf.time);
  EXPECT_FALSE(parse_candump("hello").has_value());
  EXPECT_FALSE(parse_candump("(1.0) vcan0 XYZ#00").has_value());
  const auto standard = parse_candump("(0.5) vcan0 100#0102");
  ASSERT_TRUE(standard.has_value());
  EXPECT_FALSE(standard->frame.extended);
  EXPECT_EQ(standard->frame.dlc, 2);
  EXPECT_EQ(standard->time, 500'000);
}
