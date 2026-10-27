#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>

#include "haul/can/bus.hpp"

using namespace haul::can;
using namespace std::chrono_literals;

TEST(UdpBus, CarriesStandardAndExtendedFramesBothWays) {
  auto a = open_bus("udp:47101:127.0.0.1:47102");
  auto b = open_bus("udp:47102:127.0.0.1:47101");
  Frame f;
  f.id = 0x200;
  f.dlc = 8;
  f.data = {1, 2, 3, 4, 5, 6, 7, 8};
  ASSERT_TRUE(a->send(f));
  const auto got = b->receive(500ms);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(*got, f);

  Frame ext;
  ext.id = 0x18FEF115;
  ext.extended = true;
  ext.dlc = 3;
  ext.data = {0xAA, 0xBB, 0xCC};
  ASSERT_TRUE(b->send(ext));
  const auto back = a->receive(500ms);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(*back, ext);

  EXPECT_FALSE(a->receive(10ms).has_value());  // times out when nothing is sent
}

TEST(Bus, RejectsBadSpecs) {
  EXPECT_THROW(open_bus("carrier-pigeon:1"), std::invalid_argument);
  EXPECT_THROW(open_bus("udp:notaport:127.0.0.1:1"), std::exception);
  EXPECT_THROW(open_bus("udp:47103:not-an-ip:47104"), std::exception);
}

// Needs a vcan interface (CI creates one: `ip link add dev vcan0 type vcan`). Set
// HAUL_TEST_VCAN=vcan0 to run it; otherwise it is skipped, not failed.
TEST(SocketCan, LoopsFramesBackOnVcan) {
  const char* ifname = std::getenv("HAUL_TEST_VCAN");
  if (ifname == nullptr) GTEST_SKIP() << "set HAUL_TEST_VCAN=vcan0 to run against a virtual CAN interface";
  auto tx = open_bus(std::string("socketcan:") + ifname);
  auto rx = open_bus(std::string("socketcan:") + ifname);
  Frame f;
  f.id = 0x123;
  f.dlc = 2;
  f.data = {0xBE, 0xEF};
  ASSERT_TRUE(tx->send(f));
  const auto got = rx->receive(500ms);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(*got, f);
}
