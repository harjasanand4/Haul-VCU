#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "haul/core/latency_stats.hpp"
#include "haul/core/spsc_ring.hpp"
#include "haul/core/time.hpp"
#include "haul/core/units.hpp"
#include "haul/vcu/monitors.hpp"

using namespace haul;

TEST(SpscRing, FillsDrainsAndWrapsInOrder) {
  SpscRing<int, 4> ring;
  EXPECT_FALSE(ring.try_pop().has_value());
  for (int round = 0; round < 3; ++round) {  // indices run past N, so the wrap-around is exercised
    for (int i = 0; i < 4; ++i) EXPECT_TRUE(ring.try_push(round * 10 + i));
    EXPECT_FALSE(ring.try_push(99)) << "a full ring must refuse, not overwrite";
    EXPECT_EQ(ring.size_approx(), 4u);
    for (int i = 0; i < 4; ++i) EXPECT_EQ(ring.try_pop(), round * 10 + i);
    EXPECT_FALSE(ring.try_pop().has_value());
  }
}

// Two threads hammer the ring with a tiny capacity so it is constantly full or empty. Every item
// carries its sequence number; the consumer checks nothing is lost, duplicated or reordered.
// Run this under ThreadSanitizer (the CI does) to check the memory ordering too.
TEST(SpscRing, TwoThreadStressKeepsEveryItemInOrder) {
  struct Item {
    std::uint64_t seq;
    std::uint64_t check;
  };
  constexpr std::uint64_t kItems = 2'000'000;
  auto ring = std::make_unique<SpscRing<Item, 8>>();
  std::atomic<bool> ok{true};
  std::thread consumer([&] {
    std::uint64_t expect = 0;
    while (expect < kItems) {
      if (auto item = ring->try_pop()) {
        if (item->seq != expect || item->check != ~expect) ok = false;
        ++expect;
      }
    }
  });
  for (std::uint64_t i = 0; i < kItems; ++i) {
    while (!ring->try_push(Item{i, ~i})) std::this_thread::yield();
  }
  consumer.join();
  EXPECT_TRUE(ok.load());
  EXPECT_EQ(ring->size_approx(), 0u);
}

TEST(LatencyStats, PercentilesAndMax) {
  LatencyStats s;
  for (int i = 1; i <= 100; ++i) s.add(i);
  EXPECT_EQ(s.count(), 100u);
  EXPECT_DOUBLE_EQ(s.max(), 100.0);
  EXPECT_DOUBLE_EQ(s.percentile(0.0), 1.0);
  EXPECT_NEAR(s.percentile(0.5), 50.0, 1.0);
  EXPECT_NEAR(s.percentile(0.99), 99.0, 1.0);
  EXPECT_DOUBLE_EQ(LatencyStats{}.percentile(0.5), 0.0);
}

TEST(Units, ConversionsAndRateLimit) {
  EXPECT_DOUBLE_EQ(kph_to_mps(36.0), 10.0);
  EXPECT_DOUBLE_EQ(mps_to_kph(10.0), 36.0);
  EXPECT_NEAR(grade_pct_to_rad(100.0), std::numbers::pi / 4.0, 1e-12);
  EXPECT_DOUBLE_EQ(rate_limit(0.0, 10.0, 1.0, 2.0), 1.0);
  EXPECT_DOUBLE_EQ(rate_limit(0.0, -10.0, 1.0, 2.0), -2.0);
  EXPECT_DOUBLE_EQ(rate_limit(0.0, 0.5, 1.0, 2.0), 0.5);
  EXPECT_EQ(seconds(1.5), 1'500'000);
  EXPECT_EQ(ms(10), 10'000);
}

TEST(Debouncer, NeedsTheConditionToHoldForTheOnAndOffTimes) {
  vcu::Debouncer d(ms(100), ms(50));
  EXPECT_FALSE(d.update(true, 0));
  EXPECT_FALSE(d.update(true, ms(90)));
  EXPECT_TRUE(d.update(true, ms(100)));
  EXPECT_TRUE(d.update(false, ms(110)));  // a short dropout doesn't clear it
  EXPECT_TRUE(d.update(true, ms(120)));   // and it resets the off timer
  EXPECT_TRUE(d.update(false, ms(130)));
  EXPECT_TRUE(d.update(false, ms(170)));
  EXPECT_FALSE(d.update(false, ms(180)));
}

TEST(Debouncer, ASingleNoisySampleDoesNotTrip) {
  vcu::Debouncer d(ms(150), 0);
  for (Micros t = 0; t < seconds(5); t += ms(10)) {
    const bool glitch = (t / ms(10)) % 20 == 0;  // one bad sample every 200 ms
    EXPECT_FALSE(d.update(glitch, t));
  }
}
