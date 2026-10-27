#include "haul/rt/vcu_node.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <thread>

#include "haul/core/rt_thread.hpp"
#include "haul/core/spsc_ring.hpp"
#include "haul/core/units.hpp"

namespace haul::rt {

namespace {

using Clock = std::chrono::steady_clock;

struct RxItem {
  can::Frame frame;
  Micros time;
};

struct TxItem {
  can::Frame frame;
  Clock::time_point queued;
};

struct TelemetryItem {
  Micros time;
  vcu::VcuState state;
  const char* reason;  // a string literal from the state machine: copying the pointer is enough
  std::uint16_t faults;
  float speed_kph, setpoint_kph, target_kph, traction_kn, brake_pct;
  float jitter_us, compute_us;
};

Micros since(Clock::time_point start) {
  return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start).count();
}

}  // namespace

NodeReport run_vcu_node(vcu::Vcu& vcu, can::Bus& bus, const NodeOptions& opt, const std::atomic<bool>& stop) {
  NodeReport report;
  const auto start = Clock::now();
  auto rx_ring = std::make_unique<SpscRing<RxItem, 1024>>();
  auto tele_ring = std::make_unique<SpscRing<TelemetryItem, 1024>>();
  auto tx_ring = std::make_unique<SpscRing<TxItem, 256>>();
  std::atomic<std::uint32_t> tx_signal{0};  // bumped by the control thread after queueing frames
  std::atomic<bool> tx_stop{false};
  long tx_errors = 0, tx_overruns = 0;
  std::atomic<long> rx_frames{0}, rx_overruns{0};
  std::atomic<bool> control_done{false};

  std::thread rx([&] {
    prefer_interactive_scheduling();
    while (!stop.load(std::memory_order_relaxed)) {
      auto f = bus.receive(std::chrono::milliseconds(5));
      if (!f) continue;
      rx_frames.fetch_add(1, std::memory_order_relaxed);
      if (!rx_ring->try_push({*f, since(start)})) rx_overruns.fetch_add(1, std::memory_order_relaxed);
    }
  });

  // The bus write is a system call, and on some systems (UDP on macOS) it takes hundreds of
  // microseconds. It happens here, so the control loop only ever copies a frame into a ring:
  // the way an AUTOSAR COM layer hands frames to the CAN driver.
  std::thread tx([&] {
    prefer_interactive_scheduling();
    for (;;) {
      const auto seen = tx_signal.load(std::memory_order_acquire);  // before draining: no lost wake-up
      while (auto item = tx_ring->try_pop()) {
        const auto t0 = Clock::now();
        if (!bus.send(item->frame)) ++tx_errors;
        const auto t1 = Clock::now();
        report.send_time.add(std::chrono::duration<double, std::micro>(t1 - t0).count());
        report.tx_latency.add(std::chrono::duration<double, std::micro>(t1 - item->queued).count());
      }
      if (tx_stop.load(std::memory_order_acquire) && tx_ring->size_approx() == 0) break;
      tx_signal.wait(seen, std::memory_order_acquire);
    }
  });

  std::thread telemetry([&] {
    std::ofstream csv;
    if (!opt.telemetry_csv.empty()) {
      csv.open(opt.telemetry_csv);
      csv << "t,state,faults,speed_kph,setpoint_kph,target_kph,traction_kn,brake_pct,jitter_us,compute_us\n";
    }
    Micros last_print = 0, last_row = -kMicrosPerSecond;
    auto last_state = vcu::VcuState::Init;
    while (!control_done.load(std::memory_order_acquire) || tele_ring->size_approx() > 0) {
      auto item = tele_ring->try_pop();
      if (!item) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      if (csv.is_open() && item->time - last_row >= ms(100)) {
        last_row = item->time;
        char buf[200];
        std::snprintf(buf, sizeof buf, "%.3f,%s,%u,%.2f,%.2f,%.2f,%.1f,%.1f,%.0f,%.0f\n", to_seconds(item->time),
                      vcu::to_string(item->state), item->faults, item->speed_kph, item->setpoint_kph, item->target_kph,
                      item->traction_kn, item->brake_pct, item->jitter_us, item->compute_us);
        csv << buf;
      }
      if (opt.print_status && item->state != last_state) {
        // Every transition, the moment it happens, with the reason and the faults behind it.
        std::printf("%8.2f s  %s -> %s: %s  (faults %s)\n", to_seconds(item->time), vcu::to_string(last_state),
                    vcu::to_string(item->state), item->reason, vcu::describe(vcu::FaultSet(item->faults)).c_str());
        std::fflush(stdout);
      }
      last_state = item->state;
      if (opt.print_status && item->time - last_print >= kMicrosPerSecond) {
        last_print = item->time;
        std::printf("%8.1f s  %-14s %6.1f km/h (set %5.1f, target %5.1f)  drive %7.1f kN  brake %5.1f %%  faults %s\n",
                    to_seconds(item->time), vcu::to_string(item->state), item->speed_kph, item->setpoint_kph,
                    item->target_kph, item->traction_kn, item->brake_pct,
                    vcu::describe(vcu::FaultSet(item->faults)).c_str());
        std::fflush(stdout);
      }
    }
  });

  prefer_interactive_scheduling();
  if (opt.realtime) {
    report.realtime_granted = make_thread_realtime(vcu::Vcu::kPeriod, ms(1), ms(5), opt.priority);
    if (!report.realtime_granted)
      std::fprintf(stderr, "haul-vcu: real-time scheduling not granted, running best-effort\n");
  }

  // Control loop: absolute deadlines, so a late cycle doesn't push every later cycle back.
  const auto period = std::chrono::microseconds(vcu::Vcu::kPeriod);
  auto next = Clock::now() + period;
  while (!stop.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_until(next);
    const auto woke = Clock::now();
    const double jitter_us = std::chrono::duration<double, std::micro>(woke - next).count();
    const Micros now = since(start);

    while (auto item = rx_ring->try_pop()) vcu.on_frame(item->frame, item->time);
    const auto batch = vcu.step(now);
    const auto stepped = Clock::now();
    for (const auto& f : batch) {
      if (!tx_ring->try_push({f, stepped})) ++tx_overruns;
    }
    tx_signal.fetch_add(1, std::memory_order_release);
    tx_signal.notify_one();

    const double compute_us = std::chrono::duration<double, std::micro>(Clock::now() - woke).count();
    report.wake_jitter.add(jitter_us);
    report.compute.add(compute_us);
    ++report.cycles;
    if (jitter_us + compute_us > static_cast<double>(vcu::Vcu::kPeriod)) ++report.deadline_misses;

    const auto& d = vcu.debug();
    tele_ring->try_push({now, d.state, d.state_reason, d.faults.bits(),
                         static_cast<float>(mps_to_kph(d.speed_estimate_mps)),
                         static_cast<float>(mps_to_kph(d.setpoint_mps)), static_cast<float>(mps_to_kph(d.target_mps)),
                         static_cast<float>(d.traction_request_n / 1000.0), static_cast<float>(d.brake_request_pct),
                         static_cast<float>(jitter_us), static_cast<float>(compute_us)});

    next += period;
    if (Clock::now() > next + 10 * period) next = Clock::now() + period;  // after a long stall, resync
  }

  control_done.store(true, std::memory_order_release);
  tx_stop.store(true, std::memory_order_release);
  tx_signal.fetch_add(1, std::memory_order_release);
  tx_signal.notify_one();
  rx.join();
  tx.join();
  telemetry.join();
  report.tx_errors = tx_errors;
  report.tx_overruns = tx_overruns;
  report.rx_frames = rx_frames.load();
  report.rx_overruns = rx_overruns.load();
  return report;
}

}  // namespace haul::rt
