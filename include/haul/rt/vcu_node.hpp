#pragma once

#include <atomic>
#include <string>

#include "haul/can/bus.hpp"
#include "haul/core/latency_stats.hpp"
#include "haul/vcu/vcu.hpp"

namespace haul::rt {

struct NodeOptions {
  bool realtime = false;  // Linux: SCHED_FIFO + mlockall (needs CAP_SYS_NICE); macOS: time-constraint policy
  int priority = 80;
  std::string telemetry_csv;  // write a telemetry row every 100 ms
  bool print_status = true;   // one status line per second
};

struct NodeReport {
  long cycles = 0;
  long deadline_misses = 0;  // cycles that started late or overran the 10 ms period
  long rx_frames = 0;
  long rx_overruns = 0;  // frames dropped because the control loop fell behind
  long tx_errors = 0;
  long tx_overruns = 0;  // frames dropped because the tx thread fell behind
  // One hour of 10 ms cycles, reserved up front so recording never allocates in the control loop.
  static constexpr std::size_t kSamples = 360'000;
  LatencyStats wake_jitter{kSamples, kSamples};         // how late each cycle started, us
  LatencyStats compute{kSamples, kSamples};             // time spent inside the cycle, us
  LatencyStats send_time{2 * kSamples, 2 * kSamples};   // one bus write on the tx thread, us
  LatencyStats tx_latency{2 * kSamples, 2 * kSamples};  // frame produced -> written to the bus, us
  bool realtime_granted = false;
};

// Runs a Vcu against a real bus with four threads:
//
//   rx         blocks on the bus, timestamps frames, pushes them into a lock-free SPSC ring
//   control    wakes every 10 ms on an absolute deadline (no drift), drains the ring, runs
//              Vcu::step, queues the frames to send; never blocks, never makes a system call
//              other than the sleep and a wake-up for the tx thread
//   tx         writes queued frames to the bus (a system call that can be slow)
//   telemetry  pops samples from a second SPSC ring and does the slow work: CSV, printing
//
// Stops when `stop` becomes true (e.g. from SIGINT).
NodeReport run_vcu_node(vcu::Vcu& vcu, can::Bus& bus, const NodeOptions& options, const std::atomic<bool>& stop);

}  // namespace haul::rt
