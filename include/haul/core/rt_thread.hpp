#pragma once

#include "haul/core/time.hpp"

namespace haul {

// Scheduling for the threads that keep time (the VCU's rx and control threads, the truck
// simulator's loop). Best effort: every call reports whether it got what it asked for.

// macOS: QoS "user interactive", so the thread runs on a performance core without timer
// coalescing. No privileges needed. A no-op elsewhere.
bool prefer_interactive_scheduling();

// Real-time scheduling for a periodic thread.
//   Linux: mlockall + SCHED_FIFO at `linux_priority` (needs CAP_SYS_NICE).
//   macOS: the Mach time-constraint policy (period, computation, constraint), which is what
//          audio threads use. No privileges needed.
bool make_thread_realtime(Micros period, Micros computation, Micros constraint, int linux_priority);

}  // namespace haul
