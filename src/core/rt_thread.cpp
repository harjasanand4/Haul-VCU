#include "haul/core/rt_thread.hpp"

#include <cstdint>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <pthread.h>
#endif

namespace haul {

bool prefer_interactive_scheduling() {
#if defined(__APPLE__)
  return ::pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0) == 0;
#else
  return false;
#endif
}

bool make_thread_realtime(Micros period, Micros computation, Micros constraint, int linux_priority) {
#if defined(__linux__)
  (void)period;
  (void)computation;
  (void)constraint;
  // Lock memory so a page fault can't stall the loop, then ask for a real-time priority.
  ::mlockall(MCL_CURRENT | MCL_FUTURE);
  sched_param sp{};
  sp.sched_priority = linux_priority;
  return ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &sp) == 0;
#elif defined(__APPLE__)
  (void)linux_priority;
  mach_timebase_info_data_t tb{};
  if (::mach_timebase_info(&tb) != KERN_SUCCESS || tb.numer == 0) return false;
  const auto to_abs = [&](Micros us) {
    return static_cast<std::uint32_t>(static_cast<double>(us) * 1000.0 * static_cast<double>(tb.denom) /
                                      static_cast<double>(tb.numer));
  };
  thread_time_constraint_policy_data_t policy{};
  policy.period = to_abs(period);
  policy.computation = to_abs(computation);
  policy.constraint = to_abs(constraint);
  policy.preemptible = 1;
  const auto count = static_cast<mach_msg_type_number_t>(sizeof(policy) / sizeof(integer_t));
  return ::thread_policy_set(::pthread_mach_thread_np(::pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
                             reinterpret_cast<thread_policy_t>(&policy), count) == KERN_SUCCESS;
#else
  (void)period;
  (void)computation;
  (void)constraint;
  (void)linux_priority;
  return false;
#endif
}

}  // namespace haul
