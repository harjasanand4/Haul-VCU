#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>
#include <type_traits>

namespace haul {

// Lock-free single-producer / single-consumer ring buffer.
//
// One thread pushes, one thread pops, nobody blocks and nothing is allocated after
// construction, so the control loop can hand data to other threads without ever waiting on a
// mutex held by a lower-priority thread (priority inversion). The head and tail indices live
// on separate cache lines so producer and consumer don't fight over one line, and each side
// keeps a cached copy of the other's index to avoid touching shared state on every call.
template <typename T, std::size_t N>
class SpscRing {
  static_assert(N >= 2 && (N & (N - 1)) == 0, "capacity must be a power of two");
  static_assert(std::is_trivially_copyable_v<T>, "elements are copied by value between threads");

 public:
  // Producer side.
  bool try_push(const T& value) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_cache_ == N) {
      tail_cache_ = tail_.load(std::memory_order_acquire);
      if (head - tail_cache_ == N) return false;  // full
    }
    buffer_[head & (N - 1)] = value;
    head_.store(head + 1, std::memory_order_release);  // publish the element
    return true;
  }

  // Consumer side.
  std::optional<T> try_pop() {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_cache_) {
      head_cache_ = head_.load(std::memory_order_acquire);
      if (tail == head_cache_) return std::nullopt;  // empty
    }
    T value = buffer_[tail & (N - 1)];
    tail_.store(tail + 1, std::memory_order_release);  // free the slot
    return value;
  }

  std::size_t size_approx() const {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }
  static constexpr std::size_t capacity() { return N; }

 private:
  static constexpr std::size_t kCacheLine = 64;

  alignas(kCacheLine) std::atomic<std::size_t> head_{0};  // written by the producer
  std::size_t tail_cache_ = 0;                            // producer's view of tail
  alignas(kCacheLine) std::atomic<std::size_t> tail_{0};  // written by the consumer
  std::size_t head_cache_ = 0;                            // consumer's view of head
  alignas(kCacheLine) std::array<T, N> buffer_{};
};

}  // namespace haul
