#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "haul/can/frame.hpp"

namespace haul::can {

// A CAN interface the real-time nodes can talk through.
//
//   socketcan:vcan0              Linux SocketCAN (real hardware or a virtual vcan device)
//   udp:9001:127.0.0.1:9002      CAN frames tunnelled over UDP (listen port : peer host : peer port),
//                                which also works on macOS, where SocketCAN doesn't exist.
class Bus {
 public:
  virtual ~Bus() = default;
  virtual bool send(const Frame& f) = 0;
  // Waits up to `timeout` for a frame. Returns nullopt on timeout.
  virtual std::optional<Frame> receive(std::chrono::milliseconds timeout) = 0;
  virtual std::string describe() const = 0;
};

std::unique_ptr<Bus> open_bus(const std::string& spec);

}  // namespace haul::can
