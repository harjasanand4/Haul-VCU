// Linux SocketCAN. On other platforms open_socketcan() throws with a pointer to the UDP bus.

#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>

#include "haul/can/bus.hpp"

#if defined(__linux__)
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#endif

namespace haul::can {

#if defined(__linux__)

namespace {

class SocketCanBus final : public Bus {
 public:
  explicit SocketCanBus(const std::string& ifname) : ifname_(ifname) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error(std::string("socketcan socket: ") + std::generic_category().message(errno));
    ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      const std::string err = std::generic_category().message(errno);
      ::close(fd_);
      throw std::runtime_error("socketcan: no interface " + ifname + " (" + err +
                               "). Create one with: sudo ip link add dev " + ifname +
                               " type vcan && sudo ip link set up " + ifname);
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
      const std::string err = std::generic_category().message(errno);
      ::close(fd_);
      throw std::runtime_error("socketcan bind " + ifname + ": " + err);
    }
  }

  ~SocketCanBus() override { ::close(fd_); }
  SocketCanBus(const SocketCanBus&) = delete;
  SocketCanBus& operator=(const SocketCanBus&) = delete;

  bool send(const Frame& f) override {
    can_frame cf{};
    cf.can_id = f.extended ? (f.id | CAN_EFF_FLAG) : f.id;
    cf.can_dlc = f.dlc;
    std::memcpy(cf.data, f.data.data(), 8);
    return ::write(fd_, &cf, sizeof cf) == static_cast<ssize_t>(sizeof cf);
  }

  std::optional<Frame> receive(std::chrono::milliseconds timeout) override {
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, static_cast<int>(timeout.count())) <= 0) return std::nullopt;
    can_frame cf{};
    if (::read(fd_, &cf, sizeof cf) != static_cast<ssize_t>(sizeof cf)) return std::nullopt;
    if (cf.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) return std::nullopt;
    Frame f;
    f.extended = (cf.can_id & CAN_EFF_FLAG) != 0;
    f.id = cf.can_id & (f.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    f.dlc = cf.can_dlc > 8 ? 8 : cf.can_dlc;
    std::memcpy(f.data.data(), cf.data, 8);
    return f;
  }

  std::string describe() const override { return "socketcan:" + ifname_; }

 private:
  int fd_ = -1;
  std::string ifname_;
};

}  // namespace

std::unique_ptr<Bus> open_socketcan(const std::string& ifname) {
  return std::make_unique<SocketCanBus>(ifname);
}

#else

std::unique_ptr<Bus> open_socketcan(const std::string&) {
  throw std::runtime_error("SocketCAN is Linux-only. Use a UDP bus instead, e.g. udp:9001:127.0.0.1:9002");
}

#endif

}  // namespace haul::can
