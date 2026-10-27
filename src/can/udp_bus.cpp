// CAN over UDP: each datagram carries one frame in a fixed 16-byte layout
// (id: u32 big-endian, flags: u8 [bit0 = extended], dlc: u8, 2 reserved, data: 8 bytes).
// Point to point, like the cannelloni tool: one node listens on a port and sends to a peer.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>

#include "haul/can/bus.hpp"

namespace haul::can {

std::unique_ptr<Bus> open_socketcan(const std::string& ifname);  // socketcan_bus.cpp

namespace {

class UdpBus final : public Bus {
 public:
  UdpBus(std::uint16_t listen_port, const std::string& peer_host, std::uint16_t peer_port)
      : listen_port_(listen_port), peer_host_(peer_host), peer_port_(peer_port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error(std::string("udp socket: ") + std::generic_category().message(errno));
    int yes = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(listen_port);
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof local) < 0) {
      const std::string err = std::generic_category().message(errno);
      ::close(fd_);
      throw std::runtime_error("udp bind to port " + std::to_string(listen_port) + ": " + err);
    }
    peer_.sin_family = AF_INET;
    peer_.sin_port = htons(peer_port);
    if (::inet_pton(AF_INET, peer_host.c_str(), &peer_.sin_addr) != 1) {
      ::close(fd_);
      throw std::runtime_error("bad peer address " + peer_host);
    }
  }

  ~UdpBus() override { ::close(fd_); }
  UdpBus(const UdpBus&) = delete;
  UdpBus& operator=(const UdpBus&) = delete;

  bool send(const Frame& f) override {
    std::uint8_t buf[16]{};
    const std::uint32_t id = htonl(f.id);
    std::memcpy(buf, &id, 4);
    buf[4] = f.extended ? 1 : 0;
    buf[5] = f.dlc;
    std::memcpy(buf + 8, f.data.data(), 8);
    const auto n = ::sendto(fd_, buf, sizeof buf, 0, reinterpret_cast<const sockaddr*>(&peer_), sizeof peer_);
    return n == static_cast<ssize_t>(sizeof buf);  // ECONNREFUSED while the peer isn't up yet is fine
  }

  std::optional<Frame> receive(std::chrono::milliseconds timeout) override {
    pollfd p{fd_, POLLIN, 0};
    const int r = ::poll(&p, 1, static_cast<int>(timeout.count()));
    if (r <= 0) return std::nullopt;
    std::uint8_t buf[16];
    const auto n = ::recv(fd_, buf, sizeof buf, 0);
    if (n != static_cast<ssize_t>(sizeof buf)) return std::nullopt;
    Frame f;
    std::uint32_t id = 0;
    std::memcpy(&id, buf, 4);
    f.id = ntohl(id);
    f.extended = (buf[4] & 1u) != 0;
    f.dlc = buf[5] > 8 ? 8 : buf[5];
    std::memcpy(f.data.data(), buf + 8, 8);
    return f;
  }

  std::string describe() const override {
    return "udp:" + std::to_string(listen_port_) + "->" + peer_host_ + ":" + std::to_string(peer_port_);
  }

 private:
  int fd_ = -1;
  std::uint16_t listen_port_;
  std::string peer_host_;
  std::uint16_t peer_port_;
  sockaddr_in peer_{};
};

std::uint16_t parse_port(const std::string& s) {
  const int p = std::stoi(s);
  if (p <= 0 || p > 65535) throw std::invalid_argument("bad port " + s);
  return static_cast<std::uint16_t>(p);
}

}  // namespace

std::unique_ptr<Bus> open_bus(const std::string& spec) {
  if (spec.rfind("socketcan:", 0) == 0) return open_socketcan(spec.substr(10));
  if (spec.rfind("udp:", 0) == 0) {
    // udp:<listen_port>:<peer_host>:<peer_port>
    const auto a = spec.find(':', 4);
    const auto b = spec.rfind(':');
    if (a == std::string::npos || b == a) throw std::invalid_argument("expected udp:<listen>:<peer_host>:<peer_port>");
    return std::make_unique<UdpBus>(parse_port(spec.substr(4, a - 4)), spec.substr(a + 1, b - a - 1),
                                    parse_port(spec.substr(b + 1)));
  }
  throw std::invalid_argument("unknown bus '" + spec + "' (use socketcan:<if> or udp:<listen>:<host>:<port>)");
}

}  // namespace haul::can
