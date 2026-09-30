#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

namespace uav_e1r {

struct PacketTime {
  uint8_t mode = 0;
  uint8_t status = 0;
  int64_t raw_ns = 0;
  uint16_t sequence = 0;
};

inline uint64_t big_endian(const uint8_t *p, size_t bytes) {
  uint64_t n = 0;
  for (size_t i = 0; i < bytes; ++i) n = (n << 8) | p[i];
  return n;
}

inline bool packet_time(const uint8_t *data, size_t length, bool difop, PacketTime &out) {
  constexpr std::array<uint8_t, 8> difop_magic{0xa5, 0xff, 0, 0x5a, 0x11, 0x11, 0x55, 0x55};
  constexpr std::array<uint8_t, 4> msop_magic{0x55, 0xaa, 0x5a, 0xa5};
  if (length != (difop ? 256u : 1200u)) return false;
  if (std::memcmp(data, difop ? difop_magic.data() : msop_magic.data(), difop ? 8 : 4)) return false;
  const size_t off = difop ? 103 : 10;
  const uint64_t sec = big_endian(data + off, 6);
  const uint64_t us = big_endian(data + off + 6, 4);
  if (us >= 1000000 || sec > uint64_t(std::numeric_limits<int64_t>::max() / 1000000000LL)) return false;
  out.mode = data[difop ? 101 : 9];
  out.status = difop ? data[102] : 0;
  out.raw_ns = int64_t(sec) * 1000000000LL + int64_t(us) * 1000;
  out.sequence = difop ? 0 : uint16_t(big_endian(data + 4, 2));
  return true;
}

struct ClockState {
  bool valid = false;
  int utc_offset = 0;
  int64_t updated_steady_ns = 0;
};

// Readiness gates are deliberately separate from geometrical decoding. Host-age
// checks reject gross epoch/offset mistakes; they do not certify capture accuracy.
class TimeGate {
 public:
  int64_t status_max_age_ns = 5000000000LL;
  int64_t difop_max_age_ns = 1000000000LL;
  int64_t timestamp_tolerance_ns = 2000000000LL;
  ClockState clock;
  PacketTime difop;
  int64_t difop_received_steady_ns = -1;

  void observe_difop(const PacketTime &value, int64_t steady_ns) {
    difop = value;
    difop_received_steady_ns = steady_ns;
  }

  std::string check_clock(int64_t steady_ns) const {
    if (!clock.valid || clock.utc_offset < 0 || clock.utc_offset > 128) return "PTP clock status invalid";
    const auto age = steady_ns - clock.updated_steady_ns;
    if (age < -100000000LL || age > status_max_age_ns) return "PTP clock status stale";
    return {};
  }

  std::string check_difop(int64_t wall_ns, int64_t steady_ns) const {
    auto reason = check_clock(steady_ns);
    if (!reason.empty()) return reason;
    if (difop_received_steady_ns < 0 || steady_ns - difop_received_steady_ns > difop_max_age_ns)
      return "DIFOP status missing or stale";
    if (difop.mode != 3 || difop.status != 1) return "E1R gPTP not synchronized";
    if (std::abs(wall_ns - utc_ns(difop.raw_ns)) > timestamp_tolerance_ns)
      return "DIFOP timestamp differs from host UTC";
    return {};
  }

  std::string check_msop(const PacketTime &packet, int64_t wall_ns, int64_t steady_ns) const {
    auto reason = check_difop(wall_ns, steady_ns);
    if (!reason.empty()) return reason;
    if (packet.mode != 3) return "MSOP is not gPTP time";
    if (std::abs(wall_ns - utc_ns(packet.raw_ns)) > timestamp_tolerance_ns)
      return "MSOP timestamp differs from host UTC";
    return {};
  }

  int64_t utc_ns(int64_t raw_ns) const { return raw_ns - int64_t(clock.utc_offset) * 1000000000LL; }
  double utc_seconds(double raw_seconds) const { return raw_seconds - clock.utc_offset; }
};

}  // namespace uav_e1r
