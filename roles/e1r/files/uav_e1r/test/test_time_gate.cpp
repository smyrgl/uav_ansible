#include "uav_e1r/time_gate.hpp"
#include <cassert>
#include <iostream>
#include <vector>

using namespace uav_e1r;
void put(std::vector<uint8_t> &p, size_t offset, uint64_t n, size_t bytes) {
  for (size_t i = 0; i < bytes; ++i) { p[offset + bytes - 1 - i] = n & 255; n >>= 8; }
}
int main() {
  const int64_t now = 1790000000000000000LL;
  const int64_t steady = 10000000000LL;
  TimeGate gate;
  PacketTime p{3, 1, now + 37000000000LL, 12};
  assert(gate.check_msop(p, now, steady) == "PTP clock status invalid");
  gate.clock = {true, 37, steady};
  assert(!gate.check_msop(p, now, steady).empty());
  gate.observe_difop(p, steady);
  assert(gate.check_msop(p, now, steady).empty());
  assert(gate.utc_ns(p.raw_ns) == now);
  assert(gate.utc_seconds(1790000037.123456) == 1790000000.123456);
  p.mode = 0;
  assert(gate.check_msop(p, now, steady) == "MSOP is not gPTP time");
  p.mode = 3;
  assert(gate.check_msop(p, now, steady + 1000000001) == "DIFOP status missing or stale");
  gate.difop.status = 2;
  assert(gate.check_msop(p, now, steady) == "E1R gPTP not synchronized");
  gate.difop.status = 1;
  p.raw_ns = now;
  assert(gate.check_msop(p, now, steady) == "MSOP timestamp differs from host UTC");
  assert(gate.check_clock(steady + 5000000001) == "PTP clock status stale");
  assert(gate.check_clock(steady - 200000000) == "PTP clock status stale");
  // Verify the actual vendor byte offsets and malformed timestamp rejection.
  std::vector<uint8_t> msop(1200);
  msop[0]=0x55; msop[1]=0xaa; msop[2]=0x5a; msop[3]=0xa5; msop[9]=3;
  put(msop, 4, 287, 2); put(msop, 10, 1790000037, 6); put(msop, 16, 123456, 4);
  PacketTime decoded;
  assert(packet_time(msop.data(), msop.size(), false, decoded));
  assert(decoded.raw_ns == 1790000037123456000LL && decoded.mode == 3 && decoded.sequence == 287);
  put(msop, 16, 1000000, 4);
  assert(!packet_time(msop.data(), msop.size(), false, decoded));
  assert(!packet_time(msop.data(), 100, false, decoded));
  std::vector<uint8_t> difop(256);
  const uint8_t magic[] = {0xa5,0xff,0,0x5a,0x11,0x11,0x55,0x55};
  std::memcpy(difop.data(), magic, 8); difop[101]=3; difop[102]=1;
  put(difop, 103, 1790000037, 6); put(difop, 109, 1, 4);
  assert(packet_time(difop.data(), difop.size(), true, decoded));
  assert(decoded.status == 1 && decoded.raw_ns == 1790000037000001000LL);
  std::cout << "E1R timestamp/gating contract passed\n";
}
