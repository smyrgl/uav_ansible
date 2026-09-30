#include "uav_avia/point.hpp"
#include <cassert>
#include <cmath>
#include <limits>
int main() {
  using namespace uav_avia;
  uint8_t raw[14]{};std::vector<uint8_t> cloud;
  assert(!append_point(cloud,raw,0,1,0,0));assert(cloud.empty());
  put<int32_t>(raw,1234);put<int32_t>(raw+4,-2500);put<int32_t>(raw+8,750);
  raw[12]=220;raw[13]=0x12;
  const uint64_t ts=1800000000123456789ULL;
  assert(append_point(cloud,raw,95,ts,1,0x4000));
  assert(cloud.size()==point_step);
  assert(std::abs(read<float>(cloud.data())-1.234f)<1e-6);
  assert(read<float>(cloud.data()+4)==-2.5f);
  assert(read<float>(cloud.data()+12)==220);
  assert(cloud[16]==0x12 && cloud[17]==5 && cloud[18]==1);
  const auto restored=uint64_t(read<uint32_t>(cloud.data()+20)) |
                      (uint64_t(read<uint32_t>(cloud.data()+24))<<32);
  assert(restored==ts);assert(read<uint32_t>(cloud.data()+28)==95*4167);
  assert(read<uint32_t>(cloud.data()+32)==0x4000);
  // A GPS-format timestamp is opaque too; never silently reinterpreted as epoch ns.
  assert(append_point(cloud,raw,0,0x0102030405060708ULL,3,0));
  assert(read<uint32_t>(cloud.data()+point_step+20)==0x05060708U);
}
