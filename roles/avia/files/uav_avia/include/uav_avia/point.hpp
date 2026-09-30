#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <stdexcept>
namespace uav_avia {
// Exact little-endian Livox packet timestamp is retained, without asserting UTC.
// 4167 ns and six scan lines match the official Avia ROS driver table.
constexpr uint32_t point_step = 36;
constexpr uint32_t point_interval_ns = 4167;
template<class T> T read(const uint8_t* p) { T v; std::memcpy(&v,p,sizeof(v)); return v; }
template<class T> void put(uint8_t* p,T v) { std::memcpy(p,&v,sizeof(v)); }
inline bool append_point(std::vector<uint8_t>& out,const uint8_t* raw,
                         uint32_t index,uint64_t packet_time,uint8_t type,uint32_t status) {
  const auto x=read<int32_t>(raw), y=read<int32_t>(raw+4), z=read<int32_t>(raw+8);
  if (x==0 && y==0 && z==0) return false; // Vendor-defined no-return sentinel.
  const auto pos=out.size();out.resize(pos+point_step);auto* p=out.data()+pos;
  put<float>(p,x*.001f);put<float>(p+4,y*.001f);put<float>(p+8,z*.001f);
  put<float>(p+12,raw[12]);p[16]=raw[13];p[17]=index%6;p[18]=type;p[19]=0;
  put<uint32_t>(p+20,static_cast<uint32_t>(packet_time));
  put<uint32_t>(p+24,static_cast<uint32_t>(packet_time>>32));
  put<uint32_t>(p+28,index*point_interval_ns);put<uint32_t>(p+32,status);
  return true;
}
}
