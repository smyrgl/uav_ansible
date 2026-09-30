#include "uav_e1r/time_gate.hpp"
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/driver/decoder/decoder_RSE1.hpp>
#include <cassert>
#include <iostream>
#include <vector>

using Cloud = PointCloudT<PointXYZIRT>;
using Decoder = robosense::lidar::DecoderRSE1<Cloud>;
void put(std::vector<uint8_t> &p, size_t off, uint64_t n, size_t bytes) {
  for (size_t i = 0; i < bytes; ++i) { p[off + bytes - 1 - i] = n & 255; n >>= 8; }
}
std::vector<uint8_t> packet(int seq, int usec) {
  std::vector<uint8_t> p(1200);
  p[0]=0x55; p[1]=0xaa; p[2]=0x5a; p[3]=0xa5; p[8]=4; p[9]=3;
  put(p,4,seq,2); put(p,10,1790000037,6); put(p,16,usec,4);
  for (int i=0; i<96; ++i) {
    const int off=32+i*12;
    put(p,off,i,2); put(p,off+2,200,2); put(p,off+4,32767,2); p[off+10]=42;
  }
  return p;
}
int main() {
  robosense::lidar::RSDecoderParam params;
  params.use_lidar_clock=true; params.ts_first_point=true;
  Decoder decoder(params);
  decoder.point_cloud_=std::make_shared<Cloud>();
  int frames=0;
  uav_e1r::TimeGate gate; gate.clock.utc_offset=37;
  decoder.regCallback([](const auto &error) { throw std::runtime_error(error.toString()); },
    [&](uint16_t, double stamp) {
      ++frames;
      const auto cloud=decoder.point_cloud_;
      assert(cloud->points.size()==288*96);
      if(frames==2) {
        assert(std::abs(gate.utc_seconds(stamp)-1790000000.1)<1e-6);
        const auto &p=cloud->points.at(95);
        assert(std::abs(p.x-32767.0f/32768)<1e-6f && p.y==0 && p.z==0);
        assert(p.intensity==42 && p.ring==0);
        assert(std::abs(gate.utc_seconds(p.timestamp)-1790000000.100095)<1e-6);
        for(const auto &point: cloud->points) {
          const double utc=gate.utc_seconds(point.timestamp);
          assert(utc>=1790000000.1 && utc<1790000000.2);
        }
      }
      decoder.point_cloud_=std::make_shared<Cloud>();
    });
  for(int frame=0; frame<2; ++frame) {
    for(int seq=0; seq<288; ++seq) {
      const auto p=packet(seq,frame*100000+seq*347);
      decoder.processMsopPkt(p.data(),p.size());
    }
  }
  const auto p=packet(0,200000); decoder.processMsopPkt(p.data(),p.size());
  assert(frames==2);
  std::cout << "Pinned vendor decoder geometry, firing-time units, and UTC conversion passed\n";
}
