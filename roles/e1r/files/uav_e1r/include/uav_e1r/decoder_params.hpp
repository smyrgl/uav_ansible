#pragma once

#include <rs_driver/driver/driver_param.hpp>

namespace uav_e1r {

// The E1R decoder's own distance window is [DISTANCE_MIN, DISTANCE_MAX] =
// [0, 200] m, inclusive, so a channel with no return (distance 0) passes it
// and decodes to (0, 0, 0) with the channel's intensity: 6.9 % of points on the
// bench, 2026-10-02. A user window replaces both bounds (rs_driver
// DistanceSection), so it starts one distance step (5 mm) above zero and keeps
// the decoder's maximum; everything outside becomes a NaN point
// (dense_points = false), keeping its firing time.
constexpr float kMinDistanceM = 0.005f;   // decoder_RSE1.hpp DISTANCE_RES
constexpr float kMaxDistanceM = 200.0f;   // decoder_RSE1.hpp DISTANCE_MAX

inline robosense::lidar::RSDecoderParam decoder_params() {
  robosense::lidar::RSDecoderParam params;
  params.use_lidar_clock = true;
  params.ts_first_point = true;
  params.dense_points = false;
  params.wait_for_difop = true;
  params.min_distance = kMinDistanceM;
  params.max_distance = kMaxDistanceM;
  return params;
}

}  // namespace uav_e1r
