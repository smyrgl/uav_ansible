#pragma once
// UTC helpers for Livox SDK1 "GPS" synchronization: PPS on the Sync pair plus
// the UTC time of each pulse sent over the SDK (LidarSetUtcSyncTime).
// Pure functions, no SDK or ROS, so the wire contract is unit-tested.
#include <cstdint>
#include <cstring>
#include <ctime>

namespace uav_avia {

constexpr int64_t ns_per_s = 1000000000LL;

// Field order of the SDK's LidarSetUtcSyncTimeRequest and of a timestamp_type 3
// packet stamp: year since 2000, month 1-12, day 1-31, hour 0-23, then the
// microseconds since the top of that hour (little-endian uint32). This is what
// the SDK's own GPRMC parser produces.
struct UtcFields {
  uint8_t year, month, day, hour;
  uint32_t microsecond;
};

// Calendar fields for a whole UTC second: "the UTC time of the pulse".
inline bool utc_fields(int64_t utc_sec, UtcFields& f) {
  const std::time_t t = static_cast<std::time_t>(utc_sec);
  std::tm tm{};
  if (!gmtime_r(&t, &tm) || tm.tm_year < 100 || tm.tm_year > 355) return false;
  f.year = static_cast<uint8_t>(tm.tm_year - 100);
  f.month = static_cast<uint8_t>(tm.tm_mon + 1);
  f.day = static_cast<uint8_t>(tm.tm_mday);
  f.hour = static_cast<uint8_t>(tm.tm_hour);
  f.microsecond = static_cast<uint32_t>(tm.tm_min * 60 + tm.tm_sec) * 1000000u;
  return true;
}

// An 8-byte timestamp_type 3 packet stamp to UTC nanoseconds since the epoch.
inline bool utc_stamp_ns(const uint8_t* b, int64_t& ns) {
  uint32_t us;
  std::memcpy(&us, b + 4, sizeof(us));
  if (b[1] < 1 || b[1] > 12 || b[2] < 1 || b[2] > 31 || b[3] > 23 || us >= 3600000000u) return false;
  std::tm tm{};
  tm.tm_year = b[0] + 100;
  tm.tm_mon = b[1] - 1;
  tm.tm_mday = b[2];
  tm.tm_hour = b[3];
  const std::time_t hour_start = timegm(&tm);
  if (hour_start == static_cast<std::time_t>(-1)) return false;
  ns = (static_cast<int64_t>(hour_start) * 1000000LL + us) * 1000LL;
  return true;
}

// The pulse to label at wall time now_ns: the whole second that began
// (now_ns mod 1 s) ago, if that offset is within [lo_ns, hi_ns]. The Avia
// manual requires the UTC command 10-500 ms after the rising edge it describes.
inline bool pulse_second(int64_t now_ns, int64_t lo_ns, int64_t hi_ns, int64_t& sec) {
  const int64_t frac = ((now_ns % ns_per_s) + ns_per_s) % ns_per_s;
  if (frac < lo_ns || frac > hi_ns) return false;
  sec = (now_ns - frac) / ns_per_s;
  return true;
}

// Status word (SDK LidarErrorCode): pps_status is bit 9, ptp_status bit 13,
// time_sync_status bits 14-16.
inline bool status_pps_ok(uint32_t status) { return (status >> 9) & 1U; }
inline bool status_ptp_ok(uint32_t status) { return (status >> 13) & 1U; }
inline uint32_t status_time_sync(uint32_t status) { return (status >> 14) & 7U; }
constexpr uint32_t time_sync_ptp = 1;  // IEEE 1588 PTP
constexpr uint32_t time_sync_gps = 2;  // PPS + UTC ("GPS") synchronization
constexpr uint8_t timestamp_ptp = 1;   // kTimestampTypePtp: PTP nanoseconds
constexpr uint8_t timestamp_utc = 3;   // kTimestampTypePpsGps: UTC calendar stamp

// Sensor time is trusted only when the Avia itself reports the matching source
// live (PTP: ptp_status and sync mode 1; PPS + UTC: pps_status and sync mode 2)
// and the stamp, converted to UTC, is plausibly just before its receipt. A
// label one second off, or a TAI/UTC mix-up, fails the latency bound.
inline bool sensor_time_trusted(uint8_t type, uint32_t status, int64_t latency_ns,
                                int64_t min_latency_ns, int64_t max_latency_ns) {
  bool source = false;
  if (type == timestamp_utc) source = status_pps_ok(status) && status_time_sync(status) == time_sync_gps;
  else if (type == timestamp_ptp) source = status_ptp_ok(status) && status_time_sync(status) == time_sync_ptp;
  return source && latency_ns >= min_latency_ns && latency_ns <= max_latency_ns;
}

}  // namespace uav_avia
