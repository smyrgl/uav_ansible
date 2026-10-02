#include "uav_avia/time.hpp"
#include <cassert>
#include <cstring>
int main() {
  using namespace uav_avia;
  UtcFields f{};
  // 2026-10-01T18:47:05Z
  assert(utc_fields(1790880425, f));
  assert(f.year == 26 && f.month == 10 && f.day == 1 && f.hour == 18);
  assert(f.microsecond == (47u * 60u + 5u) * 1000000u);
  // A packet stamp in the same layout decodes back to the same instant, with sub-second microseconds.
  uint8_t b[8] = {f.year, f.month, f.day, f.hour, 0, 0, 0, 0};
  const uint32_t us = f.microsecond + 123456;
  std::memcpy(b + 4, &us, 4);
  int64_t ns = 0;
  assert(utc_stamp_ns(b, ns));
  assert(ns == 1790880425LL * ns_per_s + 123456000LL);
  // Last second of the year: microseconds within the hour reach 3599 s.
  assert(utc_fields(1798761599, f));
  assert(f.year == 26 && f.month == 12 && f.day == 31 && f.hour == 23 && f.microsecond == 3599000000u);
  // Invalid stamps are rejected, never silently decoded.
  uint8_t bad_month[8] = {26, 13, 1, 0, 0, 0, 0, 0};
  assert(!utc_stamp_ns(bad_month, ns));
  uint8_t bad_us[8] = {26, 10, 1, 0, 0, 0, 0, 0};
  const uint32_t too_big = 3600000000u;
  std::memcpy(bad_us + 4, &too_big, 4);
  assert(!utc_stamp_ns(bad_us, ns));
  // Before 2000 does not fit a year-since-2000 byte.
  assert(!utc_fields(946684799, f));
  // Send window: 100 ms after the top of a second labels that second; outside it, nothing.
  int64_t sec = 0;
  const int64_t top = 1790880425LL * ns_per_s;
  assert(pulse_second(top + 100000000, 60000000, 440000000, sec) && sec == 1790880425);
  assert(!pulse_second(top + 5000000, 60000000, 440000000, sec));
  assert(!pulse_second(top + 600000000, 60000000, 440000000, sec));
  // PPS + UTC trust needs PPS OK (bit 9), GPS sync (2 in bits 14-16), a UTC stamp and a sane latency.
  const uint32_t gps_pps = (1u << 9) | (2u << 14);
  assert(sensor_time_trusted(3, gps_pps, 800000, -1000000, 50000000));
  assert(!sensor_time_trusted(3, 2u << 14, 800000, -1000000, 50000000));                // no PPS
  assert(!sensor_time_trusted(3, (1u << 9) | (3u << 14), 800000, -1000000, 50000000));  // PPS-only mode
  assert(!sensor_time_trusted(0, gps_pps, 800000, -1000000, 50000000));                 // uptime stamp
  assert(!sensor_time_trusted(3, gps_pps, ns_per_s + 800000, -1000000, 50000000));      // a second late
  assert(!sensor_time_trusted(3, gps_pps, -ns_per_s + 800000, -1000000, 50000000));     // a second early
  // PTP trust needs ptp_status (bit 13) and PTP sync (1); a TAI stamp read as UTC is 37 s off.
  const uint32_t ptp = (1u << 13) | (1u << 14);
  assert(sensor_time_trusted(1, ptp, 600000, -1000000, 50000000));
  assert(!sensor_time_trusted(1, 1u << 14, 600000, -1000000, 50000000));               // master lost
  assert(!sensor_time_trusted(1, (1u << 13) | (4u << 14), 600000, -1000000, 50000000));  // sync abnormal
  assert(!sensor_time_trusted(1, ptp, -37 * ns_per_s + 600000, -1000000, 50000000));   // TAI vs UTC
  assert(!sensor_time_trusted(1, gps_pps, 600000, -1000000, 50000000));                 // PTP stamp, GPS flags
}
