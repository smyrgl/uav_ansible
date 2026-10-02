# E1R ROS driver

This role installs `uav_e1r`, a small ROS 2 adapter around RoboSense's unmodified
`DecoderRSE1` from rs_driver commit
`897b14d3bdb6186a75df27ba51b65b5bd5557723`. Geometry and per-point microsecond
firing offsets are decoded by that vendor implementation. PCL is not required.

The shared ROS role must first create `uav-ros`, `/opt/uav/ros`, and the common
environment/Cyclone files. The root-owned time-status producer must maintain
`/run/uav/time/ptp-status.json`; publication remains blocked if it is unavailable.

Outputs:

* `/e1r/points`: best-effort PointCloud2, `e1r_nominal_lidar_frame`; float32 x/y/z,
  uint8 intensity, uint16 ring, float64 `timestamp` in **UTC seconds**. The cloud
  header is first-point UTC. Invalid ranges remain NaN. Per-point timing is
  preserved, with the same PTP-to-UTC offset removed from every point.

  No returns: the decoder's own distance window is [0, 200] m inclusive, so a
  channel with no return (distance 0) passed it and decoded to (0, 0, 0) with
  its intensity, 6.9 % of the points on the bench, contradicting the NaN above.
  Since 2026-10-02 the adapter passes a user window of [5 mm, 200 m]
  (`include/uav_e1r/decoder_params.hpp`, one distance step above zero; a user
  window replaces both bounds), so they are NaN points that keep their firing
  time, and the decoder test covers it. Measured on the bench: 27,648 points
  per frame in 432 firing slots of 64 points, 216 µs apart (95.7 ms per frame),
  frames starting on the UTC 100 ms grid; `ring` is always 0.
* `/e1r/difop_raw`: best-effort UInt8MultiArray retaining the complete 256-byte
  device status packet, including its original raw PTP timestamp and IMU bytes.
* `/diagnostics`: `e1r/driver`, hardware ID `E1R@192.168.144.95`, every second.

Timing gates require: a fresh (<=5s), same-boot verified PTP/UTC offset; DIFOP
mode3/status1 received within1s; MSOP mode3; both packet timestamps within2s of
host UTC after conversion. The2s threshold detects gross epoch/offset errors;
it is not a precision synchronization claim. A failed gate discards the
accumulated scan. Publication resumes only at complete vendor frame boundaries.
Offset changes and backward clock jumps also discard accumulated scans.

The time JSON contract uses `current_utc_offset`, `ptp_timescale`,
`utc_offset_verified`, `utc_offset_valid`, `updated_monotonic_ns`, and `boot_id`.
`utc_offset_valid` is diagnostic metadata: the configured static gPTP master does
not claim external traceability. `utc_offset_verified` comes from the separate
host-clock/PHC/PMC validation and is the actual conversion gate.

IMU publication is intentionally deferred until sensor units and the IMU frame
are verified. No receive-time fallback produces apparently synchronized clouds.

Local contract test (no ROS dependencies):

```
c++ -std=c++17 -I files/uav_e1r/include files/uav_e1r/test/test_time_gate.cpp -o /tmp/test_e1r_time
/tmp/test_e1r_time
```

Full ROS compile and CTest are performed by the role when sources change.
