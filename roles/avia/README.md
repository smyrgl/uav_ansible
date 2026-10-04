# Original Livox Avia bench driver

The `avia` role builds a small ROS 2 Jazzy C++ adapter around the unmodified official Livox SDK1 v2.3.1 source, pinned at `14c533dd7175bd90a6b568c0aa1733f35d36cb89`. It does not depend on PCL or the older upstream ROS wrapper. A vendor-target-only `-Wno-c++20-compat` setting accommodates bundled fmt's legal C++11 `char8_t` identifier on GCC 13; it does not turn off other build errors.

`uav-avia.service` is a boot-enabled system service running as `uav-ros`, supervised with `Restart=on-failure`, and grouped under `uav-ros.target`. It uses the common ROS environment and `/etc/uav/ros/avia.yaml`. SDK discovery only accepts original Avia type 7 with the configured broadcast code AND address. The SDK owns connection, heartbeat, and rediscovery. Setup requires acknowledged Cartesian, single first-return, IMU-200-Hz, and start commands; failure restarts the service. The service writes no IP, firmware or extrinsic settings. Once a second it sends the UTC time of that second's PPS pulse (`LidarSetUtcSyncTime`), which is runtime state on the Avia, not stored configuration. The firmware version is logged at connect and reported in `avia/driver`.

## Point data and clocks

`/avia/points` is a standard `sensor_msgs/msg/PointCloud2`, best effort, volatile, depth 2, approximately 10 Hz. Clouds contain the nonzero returns received during each publish interval. Livox's `(0,0,0)` no-return sentinel is removed; reflectivity and tag are preserved without confidence filtering. There is no voxelization or motion compensation. The SDK is configured for Cartesian single returns, and this adapter intentionally rejects unexpected packet versions, formats, or counts.

### Header time

With `header_stamp: sensor_utc`, the default, a cloud's header carries the Avia's own sample time of its first packet, converted to UTC, when that packet passes every check:

- the status word shows a live source: PTP lock (bit 13) with sync mode 1, or PPS (bit 9) with GPS sync mode 2;
- the stamp type matches that source: 1 for PTP, 3 for PPS plus UTC;
- receipt time minus sample time is between `sensor_latency_min_sec` and `sensor_latency_max_sec`, by default -1 and +50 ms. A label a whole second off, or a TAI/UTC mix-up, fails this bound.

Otherwise the header is the first packet's host receipt time, and `avia/clock` says why. `header_stamp: host_receipt` always uses receipt time.

### Sync sources

Livox gives PTP the highest priority, then GPS (PPS plus UTC), then PPS alone.

- **PPS plus UTC.** The receiver's PPS reaches the Avia's RS-485 Sync pair: M12 pin 12 is Sync+ (RS485_A) and pin 11 is Sync-; the manual's description for pin 11 is a copy error. The manual asks for a 20-200 ms high time; the G5 sends its 5 ms default (measured 0.50 % duty on the Jetson's PPS GPIO on 2026-10-04) and the Avia accepted it: GPS sync mode 2 on that pulse. The Avia only re-selects its sync source at power-up: with the PTP master stopped underneath it, it sat in mode 4 ("time sync abnormal") with `pps_status` 1 until it was power-cycled, after which it came up in mode 2. The UTC time of each pulse must arrive 10-500 ms after its rising edge. A thread wakes 100 ms after each UTC second (`utc_sync_phase_sec`) and sends that second, but only while the kernel reports the clock synchronized within `utc_sync_max_clock_error_sec` (50 ms), so a label can never land on the wrong pulse. The SDK queues commands through its I/O loop, which adds 1-50 ms; sends measured 101-150 ms after the second on the wire. The push is not gated on the Avia's own PPS flag: it is harmless without a pulse, and the status word shows what the Avia makes of it.
- **PTP (off since 2026-10-04, `avia_ptp_enabled: false`; it masked the PPS path because Livox ranks PTP first).** When enabled, the `time_sync` role runs a master on the drone LAN: ptpd, software time stamps, 8 Hz. The Avia locks only to a master announcing the PTP timescale. ptp4l on the system clock announces an arbitrary one and never got a lock in 20 minutes; ptpd got one in 20 s. Its stamps are therefore TAI, and `ptp_utc_offset_sec` (37) converts them. Measured after lock: receipt minus corrected sample time 0.66-1.58 ms, median 1.09 ms, against a 0.4 ms packet span.

`avia/clock` reports the policy, the source in use, `timing_validated`, the stamp type, sync mode, PPS and PTP flags, UTC command counts (sent, acknowledged, rejected, skipped and the last skip reason), the host clock's error bound, and the receipt-latency minimum, mean and maximum for the last second. Changes in the PPS flag, PTP lock, sync mode and header source are logged once each.

Every point has a 36-byte record:

| Field | Type | Meaning |
| --- | --- | --- |
| x, y, z | float32 | Livox coordinates converted from millimetres to metres |
| intensity | float32 | Unscaled Livox reflectivity byte (0–255) |
| tag | uint8 | Unmodified Livox return/noise tag |
| line | uint8 | Point index modulo six, following the official Avia driver |
| timestamp_type | uint8 | Original Livox packet timestamp type |
| sensor_time_low, sensor_time_high | uint32 | Exact original eight timestamp bytes, interpreted as a little-endian 64-bit value |
| point_offset_ns | uint32 | Index within packet × 4167 ns, following the official Avia driver |
| status_code | uint32 | Original packet status word |

Reconstruct `packet_time_raw = low | (uint64(high) << 32)`. For timestamp type 0 this is sensor uptime in nanoseconds; type 1 is the PTP clock value. For type 3 the bytes encode Livox's UTC calendar format and must be decoded accordingly; do **not** add offsets to the opaque integer as if it were epoch nanoseconds. The raw value and offset are retained separately to avoid silently converting clock domains. Point timestamps are not tied to the ROS header by a measured offset.

The ROS timer uses a steady clock. The buffer is bounded to 100,000 points; an executor backlog older than 0.5 s is discarded. No stale buffered cloud is emitted after the 2 s receipt timeout. Packet timestamp regressions and clock-type changes clear the aggregation buffer. Diagnostics report packet gaps (a conservative >600 µs interval for type 0/1), regressions, unsupported formats, buffer drops, receipt age, and output rate. A packet-gap count is an observation, not a precise loss estimator.

### LiDAR-inertial outputs

For FAST-LIO (the `lio` role) the driver also publishes:

- `/avia/imu`: the built-in IMU at 200 Hz (`sensor_msgs/Imu`, reliable). The SDK
  reports acceleration in g; the topic carries m/s². Frame `avia_imu`
  (`avia_imu_frame_id`): LiDAR axes, origin at (−41.65, −23.26, +28.40) mm in the
  LiDAR frame (Livox's factory offset). Stamped like the clouds: sensor UTC when
  the packet passes the same checks, host receipt otherwise. No orientation
  (`orientation_covariance[0] = -1`), covariances unknown (0).
- `/avia/custom`: Livox `CustomMsg` (reliable). `timebase` and the header stamp
  are the frame's first point in sensor UTC; each point's `offset_time` is its
  packet's stamp plus its slot (4167 ns) minus that. Published only for frames
  whose first packet carried trusted sensor time, so the LiDAR and the IMU are
  never on different clocks, and only while something subscribes. Points whose
  time cannot be decoded or falls outside the frame are dropped and counted.

`livox_ros_driver2` in this role is message definitions only (wire-compatible
with Livox's driver, which FAST-LIO depends on); Livox's driver and SDK2 are not
used. Measured 2026-10-02: IMU 203.8 Hz, receipt latency median 1.8 ms (p99
4.8 ms), |a| 9.74 m/s² at rest; frames 10.0 Hz, ~16,000 points indoors, offsets
0–99.6 ms. `avia/driver` reports `imu_published_sensor_time`,
`imu_published_receipt_time`, `lio_frames_published` and `lio_points_dropped`.

## Frames and visualization

`avia_nominal_lidar_frame` is added by the description role beneath `avia_link`, which originally represents the mounting-hole centroid. The provisional transform is XYZ `(0.0525, 0, 0.0324)` m, RPY `(0,0,0)`: front housing plane from the CAD mesh, scan-origin height 32.4 mm from the Avia manual. Axes are X forward, Y left, Z up. This is a mechanical estimate pending extrinsic calibration. Override `robot_description_avia_lidar_xyz/rpy` through Ansible; don't add a competing TF publisher.

In Foxglove, enable `/avia/points` in the 3D panel. Color by `intensity`; a 0.5-second decay gives the nonrepetitive scan time to fill in. The saved bench layout includes this selection. `avia/driver` describes reception and device status; `avia/clock` remains WARN for unvalidated timing. Disconnected E1R/GNSS hardware does not block Avia startup.

## Deployment and checks

From the canonical repository, synchronize source and run the normal playbook serially. Initial installation includes `ros_runtime,robot_description,avia,ros_health,foxglove`; subsequent driver-only edits use `--tags avia`. Do not run simultaneous colcon builds in `/opt/uav/ros`.

On Jetson:

```sh
sudo systemctl restart uav-avia
systemctl status uav-avia
journalctl -u uav-avia -f
```

The build runs a binary point-contract test covering units, no-return filtering, exact 64-bit timestamp preservation, offsets, tags, line numbers and opaque GPS-format bytes. Runtime verification must also check live ROS reception as `uav-ros`, Foxglove delivery, TF connectivity, and supervised restart recovery. Boot enablement is configuration evidence; a full aircraft power-cycle is a separate check.

Primary references: [SDK1](https://github.com/Livox-SDK/Livox-SDK/tree/v2.3.1), [original ROS driver's point timing table](https://github.com/Livox-SDK/livox_ros2_driver/blob/1565b976ab1905f911cd2a757fc32433422b701c/livox_ros2_driver/livox_ros2_driver/lds.h), [Avia manual](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Download/Avia/Livox%20Avia%20User%20Manual%20202204.pdf).
