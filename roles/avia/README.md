# Original Livox Avia bench driver

The `avia` role builds a small ROS 2 Jazzy C++ adapter around the unmodified official Livox SDK1 v2.3.1 source, pinned at `14c533dd7175bd90a6b568c0aa1733f35d36cb89`. It does not depend on PCL or the older upstream ROS wrapper. A vendor-target-only `-Wno-c++20-compat` setting accommodates bundled fmt's legal C++11 `char8_t` identifier on GCC 13; it does not turn off other build errors.

`uav-avia.service` is a boot-enabled system service running as `uav-ros`, supervised with `Restart=on-failure`, and grouped under `uav-ros.target`. It uses the common ROS environment and `/etc/uav/ros/avia.yaml`. SDK discovery only accepts original Avia type 7 with the configured broadcast code AND address. The SDK owns connection, heartbeat, and rediscovery. Setup requires acknowledged Cartesian, single first-return, IMU-200-Hz, and start commands; failure restarts the service. No IP, firmware, GPS, PPS, or extrinsic settings are written by the service.

## Point data and clocks

`/avia/points` is a standard `sensor_msgs/msg/PointCloud2`, best effort, volatile, depth 2, approximately 10 Hz. Clouds contain the nonzero returns received during each publish interval. Livox's `(0,0,0)` no-return sentinel is removed; reflectivity and tag are preserved without confidence filtering. There is no voxelization or motion compensation. The SDK is configured for Cartesian single returns, and this adapter intentionally rejects unexpected packet versions, formats, or counts.

The ROS header is **the first retained packet's host receipt time**, not hardware capture time. It is suitable for this static bench display; it is not synchronized acquisition time. The `avia/clock` diagnostic always reports `fusion_ready=false`, including if PTP/PPS/GPS traffic appears later. Enabling a hardware-time policy requires explicit implementation and validation.

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

IMU packet receipt is counted for transport health. **No IMU ROS topic is published in this pass**; units, capture timestamps, covariances, and the IMU frame must be handled deliberately before fusion.

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
