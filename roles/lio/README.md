# LiDAR-inertial odometry (FAST-LIO2 on the Avia)

The `lio` role builds and runs [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
(HKU MARS) on the Livox Avia and the Avia's own IMU, plus a bridge that turns
its output into `base_link` odometry. Output: `/lio/odometry`
(`nav_msgs/Odometry`, `camera_init` → `base_link`, twist in `base_link`), FAST-LIO's
own `/Odometry` and `/cloud_registered`, and the `lio` diagnostic. Nothing fuses
it yet; it is the primary candidate for the Jetson's EKFs and for PX4 velocity
once validated in flight against RTK.

## Why FAST-LIO, and on the Avia

The Avia is FAST-LIO's reference sensor (its default config is `avia.yaml`).
LiDAR ranges directly, so it keeps metric geometry at mapping altitude where
stereo depth collapses; it works without light or texture; and FAST-LIO
undistorts every point with the IMU across each 100 ms scan. Its weak spot is
geometric degeneracy: over flat, featureless ground a scan is one plane and
the horizontal translation is unconstrained. Visual odometry (the `vslam` role)
covers that case near the ground, and the down-looking E1R would add geometry
once it has a calibrated extrinsic.

## Inputs (avia role)

- `/avia/imu`: the Avia's built-in IMU at 200 Hz, `sensor_msgs/Imu` in m/s²
  (the SDK reports g), frame `avia_imu`, stamped like the clouds (sensor UTC
  via PTP when trusted).
- `/avia/custom`: Livox `CustomMsg` (the `livox_ros_driver2` message package in
  the avia role, definitions only), timebase = the frame's first point, every
  point's `offset_time` from its packet stamp plus its slot (4167 ns). Published
  only for frames on trusted sensor time, and only when subscribed.

Both run on the Avia's PTP-locked clock, so `time_sync_en` stays false.
Measured: IMU 203.8 Hz, receipt latency median 1.8 ms (p99 4.8 ms); frames
10.0 Hz, ~16,000 points indoors, offsets 0–99.6 ms.

## Build

FAST-LIO is GPL-2.0, so it is fetched at build time, never vendored here: the
`ROS2` branch pinned at `a4743b0` (2025-01-15) with its ikd-Tree submodule, into
`/opt/uav/vendor/FAST_LIO`. Its CMakeLists forces C++14, but Jazzy's rclcpp
headers need C++17 (`std::is_convertible_v`, ...), so colcon builds a copy in
`/opt/uav/ros/src/FAST_LIO` with the standard switched; the checkout stays
pristine. On aarch64 its CMake disables the OpenMP nearest-neighbour search
(x86 only), so it runs single-threaded. FAST-LIO opens
`<source>/Log/pos_log.txt` unconditionally and `fclose`s it on exit, so `Log`
and `PCD` are writable by the service user (`ReadWritePaths`).

## Configuration choices (`/etc/uav/ros/fast_lio.yaml`)

| Setting | Value | Why |
| --- | --- | --- |
| `lidar_type`, `scan_line` | 1 (Livox), 6 | the Avia |
| `blind` | 1.5 m | FAST-LIO's avia.yaml drops everything within 4 m; 1.5 m keeps the near ground and still drops the airframe |
| `extrinsic_T`, `extrinsic_R` | (0.04165, 0.02326, −0.0284), identity | the Avia's factory LiDAR-to-IMU offset (FAST-LIO's avia.yaml) |
| `point_filter_num`, filters | 3, 0.5 m, 0.5 m | FAST-LIO's Avia defaults |
| Publishing | `/cloud_registered` (sparse), no path, no PCD | for the dashboard; map saving later |

FAST-LIO's frames are hard-coded (`camera_init`, its start frame, and `body`,
the IMU) and it always broadcasts `camera_init → body` on `/tf`. On its own that
is a separate tree (`base_link` can only have one parent, the EKF's), so the
bridge anchors `camera_init` under `odom` with a static transform, computed when
the first pose arrives and again after any reset: FAST-LIO's `base_link` pose
then coincides with the EKF's at that instant, and its path and
`/cloud_registered` render in the aircraft's tree. Checked 2026-10-02: one TF
root (`map`), and `base_link → body` through the two estimators within 1 cm of
the nominal lever arm.

## The bridge (`/usr/local/lib/uav/lio_bridge.py`)

FAST-LIO publishes the pose of the IMU ("body") with no twist. The bridge moves
every pose to `base_link` through the IMU's lever arm (`lio_imu_lever_arm`:
base_link → `avia_nominal_lidar_frame` from the URDF, (0.262, −0.003, 0.110) m,
level, plus the IMU's position in the LiDAR frame), so differencing `base_link`
positions accounts for rotation about `base_link`. The twist is a central
difference of three consecutive poses in the middle pose's body frame (one
frame, 100 ms, of delay), with a diagonal 5 cm/s floor. Both the URDF mount and
the axes are nominal until extrinsic calibration.

## The map (`/lio/map`)

FAST-LIO's own `/Laser_map` cannot serve: its 1 Hz timer appends only the
current scan to an unfiltered, ever-growing cloud and republishes all of it
every second, and its downsampled ikd-tree map is compiled out (`if(0)`).
`uav-lio-map.service` (`/usr/local/lib/uav/lio_map.py`) instead keeps one point
per voxel from every registered scan (`/cloud_registered`, with
`dense_publish_en` on: full undistorted scans, ~11 MB/s on the local DDS), in
`camera_init` (anchored under `odom`), at two resolutions:

- **fine**, `lio_map_voxel_m` (5 cm): only the newly filled voxels go out, every
  `lio_map_update_period_s` (1 s), on `/lio/map/updates`, so the bandwidth
  follows new surface, not map size (next to nothing standing still). A viewer
  accumulates them: the bench layout gives the topic a one-day decay. When a
  subscriber appears, the whole fine map goes out once first, so a viewer that
  connects late still gets the detail; `/lio/map/resend` repeats that for a
  second Foxglove client (the bridge shares one ROS subscription, so only the
  first is noticed).
- **overview**, `lio_map_overview_voxel_m` (20 cm): the whole map, latched on
  `/lio/map`, every `lio_map_publish_period_s` (10 s) when it changed, so a
  viewer that connects late still sees everything at once (and the fine
  detail from then on). 16 bytes a point.

The first version (2026-10-02) published only a 20 cm map: about 1,500 points
for the bench room against the raw Avia's ~16,000 per frame at ~2 cm spacing,
which is why it looked sparse next to `/avia/points`. 5 cm is ~16× the points
on surfaces; the Avia's range precision is about 2 cm. The fine map stops
growing at `lio_map_max_points` (4 M; 64 MB of points plus ~70 bytes a voxel
of index) and the `lio/map` diagnostic says so.

- Foxglove: re-import the bench layout (or add `/lio/map` and
  `/lio/map/updates`, the latter with a long decay), coloured by height.
- `ros2 service call /lio/map/save std_srvs/srv/Trigger` writes the fine map as
  a binary PCD (x, y, z, intensity) to `/data/maps/lio_map_<UTC>.pcd`, in
  `camera_init` coordinates; `/lio/map/reset` clears both maps. A gap of more
  than 10 s in the scans (FAST-LIO restarted, new origin) clears them too.
- The flight recorder excludes the derived clouds (`/cloud_registered`,
  `/lio/map`, ...); bags keep `/avia/custom` and `/avia/imu`, from which FAST-LIO
  re-creates them.

## Measured on the bench (2026-10-02, indoors, vehicle still)

| Quantity | Value |
| --- | --- |
| `/Odometry` | 10.0 Hz, 17 ms after the scan's end (max 30 ms) |
| `/lio/odometry` | 10.0 Hz, 119 ms after the stamped (middle) pose |
| Drift | 2.8 cm (2.5 cm of it in z) and 0.05° in 60 s |
| Velocity noise | ±1.5 / 2.0 / 3.4 cm/s (x / y / z) |
| Load | FAST-LIO ~10 % of one core, 156 MB; the Avia driver ~13 % with the CustomMsg |

Indoors at close range the Avia is noisier than the D555; outdoors at altitude
the comparison should reverse. Next: fly both in shadow against RTK, then
register the E1R into the same map with these poses, calibrate the E1R against
that map (HKU MARS's mlcc targets this non-overlapping case), and only then
consider feeding the E1R into the odometry.
