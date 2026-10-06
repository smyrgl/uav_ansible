# LiDAR-inertial odometry (FAST-LIO2 on the Avia)

The `lio` role builds and runs [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
(HKU MARS) on the Livox Avia and the Avia's own IMU, plus a bridge that turns
its output into `base_link` odometry. Output: `/lio/odometry`
(`nav_msgs/Odometry`, `camera_init` → `base_link`, twist in `base_link`), FAST-LIO's
own `/Odometry` and `/cloud_registered`, the E1R registered into the same frame
(`/lio/registered/e1r`), a voxel map of both (`/lio/map`), and the `lio`
diagnostics. Nothing fuses it yet; it is the primary candidate for the Jetson's EKFs and for PX4 velocity
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
`/opt/uav/vendor/FAST_LIO`. colcon builds a copy in `/opt/uav/ros/src/FAST_LIO`
with three edits (`files/patch_fast_lio.py`, GPL-2.0 like what it patches; every
edit replaces exact text and a missing anchor stops the build); the checkout
stays pristine, and the copy records the script's fingerprint so a changed
script rebuilds it:

1. **C++17.** Its CMakeLists forces C++14; Jazzy's rclcpp headers need C++17
   (`std::is_convertible_v`, ...).
2. **A gravity-aligned world frame.** `IMU_init` takes gravity from the mean
   acceleration but leaves the attitude at identity, so `camera_init` was the
   IMU's attitude at start-up: 5.4° off on 2026-10-05 with the Avia level, and
   45° with the A-S+ cage. The initial attitude is now the rotation that takes
   the measured specific force to +z: `camera_init` is level, its x along the
   IMU's heading at start-up, gravity (0, 0, −g). What remains is the
   accelerometer bias of the first scans, a fraction of a degree.
3. **Exit when lost.** No scan matched to the map for `lio_lost_exit_s` (3 s)
   of LiDAR time, or faster than `lio_max_speed_mps` (30 m/s) for
   `lio_overspeed_s` (0.5 s): FAST-LIO prints "FAST-LIO lost: ..." and exits,
   and systemd restarts it (see *Divergence*).

On aarch64 its CMake disables the OpenMP nearest-neighbour search
(x86 only), so it runs single-threaded. FAST-LIO opens
`<source>/Log/pos_log.txt` unconditionally and `fclose`s it on exit, so `Log`
and `PCD` are writable by the service user (`ReadWritePaths`).

## Configuration choices (`/etc/uav/ros/fast_lio.yaml`)

| Setting | Value | Why |
| --- | --- | --- |
| `lidar_type`, `scan_line` | 1 (Livox), 6 | the Avia |
| `blind` | 1.0 m | the Avia returns nothing closer than 1.00 m (minimum range over twelve bags), and in the A-S+ cage no part of the airframe is in its view (integration report: 0 %), so the blind radius is the sensor's own. On the skids, 45° nose-down, the Avia sees flat ground only out to 2.8 m: 1.5 m left a sliver of it (the top 6° of 77°), 1.0 m twice that |
| `extrinsic_T`, `extrinsic_R` | from TF, `/run/uav-lio/fast_lio_extrinsic.yaml` | written before every start by `lio_extrinsic.py` from `avia_imu → avia_nominal_lidar_frame` (robot_description; Livox's factory offset, (0.04165, 0.02326, −0.0284), identity) |
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

## Geometry: from the URDF, nowhere else

No sensor pose is configured in this role. robot_description publishes the
Avia's ranging origin (`avia_nominal_lidar_frame`) and its built-in IMU
(`avia_imu`, Livox's factory offset) from `/etc/uav/ros/description.yaml`,
under the mount the URDF gives the Avia (45° nose-down in the A-S+ cage).
FAST-LIO's IMU-to-LiDAR extrinsic is written from TF before every start
(`lio_extrinsic.py`, `ExecStartPre`; the unit waits for `uav-description`),
and the bridge, the E1R registration and the map view look up
`base_link → avia_imu` in `/tf_static`. A re-mount or a calibration is a
change to the description; everything here follows at its next start. The
replay host reads the same frames from each bag's own `/tf_static`, so old bags
replay with the geometry they were flown with (bags from before
`avia_imu` get Livox's factory offset as that frame).

## The bridge (`/usr/local/lib/uav/lio_bridge.py`)

FAST-LIO publishes the pose of the IMU ("body") with no twist. The bridge moves
every pose to `base_link` through the IMU's full mount from `/tf_static`,
`T_w_base = T_w_imu T_base_imu^-1`: position and attitude. With the Avia
pitched 45°, the old axes-aligned lever arm would have reported the IMU's
attitude as base_link's and turned a level 10 m leg into a 45° climb. The twist
is a central difference of three consecutive `base_link` poses in the middle
pose's body frame (one frame, 100 ms, of delay), with a diagonal 5 cm/s floor.
Without the transform the bridge drops poses and its row says so. The mount is
nominal until extrinsic calibration.

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
  `/lio/map`, `/lio/registered/e1r`, ...); bags keep `/avia/custom`, `/avia/imu`
  and `/e1r/points`, from which they are re-created.
- The map takes every registered cloud in FAST-LIO's frame (`--scan-topics`);
  the `lio/map` diagnostic counts the fine voxels each one filled first
  (`points_from <topic>`). The 20 cm overview is fed only the fine map's new
  points: a coarse voxel seen for the first time always holds a fine voxel seen
  for the first time (they nest), so it ends up with the same voxels for a
  fraction of the work (back to every scan once the fine map is full).

### Stray returns

Indoors, about 0.3 % of the Avia's points (measured 2026-10-02) are strays: at
random ranges along real beam directions, out to ~430 m and from −160 to
+125 m in height, mean reflectivity 12.8 against 24.6 for real returns. The
Livox tag doesn't flag them; its noise bits are 0 on every one. Each lands in a
5 cm voxel of its own and never deduplicates. After ten minutes on the bench
they were 36 % of the fine map and 93 % of the overview: a dust over every
viewer, the saved PCDs, and any height-coloured range.

So a point joins the map only once its cell (0.5 m across, 0.25 m tall) holds
`lio_map_promote_voxels` (3) distinct fine voxels. Then the cell's held points
join together, and its later points go straight in. A surface fills its cells
within a scan or two; a stray never completes one. That holds for the fine map,
`/lio/map/updates`, the overview and `/lio/map/save` alike.

- The filter keeps only the voxels of cells still waiting. It caps them at
  100k cells (`--max-held-cells`) and gives up the oldest first: strays never
  complete, and a real cell completes within seconds. Promoted cells are kept
  in a sorted int64 index, 8 bytes a cell.
- The cost is a short delay. A cell's first voxels show once it holds three,
  usually in the same scan. A thin, sparse thing (a wire, a far branch) may
  never fill three voxels of one cell and stays out.
- The `lio/map` diagnostic shows it at work: `promoted_cells`, `held_cells`,
  `held_points` (mostly strays) and `held_points_dropped`.
- `lio_map_promote_voxels: 1` maps every point, as before.

## The E1R in the same map (`/lio/registered/e1r`)

The E1R looks straight down and never shares a field of view with the Avia, so
it cannot be matched against the Avia's scans; its points are placed with
FAST-LIO's trajectory and the E1R's mount instead. `uav-lio-e1r.service`
(`/usr/local/lib/uav/lio_register.py`):

- reads `/e1r/points`, whose every point carries its UTC firing time on the
  same PTP master as the Avia. Measured: frames start on the UTC 100 ms grid
  (x.000220 s), 27,648 points in 432 firing slots of 64 points, 216 µs apart,
  95.7 ms from first to last;
- interpolates the Avia IMU's pose at each firing time between FAST-LIO's
  10 Hz `/Odometry` poses (linear in position, slerp in attitude) and composes
  it with the mount, once per slot: base_link → `e1r_nominal_lidar_frame` from
  `/tf_static` (the URDF: (−0.084, 0, −0.079) m, pitched +90° so the boresight
  points down and the sensor's "up" points forward), expressed in the IMU's
  frame through `base_link → avia_imu` from the same tree
  (`T_imu_s = T_base_imu^-1 T_base_s`). Only the static tree is read: following
  `/tf` (~70 Hz of small messages) cost about 9 % of a core in Python;
- waits for the first pose after a frame's last point (median 19 ms; FAST-LIO
  publishes ~20 ms after its own scan ends), drops points between poses more
  than `lio_e1r_max_pose_gap_s` (0.25 s) apart (FAST-LIO stalled or restarted
  with a new origin; never extrapolated), and frames still waiting after
  `lio_e1r_max_wait_s` (0.5 s);
- drops points outside the range gate, `lio_e1r_min_range_m` (0.1 m): no
  returns (NaN since the e1r driver fix of 2026-10-02; before it they were
  points at the sensor's origin, which would have drawn the trajectory into the
  map) and the near field;
- publishes `/lio/registered/e1r` (`camera_init`, x y z intensity, stamped with
  the frame's first point) and the `lio/e1r` diagnostic: output rate, pose
  wait, latency, drop counters by cause, and the extrinsic in use.

The interpolation is exact for constant linear and angular velocity (tested:
4,000 points fired across 0.7 s at 16 m/s and 1.5 rad/s of yaw on a 10° roll,
through the nominal mounts (the IMU pitched 45°), land within 1 µm). What remains is curvature over the 100 ms between poses: an
angular acceleration α leaves up to α Δt²/8 of attitude error mid-interval,
0.6 mrad (6 mm at 10 m) at 0.5 rad/s², 6 mrad at an aggressive 5 rad/s².
Propagating the Avia IMU between poses would remove most of it; mapping flight
does not need it yet. A clock offset δ between the two LiDARs would add
v δ + ω × r δ, which is why both must stay on the PTP master.

Bench check (2026-10-02, vehicle standing on a bench): 10.0 Hz out, 44 ms from
a frame's last point to publication, 32 of 13.9 M points unbracketed (start-up
only), 8.3 % range-gated (6.9 % no returns, 1.4 % within 0.1 m). The E1R sees
the bench top 0.260 m below base_link, flat to 14.5 mm rms over 0.9 × 0.5 m;
the Avia, blind to 1.5 m, sees only the room floor 0.88 m below, so on the
bench the two sensors share no surface. The extrinsic check (and the input for
calibration) needs one: the vehicle standing on the floor (the E1R sees it
directly below, the Avia from 1.5 m out), or carried around so that the E1R
passes over floor the Avia has mapped. A height step or a tilt between the two
floors in the map is the E1R's extrinsic error relative to the Avia's (both
mounts are nominal).

Foxglove: the bench layout has `/lio/registered/e1r` hidden, flat magenta.
Drawn over the raw `/e1r/points` it differs by the drift between the EKF and
FAST-LIO (the raw cloud hangs off the EKF's `base_link`), not by the extrinsic;
compare it with the map instead.

## Divergence (`/lio/health`, `uav-lio-watchdog.service`)

FAST-LIO does not say when it loses track, and it does not come back by itself.
On 2026-10-02 the Avia on the bench faced an obstruction inside its blind
zone, and FAST-LIO got almost nothing to match from its first scan. It
propagated on the IMU alone and reached ALT −155 km. The bridge stopped
publishing once the speed passed 30 m/s. But the E1R registration went on
placing frames with the runaway poses and reported OK, and the map filled to
its 4 M cap with them (3.94 M of the points).

**Detection.** `lio_watchdog.py` follows two signals, through `lio_health.py`:
- **FAST-LIO's log** (the unit's journal, hence `SupplementaryGroups=
  systemd-journal`):
  - "No Effective Points!" comes once per filter iteration of a scan whose
    update found nothing to match;
  - "No point, skip this scan!" comes for a scan without usable points;
  - "FAST-LIO lost: ..." is its last line before it exits (build edit 3): a
    divergence at once.
- **Its odometry:** a speed above 30 m/s, or a leap of more than 5 m between
  consecutive poses.

A run of evidence without a gap over 0.5 s is DEGRADED. A run sustained for
`lio_watchdog_sustain_s` (2 s, about 20 failed scans) is DIVERGED. A
featureless moment, or the usual "No point" of the first scan, stays
DEGRADED. DIVERGED holds until FAST-LIO starts over: "IMU Initial Done" in
its log, or its stamps going back or skipping more than 5 s. That starts a
new epoch.

**The verdict** is on `/lio/health`: `std_msgs/String`, JSON, latched,
published on every change and every 2 s. It carries:
- the state, the epoch and the reason;
- the onset (the run's start less 1 s) and `since`, on the host's monotonic
  clock, which every process on the Jetson shares;
- a watchdog session id, so a restarted watchdog's epoch 0 isn't read as
  FAST-LIO restarting.

**On DIVERGED:**
- **lio_map** freezes: it refuses scans and takes back every point that
  arrived since the onset. It keeps a mark at every addition of the last
  10 minutes, then rebuilds the overview and the stray filter's cells from
  what is left. It bumps `/lio/map/epoch` and sends the whole (clean) map on
  `/lio/map/updates`.
- **lio_register** stops registering E1R frames (`frames_diverged`).
- **Both rows go ERROR.** The lidar view says "MAP FROZEN".

On a new epoch the map starts over and the registration forgets its poses: a
restarted FAST-LIO has a new origin, and a restart pauses the scans for a few
seconds, under the map's 10 s gap rule. The bridge's `/lio/odometry` is left
as it was.

**Recovery is a crash loop, on purpose.** FAST-LIO exits when it is lost
(build edit 3, 3 s without a matched scan: longer than the 2 s that make the
watchdog's DIVERGED, so the map is frozen first) and `uav-lio.service`
restarts it 2 s later (`Restart=always`, `StartLimitIntervalSec=0`), every
time, with nothing counting or giving up. An aircraft facing a wall restarts
every few seconds until it is moved, which costs a log line each time; an
aircraft that lost track in flight gets FAST-LIO back as soon as there is
something to match. Until 2026-10-06 the watchdog restarted it instead, through
a polkit rule, at most five times an hour, and then gave up for the hour: on
the pad with the 45° Avia that could have spent the budget before take-off.
The rule is removed on the next converge. FAST-LIO's IMU initialisation
assumes the aircraft is still, so a restart in flight starts from a level
frame estimated in motion; it is a shadow estimator here, nothing flies on it.

- The `lio/watchdog` row shows the state, the reason, how long it has been
  diverged, the epoch (FAST-LIO's restarts) and the log lines read. "Blind to
  FAST-LIO's log" means the journal can't be read.
- `ros2 topic echo /lio/health` shows the verdict itself.

## Measured on the bench (2026-10-02, indoors, vehicle still)

| Quantity | Value |
| --- | --- |
| `/Odometry` | 10.0 Hz, 17 ms after the scan's end (max 30 ms) |
| `/lio/odometry` | 10.0 Hz, 119 ms after the stamped (middle) pose |
| Drift | 2.8 cm (2.5 cm of it in z) and 0.05° in 60 s |
| Velocity noise | ±1.5 / 2.0 / 3.4 cm/s (x / y / z) |
| Load | FAST-LIO ~10 % of one core, 156 MB; the Avia driver ~13 % with the CustomMsg; bridge ~10 %; with the E1R: registration ~15 %, map ~11 % |

Indoors at close range the Avia is noisier than the D555; outdoors at altitude
the comparison should reverse. Next: fly both in shadow against RTK, compare
the E1R's floor with the Avia's (above), calibrate the E1R against the map
(HKU MARS's mlcc targets this non-overlapping case), and only then consider
feeding the E1R into the odometry.
