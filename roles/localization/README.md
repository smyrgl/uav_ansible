# Localization (robot_localization, REP-105 dual EKF)

Frames: `utm → map → odom → base_link → sensor frames` (the URDF provides
everything below `base_link`).

- `uav-ekf-odom.service`: `ekf_odom`, `world_frame: odom`, publishes
  `odom → base_link` and `/odometry/filtered/odom`. Continuous sources only,
  so it never jumps. Today: `/px4/imu/data` orientation and angular velocity.
  Add visual, LiDAR or flow odometry here once those sensors are calibrated.
- `uav-ekf-map.service`: `ekf_map`, `world_frame: map`, the same continuous
  sources plus `/odometry/gps`; publishes `map → odom` (allowed to jump) and
  `/odometry/filtered/map`.
- `uav-navsat.service`: `navsat_transform_node` fed by `/gnss/navsatfix`,
  `/px4/imu/data` and `/odometry/filtered/map`; publishes `/odometry/gps` in
  `map`, `/gps/filtered`, and broadcasts `utm → map`.

The datum (map origin and yaw) is the first fix unless `localization_datum`
is set to `[lat_deg, lon_deg, yaw_rad]`, which pins `map` to a physical spot
across sessions. `magnetic_declination_radians` and `yaw_offset` are 0
because PX4's attitude is true-north referenced and the bridge emits ENU.

PX4's own estimate, `/px4/odometry`, is in PX4's `px4_local` frame (EKF2's
origin, reset on estimator resets), is not fused (an EKF of an EKF) and carries
no TF. Both `px4_local` and `map` are ENU and true-north aligned, so
`map → px4_local` is a translation between two geodetic origins
(`/px4/local_position/origin` vs the map datum); a small node can publish it
to overlay PX4's estimate on ours in Foxglove.

Linear acceleration is not fused in either EKF yet: PX4's `sensor_combined` is
raw, not bias-corrected, and without a position source its integration drifted
250 m in a minute on the bench.

Outdoors: watch `/gnss/navsatfix` get a fix, `/odometry/gps` start,
`map → odom` jump onto it and `/odometry/filtered/map` follow, with `map` as
the fixed frame in Foxglove.
