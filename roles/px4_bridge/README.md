# PX4 bridge (uORB → ROS-native, ENU/FLU)

`uav-px4-bridge.service` subscribes to PX4 1.17's `/fmu/out/*` (px4_msgs over
XRCE-DDS) and republishes what the ROS side needs, converted from PX4's
NED/FRD conventions to ROS's ENU/FLU (REP-103), stamped with PX4's
XRCE-synchronised epoch timestamps (boot-time stamps are detected and
reported). No GNSS data from PX4 is republished: the Jetson's own receiver
(`gnss_ros`) has the full covariances.

| uORB topic | ROS topic | Type | Notes |
| --- | --- | --- | --- |
| `sensor_combined` | `/px4/imu/data_raw` | `sensor_msgs/Imu` | gyro + accel, FLU; orientation unknown |
| `sensor_combined` + `vehicle_attitude` | `/px4/imu/data` | `sensor_msgs/Imu` | + orientation FLU→ENU (yaw 0 = east, true north); orientation covariance from `vehicle_odometry` |
| `vehicle_odometry` | `/px4/odometry` | `nav_msgs/Odometry` | EKF2 output, pose ENU in `px4_local` (PX4's own origin; no TF published), twist in `base_link`; **for comparison only, not fused** |
| `vehicle_local_position` | `/px4/local_position/pose`, `/twist` (frame `px4_local`), `/origin` | Pose/TwistWithCovarianceStamped, NavSatFix | invalid axes are NaN; origin = PX4's local-frame reference point |
| `battery_status` | `/px4/battery` | `sensor_msgs/BatteryState` | current negative when discharging; cells, warning |
| `vehicle_land_detected` | `/px4/landed` | `std_msgs/Bool` | |
| `home_position` | `/px4/home` | `geographic_msgs/GeoPointStamped` | when valid |
| `vehicle_status`, `estimator_status_flags`, `timesync_status` | `/diagnostics` (`px4/bridge`) | | arming, mode, failsafe, EKF2 fusion flags, rates, timesync RTT |

Frame conversion follows px4_ros_com's `frame_transforms`
(`q_ros = NED_ENU_Q · q · AIRCRAFT_BASELINK_Q`), implemented as the matrix
product `R_ned_enu · R(q) · R_frd_flu` in `frames.py` and pinned by tests that
recompute px4_ros_com's quaternion composition. Bias-corrected
`vehicle_angular_velocity` / `vehicle_acceleration` are not in PX4's default
DDS topic list; `sensor_combined` (raw, averaged) is used until the firmware's
`dds_topics.yaml` adds them. `/px4/imu/data` covariances for rates and
acceleration are configured constants (`px4_bridge_gyro_stddev_rad_s`,
`px4_bridge_accel_stddev_m_s2`).
