# GNSS ROS driver (Septentrio, read-only)

`ros-jazzy-septentrio-gnss-driver` as `uav-gnss-ros.service`, a TCP client of
the SBF fan-out (`127.0.0.1:28785`, the `gnss` role). `configure_rx: false`:
the receiver is configured by its boot profile and the driver never sends it a
command; the fan-out is read-only anyway. Topics under `/gnss`:

| Topic | Type | Source blocks |
| --- | --- | --- |
| `navsatfix` | `sensor_msgs/NavSatFix` (full 3×3 covariance) | PVTGeodetic + PosCovGeodetic |
| `gpsfix` | `gps_msgs/GPSFix` (DOPs, speed, track, heading) | PVTGeodetic, DOP, AttEuler, ChannelStatus |
| `pose` / `twist` | `PoseWithCovarianceStamped` / `TwistWithCovarianceStamped` (ENU) | + AttEuler, VelCovGeodetic |
| `atteuler`, `attcoveuler` | driver messages: dual-antenna heading/pitch | AttEuler, AttCovEuler |
| `gpst` | `sensor_msgs/TimeReference` | ReceiverTime |
| `aimplusstatus`, `galauthstatus`, `diagnostics` | RF interference, OSNMA, receiver status | RFStatus, GALAuthStatus, ReceiverStatus, QualityInd |

Frames follow the URDF: `gnss_main_link` (ARP of the main antenna),
`gnss_aux_link`, `base_link`, `odom`. Stamps are the receiver's own time
(`gnss_ros_use_gnss_time: true`): the 50 Hz PVT (the receiver's limit with
attitude on) then carries an exact 20 ms cadence instead of host-receipt
jitter, and chrony disciplines the host from the same receiver's PPS, so the
two time bases agree. The heading is the receiver's multi-antenna attitude:
the direction of the lateral Main-Aux1 baseline with the receiver's attitude
offset (90°) already applied, so it is the vehicle's heading. With
`use_ros_axis_orientation: true` the driver publishes it in REP-103 terms:
`/gnss/atteuler` carries ENU yaw, counter-clockwise from east, so yaw = 90° -
heading (a 99.4° heading reads 350.6°), and pitch changes sign.

`robot_localization` consumes `navsatfix`; `gpsfix` is for dashboards.
