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
`gnss_aux_link`, `base_link`, `odom`. Stamps are host receipt time
(`use_gnss_time: false`); in the field chrony disciplines the host from the
same receiver's PPS, so the two agree. The moving-baseline heading is the
direction of the (lateral) aux–main baseline as the receiver reports it; the
attitude offset is a receiver setting, not applied by the driver here.

`robot_localization` consumes `navsatfix`; `gpsfix` is for dashboards.
