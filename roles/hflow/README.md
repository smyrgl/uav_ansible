# H-Flow observer (DroneCAN → ROS 2)

The Holybro H-Flow (PixArt PAA3905E1 optical flow, Broadcom AFBR-S50LV85D
time-of-flight rangefinder, DroneCAN) hangs on the flight controller's CAN2
bus, and the Holybro baseboard wires the Jetson's `can0` (mttcan) onto that
same bus. PX4 (node 10) consumes the sensor for EKF2; this role lets ROS 2 see
the same frames directly, with host timestamps and without going through the
autopilot.

## Bus safety: listen-only

`/etc/systemd/network/70-uav-can0.network` brings `can0` up at boot at
1 Mbit/s (`UAVCAN_BITRATE`) in **listen-only** mode. The controller never
transmits, not even the ACK bit, so a Jetson fault cannot disturb the bus PX4
relies on; PX4 and the H-Flow acknowledge each other. The DroneCAN node in the
observer is anonymous (no node ID) and only decodes.

Measured on the bench (2026-09-30): node 124 sends
`com.hex.equipment.flow.Measurement` (20200) at ~72 Hz and
`uavcan.equipment.range_sensor.Measurement` (1050) at ~50 Hz, plus NodeStatus;
node 10 (PX4) sends ESC commands, lights, safety state and GlobalTimeSync. The
H-Flow's own timestamps are zero (it does not sync), so every topic carries
host receipt time.

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/hflow/sensor_optical_flow` | `px4_msgs/SensorOpticalFlow` | Exactly what PX4 builds from this message: `pixel_flow` and `delta_angle` in the sensor's FRD axes (`hflow_nominal_frd_frame`), `integration_timespan_us`, `quality`, latest fresh distance |
| `/hflow/distance_sensor` | `px4_msgs/DistanceSensor` | Laser type, downward facing, clipped readings carry the sensor limit with quality 0 |
| `/hflow/range` | `sensor_msgs/Range` | ROS-native companion (`+x` along the beam in `hflow_nominal_range_frame`; −inf/+inf/NaN for too close/too far/unknown), rendered by Foxglove |
| `/diagnostics` | `hflow/driver` | CAN state, rates, quality, node status |

No mavros: PX4's own message set (`/opt/uav/px4_msgs`, the same overlay the
health node uses) keeps the Jetson topics comparable with `/fmu/out`, and
consumers convert frames with TF or `px4_ros_com` `frame_transforms`.

## Frames

The URDF (`x950_description`) carries `hflow_link` at the mount datum plus
nominal `hflow_nominal_frd_frame`, `hflow_nominal_flow_optical_frame` and
`hflow_nominal_range_frame`, with FOV guides. PX4 has `SENS_FLOW_ROT = 0`
(connector aft, arrow forward), which the model follows.

## Runtime

`uav-hflow.service`, user `uav-ros`, Python venv `/opt/uav/hflow-venv`
(pinned `dronecan`), package built into the shared `/opt/uav/ros` workspace.
Pure mapping tests run on the target at every converge.
