# Grouped sensor health

`uav-ros-health.service` publishes one `DiagnosticStatus` per sensor at 1 Hz on
`/uav/health`, with a copy on `/diagnostics`. In Foxglove choose `/uav/health` in
both Diagnostics Summary and Detail panels. Click a sensor to see its connection,
data, timing, and driver evidence grouped by key prefix. The supplied X950 layout
selects this topic; existing imported layouts need this one-time topic change.
Raw driver diagnostics remain on `/diagnostics` for engineering detail.

Severity policy:

- **OK:** observed checks are healthy.
- **WARN:** connected but degraded, timing unverified/faulty, not integrated,
  or monitoring evidence insufficient to establish connection.
- **ERROR:** an observed sensor data connection has been lost, the live driver
  reports disconnection, or the PX4 heartbeat is absent after startup grace.
  This describes the monitored connection, not a diagnosis of power or wiring.
- A stopped health node becomes stale in Foxglove after five seconds.

Timing faults never promote a connected sensor to ERROR. This does not relax
E1R's strict timestamp gates or certify data for flight/fusion.

| Entry | Live evidence |
|---|---|
| D555 | Native RGB and depth receipt age/rate, image layout/frame, timestamp progression and clock domain |
| Avia | ROS cloud receipt, nonempty/layout/frame, SDK connection state, packet rates and timestamp policy |
| E1R | ROS clouds, MSOP/DIFOP packet freshness, gPTP status, verified UTC offset and host PHC evidence |
| H-FLOW | Configured PX4 optical-flow instance and downward distance instance, measurement progression, flow quality and range validity |
| GNSS | CRC-valid SBF via the read-only broker, fresh PVT solution, chrony's selected PPS source and clock error |
| PX4 / MAVLink | Heartbeats specifically from system 1/component 1 via the router; camera heartbeats do not count |
| PX4 / DDS | Actual periodic uORB sample receipt, rates, timestamp progress and timesync evidence |
| Hadron | Explicit not-integrated/unknown until a thermal health source is configured |

H-FLOW's default association is PX4 flow instance 0 and downward (orientation 25)
range instance 0, verified during bench bringup. MAVLink does not establish the
CAN vendor identity; update the configured IDs if additional flow/range sensors
are added. The monitor requests a current flow/range message alternately once per
second (0.5 Hz each). This uses `MAV_CMD_REQUEST_MESSAGE` only: it does not alter
stream rates, PX4 parameters, modes, arming, or actuator state. It connects to the
existing router's local TCP endpoint, never directly to PX4's UDP peer port.
Loss of this telemetry path makes H-FLOW unknown, not proven disconnected.

The GNSS observer never opens the serial receiver. It reads the broker at
127.0.0.1:28785 and checks SBF CRCs. Position validity and PPS lock are independent.
PHC evidence must be current and from this boot. Monitoring uses monotonic receipt
time; wall-clock proximity alone never proves sensor synchronization. Image and
cloud payloads are not retained; geometric accuracy/image content are not certified.
DDS still deserializes the input messages. Network and chrony probes run in bounded
background workers, so they cannot block diagnostic publication.

Defaults: image timeout 2 s, minimum 15 Hz over 5 s, startup grace 10 s;
LiDAR/driver/GNSS evidence 3 s, H-FLOW measurements 4 s. PPS bench tolerance 10 ms
with a reference younger than 120 s. These are observability thresholds, not
flight-readiness requirements. Tune startup parameters in `/etc/uav/ros/health.yaml`.

Deployment is owned by this Ansible role, including an isolated, root-owned
`/opt/uav/health-venv` with pinned pymavlink and access to system ROS packages.
The process runs as `uav-ros`; no sudo or device access is used at runtime.

```sh
ansible-playbook -i jethawk, site.yml --tags ros_health -e target_user=john
```

Pure logic/protocol tests (ROS serialization skips without Jazzy):

```sh
cd roles/ros_health/files/uav_sensor_health
PYTHONPATH=. python3 -m unittest discover -s test -v
```

For target validation, source `/etc/uav/ros/environment`, Jazzy, `/opt/uav/px4_msgs/install/local_setup.bash` and the workspace,
then run the suite with the health venv as `uav-ros`. Verify actual `/uav/health`
messages and recovery, not just systemd's active state. Do not inject test data
into the live ROS domain or disconnect hardware to test diagnostics.

## Component labels and PX4 DDS

`/uav/health` contains human-readable names: D555 Camera, Avia LiDAR, E1R
LiDAR, H-FLOW Landing Sensor, GNSS Receiver, Hadron Thermal Camera,
PX4 / MAVLink, and PX4 / DDS. `hardware_id` is deliberately empty because
Foxglove prefixes it to every title. Technical identifiers remain under
`Identity/source_id` in the details.

The role builds the official `PX4/px4_msgs` release/1.17 at pinned commit
`86d8239e962f6939e05c3737784f60c02fa884db` in `/opt/uav/px4_msgs`.
The health service uses the shared ROS middleware configuration (Cyclone DDS).
PX4 1.17 requires the compatible XRCE Agent 2.4.3 pairing installed by
`px4_link`. Live Cyclone subscribers under the `uav-ros` service account
received advancing PX4 attitude, IMU, status, and timesync samples after this
pairing was deployed. The earlier Fast DDS discovery workaround is removed:
discovery alone did not establish sample delivery. Diagnostics report the
actual RMW implementation selected at runtime.

The DDS row measures receipt age, rate, and source timestamp progress for
status, attitude, local position, odometry, combined IMU, GPS, battery and
DDS timesync. No fresh monitored topics after startup grace is ERROR;
partial delivery, frozen timestamps, low rates and timing faults are WARN.
Event-only uORB topics are excluded from periodic health expectations.
This observer publishes no PX4 command or control topics. DDS timesync
residual/RTT limits are bench checks, not proof of sensor hardware sync.

## D555 streams are watched through CameraInfo, not Image

Since 2026-09-30 the observer subscribes to `<image topic>/camera_info` for the
colour and depth streams. The D555 sends every subscriber its own unicast copy
of each image (~200 Mbit/s per stream at 896×504, ~500 Mbit/s at 1280×800), and
the observer only needs the header: stamp, frame and size arrive in the
CameraInfo of the same frame at a few hundred bytes. Rate, freshness, size and
timestamp checks are unchanged; `encoding`, `row_step` and `image_bytes` are
not reported (`source = camera_info header`). The native CameraInfo carries the
known occasional frame-id mislabel between the colour and depth streams, which
shows up as a changing `frame_id` value, not as a failure.

## H-FLOW row comes from the CAN listener

Since 2026-09-30 the H-FLOW row is built from `/hflow/sensor_optical_flow` and
`/hflow/range` (the `hflow` role's listen-only DroneCAN observer) plus its
`hflow/driver` diagnostics, not from PX4's MAVLink OPTICAL_FLOW_RAD and
DISTANCE_SENSOR streams. A down CAN interface reported by the listener is a
disconnection (ERROR); a frozen or low-quality stream is a warning.
