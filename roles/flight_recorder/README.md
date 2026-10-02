# Flight recorder (rosbag2, mcap)

`uav-flight-recorder.service` runs `uav_flight_recorder`, which starts a
`ros2 bag record` the moment the vehicle arms, keeps it rolling through
disarm, and stops it after a post-roll:

| Event | Action |
| --- | --- |
| autopilot armed (or a manual request: `/flight_recorder/manual` true, or the transmitter switch on) | start `ros2 bag record -s mcap --storage-preset-profile zstd_fast -a --exclude-regex ... -o <dir>/flight_<UTC>` |
| disarm | keep recording for `flight_recorder_post_roll_sec` (30 s) |
| safety engaged after disarm (`ros` source only) | stop `flight_recorder_safed_grace_sec` (5 s) later |
| re-arm during post-roll | the same bag continues |
| manual request withdrawn, never armed | stop immediately |
| free space under `flight_recorder_min_free_gb` (50 GB) | refuse to start; stop a running bag |

## Armed state

By default (`flight_recorder_arm_source: mavlink`) the armed flag is bit 7
(`MAV_MODE_FLAG_SAFETY_ARMED`) of the autopilot's HEARTBEAT `base_mode`, read
from mavlink-router's TCP server (`flight_recorder_mavlink_endpoint`), so it
arrives over the TELEM2 serial link. Only system 1, component 1 counts; the
camera component's heartbeat cannot stand in for it. The connection is
read-only and reconnects every 2 s. Losing the link keeps the last known
state, so a dropout mid-flight never closes the bag early; a disarm seen
after the link returns starts the post-roll. PX4 sends HEARTBEAT at 1 Hz, so
a bag starts up to a second after arming, plus rosbag2's own start-up: the
first message lands about 1.1 s after `ros2 bag record` starts (DDS discovery
and subscriptions; measured on three switch-requested bags, 2026-10-02). The
first ~2 s after arming are therefore not in the bag; the PX4 ulog has them. pymavlink is pinned in
`flight_recorder_venv`, whose python runs the node with the system ROS on
its path.

Why not `/px4/armed` (XRCE-DDS over the FC's Ethernet): on 2026-10-02 the
Ethernet stayed down for whole boots while TELEM2 kept working, with the FMU
module marginally seated on the baseboard. The module's whole Ethernet
interface (RMII, plus the baseboard PHY's management bus and power enable) is
on the second of its two board-to-board connectors (PAB X2), TELEM2 and the
module's power on the first (X1), so a partly mated X2 takes out only the
Ethernet. `flight_recorder_arm_source:
ros` restores the topic source, with `/px4/armed` and `/px4/safety_off`
(latched booleans from the PX4 bridge). Only that source can stop early on a
safed vehicle, and on this FC the safety switch is bypassed anyway
(`CBRK_IO_SAFETY` 22027, the PX4 default), so in practice the post-roll ends
every bag either way.

## Transmitter switch

`flight_recorder_rc_channel` (11) requests a bag like the manual topic, and
the two are combined (either one requests). In `switch` mode
(`flight_recorder_rc_mode`, the default) it records while the switch is on;
in `button` mode each press starts or stops. It is read from the autopilot's
RC_CHANNELS on the same read-only router link and counts only while
SYS_STATUS lists the RC receiver as present, after a two-frame debounce.
While the receiver is absent the last request holds, so an RC loss never ends
a bag. The policy rules are unchanged: a request withdrawn without arming
closes the bag at once, and once the vehicle has armed the bag runs to the
post-roll whatever the switch says. A switch left on records from boot. This
is the way to record calibration passes with the vehicle disarmed, without a
laptop; flip it a couple of seconds before the pass, since capture begins about
1.1 s after the flip. RC_CHANNELS forwards SBUS failsafe positions unmarked, so set this
channel to hold in the handheld's failsafe settings.

## Output

Each bag directory gets a `flight.json` (start and stop times and reasons,
duration, bytes); `<bag>.log` beside it holds rosbag2's own output. A
"Flight Recorder" row on `/diagnostics` shows the state, the open bag's size,
free space, the last flight and the arm source; with the MAVLink source it
also reports the link, the heartbeat count and its age, and goes WARN after
`flight_recorder_mavlink_stale_sec` (3 s) without a heartbeat, since an
arming would then go unseen.

Everything is recorded except `^/realsense/`: the D555 unicasts a copy of every
raw stream per subscriber, so a second subscriber on the raw colour or depth
would starve the encoder's link. The bag takes `/d555/color/video` (H.265,
about 0.7 MB/s) instead (the throttled depth is off since 2026-10-02). With both
LiDARs that was roughly 14 MB/s, 50 GB per flight hour. Since 2026-10-02 bags
also hold the odometry inputs and outputs: `/avia/custom` (about 20 bytes a
point, ~3 MB/s), `/avia/imu`, `/Odometry`, `/lio/odometry` and
`/vslam/odometry`, so FAST-LIO can be re-run offline from a bag. Its derived
clouds (`/cloud_registered` at ~11 MB/s, the registered E1R at ~4.4 MB/s,
`/lio/map` and the like) are excluded
(`flight_recorder_exclude_regex` in `group_vars`). PX4's own ulog on the SD card remains the primary flight log; the
bag is the companion-side record in the same UTC time base.

Bench test without arming: flip the channel-11 switch on, wait, then off; or
`ros2 topic pub --once /flight_recorder/manual std_msgs/msg/Bool "{data: true}"`,
wait, then publish false. The bag closes at once because the vehicle never
armed.

Pre-roll (seconds before arming) is not recorded; rosbag2's snapshot mode could
add it later at the cost of a second, always-on recorder.
