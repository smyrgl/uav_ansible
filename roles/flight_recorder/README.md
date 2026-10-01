# Flight recorder (rosbag2, mcap)

`uav-flight-recorder.service` runs `uav_flight_recorder`, which starts a
`ros2 bag record` the moment the vehicle arms, keeps it rolling through
disarm, and stops it once the vehicle is safed:

| Event | Action |
| --- | --- |
| `/px4/armed` true (or `/flight_recorder/manual` true) | start `ros2 bag record -s mcap --storage-preset-profile zstd_fast -a --exclude-regex ... -o <dir>/flight_<UTC>` |
| disarm | keep recording for `flight_recorder_post_roll_sec` (30 s) |
| safety engaged after disarm | stop `flight_recorder_safed_grace_sec` (5 s) later |
| re-arm during post-roll | the same bag continues |
| manual request withdrawn, never armed | stop immediately |
| free space under `flight_recorder_min_free_gb` (50 GB) | refuse to start; stop a running bag |

`/px4/armed` and `/px4/safety_off` are latched booleans from the PX4 bridge.
Each bag directory gets a `flight.json` (start and stop times and reasons,
duration, bytes); `<bag>.log` beside it holds rosbag2's own output. A
"Flight Recorder" row on `/diagnostics` shows the state, the open bag's size,
free space and the last flight.

Everything is recorded except `^/realsense/`: the D555 unicasts a copy of every
raw stream per subscriber, so a second subscriber on the raw colour or depth
would starve the encoder's link. The bag takes `/d555/color/video` (H.265,
about 0.7 MB/s) and `/d555/depth/throttled` (about 1.8 MB/s) instead. With both
LiDARs that is roughly 14 MB/s, 50 GB per flight hour, about 34 hours on the
1.7 TB free. PX4's own ulog on the SD card remains the primary flight log; the
bag is the companion-side record in the same UTC time base.

Bench test without arming: `ros2 topic pub --once /flight_recorder/manual
std_msgs/msg/Bool "{data: true}"`, wait, then publish false; the bag closes at
once because the vehicle never armed.

Pre-roll (seconds before arming) is not recorded; rosbag2's snapshot mode could
add it later at the cost of a second, always-on recorder.
