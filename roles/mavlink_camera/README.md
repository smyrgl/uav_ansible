# D555 MAVLink camera

The D555 publishes native ROS2 images over Ethernet. This role installs a Jazzy
subscriber, a Jetson NVENC encoder and RTSP server, and a MAVLink Camera Protocol
v2 component. QGroundControl discovers the component through the existing
mavlink-router TCP endpoint; no additional PX4 connection or router restart is
needed.

## Current bench configuration

- Camera: D555 serial 261622302751, Ethernet 192.168.144.70.
- ROS: domain 0, Cyclone DDS restricted to enP8p1s0, BEST_EFFORT/VOLATILE images.
- Input: `/realsense/D555_261622302751_Color`, 1280x800 YUY2 at 30 Hz (the RGB
  imager's native size; `d555_rgb_profile`, set at boot by `uav-d555-profile`).
- Output: H.265 Main 8-bit, 8 Mbit/s target, no B-frames, IDR every 30 frames.
- Image rotated 180 degrees in the presentation/capture branch. Original ROS
  topics and CameraInfo remain unchanged.
- RTSP: advertised as `rtsp://192.168.144.1:8554/rgb` (`mavlink_camera_rtsp_host`,
  the drone-LAN address since 2026-10-02; the server binds every interface), TCP
  or UDP; RTP packets limited to 1200 bytes. H.264 is available by changing
  `mavlink_camera_codec`. The name `jethawk` only ever resolved, through the
  house DNS, to the Jetson's Wi-Fi address; with the Wi-Fi gone the Siyi GCS,
  which has no resolver for it, showed no video until the address went in.
- MAVLink: system 1, camera component 100, router `127.0.0.1:5760`.
- QGC telemetry connection: TCP `jethawk:5760` (the router listens on every
  interface; on the Siyi link use `192.168.144.1`).
- Jetson local D555 interface MTU 9000, matching the camera. A dedicated netplan
  fragment persists this without replacing addresses or Wi-Fi settings.

The camera's stream profiles are ROS 2 parameters of its node
(`ros2 param describe /D555_<serial> RGB.Profile` lists the choices) and they
answer over Cyclone DDS; a user-scope unit, `uav-d555-profile`, sets
`d555_rgb_profile`, `d555_depth_profile`, the stereo IR profiles
(`d555_ir_profile`, for the `vslam` role) and `Depth.option.Emitter_Mode`
(`d555_emitter_mode`, Off since 2026-10-02: the projector's dots would be
tracked as features, and nothing consumes depth) at boot, retrying until the
camera answers. `ros2 param set` parses its value as YAML, where a bare `Off`
or `On` is a boolean, and exits 0 even when the camera refuses; the script
therefore sends `!!str <value>` and checks the reply for
"Set parameter successful". 1280x800 YUY2 at 30 Hz is ~500 Mbit/s, and the D555 sends
every subscriber its own unicast copy, so this pipeline must stay the only
image subscriber on the colour stream (2026-09-30: the health observer moved to
CameraInfo and the nvblox POC was retired for exactly this reason).

## Provision/update

From this repository on the Mac:

```sh
ansible-playbook -i jethawk, site.yml --tags mavlink_camera -e target_user=john
```

The canonical source is `roles/mavlink_camera/files/uav_camera`, an ament_python
package built with colcon on the Jetson under `/home/john/uav-camera/ws`.
Runtime dependencies include the NVIDIA GStreamer plugins already supplied by
JetPack. MAVLink Python bindings are pinned to 2.4.49 in a dedicated environment
that can import the system ROS, GStreamer, NumPy, and OpenCV libraries.

Tune settings in `group_vars/all.yml`, then rerun the role. Configuration changes
restart the camera service and interrupt an active recording; stop recording
before deployment. Existing media is retained.

## Persistent service

`uav-camera.service` is a systemd **user service** for john, enabled under
`default.target`; lingering starts the user manager at boot without a login.
The service explicitly loads Jazzy, the package overlay, ROS domain, Cyclone
configuration, and the isolated Python environment. It retries failed starts and
reconnects to the MAVLink router. Normal shutdown finalizes active recording.
A three-second encoder-output watchdog rebuilds a stalled encoder when raw
frames remain fresh. The service needs no root privileges; video/render group access is inherited.

Run on jethawk:

```sh
systemctl --user status uav-camera
journalctl --user -u uav-camera -f
systemctl --user restart uav-camera
systemctl --user stop uav-camera
```

## Further streams

The camera component can advertise streams that other processes serve, as
stream_id 2, 3 and so on. Today there is one, the LiDAR map view
(`roles/lidar_view`): `mavlink_camera_extra_streams` in group_vars builds it,
and it reaches the node as a JSON `extra_streams` parameter. Each stream's
`VIDEO_STREAM_INFORMATION` carries `count` = 1 + extras. A request for stream
0 returns every stream; that is how QGC asks, and it then retries any that are
missing.

`RUNNING` is set while the stream's server accepts a TCP connection on its
probe address. The probe result is cached for 5 s.

`VIDEO_STOP_STREAMING` is acknowledged and not obeyed, for every stream.
Switching streams, QGC sends STOP for the stream it leaves and START for the
one it shows. Honouring STOP for the RGB would cut it for every other viewer
the moment one QGC switched to the LiDAR view, and keep it cut until someone
switched back (seen on 2026-10-03: the RGB RTSP stalled with the D555 at
30 fps). It also saved nothing: RTSP sends only to clients that PLAY, and the
encoder runs regardless for recordings and Foxglove. `VIDEO_START_STREAMING`
forces a keyframe for the new viewer. An extra stream flagged
`thermal` is the one QGC overlays on the main stream instead of listing it in
the selector (see the lidar_view README).

## Photos and recordings

QGC can take single photos, request interval capture, and start/stop recording.
Photos require a new RGB frame after the request. Each JPEG gets a JSON sidecar;
recordings are Matroska (`.mkv`) with a JSON sidecar. Files live under
`/home/john/camera-media` on NVMe. Retrieve them using SCP; MAVLink file download
and a photo HTTP gallery are not implemented.

Recording uses the same encoded stream and is independent of ground viewers.
It continues if a viewer disconnects or live streaming is stopped. It does not
resume an earlier recording automatically after a service/aircraft restart.
Disk, stale-source and encoder errors are reported through MAVLink STATUSTEXT
and the ROS `~/status` topic. No automatic deletion/retention policy is enabled.

Sensor timestamps in the present firmware are **device uptime**, not Unix UTC.
Sidecars retain the original sensor stamp, original optical frame, source
intrinsics, host callback/processing times, and the pixel transform for the 180
rotation. Intrinsics refer to original source pixels; apply the saved transform
before using them with a rotated photo. These host times are not a substitute
for the planned sensor clock/extrinsic calibration.

## Transmitter buttons

The camera also serves two transmitter buttons, read from the autopilot's
RC_CHANNELS on its router connection: `mavlink_camera_rc_photo_channel` (12)
takes one photo and `mavlink_camera_rc_video_channel` (13) starts or stops
recording. They act like the GCS commands with nobody to acknowledge; QGC
still sees the capture event and the recording status. `button` mode
(`mavlink_camera_rc_button_mode`) acts on a momentary press, `toggle` on every
flip of a latching button or switch. A press is ignored, with a STATUSTEXT,
while the camera is busy, an interval sequence runs or no fresh frame exists.

RC_CHANNELS carries no loss or failsafe marker: PX4 forwards whatever the
receiver sends, failsafe positions included (the IO firmware keeps decoding
SBUS values while the failsafe bit is set, and SBUS RSSI stays at maximum). A
press therefore counts only while SYS_STATUS lists the RC receiver as present
(PX4 drops it from the present mask while manual control is invalid; 1 Hz),
after a two-frame debounce, and not in a frame where three or more other
channels jump by over 150 µs at once (a jump to failsafe positions, or back).
After a loss or a gap the current positions are a new baseline, not a press.
What remains is the second or so before SYS_STATUS reports a loss, if the
receiver's failsafe moves only these channels: set them to hold in the
handheld's failsafe settings.

PX4 streams RC_CHANNELS at 5 Hz on TELEM2, slow enough to miss a short press,
so while buttons are enabled the camera asks for 20 Hz (SET_MESSAGE_INTERVAL)
whenever the last 5 s fell short; PX4 forgets the rate on reboot. `~/status`
reports `rc_buttons`: channels, presses, receiver presence, measured rate and
suppressed frames.

Geotagging/camera pose is not implemented. CAMERA_IMAGE_CAPTURED reports NaN
orientation and INT32_MAX integer pose fields as application-level unavailable
sentinels; MAVLink has no standard integer-invalid value for those fields. Do not
interpret them as positions. Sidecars explicitly identify pose as unavailable.

## Reaching the drone-LAN address from a Mac

Nothing to set up. The stream URIs name the drone-LAN address `192.168.144.1`.
QGC on the Siyi GCS reaches it over the datalink. The Jetson also advertises
that address to the tailnet as a /32 route (`tailscale` role), so QGC on any
tailnet client reaches the same URIs over whatever underlay the Jetson has:
Wi-Fi on the bench, the GCS in flight.

The `bench-route-macos.sh` helper that used to add a static host route via the
Jetson's Wi-Fi address is gone (2026-10-03). Its route outlived every lease
change until the Mac rebooted: via the old `.40` lease, it shadowed the tailnet
route and failed with "No route to host". If `route -n get 192.168.144.1` on the
Mac shows a gateway on `en0` rather than a `utun` interface, delete the
leftover:

```sh
sudo route -n delete -host 192.168.144.1
```

## Verification

Unit tests on the actual Jazzy host:

```sh
source /opt/ros/jazzy/setup.bash
cd /home/john/uav-camera/ws/src/uav_camera
/home/john/uav-camera/venv/bin/python -m pytest -q
```

`tools/verify_camera.py` tests camera discovery through the MAVLink router.
Add `--capture` to take one photo during a five-second onboard recording; it
refuses to interrupt an already active recording. The probe sends camera
commands only. Run with a Python environment containing pymavlink 2.4.49.

Bench verification on 2026-09-24:
- Actual native DDS RGB received at 30 Hz after correcting RMW/MTU configuration.
- QGC on macOS automatically discovered the camera and displayed H.265 video;
  user confirmed the 180-degree rotation.
- MAVLink request, recording, photo-during-recording, completion ACK and capture
  event verified from this Mac through its host route.
- Recorded H.265 MKV decoded completely: 179 frames over 6.06 seconds at
  896x504; JPEG and source calibration/rotation sidecar inspected.
- H.264 and H.265 hardware encode, one-second IDR cadence, TCP/UDP decoding, and
  stream pause/resume tested on the Jetson. Pause is gone since 2026-10-03 (STOP
  is ignored, above).

Flight radio bandwidth, glass-to-glass latency and the UniRC's decoder still need
validation with the actual SIYI link. The bitrate is a bench starting point, not
an established flight-link budget. Cold-boot behavior is configured but has not
been tested by rebooting the aircraft during this session.

## Dashboard video topic

The node republishes every encoded access unit as
`foxglove_msgs/CompressedVideo` on `mavlink_camera_video_topic`
(`/d555/color/video`, `format` = `mavlink_camera_codec`). It is the same NVENC
output that feeds RTSP and recordings: one encode, Annex B byte stream,
keyframes self-contained (VPS/SPS/PPS inserted every IDR, one per second, no
B-frames), so Foxglove's Image panel decodes it directly. Messages are stamped
with the frame's capture time in UTC: its device stamp (mid-exposure) through
the clock model the D555 adapter publishes on `/d555/clock` (foxglove role,
"D555 time"). Without a valid, fresh model the stamp is host receive time.
`~/status` counts `video_stamped_capture_utc` and `video_stamped_receipt` and
reports `clock_model`. Photos use the same capture time for MAVLink
`time_utc_us` and record `capture_utc_us` beside `received_utc_us` in their JSON
sidecar. Measured: a video message arrives about 113 ms after its capture time
(transfer, conversion, NVENC). `video_frame_id` is a label for overlays, and the picture is already
rotated by `rotation_degrees`, so a panel should not rotate it again. `~/status`
reports `video_frames_published` and `video_frames_dropped`. The publisher is
best effort and fed through a small queue drained by the node's executor: a
reliable writer can block in `publish()` while a slow or departing reader is
in play, and this data comes from the encoder's streaming thread, where any
blocking shows up as an NVENC stall and a pipeline rebuild (measured
2026-10-01 with a rosbag2 reader).

Why not the camera's own compressed streams: the D555 firmware (as shipped,
2026-09-30) offers `/realsense/<serial>_Color/compressed` (JPEG) and `/h264`,
but one subscriber on either throttles the colour imager to ~3.5 fps, which
also starves this node's RTSP feed, and the camera replays stale
width×height-byte buffers (see the Foxglove role README for the measurement).
The Foxglove bridge hides both topics.
