# Foxglove bridge

Connect Foxglove using **Foxglove WebSocket** (subprotocol `foxglove.sdk.v1`)
at `ws://jethawk:8765` from the house Wi-Fi, or `ws://192.168.144.1:8765` on
the drone LAN. The bridge binds every interface (`foxglove_bind_address`), like
the router's TCP port; no cloud gateway is enabled, and there is no
authentication, so it is reachable by whoever can reach the Jetson. Do not
route to the drone-LAN address from the Wi-Fi: the Jetson's Wi-Fi lease moves,
and a static host route via that address goes stale (2026-09-30). If the bind
is ever confined to the drone LAN again, a tunnel still works:

```sh
ssh -N -L 8765:192.168.144.1:8765 jethawk   # then connect to ws://localhost:8765
``` Raw point
clouds and camera images can consume substantial bandwidth over the radio, so
subscribe to the panels you need. Video for piloting remains on the existing
encoded camera stream.

The bridge exposes topics, the connection graph, read-only ROS parameter
services, and the X950 STL meshes. It does not offer client topic publication or
the parameter-setting capability. Asset retrieval accepts only
`package://x950_description/meshes/<safe-name>.stl`; neither arbitrary local files
nor arbitrary package resources are exposed.

In a 3D panel select `base_link` as the fixed/display frame, enable the
`/robot_description` topic and the E1R point cloud topic. The model's sensor
transforms remain nominal until extrinsic calibration. Add an Image panel for
each native D555 stream as needed. Foxglove's Topic Graph shows discovered nodes
and connections; service process health is separate from receipt of valid data.

Import `files/x950-bench-layout.json` from this role into Foxglove for a starting
layout with the robot, `/e1r/points`, `/avia/points`, the FAST-LIO voxel map
(`/lio/map`, coloured by height; see the lio role), the D555 colour video, and diagnostics
summary/detail on `/uav/health` (one row per sensor; grouped data/timing details).
Existing imported layouts need their Diagnostics Summary and Detail topics changed
from `/diagnostics` to `/uav/health`. Raw driver diagnostics remain on `/diagnostics`.
A copy is deployed at `/etc/uav/ros/x950-bench-layout.json`.
Click a diagnostic row to populate its detail panel. The images use display-only
180-degree rotation, with synchronization/rectification disabled while camera
timestamps remain device-relative. Depth is displayed as raw 16-bit values with
a 0–10000 color range; this setting does not assert a calibrated distance scale.

Native camera `yuv422_yuy2` and depth `16UC1` encodings are supported by the current
[Foxglove Image panel](https://docs.foxglove.dev/docs/visualization/panels/image).
The layout structure follows Foxglove's official
[camera example](https://github.com/foxglove/foxglove-sdk/blob/main/python/foxglove-sdk-examples/oak-camera-streaming/foxglove/oak_layout.json)
and [URDF example](https://github.com/foxglove/foxglove-sdk/blob/main/python/foxglove-sdk-examples/so101-visualization/foxglove/lerobot_layout_with_tf.json).
Diagnostic panel fields were checked against the installed Foxglove application.

The bridge excludes `/realsense/D555_261622302751/tf_static` from visualization:
that factory optical-frame tree otherwise conflicts with the description's
nominal aliases. The original topic remains untouched on ROS. The visual model
uses nominal mounting and optical poses; factory intrinsics/extrinsics and the
future measured aircraft calibration are not yet integrated into this TF tree.

Runtime: `uav-foxglove.service`, service account `uav-ros`, shared DDS environment
`/etc/uav/ros/environment`, logs in the system journal and `/var/log/uav-ros`.
The unit restarts after unexpected exit and is enabled at boot.

Parameter names are verified against
[Foxglove bridge 3.5.0](https://github.com/foxglove/foxglove-sdk/tree/ros-v3.5.0/ros/src/foxglove_bridge).

The bridge keeps raw `/fmu/*` excluded during sensor bringup. Firmware-matched
PX4 1.17 definitions now live in `/opt/uav/px4_msgs`; the health service sources
that overlay and monitors periodic topic delivery via Fast DDS. The Cyclone
bridge does not source that overlay yet. Its `/uav/health` summary includes
separate PX4 MAVLink and DDS rows without exposing raw flight-control topics.

## Native D555 CameraInfo frame-name compatibility

D555 native Image messages already use canonical optical frame IDs. Its native
CameraInfo messages currently serialize the same names in a 64-byte CDR string
with 37 trailing NUL bytes. Foxglove treats those as different coordinate frames.
`uav-d555-frames.service` republishes the two small CameraInfo streams on
`/d555/color/camera_info` and `/d555/depth/camera_info`, with validated canonical
frame IDs and all calibration fields preserved. The original native CameraInfo
topics remain on ROS.

### D555 time: everything under `/d555` is UTC

The D555 stamps in its own hardware clock. Over PoE it has no PTP client
(firmware 7.58 sends no PTP at all) and hardware sync is USB-only, so the same
service fits that clock to UTC on the host, the way librealsense's global time
does (`d555_clock.py`). The source is the camera's IMU: 100 Hz in small packets,
each carrying the DDS source timestamp (the camera's clock when it was sent) and
the host's DDS receive timestamp (UTC, chrony on the GNSS PPS). Latency only ever
adds, so a line through the per-second minima of receive minus send, over a 60 s
window, gives offset and skew; congested seconds are rejected, and a 32-bit
microsecond wrap of device time (period 4295 s) is unwrapped while any other
backwards jump starts a new model (camera restart).

Every `/d555` output carries the mapped capture time while the model is valid,
and host receive time until it is (about 10 s after start):

| Topic | Contents |
| --- | --- |
| `/d555/imu` | the camera's IMU, `sensor_msgs/Imu`, 100 Hz |
| `/d555/color/camera_info`, `/d555/depth/camera_info` | CameraInfo, canonical frame IDs |
| `/d555/depth/throttled` | depth image at `foxglove_depth_throttle_hz` (off since 2026-10-02) |
| `/d555/clock` | the model as JSON, latched: offset, skew, residual, validity |

The camera node applies `/d555/clock` to `/d555/color/video` and photo times.
`d555/clock` on `/diagnostics` reports the model; the health node's D555 row
verifies timing from it. Measured 2026-10-02: skew -9 ppm, bin-minimum residual
49 us; receipt minus mapped stamp 0.7-4.6 ms for the IMU, 53-56 ms for colour,
31-37 ms for depth, never negative. The mapping cannot see the minimum one-way
transport latency, so mapped times are late by about that much (sub-millisecond),
never early. The service uses about a quarter of one core, mostly Python
per-message work on the 100 Hz IMU. Only the bridge node remaps its subscriptions to the normalized topics,
keeping the existing native topic labels in Foxglove and preserving saved layouts.
The internal adapter topics are hidden from duplicate bridge advertisement. Restart the Foxglove connection to clear any
orphan names cached before this fix.

A direct native-ROS raw-message probe on 2026-09-25 also found occasional
CameraInfo/frame mismatches upstream of the adapter. In 8 seconds, Color had
135 correctly labeled messages and one infrared-2 label; Depth had 200 correct
messages plus one infrared-1 and one color label. All native header strings had
CDR length 64. The adapter rejects and logs inconsistent labels rather than
rewriting them to the expected camera. The native publisher issue remains for
future camera-pipeline validation; no factory calibration values are altered.

Avia point clouds are available on `/avia/points`, with `avia/driver` and `avia/clock` diagnostics. The saved bench layout enables a 0.5-second cloud decay. Headers carry the Avia's validated sample time in UTC when it is synchronized, and host receipt time otherwise; measured extrinsics remain pending.

## Health display latency

`message_backlog_size` bounds the Foxglove SDK per-client queue (its default
is 1024 messages). With simultaneous raw images/clouds, the original connection
delivered diagnostics over 11 seconds late, making every status gray under the
five-second stale timeout. The bounded queue drops old visualization messages
on overflow rather than retaining that backlog. At 16 the same layout delivered
health updates in about 0.25 seconds and visibly rendered ERROR red, WARN amber
and OK green. The stale timeout remains enabled. This does not guarantee latency
on every radio link; raw images still require sufficient bandwidth.

The queue also carries fetch-asset responses on the SDK's control plane, and
the SDK closes the connection when one of those finds the queue full instead of
dropping it (`ShutdownReason::ControlPlaneQueueFull` in the SDK's
`connected_client.rs`; nothing is logged on either side). The 3D panel
requests every mesh of the URDF at once, so at 16 the bridge served exactly the
first three of the 42 X950 meshes (the Avia parts) and dropped the client; the
rest rendered as missing (2026-09-30; bursts of 12 passed, 16 did not). The
value is now `foxglove_message_backlog_size` (128): room for the whole model
plus the images a layout is already streaming. If a mesh still fails to load,
the alternative is client-side: point Foxglove's ROS package path at a local
copy of `x950_description`, which resolves `package://` without the bridge.

## Image panels and the D555's per-subscriber unicast

The D555 sends every subscriber its own unicast copy of a stream, so a raw
1280×800 colour panel costs ~500 Mbit/s of LAN and competes with the camera
pipeline that feeds the RTSP link. The bench layout therefore shows colour
through `/d555/color/video` (`foxglove_msgs/CompressedVideo`, the camera
node's own NVENC output, H.265 by default at `mavlink_camera_bitrate`, decoded
by Foxglove's Image panel). Depth used to come through `/d555/depth/throttled`,
which the D555 adapter republishes at `foxglove_depth_throttle_hz` with UTC
stamps from one raw (serialized) depth copy; since 2026-10-02 that rate is 0
(`group_vars`), and the layout has no depth panel: the 27 MB/s went to the
stereo IR pair for VSLAM. Set a rate again to bring depth back, minding the
link budget. The D555 has no compressed depth stream.

Do not use the camera's own `/realsense/<serial>_Color/compressed` (JPEG) or
`/h264` topics, and the bridge hides them from Foxglove. Since 2026-10-01 it
also hides the raw `_Color`, `_Depth`, `_Infrared_*` and `_Depth_Color_Points`
streams: one session with raw panels pulled 77 MB/s, which with the camera
node and the depth relay filled the 1 Gbit camera link and throttled the
imager to 9 fps. Raw streams remain available to ROS tools on the Jetson. Measured 2026-09-30
with firmware as shipped: as soon as either had one subscriber the imager's
frame counter dropped from 29.9 to ~3.5 fps (every stream, RTSP included), the
JPEG topic cycled four stale payloads padded to exactly width×height bytes,
one of them not a JPEG at all, and the H.264 topic mixed valid access units
with 1 MB buffers ending in garbage. With no compressed subscriber the imager
returned to 29.9 fps immediately. That is what Foxglove showed as "rotating
between three frames, one of them an error".

