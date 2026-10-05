# D555 IR relay: the bag's copy of the stereo infrared pair

`uav-d555-ir-relay.service` is the one camera-side reader of the D555's two
infrared streams and republishes them locally, so the flight recorder (and any
other consumer on the Jetson) gets the frames without asking the camera for
another copy.

## Why a relay

The D555 is its own DDS publisher. It sends every subscriber a private unicast
copy of each stream, and everything on the drone LAN converges on the Jetson's
single 1 GbE port. Measured on the bench on 2026-10-05, idle, with one reader
per stream:

| Flow | Profile | Mbit/s |
|---|---|---|
| Colour, raw YUY2, to the H.265 encoder (`mavlink_camera`) | 1280×800 @ 30 | 491 |
| Infrared 1 + 2, raw Y8 | 896×504 @ 29.3, each | 212 |
| Avia clouds, PX4 XRCE, SIYI | | ~30 |
| Port total | | 747 of ~940 |

A rosbag2 subscriber on the pair would be a second copy, another 212 Mbit/s,
and the port would be over its ceiling. Every stream on it is best-effort, so
the drops would land on the operator video, the Avia clouds the recorder's gap
gate is written against, and the FC's XRCE link. That is why `^/realsense/`
has always been excluded from the bags. The frames already arrive once; the
second copy is a DDS artefact. The relay removes it.

## What it does

- Subscribes (raw, best-effort) to `/realsense/D555_<serial>_Infrared_1` and
  `_Infrared_2` and their `camera_info`, and publishes
  `/d555/infra1/image`, `/d555/infra1/camera_info`, `/d555/infra2/image`,
  `/d555/infra2/camera_info` (best-effort, keep-last 5).
- **Lazy** (`d555_relay_lazy`): the camera is read only while an output topic
  has a reader, checked at 1 Hz. The recorder's `-a` subscribes when a bag
  starts, so the pair flows while recording and costs the link nothing between
  bags: the bench port measured 527 Mbit/s idle and 747 while a bag ran.
- **Release is a restart.** Destroying the subscriptions does not make the
  camera stop: the D555 keeps unicasting to the vanished reader and stops only
  when that reader's DDS participant goes away (2026-10-05: 747 Mbit/s stayed on
  the port after the release, 527 after a process restart). So after
  `d555_relay_release_after_s` (5 s) without a local reader the node exits 0 and
  the unit's `Restart=always` brings it back idle two seconds later. A bag that
  starts in that gap finds the output topics as soon as the relay is up; rosbag2
  adds topics that appear while it records.
- **UTC stamps.** The camera stamps in its own clock. The relay maps each stamp
  to UTC with the adapter's model from `/d555/clock` (foxglove role: the same
  mapping the camera node and the vslam bridge apply), so image and
  `camera_info` of one frame carry one UTC stamp like every other sensor in the
  bag. While the model is missing or stale the frame is stamped at receipt and
  counted (`stamped_receipt`), so a bag's quality is visible in the diagnostic.
- **Canonical frame ids.** The camera pads `frame_id` with NULs, and now and
  then labels a frame with the other stream's name; those frames are dropped and
  counted (`dropped_frame_label`), as the vslam bridge does for `camera_info`.
- **No deserialization of images.** The CDR buffer passes through with only the
  header rebuilt (`restamp`): nothing after the header of a `sensor_msgs/Image`
  needs more than 4-byte alignment, and the rebuilt header is a multiple of
  4 bytes, so the payload is reused byte for byte. `camera_info` is small and is
  relayed typed. The tests build an Image by hand and, on the target with ROS
  sourced, round-trip one through rclpy's own serializer.
- One `DiagnosticStatus` `d555/ir_relay` on `/diagnostics`: readers, frames,
  stamp kinds, drops, the clock model's state.

## Measured (bench, 2026-10-05, 49 s manual bag)

| | |
|---|---|
| Camera readers per IR stream while recording | 1 (the relay) |
| Port load idle / recording | 527 / 747 Mbit/s |
| Relay CPU | 19 % of a core |
| rosbag2 CPU, everything plus the pair, zstd_fast | 54 % of a core |
| Bag growth | 29 MB/s |
| Frames relayed / dropped / stamped at receipt | 1405 per stream / 0 / 0 |
| `camera_info` stamps equal to an image stamp | 1398 of 1399 |

## What the bag gains

Lossless, UTC-stamped, hardware-synchronised global-shutter stereo at 29.3 Hz
with intrinsics: about 95 GB per flight hour before zstd (26 GB/h for everything
else today). It is what an offline cuVSLAM run, stereo or camera-to-LiDAR
calibration, and neural reconstruction (NuRec on the replay host) need. The
colour stream is recorded raw too, by the camera node itself
(`mavlink_camera_raw_topic`, `/d555/color/image`, 61 MB/s), since that node is
the camera's one colour reader; the H.265 operator stream stays the review copy.

## Verify

```sh
ros2 topic info -v /realsense/D555_261622302751_Infrared_1   # one reader: d555_ir_relay (while a bag runs)
ros2 topic hz /d555/infra1/image                             # ~29.3 Hz (this subscriber wakes the lazy relay)
```

Port load: `awk '/enP8p1s0/{print $2}' /proc/net/dev` twice, 10 s apart, ×8/10.
Tests: `cd roles/d555_relay/files && python3 -m unittest -v test_d555_ir_relay`.
