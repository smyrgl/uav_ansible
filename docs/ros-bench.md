# ROS sensor bench on jethawk

Connect Foxglove with the Foxglove WebSocket connection (subprotocol `foxglove.sdk.v1`) to `ws://jethawk:8765` from the house Wi-Fi, or `ws://192.168.144.1:8765` on the SIYI link; the bridge binds every interface. Do not add a host route to the drone-LAN address via the Jetson's Wi-Fi address: that lease moves, and the stale route fails with "No route to host".

Import `roles/foxglove/files/x950-bench-layout.json` into Foxglove. A deployed copy is at `/etc/uav/ros/x950-bench-layout.json`. It displays the X950 model, E1R and Avia points, D555 color/depth images rotated 180 degrees, and grouped `/uav/health` diagnostics. Images are deliberately not time-synchronized to the E1R until camera acquisition timestamps are resolved. Raw images and clouds are intended for the bench; use the existing encoded RTSP stream for radio video.

## Startup and recovery

System-level systemd units run ROS nodes as the unprivileged `uav-ros` account, with no interactive login required. Configuration and installations are managed by Ansible. Individual services restart on failure and are enabled at boot; `uav-ros.target` groups the five inspection services:

| Unit | Purpose |
| --- | --- |
| `uav-description.service` | Headless robot_state_publisher, URDF and static transforms |
| `uav-e1r.service` | Pinned official RoboSense decoder, UTC-normalized point clouds |
| `uav-avia.service` | Pinned official Livox SDK1, Avia point clouds with explicit bench receipt timestamps |
| `uav-foxglove.service` | Bridge on the drone LAN, port 8765 |
| `uav-d555-frames.service` | CameraInfo frame-name compatibility adapter, pulled in by the bridge |
| `uav-ros-health.service` | Grouped sensor, timing, MAVLink and PX4 DDS delivery health |
| `uav-ptp-status.service` | Independent read-only PHC/PTP-to-UTC mapping check |
| `uav-nvblox.service` | Isaac ROS nvblox: D555 depth+colour and Avia LiDAR fused into one GPU TSDF (see below) |
| `uav-d555-rgb.service` | D555 `yuv422_yuy2` → rectified `rgb8` + zero-distortion CameraInfo for nvblox |
| `uav-bench-odom.service` | **Bench only:** static identity `odom → base_link` standing in for odometry |

On Jetson:

```sh
sudo systemctl restart uav-ros.target
systemctl status uav-description uav-e1r uav-avia uav-foxglove uav-ros-health uav-nvblox uav-d555-rgb uav-bench-odom
journalctl -u uav-e1r -f
```

The existing D555 `uav-camera.service` remains a separate user service under `john`, with lingering already enabled. It supplies the NVENC RTSP and MAVLink Camera v2 path. Native D555 ROS data arrives over Ethernet directly from the camera.

Common ROS environment: `/etc/uav/ros/environment`; Cyclone configuration: `/etc/uav/ros/cyclonedds.xml`; ROS domain 0 on `enP8p1s0`. Runtime workspace: `/opt/uav/ros`. Service logs use journald and `/var/log/uav-ros`.

## Current data contract and limits

- `/e1r/points`: PointCloud2 in `e1r_nominal_lidar_frame`; x/y/z, intensity, ring and per-point UTC timestamp. Normal scans contain 27,648 points at approximately 10 Hz. The driver reports packet gaps; a published scan is not an assurance that the network lost no packets.
- E1R publication requires fresh local clock verification, gPTP mode 3, recent DIFOP synchronized status 1, and plausible packet timestamps. Startup/recovery partial scans are discarded. Raw PTP/TAI timestamps are converted to UTC using the independently checked offset (currently 37 seconds); both header and point times use UTC.
- `/e1r/difop_raw` retains raw status packets. E1R IMU is intentionally not published until units and coordinate conventions are verified.
- D555 CameraInfo metadata passes through a small frame-name adapter for Foxglove. Native calibration headers contain trailing NUL padding; image headers are already canonical. Bridge-local topic remaps preserve saved layout selections. Original ROS messages, capture timestamps, image payloads and calibration values are preserved; the adapter only republishes CameraInfo with validated frame IDs.
- D555 native color/depth image receipt is monitored separately from synchronization. Its headers currently use device time. A green stream status does **not** mean it is synchronized with the E1R. Native RGB timestamp regressions are counted in the clock diagnostic for follow-up.
- The URDF positions and nine native-camera frame aliases are nominal, pending measured extrinsic calibration. The native camera's separate namespaced TF tree is preserved in ROS but excluded from Foxglove to prevent competing parents.
- Raw `/fmu/*` stays excluded from the Cyclone Foxglove bridge. `/uav/health` now includes actual PX4 DDS topic delivery using pinned 1.17 definitions and a Fast DDS health observer, alongside MAVLink heartbeat and H-FLOW flow/range checks. Topic discovery alone never indicates a healthy DDS link.
- Avia network and ROS bringup are complete (details below); hardware synchronization remains pending. Hadron integration remains pending. GNSS SBF/PVT and chrony PPS are monitored live; H-FLOW is observed through PX4 telemetry. Loss of that telemetry path makes H-FLOW unknown, not proven disconnected.

## Avia network commissioning — 2026-09-25

The original Livox Avia is connected through SIYI air-unit LAN1 to the Holybro/Jetson drone LAN. Initial discovery failed because the SIYI-to-Jetson uplink was not fully seated; reseating it restored the Avia's broadcasts. No sensor cable repinning was required.

| Setting | Verified value |
| --- | --- |
| Static address | `192.168.144.80/24` |
| Gateway | `192.168.144.1` |
| MAC address | `0c:9a:e6:bc:f8:d9` |
| Broadcast code | `3JEDNAP001S5701` |
| Firmware | `11.8.0.6` (Livox release `11.08.0006`) |
| Previous factory address | `192.168.1.170/24`, gateway `192.168.1.1` |

Firmware was checked against the [official Avia download page](https://www.livoxtech.com/avia/downloads) and its release notes on 2026-09-25. `11.08.0006` (released 2021-12-26) is the latest publicly listed firmware for the original Avia and matches the installed version; no firmware flash was needed.

The sensor acknowledged the static-IP command and software restart; its configuration was read back successfully at the new address after restarting. Commissioning used a temporary `/32` Ethernet address and a route only to `192.168.1.170`, since the factory subnet overlaps bench Wi-Fi. Both temporary entries were removed; the Wi-Fi route remains unchanged. No Avia-specific host routing is needed at the final address.

A bounded 12-second UDP capture received 30,048 Cartesian single-return packets (2,884,608 point slots, about 240,374/s) and 2,446 IMU packets. Of the point slots, 453,424 contained nonzero coordinates; zero/no-return samples are not usable detections. Example valid returns were approximately 1–3 m away. Every observed packet carried status word zero and **timestamp type 0: unsynchronized sensor uptime**, with timestamps advancing through the capture. This validates communication and sample delivery, not calibration or synchronization. Sampling was stopped after that initial network-only check; the persistent ROS driver was then installed as described below.

The persistent `uav-avia.service` now publishes `/avia/points` at 10 Hz using a small adapter around official SDK1 v2.3.1, pinned at `14c533dd7175bd90a6b568c0aa1733f35d36cb89`. It is enabled at boot, runs as `uav-ros`, and restarts on failure independently of missing E1R/GNSS hardware. `/diagnostics` carries `avia/driver` reception health and a separate `avia/clock` warning. No public Avia IMU topic is published yet.

PointCloud2 headers carry the Avia's own sample time in UTC once the driver has validated it (PTP lock, or PPS plus the pushed UTC time, and a sane receipt latency), and host receipt time until then. Raw sensor packet timestamps, their type, status, and per-point offsets are preserved as additional point fields. Empty `(0,0,0)` returns are omitted. `fusion_ready=false` remains explicit while the extrinsics are nominal. Detailed field meanings, configuration, and rebuild instructions are in [the Avia role](../roles/avia/README.md).

The description adds `avia_nominal_lidar_frame` under the existing mounting-hole datum `avia_link`, at `(0.0525, 0, 0.0324)` m, aligned X forward/Y left/Z up. This uses the CAD front plane and manual scan-origin height; measured extrinsic calibration is pending. Its XYZ/RPY can be overridden through the description role. The saved Foxglove layout enables `/avia/points` with a 0.5-second decay, and the current desktop 3D view was enabled and visually checked.

Live validation as the actual `uav-ros` account received 122 clouds in roughly 12 seconds (10.03 Hz), typically 3,241–3,974 nonzero returns per cloud, with fresh host headers and the complete TF chain to `base_link`. An intentional driver-process termination exercised systemd recovery: a new PID started automatically (`NRestarts=1`), and 165 subsequent clouds arrived at 10.00 Hz. Both captures reported zero packet-gap events, unsupported packets, buffer drops and timestamp regressions; device status was normal. The point-contract unit test and full scoped Ansible deployment passed. A full Jetson power-cycle remains an operator-timed verification, not something performed during this bringup.

The shared LAN interface `enP8p1s0` reports software timestamping only and no PHC. Avia supports UDP/IPv4 IEEE 1588v2 E2E PTP, which differs from the E1R's dedicated L2/P2P gPTP setup. Software PTP or the planned PPS+UTC path needs a separate configuration and measured validation; no timing configuration was changed during network commissioning.

Sources: [Avia manual](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Download/Avia/Livox%20Avia%20User%20Manual%20202204.pdf), [Livox communication protocol](https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol), [original ROS 2 driver](https://github.com/Livox-SDK/livox_ros2_driver).

## nvblox reconstruction — 2026-09-27

Isaac ROS 4.6 `nvblox_node` runs natively (apt, JetPack 7.2, no container) as
`uav-nvblox.service` and fuses the D555 depth and colour with the Avia point
clouds into one 5 cm TSDF in the `odom` frame, meshes it and keeps a 3D ESDF.
Because the aircraft is static on the bench, `uav-bench-odom.service` publishes
an identity `odom → base_link`; that unit is the placeholder for real odometry
and must be replaced before flight (`nvblox_bench_static_odom: false`).

Data contract:

- `/nvblox_node/color_layer_marker` and `/nvblox_node/tsdf_layer_marker`:
  `visualization_msgs/Marker` cube lists of surface voxels in `odom`, published
  at 2 Hz **only while subscribed**. The saved bench layout shows the coloured one.
- `/nvblox_node/mesh`: `nvblox_msgs/Mesh` (RViz plugin or NVIDIA's `nvblox_foxglove`
  extension). `ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath
  "{file_path: '/var/lib/uav-ros/nvblox/mesh.ply'}"` writes a coloured PLY.
- 3D ESDF: served by `/nvblox_node/get_esdf_and_gradient`; nothing streams on
  `static_esdf_pointcloud` in 3D mode (2D mode publishes the slice instead).
- `/d555/color/rect/image` + `/camera_info`: rectified `rgb8`, ≤5 Hz, device
  stamps unchanged, `d = 0`. Diagnostics: `d555/rgb_adapter`.

Inputs come through the validated relays, not the native camera topics: the
D555's native CameraInfo streams interleave depth and colour intrinsics (3 of
213 depth messages carried the colour K in a 10 s sample), and nvblox pairs image
and CameraInfo by exact stamp. Motion compensation is off (the Avia cloud has no
per-point time field; enabling it aborts the node). Full rationale, the
LiDAR-model sizing (900x200 grid, 80° FOV, 0.4°/px good to ~5 m at 5 cm voxels),
the CUDA JIT cache (the Jetson debs ship `sm_75` code only) and the device-group
requirements (`video` and `render`) are in [the nvblox role](../roles/nvblox/README.md).

Bench verification (static aircraft, indoor room): TF chains from `odom` to all
three sensor frames resolve; the colour adapter delivered 479 stamp-matched
frames in 90 s; nvblox integrated depth at 8–11 Hz, LiDAR at 9.8 Hz, colour at
1.8 Hz and updated the ESDF at 2 Hz; ~20 k coloured surface voxels were streamed;
`save_ply` produced a 13.8 k-vertex coloured mesh; GPU load stayed at 0–2 %,
CPU 10–25 % per core. Check it again with:

```sh
python3 /usr/local/lib/uav/verify_nvblox.py --wait 60 --save-ply /var/lib/uav-ros/nvblox/bench.ply
```

Time bases are UTC: the Avia stamps its own sample time synchronized by the
receiver's PPS plus the driver's UTC push (GPS sync mode; the ptpd master was
retired on 2026-10-04), the D555 adapter maps the camera's hardware clock
to UTC from its IMU stream, and the PX4 bridge corrects FC stamps with the PPS.
See the avia and foxglove role READMEs for how each is validated.

## Rebuilding and deploying

The model source remains the exported `x950_description` beside this repository in `../Printables/x950_description`. Update that export from its existing Blender/build workflow first; the role copies only runtime files, not Blender/CAD intermediates. A runtime snapshot is also staged at `/home/john/Printables/x950_description` on Jetson for local reconvergence.

From the Mac's `uav_ansible` repository:

```sh
ANSIBLE_STDOUT_CALLBACK=default ansible-playbook -i jethawk, site.yml --tags ros_inspection -e target_user=john
```

The `ros_inspection` tag now includes `nvblox`; the NVIDIA apt repository and the
pinned nvblox packages come from `--tags isaac_ros` (run once, or with the full play).

Override `robot_description_source_dir` if the exported package lives elsewhere. The same tag works with Jetson's `inventory/localhost.yml` after source synchronization. Do not run parallel colcon builds in the shared workspace. These changes are present in the local and Jetson working copies; no commit/push has been made.

## Bench verification

The initial deployment verified model/TF delivery, mesh fetching and D555 images through the bridge from the Mac, plus actual service-user ROS subscriptions and E1R point fields/timestamps. Timing-status interruption/recovery passed: 10.015 Hz baseline, zero clouds after status expiry, ERROR diagnostics, then automatic recovery to OK. The underlying gPTP services stayed running. The coordinated service-group restart also passed; all four services returned active without restart loops. A complete sensor-tag deployment succeeded from the Jetson working copy. Boot enablement is configured; a full power-cycle/reboot test remains to be done when convenient for the bench setup.

## PX4 communication interruption observed 2026-09-27

At about 12:09 MDT a 15-second packet capture observed 15 PX4 heartbeats
arriving at the Jetson; the unchanged MAVLink router delivered all 15 to a
TCP client, along with attitude/GPS/flow/range. XRCE traffic also arrived.
Around 12:10:50, both PX4 MAVLink and XRCE traffic stopped without an operator
change. Subsequent captures received neither, although ICMP pings to the same
PX4 IP/MAC still succeeded. A Fast DDS subscription probe found advertised
status/attitude/IMU/timesync publishers but received zero samples in 15 seconds.
This localizes the interruption upstream of the router; it does not establish
the internal PX4 cause. Console access during the failure was unavailable.
No Pixhawk reboot, firmware change, or flight-control parameter change was made
during this diagnosis. Fresh PX4 delivery/recovery validation remains pending.

## Executor A/B (2026-10-04)

Autonomy roadmap Stage 0b, "compute recovery, measured". Every Python node
spins through a small `_spin()` helper: rclpy's default executor, or the
experimental `EventsExecutor` (callbacks run from DDS events instead of a wait
set rebuilt on every wake-up) when its unit carries `UAV_EVENTS_EXECUTOR=1`
(`ros_events_executor` in group_vars, on by default since this measurement).
CPU is utime+stime of each service's Python processes from `/proc` over 60 s,
sampled with `/home/john/uav-probes/cpu_by_unit.py` on the Jetson, bench
load (every sensor streaming, no bag). Baseline before any change
(`cpu_baseline_20261004T181138Z.json`), then the health node alone, then all:

| Service | Default executor, % of a core | EventsExecutor | Note |
|---|---|---|---|
| uav-ros-health | 58.0 | 14.5 | the guard's graph scan is 4.7 ms/s; raw cloud subscriptions changed nothing (kept: no 2.5 MB deserialization) |
| uav-vslam-bridge | 24.6 | 7.5 | |
| uav-lio-bridge | 10.1 | 3.9 | |
| uav-lio-map | 4.2 | 2.8 | |
| uav-lio-e1r | 2.3 | 1.2 | |
| uav-lio-watchdog | 1.8 | 1.0 | |
| uav-hflow | 16.7 | 16.8 | its cost is the CAN frame parsing thread, not the executor |
| uav-flight-recorder | 1.5 | 1.3 | |
| uav-d555-frames | 18.3 | crash-loops | rclpy 7.1.12 "SystemError: null argument to internal routine" at start; pinned to the default executor (`foxglove_d555_frames_events_executor: false`) |
| all Python services | 124.2 | 54.6 | without the camera node, below |
| uav-camera (user unit) | ~105 | 22.0 | not the executor: the frame now enters GStreamer as the D555's native YUY2 and nvvidconv converts and flips on the VIC; the CPU YUY2→RGB→NV12 double conversion is gone (mavlink_camera README) |

Not in the table: the gnss broker/time feeders, jetson stats and lidar_view
(unchanged, not rclpy executors in the hot path).
