# nvblox on the X950 (D555 + Avia)

Isaac ROS 4.6 `nvblox_node` fuses the D555's depth and colour and the Avia's
point clouds into one GPU TSDF at 5 cm, meshes it, and maintains a 3D Euclidean
signed distance field. Everything runs natively on JetPack 7.2 from NVIDIA's apt
repository (see `roles/isaac_ros`); there is no container.

This is a **bench proof of concept**: the aircraft does not move, so the pose
nvblox needs is a static identity `odom -> base_link` published by
`uav-bench-odom.service`. That unit is the only thing standing between this
config and a flying one, and it is named so nobody forgets it.

## Units

| Unit | Runs as | Purpose |
| --- | --- | --- |
| `uav-nvblox.service` | `uav-ros` + `video`,`render` groups | `nvblox_node` with `/etc/uav/ros/nvblox.yaml` and the topic remaps below |
| `uav-d555-rgb.service` | `uav-ros` | `yuv422_yuy2` → undistorted `rgb8` + zero-distortion CameraInfo on `/d555/color/rect/*` |
| `uav-bench-odom.service` | `uav-ros` | **bench only** static identity `odom -> base_link`; disable with `nvblox_bench_static_odom: false` |

All three are `PartOf=uav-ros.target`; the first two are wanted by it. The bench
odometry unit is enabled on its own so that turning it off in Ansible really
stops it. Configuration lives in `group_vars/all.yml` (the knobs a viewer would
change) and `defaults/main.yml` (the long tail).

Input wiring (remaps on the `nvblox_node` command line):

| nvblox topic | Source | Notes |
| --- | --- | --- |
| `camera_0/depth/image` | `/realsense/D555_<serial>_Depth` | native `16UC1`, 896x504, ~21 Hz |
| `camera_0/depth/camera_info` | `/d555/depth/camera_info` | the frame-validated relay from `roles/foxglove`, **not** the native topic |
| `camera_0/color/image` | `/d555/color/rect/image` | from `uav-d555-rgb`, `rgb8`, ≤5 Hz |
| `camera_0/color/camera_info` | `/d555/color/rect/camera_info` | same stamp as the image, `d = 0` |
| `pointcloud` | `/avia/points` | 10 Hz, ~10 k points per cloud |

Outputs worth knowing: `/nvblox_node/color_layer_marker` and `tsdf_layer_marker`
(`visualization_msgs/Marker` cube lists that Foxglove renders natively; they are
only serialised while something subscribes), `/nvblox_node/mesh`
(`nvblox_msgs/Mesh`, needs RViz or NVIDIA's `nvblox_foxglove` extension), and the
services `save_ply`, `save_map`, `load_map`, `get_esdf_and_gradient`. In
`esdf_mode: "3d"` the ESDF is **not** streamed on a topic: it is updated at
`update_esdf_rate_hz` and served on request by `get_esdf_and_gradient` (a dense
distance + gradient grid for a planner; the request's cloud is echoed on
`~/esdf_service_pointcloud` while something subscribes). Only `"2d"` publishes
the slice on `static_esdf_pointcloud` / `static_map_slice` / `static_occupancy_grid`.

## Why the inputs are adapted

**CameraInfo.** The D555's native `…_Depth/camera_info` and `…_Color/camera_info`
topics interleave each other's messages: in a 10 s sample 3 of 213 depth
CameraInfo messages carried the *colour* frame id and the *colour* K (with its
distortion), and 2 of 212 colour messages carried the depth intrinsics. nvblox
pairs image and CameraInfo by exact stamp and builds its pinhole from `k`, so a
swapped message would project one depth frame through the wrong camera. The
relay in `roles/foxglove` drops every mislabelled message; nvblox reads the relay.

**Colour encoding and distortion.** nvblox integrates colour only from `rgb8` or
`bgra8` and models every camera as a pure pinhole (`k` only). The D555 publishes
`yuv422_yuy2` with about 5 % radial distortion (`k1 = -0.053, k2 = 0.058,
k3 = -0.018`). `d555_rgb_adapter.py` converts with OpenCV, undistorts onto the
camera's own K (barrel distortion means no black corners), publishes the image
with a matching zero-distortion CameraInfo carrying the image's stamp, and rate
limits on device stamps to what nvblox actually integrates. Device capture stamps
are copied unchanged; the node validates the frame id and K the same way the
existing relay does.

**Avia through a spinning-LiDAR model.** nvblox turns a point cloud into a range
image with its `Lidar` model: `lidar_width` columns over 360° of azimuth and
`lidar_height` rows over `lidar_vertical_fov_rad` of elevation, one range per
pixel, last write wins. Before the first integration it checks that every
finite point beyond `lidar_min_valid_range_m` projects inside that image; one
point outside the vertical FOV and the whole cloud is rejected (`LiDAR
intrinsics are inconsistent`). There is no assumption about points sitting on
pixel centres, so the Avia's non-repetitive rosette is acceptable as long as the
FOV covers it: measured elevation ±37.5°, azimuth ±35.3° (spec 77.2° × 70.4°),
so the config uses 80° and a symmetric model.

Only a ~70° slice of the 360° image is ever filled, and a 100 ms cloud fills
about 28 % of that patch at 0.5°/px, 21 % at 0.4°/px, 7 % at 0.2°/px. Holes are
tolerated by nvblox's sampler: linear interpolation needs all four neighbours,
and the fallback takes the pixel the voxel falls in *if* the voxel centre lies
within 0.5 voxel of that pixel's centre ray. That fallback is what bounds the
pixel size: at range *R* the worst-case offset is half the pixel diagonal,
`R · θ/√2`, so `θ < voxel/(√2 R)` — 0.51° at 4 m and 5 cm voxels, 0.20° at
10 m. The bench config (900 × 200, 0.40°/px) is matched to a room; for outdoor
ranges move to 1800 × 400 or to 10 cm voxels. The non-repetitive pattern fills
the remaining holes over the next few clouds, which is the property that makes
the Avia workable here at all.

**Motion compensation is off.** With `use_lidar_motion_compensation: true`,
nvblox looks for a per-point `t`, `time` or `timestamp` field and aborts the
process (a glog `CHECK`) when none exists. `/avia/points` carries the raw Livox
packet timestamp and a per-point offset but no cloud-relative field yet. Adding a
`t` (uint32 ns relative to the header) to the Avia adapter would unlock nvblox's
de-skew, which matters once the aircraft moves; the E1R's `timestamp` field
(float64 UTC seconds) would need converting to nanoseconds for the same reason.

## Time bases

Three clocks meet here. The D555 stamps in device uptime (about 43 000 s at the
time of writing, i.e. 1970 in ROS time), the Avia adapter stamps host receipt
time, nvblox stamps its outputs with ROS time. This works only because the whole
TF tree is static: `tf2` answers a static lookup at any time, and nvblox's rate
limiting compares stamps within one stream. The moment `odom -> base_link` becomes
dynamic, the D555 frames will fail their TF lookup and be dropped. Real odometry
therefore has to arrive together with the sensor time-base work already planned
for this aircraft (PPS-disciplined host clock, sensor timestamps in UTC).

## CUDA JIT cache

The Isaac ROS 4.6 Jetson debs embed SASS and PTX for `sm_75` only; the Orin NX is
`sm_87`, so the driver JIT-compiles every kernel from PTX on first use. Measured
on jethawk: about 18 s per voxel-layer type, ~90 s before the mapper is even
initialised on a cold cache, then more at the first depth, colour, LiDAR, mesh
and ESDF integrations. The compiled code is cached under
`CUDA_CACHE_PATH=/var/lib/uav-ros/.nv/ComputeCache` (`StateDirectory=uav-ros`
keeps it writable under `ProtectSystem=strict`; `CUDA_CACHE_MAXSIZE` is raised to
1 GiB). Expect the first start after a converge or an nvblox upgrade to be slow;
later starts initialise in about a second.

The service account also needs the `video` group: `/dev/nvgpu/igpu0/*`,
`/dev/nvhost-*-gpu` and `/dev/nvmap` are `root:video`, and without it CUDA
reports "no CUDA-capable device is detected".

## Tuning notes

* `input_qos: SENSOR_DATA` — the D555 and the Avia publish best effort; the
  default `SYSTEM_DEFAULT` (reliable) never matches them and nvblox stays silent.
* Rates compare **message stamps**, with `>` not `>=`: a 10 Hz limit on 10 Hz
  clouds drops about half of them. Limits are set above the sensor rates where
  every frame should integrate (`integrate_lidar_rate_hz: 20`), and depth is
  halved to ~10 Hz on purpose (`15`).
* `decay_tsdf_rate_hz: 0` keeps the bench map persistent. Enable decay when the
  scene is expected to change under the aircraft.
* `map_clearing_radius_m: 15` frees blocks farther than that from `base_link`;
  raise it for flight.
* `esdf_mode: "3d"` publishes the 3D ESDF cloud; `"2d"` gives the nav2-style
  slice (`esdf_slice_*` heights are relative to `odom`).
* `layer_visualization_undo_gamma_correction: true` is NVIDIA's own setting for
  rendering the cube markers in Foxglove.

## Verification

Bench check (as `john`, with the bench ROS environment):

```sh
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
       CYCLONEDDS_URI=file:///etc/uav/ros/cyclonedds.xml ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
source /opt/ros/jazzy/setup.bash
python3 /usr/local/lib/uav/verify_nvblox.py --wait 40 --save-ply /var/lib/uav-ros/nvblox/bench.ply
```

It checks the TF chains, the rectified colour contract, the coloured surface
voxel marker and ESDF in `odom`, the unit journal for the errors nvblox prints
when a model is wrong, the integration rates nvblox reports, and writes a PLY of
the mesh you can open in MeshLab or Blender. Service controls:

```sh
systemctl status uav-nvblox uav-d555-rgb uav-bench-odom
journalctl -u uav-nvblox -f
ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath "{file_path: '/var/lib/uav-ros/nvblox/mesh.ply'}"
```

In Foxglove the saved bench layout enables `/nvblox_node/color_layer_marker`
(coloured surface voxels) with the TSDF marker and the two ESDF clouds available
but hidden (the slice cloud only exists in `"2d"` mode, the service cloud only
after a `get_esdf_and_gradient` call).

### Bench verification, 2026-09-27

Static aircraft on the bench, indoor room (D555 depth 0.8–4.1 m, Avia 1.0–4.2 m),
`verify_nvblox.py --wait 90` as `john`:

| Check | Result |
| --- | --- |
| TF | `odom → base_link → camera_depth_optical_frame / camera_color_optical_frame / avia_nominal_lidar_frame` all static |
| Colour adapter | 479 frames in 90 s (5.3 Hz), 896x504 `rgb8`, 479/479 stamps matched by a zero-distortion CameraInfo |
| nvblox rates (journal) | depth 8–11 Hz, LiDAR 9.8 Hz, colour 1.8 Hz, ESDF 2.0 Hz, tick 101 Hz |
| Surface voxels | `color_layer_marker` ~19.9 k cubes, `tsdf_layer_marker` ~20.5 k, 178 messages in 90 s |
| `save_ply` | 13 799 vertices / 20 464 faces with per-vertex colour and normals, 1.2 MB |
| Journal | no LiDAR-intrinsics, encoding or TF errors after the render-group fix |
| Load | CPU 10–25 % per core across 8 cores, GPU (`GR3D`) 0–2 %, nvblox RSS 1.2 GB, +450 MB system RAM |

Observations: depth integration drops from ~10.7 Hz to ~5–8 Hz while a layer
marker subscriber is attached, because serialising ~20 k cubes at 2 Hz runs in
the same tick thread; lower `publish_layer_rate_hz` if depth rate matters more
than the live view. nvblox's "Delay statistics" for depth and colour show large
negative values: the D555 stamps in device uptime, so the delay is meaningless
(and harmless) until the camera is on the host time base. With a cold cache the
first start took ~90 s before the mapper initialised plus further JIT at the first
integrations; the second start initialised in 0.3 s from the 47 MB cache. NVIDIA's `nvblox_foxglove` extension (in the isaac_ros_nvblox repository,
`npm run package`) renders the true mesh from `/nvblox_node/mesh` if you want
triangles instead of cubes.

## Before flight

1. Replace `uav-bench-odom` with real odometry into `odom -> base_link`
   (PX4 `vehicle_odometry` through a NED→ENU bridge with `px4_msgs`, or cuVSLAM
   from the D555's stereo IR + IMU, `ros-jazzy-isaac-ros-visual-slam`).
2. Put every sensor on the host/UTC time base (this repository's PPS work) and
   give the Avia cloud a per-point `t` field, then enable
   `use_lidar_motion_compensation`.
3. Replace the nominal D555 depth↔colour extrinsic (59 mm, CAD) with the factory
   one the camera publishes on its namespaced `tf_static`, and calibrate the
   Avia and D555 mounts against `base_link`.
4. Retune the LiDAR grid and voxel size for the operating range (section above),
   raise `map_clearing_radius_m`, and decide on TSDF decay.
