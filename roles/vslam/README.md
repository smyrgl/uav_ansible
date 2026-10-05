# Visual odometry (cuVSLAM on the D555 stereo IR)

The `vslam` role runs NVIDIA Isaac ROS cuVSLAM (`ros-jazzy-isaac-ros-visual-slam`
4.6, from the pinned `isaac_ros` apt repository) on the D555's stereo infrared
pair, plus a bridge that puts its odometry on the host UTC time base. Output:
`/vslam/odometry` (`nav_msgs/Odometry`, `vslam_odom` → `base_link`, twist in
`base_link`) and the `vslam` diagnostic. It is a shadow source for now: nothing
fuses it yet. The plan is to use it as the low-altitude and backup source next
to FAST-LIO (the `lio` role), which suits a LiDAR mapping aircraft at altitude
better (stereo depth error grows with the square of range: at f = 452 px and a
95 mm baseline, disparity is ~43/Z px, 2.1 px at 20 m).

**Status 2026-10-05: installed but not running** (`vslam_enabled: false`).
Nothing on the roadmap consumes cuVSLAM live, and the IR pair it read is now in
every bag through the `d555_relay` role, so it is evaluated offline on the
replay host instead. Re-enabling it live means pointing its inputs at the
relay's `/d555/infra{1,2}` topics (a direct camera reader would be a second
unicast copy on the port) and dropping the bridge's device-to-UTC stamp
conversion, since the relayed inputs already carry UTC stamps.

## What runs

- `uav-vslam.service`: a `component_container_mt` with cuVSLAM loaded as a
  component, from `/etc/uav/ros/vslam.launch.py`. The packaged standalone
  executable is a Bazel binary that looks for a runfiles tree the deb does not
  ship (`DLOAD SHIM RUNFILES ERROR`, exit 255); the component is how NVIDIA's own
  launch files run it. GPU access needs `video` and `render`; the CUDA cache lives
  in `/var/lib/uav-ros/.nv/ComputeCache` (cuVSLAM's warm-up measured 7 ms, so the
  ~90 s JIT seen with nvblox does not apply).
- `uav-vslam-bridge.service` (`/usr/local/lib/uav/vslam_bridge.py`):
  - relays the IR `camera_info` with the camera's own stamps (cuVSLAM pairs them
    with the images, same clock), dropping any message labelled with another
    stream's frame (the D555 does that occasionally on its native topics);
  - maps cuVSLAM's odometry stamps from the D555 clock to UTC through
    `/d555/clock` (foxglove role), and publishes nothing while that model is
    unavailable;
  - replaces the twist (below) and publishes the `vslam` diagnostic;
  - anchors `vslam_odom` under `odom` with a static transform, computed when
    the first pose arrives and again after a reset (when cuVSLAM may have
    restarted its coordinate system), so that its `base_link` pose coincides
    with the EKF's at that instant; otherwise the frame has no place in the
    TF tree and Foxglove shows it as an orphan. The same static message carries
    `vslam_odom → vslam_map` as identity: with SLAM off the two coincide, and
    cuVSLAM still stamps `tracking/slam_path` and `vis/slam_odometry` in its map
    frame.

## Configuration choices

| Setting | Value | Why |
| --- | --- | --- |
| Inputs | `/realsense/D555_<sn>_Infrared_1/2` directly (as configured; stale since 2026-10-05, see Status) | rectified Y8 896×504 at 29.3 Hz, left/right stamps identical (hardware-synchronised); the pair's one reader is now the `d555_relay`, so a live cuVSLAM must read `/d555/infra{1,2}` |
| `tracking_mode` | 0 (stereo VO) | VIO (1) needs the IMU noise model and frame; the D555 IMU runs at 100 Hz against NVIDIA's 200 Hz examples, and its DDS IMU has open upstream issues (axes, millimetre extrinsic). Evaluate later. |
| SLAM | off (`enable_localization_n_mapping: false`) | loop closure and relocalisation can jump the pose |
| TF | none; frames `vslam_odom`/`vslam_map` | robot_localization owns `odom→base_link` and `map→odom` |
| `camera_optical_frames` | `camera_infra1/2_optical_frame` on the main TF tree | the description's aliases of the nominal URDF frames: baseline 95.0 mm against the factory 95.03 mm (0.04 % scale) |
| `image_jitter_threshold_ms` | 45 | frames arrive every ~34.1 ms, over the 34 ms default |
| D555 emitter | Off (`d555_emitter_mode`) | its dot pattern moves with the camera and would be tracked |

The camera link had to make room: the IR pair is 27 MB/s at 896×504/30 fps, on a
1 GbE link already at ~93 MB/s. The Foxglove depth relay was dropped (27 MB/s),
so with VSLAM the link is back to the same ~93 MB/s. Since 2026-10-05 the pair
flows to the `d555_relay` and from it, raw, into every flight bag, instead of to
cuVSLAM; the port load is unchanged.

## The twist is computed by the bridge

cuVSLAM's own odometry twist is the displacement across its ten-pose cache,
expressed in the oldest pose's frame (`pose_cache.cpp`, release-4.6): about
0.15 s late at 30 Hz and rotated by whatever the vehicle turned since. Its
"covariance" is the spread of the last ten velocities (3×10⁻¹⁰ (m/s)² standing
still). The bridge therefore publishes a central difference of three
consecutive poses, rotated into the middle pose's body frame and stamped with
its time (one frame of delay), with a diagonal 5 cm/s floor
(`vslam_twist_stddev_mps`). A gap over 0.1 s, after which cuVSLAM may restart
its coordinate system, or an implausible jump starts a new window
(`velocity_resets`).

## Measured on the bench (2026-10-02, indoors, vehicle still)

| Quantity | Value |
| --- | --- |
| Odometry | 30.0 Hz, receipt minus mapped stamp 30 ms (65 ms after the bridge's one-frame delay) |
| cuVSLAM track time | 1.7–1.8 ms per frame |
| Drift | 0.2 cm and 0.03° in 60 s; under 2 cm in 20 min |
| Velocity noise | ±0.07–0.14 cm/s (central difference) |
| Load | container ~16 % of one core, 630 MB; bridge ~15 % |

Not yet done: a motion test of axis signs and scale, and any fusion. PX4 gets
velocity only, after shadow validation, if at all (FAST-LIO is the primary
candidate).
