# Open-source landscape for 3D UAV autonomy on the X950 (verified 2026-10-03)

Appendix to [the autonomy roadmap](autonomy-roadmap.md). Nine survey lenses were
run over the open-source landscape on 2026-10-03, each candidate checked against
primary sources (repository metadata via the GitHub API, READMEs, package files,
issues, PX4 v1.17 source); 135 distinct candidates were ranked across lenses and
the top 14 were re-verified adversarially (an agent tasked to refute every claim
with primary sources). Section 1 gives the verdicts on those 14; section 2 the
per-lens summaries as written; section 3 the full ranked list. Treat GitHub
activity dates as of 2026-10-03.

## 1. Verified shortlist

| Project | Verdict | Activity | Jazzy / aarch64 | What matters here |
|---|---|---|---|---|
| **px4-ros2-interface-lib** (Auterion) | core, later | 2026-09-25 | builds today (C++17, Jazzy CI, arm64) | Only mavros-free way to put a Jetson mode into commander. Use **release/1.17 @ 4a3370f0** (the only line message-compatible with the pinned `px4_msgs` 86d8239); main/2.x and the Cloudsmith debs track PX4 main. On v1.17 there is **no `user_selectable`** (added one day after the branch point), so a registered mode appears in QGC; the FC removes an unselected mode after ~1.2 s of missed replies and the lib exits by exception; setpoint types publish to `/fmu/in` on every `update()` even when unselected. Dedicated process, own executor thread, `Restart=`, re-register. Run `check-message-compatibility.py` against the PPS firmware once. |
| **PX4 v1.17 Offboard path** (OffboardControlMode + TrajectorySetpoint / GotoSetpoint over XRCE) | core | release/1.17 head 2026-08-06 | native | The XRCE client republishes any `/fmu/in` setpoint to uORB with **no nav_state check**; `mc_pos_control` takes the newest sample in Position/Altitude modes; FlightModeManager shares the topic. Stamps: with `UXRCE_DDS_SYNCT 1` the companion stamps in its own clock (µs); 0 becomes "now". `COM_OBL_RC_ACT 0` becomes `NAV_RCL_ACT` (Return) when RC is also lost (v1.17 backport). Stick override in Offboard needs `COM_RC_OVERRIDE` bit 1 (not set today). |
| **px4_msgs** release/1.17 | core | branch 2026-03-22 | builds today (first-party on this Jetson) | 86d8239 is byte-identical to release/1.17 HEAD and the custom firmware never touches `msg/`. Versioned topics carry `_vN` (`vehicle_status_v1`, `vehicle_local_position_v1`). No XRCE "type hash": a mismatch mis-deserialises silently; the interface-lib's FNV-1a `message_format_request` check is the safety net. Move to release/1.18 with the FC (`vehicle_status_v4`), never main. |
| **Micro XRCE-DDS Agent 2.4.3** | core (frozen) | 2.x line unchanged since 2024-03 | builds today | Only agent line PX4 v1.17 can talk to; 3.x is incompatible. Forwards anything written to `/fmu/in`: read-only is a property of the ROS nodes, not the agent. Timesync lives on the FC (`Timesync.cpp`, 1 Hz, RTT ≥ 10 ms samples rejected): the ms-level bias is plausibly sample starvation on the stalling FMU link. Open report of lost ROS 2 service replies with a Cyclone host RMW (#412): use the VehicleCommand/Ack topics, never the service. |
| **PX4 logger / ULog + dds_topics.yaml** | core | v1.17.0 | native | `trajectory_setpoint` is logged at 5 Hz only and is **not** a `/fmu/out` publication; add it and `vehicle_local_position_setpoint` to `publications:` at `rate_limit: 50` (a uORB topic may be both a subscription and a publication, as `mode_completed` already is). The ulog carries UTC anchors (`vehicle_gps_position.time_utc_usec`, `boot_time_utc_us`) because the FC reads the mosaic-G5. `SDLOG_PROFILE 19` leaves `collision_constraints` unlogged; bit 7 (147) adds them. |
| **PX4 v1.17 built-in safety** (Collision Prevention, geofence, failsafes, failure injection) | core | v1.17.0 | native | CP runs only in Position and Position Slow with `MPC_POS_MODE 4`; needs a 72 × 5° BODY_FRD map, bins fresher than 500 ms, a map update within 5 s (else zero XY, then Hold); downward sensors excluded; this suite covers ~14 of 72 sectors so `CP_GO_NO_DATA 1` is mandatory. With `CP_DIST −1` the stream is ignored and produces nothing. Failure injection on 1.17 covers motors only on hardware; the `failure` shell command is not in stock fmu-v6x firmware. |
| **rosbag2 MCAP** (record, replay with simulated clock) | core | jazzy branch 2026-09-28; 0.26.11 | Tier-1 arm64 | Bags already hold raw `/fmu/out`, `/tf`, both LiDARs, the odometries; the D555 only as H.265 colour. Replay on the aircraft must exclude `^/fmu/in/` or stop the agent. Jazzy bugs: fixed-rate `--clock` with `--playback-duration` leaves `/clock` running (#2383; fixed rolling/lyrical only), `--exclude-topics /tf_static` ineffective on MCAP (#2354), recorder can silently stop at ~10 MB/s (#2463, we write ~14). Snapshot ring is byte-bounded and drops transient-local topics. |
| **evo** | core (dev tool) | 2026-09-08 | pure Python; reads zstd MCAP | APE/RPE with SE(3) alignment against an RTK + heading PoseWithCovarianceStamped reference (NavSatFix alone scores no yaw). Interpolation sync is on master (bc52497d), not in PyPI 1.37.1. Umeyama alignment also hides a constant lever-arm error. |
| **Foxglove** (app + bridge + px4_converter) | core (observation) | 2026-10-02 | apt ros-jazzy-foxglove-bridge | Bridge configured without client publishing: cannot become a control path. Local files need a Developer seat (Free plan has 3); Lichtblick is the account-free fallback. `.ulg` and MCAP cannot be merged in one timeline: a ulog→MCAP UTC converter is required. |
| **grid_map** (+ filters, cv, sdf) | core | ROS 2 branches static since 2025-09; 2.2.2 on Jazzy apt | arm64 debs | Frozen API since 2022; one maintainer "purely on maintenance mode" (2026-04). No live point-cloud accumulation: an own node is required (E1R, per-point TF, min-z per cell, resolution tied to AGL). Slope/roughness are MathExpression filters, inpainting needs `grid_map_cv`. Pin or vendor the jazzy tag. |
| **GCOPTER / MINCO** headers (ZJU) | core (library) | **dead since 2023-06** | minor port (two one-line patches for C++17/GCC 13) | Thirteen Eigen-only headers; output `Trajectory<5>` maps 1:1 onto TrajectorySetpoint feed-forward. Unfixed defects: quickhull producing illegal polytopes on an Orin NX (#28), C++17 compile breaks (#16). Dense `VoxelMap` cannot take the 5 cm global map (2.4e9 voxels): feed FIRI points directly or use a local window. No notion of unknown space. Racer-tuned physical parameters must be re-identified. |
| **nvblox / isaac_ros_nvblox 4.6** (core 0.0.10) | core, gated | 2026-09-22 (repo); release-4.6 since 2026-08-19 | arm64 debs for Jazzy/JetPack 7.2 | The only GPU ESDF packaged for this stack; already bench-proven here (GPU 0–2 %, 1.2 GB static). **One PointCloud2 input per node** and a 360° range-image LiDAR model (the Avia fits at 1800 × 400; the E1R does not). De-skew needs a per-point `t` in ns (missing field = CHECK abort). Two open unanswered 3D-ESDF bugs on Jazzy/CUDA 13 (#154, nvblox #122): bench gate. Jazzy line frozen at 4.6 (5.0 is Lyrical, NITROS removed); the ETH Isaac-free fork shows the dependency is removable in a day but is a stale 4.0 snapshot. Apache-2.0 despite GitHub's NOASSERTION label. |
| **aerial-autonomy-stack** (JacopoPan) | reference | 2026-10-02 | Jazzy inside Docker only | Closest published wiring to this exact board (Holybro Jetson Baseboard, 6X, Orin NX, JetPack 7.2.1, PX4 1.17 over Ethernet XRCE). Read its two PX4 nodes (OffboardFlag gating, DO_SET_MODE sequencing, versioned topic names and QoS, external-mode template) line by line; do not depend on it (tmuxinator monolith, privileged containers, no launch files, no tests, branch-head deps, no PX4 flight evidence: its PX4 params file literally says "TODO"). |
| **PX4 external-mode firmware side** (ModeManagement, externalChecks) | core (read) | v1.17.0 | native | Polls each registered component every 300 ms; a reply must arrive within the same cycle; flagged after the 4th consecutive miss; an unselected unresponsive mode is **deregistered**; `can_arm_and_run=false` is scoped to the mode's own NavModes group, so a permanently failing arming check refuses entry to that mode alone. Eight registration slots. |

## 2. Per-lens summaries (as written by the survey, 2026-10-03)

## Lens: planning

State of the lens (verified 2026-10-03): LiDAR-first multirotor planning has converged on one pipeline — robocentric occupancy map (or raw kd-tree of registered points) -> front-end search (A*/JPS3D/kinodynamic A*) -> convex safe-flight-corridor (FIRI/CIRI/DecompUtil) -> sparse spline back-end (MINCO, Hermite, B-spline) -> trajectory sampled to position/velocity/acceleration/yaw references for a tracker. Nobody ships a Jazzy-native, PX4-XRCE-native planner; every candidate emits quadrotor_msgs::PositionCommand-style references into its own (often unreleased) controller. That is fine for this aircraft: PX4 TrajectorySetpoint takes position plus non-NaN velocity/acceleration as feed-forward plus yaw/yawspeed (https://docs.px4.io/main/en/flight_modes/offboard.html), so a 50-line "shadow tracker" node closes the gap and PX4 keeps position control; nothing here needs to fight MPC_*.

Recommendation (3 pulls): (1) Plumbing: Auterion px4-ros2-interface-lib (release/1.17 branch, active 2026-09) — register an external mode via the already-bridged register_ext_component_request/arming_check/config_overrides topics and never select it; the shadow tracker publishes TrajectorySetpoint on a /nav/shadow/* topic that is bagged and diffed against /px4/local_position. (2) First planner core: hku-mars SUPER (ROG-Map + CIRI + MINCO, Science Robotics 2025). Its inputs are exactly /cloud_registered + LIO odometry in the world frame, the C++ core is ROS-agnostic with ros1/ros2 adapters (Foxy upstream, Humble community fork), replan_rate 15 Hz, planning_horizon 7 m, 10 cm ROG-Map with free-space raycasting that lio_map lacks. Hazards: the authors' MPC is unreleased (issue #5/#57), the ROS 2 adapter needs a Jazzy pass, GPL/none licence irrelevant here. (3) Shared back-end library: ZJU GCOPTER headers (minco.hpp, firi.hpp, lbfgs.hpp, sfc_gen.hpp; MIT) — also what SUPER, LA-Planner and YOPO-MINCO build on, so one vendored dependency serves several planners.

Second-tier: MIGHTY (MIT ACL, ROS 2 Humble, LiDAR-born, Jetson build path, RA-L 2026, active 2026-08) is the best living ROS 2 planner if dynamic obstacles matter, but drags in acl-mapping (GitLab) and dynus_interfaces. dyn_small_obs_avoidance-ros2 is the only Jazzy/aarch64-tested LiDAR kinodynamic planner found — one week old, zero stars, but tiny and testable; a good cheap comparison planner. EGO-Swarm ros2_version (Humble, frozen 2025-03) suits a D555-only shadow planner.

Avoid: frameworks that own the aircraft (Aerostack2, MRS ROS 2, kr_autonomous_flight — all route PX4 through mavros or their own attitude controller), Gurobi-bound DYNUS (licence token and WLS network dependency in the field), MPPI (no mature ROS 2 quadrotor node; MPPI-Generic is CUDA 10-12 era, untested on CUDA 13.2; the GPU is better spent on perception), and the ROS 1 legacy (Fast-Planner, FASTER/MADER/PANTHER, Agilicious/rpg_mpc, mav_trajectory_generation, PX4-Avoidance archived; PX4 removed the trajectory_waypoint path-planning interface in v1.15).

Compute: SUPER/MIGHTY report 10-20 ms per replan on desktop x86; expect 2-3x on the A78AE, i.e. <0.5 core at 10-15 Hz if written in C++ (not rclpy). Trade-off to decide first: planner map (ROG-Map raycast occupancy) versus the existing lio_map voxels — the planner choice drives the mapping lens.

**Gaps:**
- No candidate was build-tested on ROS 2 Jazzy / Ubuntu 24.04 / aarch64 / CUDA 13.2; all ROS and platform statements come from READMEs, package.xml files and tree listings.
- SUPER: per-replan CPU time on hardware and the status of the promised MPC release were not confirmed beyond issues #5 (2025-04: 'later this year') and #57 (open 2026-01); deepwiki was rate-limited, so parameters were read from click_smooth_ros2.yaml.
- MIGHTY and DYNUS: how MIT ACL feeds PX4 on hardware is not in the repositories (an ACL tracker outside the repo); replan rates are not stated as fixed numbers in the papers.
- Aerostack2 as2_platform_pixhawk transport (px4_msgs/XRCE vs mavros) and the ROS distro inside aerial-autonomy-stack's containers were not verified.
- MPPI: no mature ROS 2 quadrotor MPPI node was found; MPPI-Generic compatibility with CUDA 13.2 is unknown (README: CUDA 10+, 11.7 recommended).
- ego-planner-swarm ros2_version (Humble, frozen 2025-03) was not checked for Jazzy build issues; EGO-Planner-v2 has no ROS 2 branch.
- kr_autonomous_flight ros2_dev CI status not checked; its README describes itself as unverified.
- 2026 papers without verified code releases: LOONG (arXiv 2601.07434), SANDO (2604.07599), Neural-Primitive (2608.20948; code not located); CORTO-Planner (RA-L 2026) is ROS 1 Noetic with 3 stars and was not evaluated further.
- Exploration planners (GBPlanner3/OmniPlanner, confirmed ROS 1 Noetic; EPIC, FALCON, FUEL, mav_active_3d_planning) were treated as out of this lens; Primitive-Planner (ROS 1 Noetic, sub-ms primitive library) noted but not rated as a candidate.
- ROG-Map's ROS 2 version exists only inside SUPER (upstream README); the standalone vittpall/ROG-Map fork has no commits since 2025-03.
- Orin NX timing estimates (2-3x slower than desktop x86 per replan) are extrapolations, not measurements.

## Lens: mapping

State of the lens for this aircraft: the only maintained, GPU, ROS 2 Jazzy, aarch64-native option that gives a planner free space and an ESDF is nvblox, and this project has already run it on this Jetson (roles/nvblox, bench 2026-09-27: Avia through the Lidar range-image model plus D555 depth at 5 cm, 3D ESDF via GetEsdfAndGradient, GPU 0-2 %, RSS 1.2 GB). What stopped it was the pose, not nvblox; FAST-LIO poses anchored under odom now exist. Everything else in the lens is either CPU research code on ROS 1 (ROG-Map ROS 1 / SUPER ROS 2 on Foxy "unstable", wavemap, UFOMap v1, voxblox, D-Map, FIESTA, EGO grid_map), dead GPU code (OHM master 2023, GIE-mapping 2023, CUDA 9-11), occupancy-only without a distance field (OctoMap, Bonxai, VDB-Mapping), or not a map at all (ikd-Tree, i-Octree: k-NN point stores, which is what lio_map already is in numpy). The 2025-2026 novelties are CPU and LiDAR-specific: hku-mars BDM (IJRR 2026, code "very soon"), D-BDM (arXiv 2026-04), DB-TSDF (ICRA 2026, Humble), Parallel OctoMapping (no code found); none ship an ESDF or a Jazzy node.

Memory is the real constraint, not compute: free space is the hog. nvblox allocates 8^3 blocks along every ray out to the integration distance (TSDF 8 B + ESDF ~20 B per voxel), so at 10 cm a 10 m-radius tube costs roughly 8-9 GB per km of flight path; a planning map must be local (map_clearing_radius_m, 10-20 cm voxels, ESDF 3D at 1-2 Hz) while lio_map stays the global surface product. ROG-Map's answer (5 cm, 30x30x12 m sliding grid, 5.96 ms/frame on CPU) is the same idea without a GPU.

Recommendation: Stage 1 = resurrect the nvblox role as the shadow planning map, fed by FAST-LIO TF (odom -> camera_init -> body) and /avia/points with a per-point t field (nvblox's de-skew CHECKs for it), 10 cm voxels, LiDAR grid 1800x400 (0.2 deg/px, the role's own bound theta < voxel/(sqrt2 R)), integration 20-30 m, clearing radius 60-80 m, esdf_mode 3d plus a 2D slice at flight altitude for QGC/Foxglove, unobserved_esdf_policy chosen deliberately (v0.0.10), diagnostics on blocks/rates/ESDF age, and bag replays against lio_map to score free-space correctness and stray handling. Stay on the Isaac ROS 4.6 debs: 5.0 moved to ROS 2 Lyrical, so the Jazzy nvblox line is frozen (mitigation: ethz-mrl/nvblox_ros2 Isaac-free fork, Apache-2.0 nvblox_ros). Keep ROG-Map as the CPU fallback/algorithm reference (fetch-at-build like FAST-LIO, GPL-3; rewrite its small ROS 2 glue for Jazzy), not as the first path. Do not add ray casting to lio_map's Python; do not port ROS 1 research maps in Stage 1. elevation_mapping_cupy (Jazzy line, 2026) is a later, separate 2.5D landing-zone product for the down-looking E1R, with a CuPy/torch-on-CUDA-13 aarch64 dependency hazard.

**Gaps:**
- Did not verify whether a single isaac_ros_nvblox 4.6 nvblox_node accepts more than one LiDAR point cloud input (Avia + E1R) or whether the E1R must enter as a depth image / second node; the parameters page lists one lidar model set.
- Did not confirm the exact nvblox 4.6 behaviour and GPU/RAM cost on the Orin NX with a moving pose at 10 cm and 1800x400 LiDAR grid; the only measurement is the project's static bench run (roles/nvblox README, 2026-09-27). My per-km memory figures are arithmetic from the voxel struct sizes (TsdfVoxel 2 floats, EsdfVoxel ~20 B) and 8^3 blocks, not measured.
- nvblox core licence: the GitHub API reports NOASSERTION for nvidia-isaac/nvblox; I did not read the LICENSE file (isaac_ros_nvblox is Apache-2.0). Irrelevant for non-commercial use but worth a glance before vendoring.
- The 'LiDAR dynamics support with motion compensation' mentioned in a search summary for isaac_ros_nvblox 2026 updates was not confirmed against the release notes text.
- hku-mars/BDM code is unreleased ('very soon' as of 2026-03); D-BDM (arXiv 2026-04) has no code link found. Parallel OctoMapping (arXiv 2603.22508) and HEPP (arXiv 2505.17438) appear to have no public code.
- DB-TSDF (robotics-upo, ICRA 2026, CPU directional-bitmask TSDF, ROS 2 Humble, MIT, last commit 2026-06) was found but not fully assessed: no ESDF, Humble-only, x86-documented; left out of the candidate list for space.
- Unofficial ROS 2 forks of EGO-Planner / Fast-Planner, qza36/ROG-Map-ROS2 and scorpio-robot/rog_map were not built or read beyond their README/metadata; stability on Jazzy/aarch64 is unknown.
- elevation_mapping_cupy's CuPy/torch availability for CUDA 13.2 on Tegra aarch64 (JetPack 7.2) was not verified; the project documents cupy-cuda12x and recommends Docker.
- OHM's 2026 side branches (codex/ndt-atomic-covariance-update) suggest some CSIRO activity but master is 2023; I did not check whether a release or CUDA 12/13 support is planned.
- supereight2, wavemap, UFO and vdb_mapping aarch64 builds were not attempted; dependency availability was checked only for OpenVDB (noble arm64 10.0.1).
- The nvblox core documentation site (nvidia-isaac.github.io/nvblox) returned 404 on the page I tried, so the Python/pip install story for the core library on Jetson was not verified; the project's path (Isaac ROS 4.6 debs) is verified by roles/isaac_ros.

## Lens: px4_plumbing

Stage 1 cannot rely on "publishing setpoints PX4 ignores": in v1.17 there is no such path. mc_pos_control consumes the single uORB topic trajectory_setpoint with no origin check whenever flag_multicopter_position_control_enabled (Position/Altitude modes), FlightModeManager publishes the same topic, and the only gate is a timestamp older than position-control start (MulticopterPositionControl.cpp:429-448); the agent's /fmu/in/trajectory_setpoint reader maps straight onto it, and goto_setpoint runs whenever no trajectory_setpoint arrived that cycle. So the shadow stack publishes only in its own namespace (/shadow/trajectory_setpoint as px4_msgs in NED plus an ENU twin, /shadow/offboard_control_mode, /shadow/obstacle_distance), never offboard_control_mode (it makes Offboard "available" and GCS-selectable) and never vehicle_command. Make it provable: a guard node that counts publishers on every /fmu/in/* topic and goes ERROR on any, and, since the aircraft already flies a one-commit firmware branch, delete the setpoint, command and odometry subscriptions from dds_topics.yaml so the FC has no DDS input path at all; re-adding them is the auditable Stage 2 flash.

Stage 2 engagement, explicit and reversible: Offboard entered only by the pilot through RC_MAP_OFFB_SW on a spare companion channel (12/13), never by vehicle_command; COM_RC_OVERRIDE=3 so stick movement beyond COM_RC_STICK_OV (a rate test on all four sticks) drops to Position; COM_OBL_RC_ACT 0 with COM_OF_LOSS_T 1 s on stream loss; kill switch (ch9) last. The external-mode route (interface-lib release/1.17) adds arming checks, mode requirements, a named mode and Hold fallback after 3x300 ms of missed arming_check_reply, but in v1.17 registration only works disarmed (COM_MODE_ARM_CHK 0), the mode is GCS-selectable the moment it registers (no not_user_selectable until 1.18), RC selection needs a COM_FLTMODEx slot set to 100+index (all six are taken), and stick override cannot fire because neither PX4 nor the library sets flag_control_auto/offboard_enabled for external modes. Offboard+RC switch first; external mode later.

Record for scoring: ulog already holds trajectory_setpoint (5 Hz only), manual_control_setpoint (full rate via profile bit 16), vehicle_local_position(_setpoint) 10 Hz, vehicle_control_mode, action_request, vehicle_command, failsafe_flags, timesync_status, pps_capture; add bit 7 (SDLOG_PROFILE 147) for obstacle_distance/collision_constraints. The MCAP lacks trajectory_setpoint and vehicle_local_position_setpoint: add them to dds_topics.yaml publications at 50 Hz so the pilot's planner-level intent lands in UTC beside /shadow/*.

Pitfalls that bite first: NED/FRD vs ENU/FLU with yaw_ned = pi/2 - yaw_enu; PX4 local-frame resets (xy/z/heading_reset_counter) that mc_pos_control compensates for its own setpoints only; px4_local has no TF into map; ulog is boot-time microseconds and the FC has no GNSS, so timesync_status/pps_capture is the only UTC anchor; EKF2 subtracts EKF2_EV_DELAY and refuses EV yaw while GNSS is active unless pose_frame is NED, so LIO goes in as FRD with reset_counter bumped on every watchdog rollback.

**Gaps:**
- The docs' claim that the OffboardControlMode stream must run for at least one second before Offboard can be selected was not found in v1.17.0 offboardCheck.cpp (which only gates on recency within COM_OF_LOSS_T); another check site or outdated docs are possible.
- Did not confirm that deleting subscriptions from dds_topics.yaml removes the uORB input path with a rebuild alone (the uxrce_dds_client generates its topic table from that YAML at build time; not re-verified against the CMake/EmPy generator).
- Did not verify EKF2 behaviour with a 10 Hz FAST-LIO vehicle_visual_odometry stream (the docs say 30-50 Hz; the source only enforces a minimum interval and a 1 s no-aid timeout).
- Did not check whether the Siyi UniRC 7 Pro's QGroundControl build (Android) shows external modes; the three QGC issues are closed on desktop builds and v5.1.5 is the latest release.
- Did not fetch the arm64 Jazzy Debian package location for px4-ros2-interface-lib (CI matrix shows linux/arm64 + jazzy, publication target unverified).
- pyulog's ulog2ros2bag storage plugin (sqlite3 vs mcap) and compatibility with Jazzy's rosbag2 API not verified.
- Did not test rclpy default-QoS publishers against the agent's /fmu/in readers on this install (known to work in the official examples).
- Only indirect reading of PX4 1.18-beta changes (RegisterExtComponentRequestV1 not_user_selectable, interface-lib SetpointConfig mechanism); 1.18 is beta/rc as of 2026-10 and not recommended for this aircraft.
- No 2025-2026 open-source project implementing an explicit shadow/advisory mode for PX4 was found; the pattern appears to be novel for this stack.

## Lens: frameworks

Nothing in this lens should be adopted wholesale. Every full stack alive on ROS 2 Jazzy either owns the aircraft through Docker plus its own middleware choice (aerial-autonomy-stack: containers + Zenoh; AirStack: NVIDIA-container-only, "ALPHA, intended for internal AirLab use"; MRS 2.x: Zenoh RMW recommended) or still reaches PX4 through MAVROS (MRS mrs_uav_px4_api, AirStack mavros_interface, kr_autonomous_flight so3cmd_to_mavros, NTNU Unified Autonomy Stack), which the project rule forbids. The DARPA SubT lineage (CMU Explorer aerial environment, CERBERUS gbplanner3, NeBula LOCUS/LAMP) is ROS 1 Noetic; CMU's Jazzy ports are ground-only. Agilicious (Betaflight, last commit 2023), Clover (RPi/mavros), PX4-Avoidance (archived 2024-08-01) and OpenUAV (2019) are dead ends for this aircraft; Auterion's SDK is the closed comparison point (ROS 2 in containers on its PX4 fork).

The one component to take as a library is Auterion's px4-ros2-interface-lib (2.2.3, 2026-09-23; release/1.17 branch; Jazzy CI; arm64 debs from its release workflow; not on the ROS build farm, so vendor at a pin). It gives Stage 1 exactly what was asked: register an external mode over the /fmu/in topics the bridge already exposes (register_ext_component_request, arming_check_reply, config_control_setpoints, mode_completed), set user_selectable=false so neither RC nor QGC can hand it control, and use checkArmingAndRunConditions as a heartbeat into the FC's health reporter. PX4 polls each registered component every 300 ms, declares it unresponsive after 3 misses, and scopes can_arm_and_run=false and "Mode is unresponsive" to that mode's group only; a standalone check with no mode scopes to NavModes::All and would block arming everywhere, so always attach the check to the shadow mode. Eight registration slots exist. Raw Offboard (what Aerostack2's pixhawk plugin uses) is the fallback: ignored unless Offboard is selected, but no health channel, and PX4 docs now steer people to the library.

Borrow, don't adopt: Aerostack2's PlatformStatus state machine and ControlMode enum as interface definitions plus its behavior-server pattern; MRS's ControlManager/tracker split and RC escalating failsafe as vocabulary for the later controller stage; AirStack's drone_safety_monitor (a ~150-line state-estimate-timeout node) as the first safety monitor to write; aerial-autonomy-stack's autopilot_interface actions, px4_custom_mode_template and py_trees mission as the wiring on identical hardware; UAS's CBF last-resort filter layering for the eventual control path.

nav2: costmaps/planners no; BehaviorTree.CPP 4.10 (ros-jazzy apt), BehaviorTree.ROS2 0.3, nav2_lifecycle_manager (bond heartbeat, 4 s timeout, respawn reconnection) and the nav2_util LifecycleNode/pluginlib pattern yes. py_trees_ros is the lighter Python executive if the repo stays rclpy-first; FlexBE, YASMIN and SMACC2 (which grew a cl_px4_mr XRCE client in 2026) are in Jazzy but add a second framework.

**Gaps:**
- NeBula (JPL CoSTAR) is not an open stack: only LOCUS and LAMP (ROS 1, last pushed 2023-04/2023-10) are public under NeBula-Autonomy; OpenUAV (Open-UAV/openuav-playground, 2019) is dead and openuav-turbovnc is a cloud sim environment; Auterion SDK/AuterionOS is closed (ROS 2 apps in containers on its PX4 fork). None assessed beyond confirming that; no entries created for them.
- Did not build px4_ros2_cpp against the owner's one-commit custom PX4 v1.17 branch; the message-hash compatibility check (scripts/check-message-compatibility.py or the on-register check) must be run, and whether the pps_capture message addition changes any checked message is unverified.
- FC-side behaviour of ModeBase::Settings user_selectable=false (hiding the mode from QGC's selector and RC assignment) was read from the library header comment and the externalChecks source, not flight-tested on v1.17.
- Aerostack2 concept docs returned 404; platform-state-machine and ControlMode details come from the message and source files. Quality of the Jazzy binaries (1.1.3) on aarch64 untested; main development still targets Humble.
- MRS ROS 2: did not confirm whether the Jazzy PPA ships arm64 or whether mrs_uav_hw_api could be implemented over px4_msgs without mavros (no such plugin exists upstream); documentation carries 'may be still outdated' warnings.
- Unified Autonomy Stack: ROS 2 distro inside its Dockerfiles, aarch64/Jetson support, and the exact field-test hardware were not confirmed; FC path inferred from ws_mavros.repos and the navigation docs page.
- AirStack: assessment of maturity relies on docs/release notes; did not inspect the 0.20 'stacks'/'fleets' layout or the develop branch.
- GitHub API via WebFetch returned 403 (unauthenticated rate limit); metadata was pulled with the authenticated gh CLI instead. Last-activity months are from pushed_at or last commit on the default branch, not per-package.
- Not examined: ModalAI VOXL SDK, Skydio/closed stacks, XTDrone (ROS 1 PX4 sim), CrazyChoir/Crazyswarm2 (Crazyflie), PegasusSimulator/Isaac Sim (simulation lens), ETH ASL mav_control_rw (2019, ROS 1), ethz-asl mav_active_3d_planning (ROS 1, planning lens), HKUST FUEL / ZJU EGO-Planner (planning lens).
- SMACC2 cl_px4_mr and auto-apms-px4 were read only at package/README level; neither has been run.

## Lens: learning

On this aircraft learned components earn their place only where geometry does not already answer the question. Two LiDARs, hardware stereo and RTK make depth, odometry and mapping classical problems that are solved today; the learned-depth families (Depth Anything 3, ESS, Fast-FoundationStereo, Metric3D, UniDepth) compete with sensors you already carry and should be treated as optional dense-fill, not as perception. The exceptions are semantics (what a surface or object is), inertial dead-reckoning across LIO/GNSS dropouts, and telemetry anomaly detection: there learning clearly beats classical and the coming flights produce the training data for free (PTP-aligned LiDAR projected into D555 frames = depth and terrain-geometry labels; RTK + FAST-LIO poses = ground truth for Air-IO; nominal-flight MCAP + ulog = unsupervised anomaly training set).

Platform reality: JetPack 7.2 ships CUDA 13.2, cuDNN 9.20, TensorRT 10.16.2 on Orin NX. Isaac ROS 4.6.0 (2026-08-18) is the first and last line that pairs Jazzy with Orin/JetPack 7.2; 5.0 (2026-09-21) moved to ROS 2 Lyrical and removed NITROS, so pin 4.6. Ultralytics documents a native TensorRT 10.x path on JetPack 7.2 (TRT 11.2 wheels do not support JetPack). TensorRT Edge-LLM runs INT4 VLMs on Orin NX under JetPack 7.2: Qwen3-VL-2B needs ~0.8 s prefill and decodes 52 tok/s, so a VLM is a mission-level advisor (1-3 s per query, 2-5 GB RAM), never in the loop. CPU, not GPU, is the scarce resource (5.2 cores busy, mostly rclpy overhead); every learned node must be a C++ TensorRT node, and training/export belongs on a desktop GPU, not the aircraft.

Recommendation for Stage 1 shadow mode: (1) a telemetry anomaly model (LSTM-AE/IsolationForest style) beside sensor-health, trained on the first nominal flights; (2) Air-IO-style learned IMU odometry as a shadow source next to FAST-LIO/cuVSLAM, trained on RTK+LIO truth; (3) SegFormer-B0 on D555 RGB (~10-14 ms FP16 on Orin NX, scaled from AGX Orin 9.4 ms) for terrain/landing-zone classes, auto-labelled from E1R/Avia geometry; (4) YOLO26n (4.1 ms FP16 on Orin NX) for people/vehicle awareness. For a learned local planner, NavRL is the only architecture that fits a LiDAR aircraft (ray-cast observation from the lio_map voxel map, velocity output, classical velocity-obstacle shield, 7 ms policy on Orin NX) and is cheap to run in shadow against the pilot's inputs; vendor the algorithm, not the Humble/mavros stack. Skip learned occupancy/scene completion (nothing is real-time on Orin at Avia rates) and VLA models (none run onboard PX4). The main trade-off is complexity against value: each learned node costs CPU and a maintenance surface for roughly zero gain in the LiDAR mapping mission, so gate each on a measured deficit (D555 depth holes, LIO divergence duration, missed anomalies) observed in the recorded flights.

**Gaps:**
- No published Orin NX numbers exist for ESS, Fast-FoundationStereo, DA3METRIC-LARGE, PromptDA or Air-IO; all Orin NX figures for those are scaled from AGX Orin, Orin Nano Super or Thor and must be measured on the aircraft.
- The MDPI Sensors 2025 LSTM-AE anomaly paper was behind a captcha; whether it releases code was not verified.
- Ultralytics' claimed native TensorRT 10.x path on JetPack 7.2 was not exercised here; the guide text is ambiguous about PyTorch wheels (TensorRT 11.2 'does not support JetPack').
- Whether the D555's on-camera depth is good enough outdoors (emitter off, sunlight) to make ESS redundant needs one recorded flight compared against the Avia; this decides the stereo-network question.
- Isaac ROS 5.0 (2026-09) moved to ROS 2 Lyrical and removed NITROS; 4.6 is the last Jazzy line for Orin and will stop receiving model/package updates, a long-term pin risk for ESS/Segformer/cuVSLAM debs.
- NavRL++ (2026) code status inside the NavRL repo was not confirmed; last repo push is 2025-07.
- UZH LAFR and learning_on_the_fly (2026) and LeCAR Agile-but-Safe were checked but excluded as learned controllers/legged work outside a no-control shadow stage; UZH Swift remains closed source; agile_autonomy is ROS1/GPL and dead since 2023.
- RNIN-VIO (seed) could not be located as a maintained repository.
- Did not survey learned LiDAR odometry (DFLIOM, LIR-LIVO) in depth; nothing found that is Jetson-tested or clearly better than FAST-LIO2 on this sensor suite.

## Lens: safety

Runtime assurance for this aircraft already lives in the FC: PX4 1.17 owns modes, failsafes, geofence (GF_ACTION 3), RC priority (COM_RC_IN_MODE 5, COM_RC_OVERRIDE 1), kill (ch 9) and offboard-loss handling (COM_OF_LOSS_T 1 s -> Position). Stage 1's job is to be provably absent from that loop; stage 2+ adds one independent companion monitor in the ASTM F3269 pattern (advanced function / monitor / safe fallback = PX4 Position mode + pilot).

(a) Inert stage 1. Run every shadow node under a uav-shadow.target in a second ROS_DOMAIN_ID. PX4's XRCE client only exists in UXRCE_DDS_DOM_ID, so nothing published in the shadow domain can reach /fmu/in. One ros2/domain_bridge (Jazzy binary, 2026-04) with a YAML allow-list forwards inputs in (/fmu/out/*, /px4/*, /lio/*, the map) and /shadow/* out for the recorder; a target-side test asserts no rule bridges ^/fmu/in/ and that `ros2 topic info -v` shows no publisher on any /fmu/in topic. The shadow planner publishes real px4_msgs TrajectorySetpoint + OffboardControlMode on /shadow/..., so serialization, rates and heartbeat timing are exercised. Keep px4_ros2_interface_lib out of stage 1: registration is itself a write. SROS2 (Fast DDS agent vs Cyclone nodes: cross-vendor security unsupported) and rmw_zenoh ACLs are heavier than this needs.

(b) Scoring. Replay bags (`ros2 bag play --clock --topics ...`) into the shadow domain, score offline from MCAP: ADE/FDE at 1/3/5 s between the setpoint-integrated path and flown /px4/odometry; would-have-collided by sweeping that path (radius + margin) through lio_map at matched time (min clearance, TTC); smoothness (accel/jerk/yaw-rate against MPC_ACC_HOR 3, MPC_JERK_AUTO 4); latency (newest-input stamp -> MCAP log_time, p50/p99, heartbeat gaps vs COM_OF_LOSS_T); intervention proxies (how often a shadow CBF filter would have altered manual_control_setpoint, which is on /fmu/out). PX4's own reference (trajectory_setpoint) is not on DDS: read it from ulog or add it to dds_topics.yaml on the custom branch. nuPlan/WOSAC and Autoware's planning_evaluator are the metric references; no UAV shadow-scoring tool exists.

(c) Engagement. 2a: feed /fmu/in/obstacle_distance from the LiDAR; collision prevention (needs MPC_POS_MODE 4, already set) slows the pilot, pilot keeps control. 2b: register an external "Assist" mode (interface lib, Jazzy CI) on a COM_FLTMODE slot (index from `commander status`), requirements manual_control + local_position, arming-check reporter; the mode filters sticks with a composite CBF (NTNU algorithm, reimplemented on px4_msgs). Takeover: any RC mode switch, stick override, kill ch 9; PX4 falls back to an internal mode if the node dies. 3: planner setpoints in that mode, with the companion monitor (Implicit-Simplex: backup-trajectory feasibility) and PX4 built-ins as independent layers.

(d) Existing pieces: domain_bridge, rosbag2, topic_tools, px4-ros2-interface-lib, PX4 built-ins; algorithms from composite_cbf and ACT3 RTA; GSN safety case via gsn2x.

**Gaps:**
- Not verified whether COM_RC_OVERRIDE stick takeover applies to external (ROS 2) modes as it does to Auto/Offboard; check PX4 commander source before stage 2b and otherwise rely on RC mode switch plus kill.
- Did not confirm a release/1.17 branch of px4-ros2-interface-lib exists (README cites release/1.16 as the example); pin main or the matching branch after a message-hash check against the custom 1.17 firmware.
- JAX CUDA wheels for aarch64 against CUDA 13.2 (cbfkit, ACT3 RTA GPU use) not verified; CPU JAX on aarch64 is fine.
- docs.ros.org blocked fetches (Anubis), so SROS2 statements come from the ros2_documentation jazzy source; Cyclone DDS security interop with the Fast DDS XRCE agent was not tested, only the docs' 'secure communication between vendors is not supported'.
- No UAV-specific open-source shadow-mode scoring tool exists; the AV metric suites are the nearest reference and the evaluator must be written (small Python over MCAP).
- domain_bridge CPU cost on the Orin NX for map-rate topics was not measured.
- Assumed trajectory_setpoint / vehicle_local_position_setpoint are in the ulog under SDLOG_PROFILE 19 (PX4 default logging); not verified against this FC's logs.
- ICAROUS development currency not fully checked (last code commit 2022-01, docs commit 2026-09); PolyCARP's standalone build on aarch64 not tested.
- Groot2 aarch64 AppImage listed on the vendor page was not tested on JetPack 7.2.
- PX4 failure-injection support matrix per unit on real fmu-v6x hardware was not enumerated beyond the docs' statement that some units apply only in SITL.

## Lens: simulation

State of the lens for THIS aircraft: the only simulation path that is official, CI-tested on PX4 v1.17 and reproduces the real plumbing (uXRCE-DDS into /fmu/*) is PX4 SITL, with either Gazebo Harmonic (the Jazzy-paired Gazebo) or SIH, PX4's in-tree physics. Everything else is pinned to old stacks (Pegasus: Isaac Sim 5.1 + PX4 1.14.3 + Ubuntu 22.04, Isaac Sim 6 migration still an open issue; MARSIM's ROS 2 port: Humble, own dynamics, no PX4), Unreal-sized (ProjectAirSim, Cosys-AirSim: Humble, Windows/Linux GPU), archived (Colosseum, 2026-07), dead (Flightmare and Agilicious 2023, RotorS 2021, rpg_trajectory_evaluation 2022) or RL-only on an EOL backend (Aerial Gym on Isaac Gym). PX4 HITL is Gazebo Classic only and "community supported": skip it.

Realistic split. Mac: PX4 SITL built from the owner's v1.17 branch (PX4's pixi-locked macOS environment, CI on Apple Silicon) plus SIH or a MuJoCo lockstep backend, for controller, mode-handoff and failsafe plumbing. Gazebo's rendering-based sensors (gpu_lidar, cameras) have a long history of not working on macOS, so no LiDAR simulation there. Jetson: SIH on the real Pixhawk 6X (SYS_HITL=2) or SITL on the Jetson itself (arm64 Ubuntu 24.04 builds are proven by PX4's official arm64 images; no v1.17 debs exist, v1.18 only), so the shadow nodes see the true XRCE topic set over the true Ethernet path with zero extra hardware. Linux NVIDIA workstation: the only place for Livox-pattern LiDAR simulation (KangjianPeng's gz-sim Harmonic plugin with an Avia pattern and CustomMsg output, Humble today and a small port to Jazzy; RGL with Livox presets and custom patterns that can encode the E1R's 120x90 flash grid, OptiX so not on Jetson; MARSIM rendering depth from the aircraft's own lio_map PCDs). Isaac Sim 6 wants an RTX 4080-class GPU and has no maintained PX4 1.17 bridge.

Replay fraction: roughly three quarters of mapper, ESDF, free-space and planner work is open-loop given a trajectory and is replay work on recorded flights (ros2 bag play --clock with use_sim_time; MCAP is Jazzy's default). Only the closed loop needs SIH/SITL. The recorder already keeps /avia/custom, /avia/imu, /e1r/points and the odometries; add the raw /fmu/out px4_msgs at native QoS (so the bridge itself can be re-run), the /fmu/in shadow setpoints, manual_control_setpoint (the pilot as the label), timesync_status and pps_capture, TF, and a ulog-to-bag offset: pyulog's ulog2ros2bag writes boot-time stamps, not UTC, so a small offset tool is needed. Evaluation: evo reads rosbag2 MCAP directly against RTK; Foxglove opens ULog natively (free plan: 10 GB, 3 users); PlotJuggler 4 and rerun both ingest MCAP and ULog on the Mac.

Record now for sim-to-real: actuator_motors/esc_status, thrust and torque setpoints, battery, air data, wind, raw IMU (SDLOG_PROFILE 19 already has the replay and high-rate bits), a PCD of every site via /lio/map/save as a future sim world, and the x950 URDF revision.

**Gaps:**
- Gazebo Harmonic GPU sensors on Apple Silicon: evidence is issue history (gz-sim #960 closed 2023-12) and forum reports, not a test of the current pixi-locked Harmonic 8.10 build; the Embree-based ashduwihch plugin as a CPU LiDAR path on macOS is untested speculation.
- Headless gpu_lidar/Ogre2 rendering on the Jetson Orin NX (EGL, arm64 Gazebo Harmonic) was not verified; only the existence of official arm64 px4-sitl-gazebo images was.
- The KangjianPeng Livox plugin was not built on Jazzy (Humble is the only tested target) and its 6-line Avia pattern fidelity against the real rosette was not checked.
- No simulator anywhere models the RoboSense E1R flash LiDAR (multi-return, 25 Hz frame structure, near-field noise) or the D555's DDS-native transport and host clock model; gz rgbd_camera/gpu_lidar approximations are untested for this stack.
- Cosys-AirSim's ROS 2 distro page returned 404; the distro is inferred (Humble-era) not confirmed. ProjectAirSim's PX4 topic path into ROS 2 was not verified.
- PlotJuggler macOS packaging and the rosbag2_storage_rrd plugin's supported distro are not stated in the sources fetched.
- The foxglove-sdk ROS 2 bridge's Jazzy release was not fetched directly (the aircraft already runs a bridge, so taken as working).
- Building PX4 SITL from the owner's one-commit v1.17 branch on arm64 Ubuntu 24.04 (Jetson) or macOS was not attempted; official arm64 images are v1.18 only, and the docs' pre-built .deb packages were not found on any v1.17 or v1.18 GitHub release (Docker Hub only).
- SIH on the real Pixhawk 6X with uXRCE over the baseboard Ethernet is documented only as 'community supported'; not verified on this FC, and the parameter isolation needed to keep it away from flight params was not designed.
- Whether timesync_status (the ulog-to-UTC offset source) is logged under SDLOG_PROFILE 19 was not checked; the alignment tool itself does not exist and must be written.
- MARSIM with the aircraft's own 5 cm lio_map PCDs (density at range, frame conventions) was not tried.
- Not evaluated beyond activity checks: OmniDrones, castacks/AirStack, damien-robotsix/rs_isaac_uav_sim, teapotlaboratories/drone-sim, LASER-Robotics/laser_gazebo_resources (Mid-360 CSV patterns + protobuf to PX4 SITL, 1 star), Tfly6/Mid360_px4_sim_plugin, Neomelt/rm_sim_26 (Jazzy + Harmonic + RGL Mid-360, ground robot).

## Lens: tasks

For a LiDAR mapping aircraft the behaviours worth building first are, in order: (1) assured return through mapped space, (2) terrain-following survey with avoidance, (3) continuous emergency-landing-site scoring, (4) bounded-volume exploration. Target following has no mapping use and PX4 already ships Follow Me; skip it.

PX4 sets the baseline: terrain following/hold (MPC_ALT_MODE 1/2) exists only in Position/Altitude modes and needs a distance_sensor; missions have no native terrain following (QGC bakes SRTM heights into waypoints); Collision Prevention works only in Position mode and only in the horizontal plane. Everything beyond that comes from the companion, and the sanctioned ROS 2 path is Auterion's px4-ros2-interface-lib (release/1.17 branch, Jazzy): external modes, mode executors, and replacement of internal RTL with automatic fallback to the built-in RTL if the external mode dies. For Stage 1 register nothing (a registered mode is selectable from QGC): run planner/scorer nodes as pure readers of /px4/* and the lio_map, publish would-be setpoints on non-bridged /shadow/* topics, and let the flight recorder capture them for offline comparison with what PX4 actually flew. Registration, mode executors and BehaviorTree.CPP sequencing are the Stage 2 switch-on.

Open-source state: LiDAR-UAV exploration is lively but ROS 1 research code: FUEL/FALCON (depth cameras, ROS 2 "future work"), EPIC (LiDAR, RAL 2025, Noetic), GBPlanner3/OmniPlanner (active Sept 2026, still Noetic + Gazebo Garden source build), SHIELD and FLARE (late-2025 papers, code not yet published). TARE has a Jazzy/ARM branch but is ground-vehicle oriented. None is a drop-in; take the algorithms (hierarchical local/global planning, observation-quality frontiers on point clouds) into small nodes later. Terrain and landing are far better served: grid_map (Jazzy 2.2.2, apt), elevation_mapping_cupy v2.2.0 (Jazzy, GPU), Patchwork++ (native ROS 2 node, C++17/Eigen) and GroundGrid (ros2-jazzy branch) form a maintained 2.5D stack. No maintained ROS 2 landing-zone package exists; PX4-Avoidance's archived safe_landing_planner is the algorithm reference (grid-binned mean/std of z), best re-expressed as a grid_map_filters chain (slope, roughness, inpainting, minimum patch size). Coverage: Fields2Cover v2.1.0 (2026-09) for polygon lawnmowers, 2D only; QGC Survey already covers the manual case. Scene graphs (Hydra/Khronos: Jazzy but "unstable", RGB-D + TensorRT semantics; Hydra++ hybrid-LiDAR code TBU) are the wrong GPU spend for an outdoor mapping aircraft now.

Map and estimator demands: RTL and avoidance need free space (ray-cast occupancy or an ESDF), not the 4M-point surface map; terrain following and landing need a robot-centric elevation map from the E1R registered with FAST-LIO poses; all need a validated, continuously monitored transform between FAST-LIO's camera_init frame and PX4's local NED origin, which the shadow stage is exactly the place to measure.

**Gaps:**
- SHIELD (arXiv 2512.23972), FLARE (RAL 2025) and Hydra++ (IROS 2026) promise code; none was found published as of 2026-10-03, so they are unrated.
- elevation_mapping_cupy documents CUDA 12 on Jazzy; running it on JetPack 7.2 / CUDA 13.2 with cupy-cuda13x was not tested, only inferred from CuPy's wheel list.
- Patchwork++ Jazzy support rests on the repo README (Humble badge) plus search summaries claiming Jazzy builds; not verified by building on aarch64.
- behaviortree_cpp shows RELEASED for Jazzy on index.ros.org; the arm64 apt binary was not checked from a package list.
- px4-ros2-interface-lib release/1.17 branch exists but its diff against main and against the FC's one-commit custom PX4 branch was not inspected.
- No maintained ROS 2 LiDAR landing-zone package was found; the conclusion 'none exists' is based on several searches, not an exhaustive GitHub crawl. Recent fusion papers (Sci. Rep. 2026) were not checked for code.
- Target-following and PX4 Follow Me were not surveyed beyond noting PX4 ships it; follow_target is not among the bridged topics.
- Framework-class stacks (Aerostack2 Jazzy status, MRS UAV System ros2 branch, kr_autonomous_flight ros2_dev) were only touched; they belong to the frameworks lens.
- TARE/CMU dev-env real-robot topic contract (registered_scan, state_estimation) was not read in detail on the jazzy branch; the aerial development environment (ROS 1, 2024-04) README was not fetched.
- Free-space / ESDF map choices (nvblox vs wavemap vs Bonxai vs own ray-caster) that assured-RTL depends on were deliberately left to the mapping/planning lens; activity dates were collected (wavemap 2024-12, Bonxai 2026-09) but not assessed.

## Lens: compute

Baseline (brief, 2026-10-03, MAXN): 5.2 of 8 A78AE cores busy, GPU 0-36 % (36 only while lidar_view is watched), 4.1 of 15.6 GB, ~15 W. The algorithms are cheap: FAST-LIO2 ~0.1 core (paper: 1.8 ms/scan on an i7-8550U, 5.2 ms on an ARM A73), cuVSLAM 1.8 ms/frame and ~2 % GPU, the full LIO chain ~0.6 core. Roughly 3 of the 5.2 cores are rclpy executor/DDS overhead across ~10 Python nodes, so headroom comes from plumbing, not from dropping sensors.

Budget table (cores / GPU % / RAM / when):
- Stage 1 shadow (offboard mirror, arming-check reply, mode executor, rclcpp): +0.05 / 0 / +0.1 GB, always. Recover first: rclpy.experimental.EventsExecutor on every Python node (author's test 16 %->6 % per 100 Hz node; est. -0.5..-1 core here), then compose the camera, sensor-health, px4-bridge and odometry bridges into one rclcpp component_container with intra-process comms (iRobot: -75 % executor CPU; est. -0.5..-1 core). Target <=4 cores.
- Stage 2 map+planner shadow: nvblox-core ESDF at 5 cm from LiDAR, est. 3-8 ms per 10 Hz update = 5-8 % GPU, 1.2 GB (AGX Orin 0.5-0.8 ms TSDF + 1.5-1.7 ms ESDF; Orin Nano 2-3x slower; the NX GPU is Nano-class). Keep its CPU <0.3 core by never publishing layer markers (the retired role measured 1-2 cores doing that). Planner: EGO-class 0.4-0.8 ms/replan (<0.1 core); SUPER-class 47.5 ms at 15 Hz (~0.7 core). Total est. 5-5.5 cores, GPU 10-15 % (+36 when watched), 6 GB.
- Stage 3 learned perception at 640, Orin NX 16 GB measured: YOLO11n FP16 4.9 / INT8 4.4 ms, 11s 7.6/5.6, 11m 14.5/10.2; RT-DETR ~20 ms (est.); SegFormer-B0 FP16 ~12 ms (est.); ESS light 9.4 / full 26 ms; Depth Anything V2-S 98.6 ms at 480x300. At 10 Hz: small class 5-8 % GPU + 0.5-1 GB leaves ~45 % GPU with the render on; medium (11m/RT-DETR/SegFormer-B0) 12-20 % leaves ~30 %; large (ESS full at 20 Hz = 52 %, DA-S at 5 Hz = 50 %) collides with lidar_view. INT8 CNNs can move to the two DLAs; memory bandwidth (102 GB/s) stays shared.

Cheapest wins, in order: (1) EventsExecutor, one line per node; (2) rclcpp composition of the five hottest nodes; (3) drop nvblox/lio_map marker publishing and keep renders subscription-gated. Power: stay MAXN, not MAXN_SUPER; a full-load Orin NX 16 GB test hit 32 W at 82-85 C (throttle 99 C), Holybro budgets 15-30 W and a 0.35 A fan port; log VDD_IN and tj in every flight bag and set the budget from the hottest flight.

Recommendation on learned vs classical: estimation, mapping, ESDF and planning stay classical (sub-core); spend the GPU on ESDF first, then one small CNN (semantic mask or detector), DLA-hosted if INT8. Caveat: Isaac ROS 4.6 lists only AGX Thor/AGX Orin on JetPack 7.2; Orin NX runs the debs via sm_75 PTX JIT.

**Gaps:**
- No first-party Isaac ROS 4.x numbers exist for Orin NX: NVIDIA dropped the Orin NX column after Isaac ROS 3.2 and the 4.6 supported-platform table lists only AGX Thor and AGX Orin on JetPack 7.2. All Orin NX figures for nvblox, RT-DETR and SegFormer-B0 are interpolations between AGX Orin and Orin Nano (Super) data and are marked as estimates.
- The rclpy EventsExecutor saving (16 % to 6 % per node) is an x86/Iron measurement; the per-node overhead on this Jetson has not been A/B measured with jtop. The 0.5-1 core recovery estimates for the executor and composition wins are therefore estimates.
- The Orin NX thermal design guide could not be downloaded (Mouser mirror 404); sustained power in the sealed airframe with the Holybro baseboard's 0.35 A fan and the module's junction temperature in flight are unmeasured. Whether the Holybro UBEC can supply a MAXN_SUPER module was not verified.
- SUPER's own paper is paywalled (Science Robotics); its planner/ROG-Map timings come from a third-party profile with unspecified hardware. The planner CPU numbers for EGO-Planner are from the 2020 paper, not an Orin measurement.
- Memory-bandwidth contention between the 4 M-point lidar_view render + NVENC and TensorRT inference was not measured on this machine; only literature on Orin Nano/AGX contention was found.
- Cyclone DDS: the repo already applies the standard tuning (16 MB SocketReceiveBufferSize, rmem_max, spdp-only multicast after a measured 13 MB/s flood); Cyclone's iceoryx shared-memory path and rmw_zenoh (Jazzy binaries exist, needs a zenohd router) were not evaluated for CPU on this platform.
- The DLA share of the Orin NX 16 GB's 100 sparse INT8 TOPS (commonly quoted as 2 x 20) was not confirmed from an NVIDIA document; the NVIDIA module comparison page returned misaligned columns via fetch and Connect Tech's page omits the split.
- The web-search budget for this session was exhausted part-way; later verification used direct fetches only, so 2025-2026 work on rclpy performance beyond the EventsExecutor and on Orin NX-specific DNN benchmarks may have been missed.

## 3. Ranked candidates (score = Σ relevance + 0.25 × maturity across lenses)

```
 12.0  PX4 ROS 2 Interface Library (px4_ros2_cpp)  [planning, frameworks]
  8.5  aerial-autonomy-stack (AAS)  [planning, frameworks]
  6.3  PX4 v1.17 Offboard mode path (OffboardControlMode + TrajectorySetpoint/GotoSetpoint over uXRCE-DDS)  [px4_plumbing]
  6.3  px4_msgs (tag v1.17.0 / branch release/1.17)  [px4_plumbing]
  6.3  Micro XRCE-DDS Agent 2.4.3 (eProsima)  [px4_plumbing]
  6.3  PX4 logger / ULog: SDLOG_PROFILE, logger_topics.txt, dds_topics.yaml publications (the pilot-vs-planner record)  [px4_plumbing]
  6.3  PX4 v1.17 built-in safety: collision prevention, geofence, failsafes, failure injection  [safety]
  6.3  rosbag2 (MCAP) replay as the shadow evaluation harness  [safety]
  6.3  rosbag2 MCAP record/play with simulated clock (Jazzy)  [simulation]
  6.3  evo (odometry/SLAM trajectory evaluation)  [simulation]
  6.3  Foxglove app + PX4 converter extension + ROS 2 bridge (foxglove-sdk)  [simulation]
  6.3  grid_map (+ grid_map_filters)  [tasks]
  6.0  GCOPTER / MINCO (header library)  [planning]
  6.0  nvblox / isaac_ros_nvblox  [mapping]
  6.0  PX4 ROS 2 Interface Library (px4-ros2-interface-lib)  [safety]
  6.0  domain_bridge  [safety]
  6.0  PX4 SIH (Simulation-In-Hardware), as SITL and on the real Pixhawk 6X  [simulation]
  6.0  px4-ros2-interface-lib (external modes, mode executors, RTL replacement)  [tasks]
  6.0  rclpy EventsExecutor (Jazzy backport)  [compute]
  6.0  rclcpp composition + intra-process comms + experimental EventsExecutor  [compute]
  6.0  FAST-LIO2 (hku-mars/FAST_LIO, ROS2 branch)  [compute]
  5.8  SUPER (with ROG-Map and CIRI)  [planning]
  5.3  BehaviorTree.CPP 4.x (+ BehaviorTree.ROS2)  [frameworks]
  5.3  PlotJuggler 4  [simulation]
  5.3  BehaviorTree.CPP v4  [tasks]
  5.3  jetson_stats (jtop) + isaac_ros_jetson_stats as the power/thermal budget meter  [compute]
  5.0  PX4 v1.17 external modes / ROS 2 Control Interface (firmware side: ModeManagement, register_ext_component_request, arming_check_request/reply, config_control_setpoints, config_overrides)  [px4_plumbing]
  5.0  px4-ros2-interface-lib (px4_ros2_cpp, px4_ros2_py)  [px4_plumbing]
  5.0  PX4 EKF2 external-vision fusion (vehicle_visual_odometry, EKF2_EV_CTRL/EV_DELAY/EV_QMIN/EV_NOISE_MD)  [px4_plumbing]
  5.0  pyulog (ulog2csv, ulog2ros2bag) and ulog_cpp  [px4_plumbing]
  5.0  PX4 SITL + Gazebo Harmonic (PX4-Autopilot gz bridge, PX4-gazebo-models)  [simulation]
  5.0  rerun (viewer/SDK) + rosbag2_storage_rrd  [simulation]
  5.0  elevation_mapping_cupy (ROS 2 Jazzy)  [tasks]
  5.0  Patchwork++ (ground segmentation)  [tasks]
  5.0  cuVSLAM (Isaac ROS Visual SLAM / PyCuVSLAM)  [compute]
  5.0  nvblox core (TSDF/ESDF on GPU) via a thin node, not the Nav2 wrapper  [compute]
  4.8  MIGHTY  [planning]
  4.8  ROG-Map (and its ROS 2 version inside SUPER)  [mapping]
  4.8  Bonxai (bonxai_core + bonxai_map ProbabilisticMap + bonxai_ros)  [mapping]
  4.8  NavRL (and NavRL++)  [learning]
  4.8  pyulog ulog2ros2bag (--mcap)  [simulation]
  4.8  MARSIM (ROS 2 port, ubuntu20_ros2 branch)  [simulation]
  4.5  NTNU composite CBF safety filter (composite_cbf ROS 2 node + PX4-CBF embedded)  [safety]
  4.5  livox_laser_simulation for Gazebo Sim (Harmonic) — KangjianPeng fork  [simulation]
  4.3  OctoMap + octomap_server (ROS 2) + dynamicEDT3D  [mapping]
  4.3  nav2 reusable pieces: nav2_lifecycle_manager, nav2_util LifecycleNode/bond, plugin pattern, nav2_behavior_tree nodes  [frameworks]
  4.3  Ultralytics YOLO26/YOLO11 (with RT-DETR / RF-DETR as alternatives)  [learning]
  4.3  BehaviorTree.CPP 4.10 + Groot2 (versus plain ROS 2 lifecycle nodes)  [safety]
  4.3  YOLO11 / YOLO26 via TensorRT (Ultralytics), RT-DETR class  [compute]
  4.3  dyn_small_obs_avoidance-ros2 (Jazzy port of HKU-MARS kinodynamic planner)  [planning]
  4.0  kr_mav_control (humble)  [planning]
  4.0  MRS UAV System ROS 2 (mrs_uav_trackers MPC tracker, mrs_uav_trajectory_generation, octomap planning)  [planning]
  4.0  PX4 Collision Prevention (CP_DIST, obstacle_distance / collision_constraints)  [px4_plumbing]
  4.0  PlotJuggler 4.x (ULog + MCAP loaders)  [px4_plumbing]
  4.0  Aerostack2  [frameworks]
  4.0  py_trees_ros (and the other Jazzy-released executives: FlexBE, YASMIN, SMACC2)  [frameworks]
  4.0  SegFormer-B0/B2 (HF Transformers weights; Isaac ROS image_segmentation package)  [learning]
  4.0  topic_tools (mux / relay / throttle / drop / delay)  [safety]
  4.0  AV planner-evaluation metric suites (nuPlan devkit, WOSAC, Autoware planning_evaluator / validators, driving_log_replayer_v2)  [safety]
  4.0  gsn2x + AMLAS / SACE (safety-case documentation)  [safety]
  4.0  RGL Gazebo Plugin (Robotec GPU Lidar)  [simulation]
  4.0  TARE planner (humble-jazzy) + CMU autonomous_exploration_development_environment  [tasks]
  4.0  Fields2Cover (+ opennav_coverage; polygon_coverage_planning is dead)  [tasks]
  3.8  EGO-Planner / EGO-Swarm (official ros2_version branch)  [planning]
  3.8  kr_autonomous_flight (with jps3d and mpl)  [planning]
  3.8  MPPI-Generic  [planning]
  3.8  VDB-Mapping / vdb_mapping_ros2  [mapping]
  3.8  elevation_mapping_cupy (Jazzy line)  [mapping]
  3.8  PX4 EKF2 / system-wide replay  [px4_plumbing]
  3.8  MRS UAV System 2.x (ROS 2)  [frameworks]
  3.8  Unified Autonomy Stack (UAstack), with gbplanner3/OmniPlanner  [frameworks]
  3.8  PromptDA (Prompt Depth Anything)  [learning]
  3.8  Aerial Gym Simulator  [learning]
  3.8  Air-IO (learned inertial odometry for UAVs; TLIO is dead)  [learning]
  3.8  Telemetry anomaly detection: PX4 flight-review-rs (offline) + LSTM-AE/Isolation-Forest onboard (Sensors 2025)  [learning]
  3.8  CBFKit  [safety]
  3.8  run-time-assurance (ACT3 RTA library)  [safety]
  3.8  PX4 system-wide / EKF2 replay (ulog)  [simulation]
  3.8  GBPlanner3 / OmniPlanner (graph-based exploration, inspection, target-reach)  [tasks]
  3.8  SUPER + ROG-Map (hku-mars)  [compute]
  3.8  EGO-Planner / ego-planner-swarm (ZJU FAST-Lab)  [compute]
  3.8  SegFormer-B0 semantic segmentation  [compute]
  3.8  DLA offload for INT8 CNNs (jetson_dla_tutorial + TensorRT)  [compute]
  3.5  DYNUS  [planning]
  3.5  D-Map, BDM and D-BDM (occupancy without / with truncated ray casting for high-res LiDAR)  [mapping]
  3.5  VLNontheFly (onboard VLN stack, IMAV 2026)  [learning]
  3.5  MuJoCo lockstep backends for PX4 SITL (mujoco_px4_sitl, roqsim px4_sitl plugin, mjc_sitl_px4)  [simulation]
  3.5  GroundGrid (LiDAR ground segmentation + terrain estimation on grid_map)  [tasks]
  3.5  PX4-Avoidance safe_landing_planner (algorithm reference)  [tasks]
  3.5  EPIC (LiDAR exploration directly on point clouds)  [tasks]
  3.5  Jetson Linux 39.2 PREEMPT_RT kernel + isolcpus/IRQ affinity (rt_tuning placeholders)  [compute]
  3.3  auto-apms-px4 (AutoAPMS PX4 extension), with SMACC2 cl_px4_mr as the state-machine counterpart  [frameworks]
  3.0  Aerostack2 (+ as2_platform_pixhawk, as2_behaviors_path_planning)  [planning]
  3.0  wavemap  [mapping]
  3.0  flight_review (PX4 log analysis web app)  [px4_plumbing]
  3.0  Depth Anything 3 + ROS 2 TensorRT wrappers (RWTH ika, GerdsenAI)  [learning]
  3.0  Isaac ROS DNN Stereo Depth (ESS, Fast-FoundationStereo, FoundationStereo)  [learning]
  3.0  TensorRT Edge-LLM (plus LLM task planners TypeFly / UAV-VLA / AeroVLA)  [learning]
  3.0  ESS DNN stereo (isaac_ros_dnn_stereo_depth)  [compute]
  3.0  Depth Anything V2 (ViT-S) monocular depth  [compute]
  2.8  YOPO / YOPO-MINCO (learned planner)  [planning]
  2.8  supereight2  [mapping]
  2.8  EGO-Planner grid_map / Fast-Planner SDFMap  [mapping]
  2.8  px4_ros_com (frame_transforms, offboard examples) and Jaeyoung-Lim/px4-offboard  [px4_plumbing]
  2.8  CMU Explorer / cmu-exploration (Aerial Navigation Development Environment, Air-FAR, autonomy_stack_* ports)  [frameworks]
  2.8  iPlanner / ViPlanner (imperative learned local planners)  [learning]
  2.8  Middleware allow-lists: SROS2 (DDS-Security) and rmw_zenoh ACLs  [safety]
  2.8  Hamilton-Jacobi reachability toolkits (hj_reachability, DeepReach)  [safety]
  2.8  Ogma + Copilot (generated runtime monitors)  [safety]
  2.8  Pegasus Simulator on Isaac Sim (and Isaac Sim 6.x itself)  [simulation]
  2.8  AirSim lineage: ProjectAirSim (IAMAI), Cosys-AirSim, Colosseum (archived)  [simulation]
  2.8  MRS UAV System (ROS 2 / Jazzy) and mrs_multirotor_simulator  [simulation]
  2.8  FUEL / FALCON (HKUST FAST-Lab exploration family; RACER is the multi-UAV variant)  [tasks]
  2.8  Hydra / Hydra-ROS / Khronos 3D scene graphs  [tasks]
  2.8  mav_active_3d_planning (informative path planning / next-best-view)  [tasks]
  2.8  CERLAB-UAV-Autonomy (CMU, complete PX4 stack with exploration and inspection)  [tasks]
  2.8  CUDA MPS on Jetson (GPU sharing between processes)  [compute]
  2.5  voxblox (ETH ASL) and its ROS 2 ports: tu-darmstadt voxblox_ros2, voxfield_ros2 (incl. FIESTA server)  [mapping]
  2.5  UFOMap v1 / UFO framework (ufo, uforos)  [mapping]
  2.5  OHM (Occupancy Homogeneous Map)  [mapping]
  2.5  GIE-mapping (GPU incremental Euclidean distance transform)  [mapping]
  2.5  PX4 message translation node (msg/translation_node + px4_msgs_old)  [px4_plumbing]
  2.5  AirStack  [frameworks]
  2.5  kr_autonomous_flight  [frameworks]
  2.5  vitfly (ViT end-to-end obstacle avoidance)  [learning]
  2.5  NASA ICAROUS / PolyCARP / DAIDALUS (and the licensed Safeguard, Safe2Ditch)  [safety]
  2.3  px4-ros2-drone-nav  [frameworks]
  2.0  Clover  [frameworks]
  1.8  ikd-Tree / i-Octree (and lio_map as the project's own point store)  [mapping]
  1.8  Agilicious  [frameworks]
  1.5  Legacy ROS 1 planners: Fast-Planner, FASTER, MADER, PANTHER, PUMA, Agilicious/rpg_mpc, mav_trajectory_generation, PX4-Avoidance  [planning]
  1.5  PX4-Avoidance  [frameworks]
  1.5  visualnav-transformer (GNM / ViNT / NoMaD)  [learning]
  1.5  Learned LiDAR occupancy / semantic scene completion (SalsaNext, MinkNet, SPVCNN, occupancy forecasting)  [learning]
  1.3  PX4-Avoidance (local_planner / global_planner / safe_landing_planner)  [px4_plumbing]```
