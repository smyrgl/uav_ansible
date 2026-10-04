# Autonomy roadmap: from shadow mode to assured behaviours on the X950

Written 2026-10-03, at the start of the manual flight-test campaign. This is the
staged plan for the autonomous-navigation framework: what gets built, in which
order, what each stage must prove before the next one starts, how the coming
manual flights and their recordings are used, and how the Orin NX's budget is
spent. It rests on a nine-lens survey of the open-source landscape with every
candidate verified against primary sources on 2026-10-03
([autonomy-landscape.md](autonomy-landscape.md)), on three independently drafted
plans and a critique of them, and on facts read from this aircraft the same day
(PX4 parameters, the DDS topic set, measured load). The Hadron thermal camera has
its own notes ([hadron-thermal.md](hadron-thermal.md)).

Nothing here changes the aircraft yet. Every stage below is a proposal until the
owner starts it.

## 1. What the research changed

The survey's yield was less a stack to adopt than a set of corrected assumptions.
Each of these shapes the plan.

1. **"Shadow mode in the offboard control path, never given control" does not
   exist on PX4 v1.17.** The uXRCE-DDS client republishes anything received on
   `/fmu/in/trajectory_setpoint` straight to uORB with no flight-mode check, and
   `mc_pos_control` takes the newest sample whenever position control is enabled,
   i.e. in the pilot's Position and Altitude modes too (release/1.17
   `MulticopterPositionControl.cpp:431-453`; the MAVLink receiver gates on
   OFFBOARD, the DDS path does not). Publishing a setpoint on `/fmu/in` while
   "not in Offboard" steers the aircraft. Shadow therefore means: the exact
   `px4_msgs` at the exact cadence on **`/shadow/*`**, the `/fmu/in` setpoint,
   command and registration **subscriptions deleted from `dds_topics.yaml`** on
   the firmware branch this project already builds, a guard that counts
   publishers on every `/fmu/in` topic, and a zero-`/fmu/in` assertion over every
   flight bag. Authority returns one auditable firmware flash at a time.
2. **No Jazzy-native, PX4-XRCE-native 3D planner exists.** Every living planner
   (SUPER, MIGHTY, DYNUS, EGO) emits references into its own, often unreleased,
   controller; every full stack (Aerostack2, MRS, AirStack, aerial-autonomy-stack,
   kr_autonomous_flight) owns the aircraft through Docker or mavros. The usable
   pieces are **libraries**: GCOPTER's thirteen Eigen headers (MINCO, FIRI,
   L-BFGS), nvblox 4.6 for a GPU ESDF, grid_map for 2.5D terrain, Auterion's
   px4-ros2-interface-lib (release/1.17 only) for external modes later, PX4's own
   Collision Prevention, rosbag2 and evo for scoring. The stack is "small rclcpp
   nodes around pinned libraries", which is what this repo already does.
3. **The missing component is free space, not a planner.** lio_map is a surface
   store: no ray casting, no notion of unknown. No planner can be safe on it. The
   first build decision is the ESDF (nvblox 4.6, already bench-proven here, versus
   ROG-Map on the CPU); the planner is a few hundred lines once an ESDF with
   explicit unknown exists.
4. **nav2's rejection is right for costmaps and planners, wrong for the rest.**
   BehaviorTree.CPP 4.10 (apt), `nav2_lifecycle_manager`'s bond heartbeat and
   the LifecycleNode pattern are exactly the size needed to gate shadow components
   and sequence behaviours later.
5. **External modes come late, not first.** On v1.17 the interface library has no
   `user_selectable` (PX4 added it one day after the 1.17 branch point), so a
   registered mode appears in QGC the moment it exists; registration is itself a
   write to `/fmu/in`; and the FC removes an unselected mode after ~1.2 s of missed
   arming-check replies, after which the library exits by exception. Learn it on
   the bench against SIH; fly it after raw Offboard is proven.
6. **Collision Prevention is not a shadow feature.** With `CP_DIST −1` PX4 ignores
   the obstacle stream and produces nothing to score; with `CP_DIST > 0` the
   Jetson's stream is flight-critical (no data → zero XY acceleration → Hold
   after 5 s) and on this forward-only sensor suite `CP_GO_NO_DATA 1` is mandatory
   or Position mode cannot translate sideways. Build and bag the publisher in
   shadow; enable it as the first authority step.
7. **The recorder cannot replay a vision stage.** Bags keep the D555 only as the
   H.265 colour stream (raw IR/depth are excluded because the camera unicasts a
   copy per subscriber), so cuVSLAM and any stereo or semantic network cannot be
   re-run from flight data. The plan is LiDAR-first unless a throttled IR/depth
   copy is added to the bag before the campaign.
8. **Bookkeeping that would have bitten:** the FC *does* have GNSS (the mosaic-G5
   on GPS2; the Septentrio driver sets CLOCK_REALTIME and the ulog carries UTC
   anchors; `roles/px4_bridge/README.md` is stale); the firmware branch is four
   commits ahead of v1.17.0, not one, and the flashed FC may lag the worktree;
   the geofence on the FC today (50 m / 30 m, Return) is smaller than every data
   pattern the plans ask for; the 3.5–4.7 ms timesync bias is most plausibly PX4's
   `Timesync` rejecting every exchange with RTT ≥ 10 ms on the stalling FMU link,
   not "offset without rate".

## 2. The Offboard executor with diverted output (what the first stages build)

Decided 2026-10-03: there is no separate "shadow mode". The aircraft knows two
controllers, the FMU and the Jetson, and the Jetson's path is PX4's standard
Offboard mode; a custom external mode (§3, Stage 5) comes only after the raw path
has flight hours. What "shadow" reduces to is one property of the executor: its
output is **diverted** to a logged `/shadow/*` twin until it is engaged, and the
firmware has no `/fmu/in` input path until the stage that needs one. The
diversion is not ceremony: it is the record that lets the scorer say the planner
would have been right before it is handed the switch.

```
                 PX4 (uav/v1.17.0-pps, dds_topics.yaml: NO /fmu/in setpoint, command or
                 registration subscriptions; + trajectory_setpoint, vehicle_local_position_setpoint
                 published at 50 Hz)
                        │ XRCE-DDS (agent 2.4.3)
   /fmu/out/* ──────────┴──────────────► px4_bridge ──► /px4/* (ENU, UTC)
   /avia, /e1r, /lio/odometry, /lio/map, /lio/health, /gnss, /tf  (existing)
                        │
          ┌─────────────┴────────────────────────────────────────────┐
          │  uav-shadow (rclcpp nodes, same ROS domain)              │
          │  esdf (nvblox 4.6, local, 10 cm)   terrain (grid_map, E1R)│
          │  return planner (A*/JPS → FIRI → MINCO, 10 Hz)            │
          │  tracker → /shadow/trajectory_setpoint (px4_msgs, NED)   │
          │            /shadow/offboard_control_mode (10 Hz)          │
          │  obstacle_distance → /shadow/obstacle_distance (72×5°)    │
          │  guard: publishers on ^/fmu/in/ == 0, else ERROR          │
          └─────────────┬────────────────────────────────────────────┘
                        ▼
   flight recorder (rosbag2 MCAP, -a) ── replay harness ── offline scorer (Python over MCAP)
   ulog ──► ulog→MCAP UTC converter ──┘
```

Rules that make it provable rather than promised:

- **Firmware**: a commit on `uav/v1.17.0-pps` (built from `PX4-Autopilot-fc`,
  never pushed upstream) deletes every `/fmu/in` subscription except
  `message_format_request`, and adds `trajectory_setpoint` and
  `vehicle_local_position_setpoint` to the publications at `rate_limit: 50`
  (the pilot's flight-task output, the label every scorer needs). A bench
  firmware variant keeps the registration topics for SIH rehearsal. Each later
  stage re-adds exactly the topics it needs; the diff is the audit trail.
- **Graph**: `shadow_guard` (rclcpp) counts publishers on every `/fmu/in` topic
  at 1 Hz and goes ERROR on any; a mavlink-router rule or sniffer alarms on any
  COMMAND_LONG / SET_MODE / PARAM_SET whose source is the Jetson (the MAVLink
  path is a second write channel the DDS argument does not cover).
- **Record**: every bag is checked for zero `^/fmu/in/` messages at sign-off.
  Replay runs with `--clock-topics-all`, `--exclude-regex '^/fmu/in'`, the XRCE
  agent stopped and `use_sim_time` on every node (Jazzy bugs #2383 and #2354 are
  worked around, not fixed).
- **The executor is the production component from day one.** It writes to
  `/fmu/in` only when `nav_state == OFFBOARD` **and** an explicit engage is set
  (or the FC is in SIH on the bench), and otherwise to `/shadow/*`: real
  `px4_msgs` types, NED, Jetson-microsecond stamps (what `UXRCE_DDS_SYNCT 1`
  expects), PX4 reset counters honoured. The XRCE client has no mode gate; this
  node is the gate, and the firmware deletion is the belt to its braces. The only
  thing that changes when authority is granted is the topic prefix.
- **A second ROS_DOMAIN_ID with `domain_bridge`** is deferred to Stage 4a, when
  the first `/fmu/in` topics are legitimately re-added and non-authorised nodes
  must be kept out; introduced then, scoped and measured.

## 3. Stages and gates

Gates are checkable conditions. Exit criteria are measurable. Flight patterns are
what the manual flights of that period should fly and record; the pilot flies,
the shadow watches. Effort figures are owner-weeks of bench and desk work and are
estimates.

### Stage 0a — Hardware and parameters (bench, before campaign flights)

- FMU module retained on both PAB connectors (X1, X2); periph-5V overcurrent
  isolated; PPS into FMU_CAP1 through an isolated stage; flashed firmware re-read
  (`ver all`) and recorded.
- `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` / `RTL_RETURN_ALT` set per site for the
  planned legs and altitudes (today's 50 m / 30 m trips every pattern below).
- RC channel map fixed and written down: ch 11 recorder (as today), which of
  ch 12/13 becomes the in-bag marker and which the Offboard switch; how the
  Jetson reads them.
- Pre-flight item added: `SYS_HITL == 0` and a parameter checksum against the
  flight set (SIH sessions leave the FC in a non-flyable state).
- **Avia time source: PPS + UTC, not PTP.** The receiver's PPS reaches the Avia's
  RS-485 Sync pair through the TTL→RS-485 converter already wired; the driver
  already pushes the UTC second 100 ms after each pulse. Outdoors with a fix:
  (1) with `ptpd` still running, read the Avia status word's `pps_status` (bit 9),
  which shows the pulse arriving regardless of the active source (Livox priority
  is PTP > GPS > PPS); (2) `avia_ptp_enabled: false`, confirm GPS sync mode 2 with
  stamp type 3 and receipt-minus-sample latency inside the driver's −1…+50 ms
  window; (3) a ~100 ms late stamp on every packet means Sync+/Sync− are swapped
  (the Avia sees the rising edge at the end of the pulse); a dead `pps_status` bit
  means the converter's driver-enable or the M12 pins (12 = Sync+, 11 = Sync−).
  Gate: median latency at or below today's PTP figure (1.09 ms; expect far less);
  then `ptpd` is retired. This replaces any idea of moving the Avia onto the i226
  segment (that would need an 802.1AS-aware switch and a second `ptp4l` profile for
  ~30 Mbit/s of traffic).
- **Gate:** 10 consecutive cold boots with XRCE connected and
  `/fmu/out/vehicle_odometry` at ~100 Hz within 30 s; zero overcurrent events on 3
  full-payload power-ups; eth0 up with the PPS lead connected on 5 boots; the
  geofence fits every flight-card manoeuvre with 10 m margin.

### Stage 0b — Record and inertness (bench, overlapping shakedown flights)

- Firmware commit as in §2; `ros2 topic list` shows no `/fmu/in` reader except
  `message_format_request`.
- `shadow_guard` and the MAVLink-source check; a `px4_msgs` FNV-1a message-hash
  check at `uav-ros.target` start that **retries until the client is connected**
  and reports "unknown" in diagnostics until then. *Done 2026-10-04 as the
  health node's "PX4 / Guard" row (rclpy, not rclcpp: a 1 Hz graph query):
  16/16 hashes matched the FC on first contact; the router forwards all MAVLink
  to the node (`SnifferSysid`).*
- Recorder hardening: stalled-growth alarm (open rosbag2 Jazzy #2463 at our
  ~14 MB/s), per-topic gap scan and topic-count check written into `flight.json`,
  end-of-flight `/lio/map/save` PCD, flight card per flight. *Done 2026-10-04:
  the stall alarm fired in the SIGSTOP test at 120 s; `postflight.py` (recorder
  role README); a gap is a missed message, nominal period + 100 ms.*
- ulog → MCAP UTC converter (pyulog + mcap): `UTC_us = hrt_us −
  timesync_status.estimated_offset`, cross-checked against
  `vehicle_gps_position.time_utc_usec` and `pps_capture`. *Written 2026-10-04:
  `tools/ulog2mcap.py`; the UTC path waits for a log from this FC.*
- Storage and offload plan: ~50 GB per flight hour; where bags go, which host
  replays them (the Mac cannot run Jazzy rclcpp natively). *2026-10-04: the
  replay host exists (roles/replay on `atomic`, inventory/replay.yml): the
  aircraft's own builds, loopback DDS on domain 42, a ZFS flights dataset,
  `uav-replay` and a scorer. Replay-against-replay of a bench bag is
  bit-identical (APE 1e-16 m, map Jaccard 1.0): the harness is deterministic.
  Fidelity against the live run needs a flight bag (a stationary bench bag has
  no trajectory extent for the alignment); the recorder now resets the live map
  when a bag starts so the saved map covers exactly the bag. Offload from the
  Jetson and the TrueNAS tier are still to do.*
- Compute recovery, measured: `rclpy.experimental.EventsExecutor` on every Python
  node with a jtop A/B per node; the camera node's 1.2 cores are a Python
  GStreamer buffer problem, not executor polling, and need a C++ or zero-copy
  path (Stage 2). *2026-10-04: health node 58 % → 14.5 % of a core, all Python
  services 124 % → 55 %; the camera node 105 % → 22 % by feeding the D555's
  native YUY2 to nvvidconv instead of converting twice on the CPU (the Stage 2
  zero-copy item is now about the remaining copies, not the conversion); the
  rest in docs/ros-bench.md "Executor A/B".*
- Decision recorded: a throttled D555 IR/depth copy in the bag, or LiDAR-first.
- Fix the stale README lines (FC GNSS; four-commit branch).
- **Gate:** 3 bags with zero gaps > 100 ms on `/fmu/out/vehicle_odometry`,
  `/avia/*`, `/e1r/points`, `/gnss/*` while armed and `/fmu/out/trajectory_setpoint`
  at ≥ 45 Hz; ulog-to-bag alignment < 2 ms median; a replay reproduces the live
  `/lio/odometry` to < 2 cm APE and lio_map to Jaccard > 0.98; the stall alarm
  fires in a SIGSTOP test; measured CPU reduction recorded.

### Stage 1 — Estimator truth, frame contract, map-quality baseline (rides on manual flights)

The thing the next flights uniquely produce is estimator truth outdoors; plumbing
can be built on the bench, these numbers cannot.

- Scripted card per flight, each manoeuvre marked: hovers at 3/10/20/30 m AGL;
  yaw-in-place at 5 and 20 m; legs at 2/5/8 m/s inside the fence; a figure-8; one
  deliberate pass over flat featureless ground (FAST-LIO degeneracy on record);
  the E1R over ground the Avia mapped earlier; repeated identical QGC Survey
  missions (a deterministic label and a regression path for LIO drift and map
  quality).
- evo APE/RPE of `/lio/odometry`, `/vslam/odometry`, `/px4/odometry` against the
  RTK + dual-antenna-heading reference, SE(3) alignment only, with the
  antenna-to-base_link lever arm measured (a static yaw-in-place gives it).
- A `camera_init → px4_local` monitor with PX4's xy/z/heading reset counters and
  `/lio/health` events logged: the transform every later stage converts through.
- Divergence catalogue; E1R–Avia extrinsic from the floor offset; D555 outdoor
  depth validity fraction (the number that decides whether any stereo network is
  ever worth its GPU); map-quality metrics (registration vs RTK, density at range,
  outdoor stray fraction) on the repeated mission.
- Vehicle identification for the trajectory optimiser: hover thrust, drag, tilt
  from steady translations at 2/5/8 m/s. The X950 does not fit a 0.61 kg racer's
  defaults.
- **Gate:** ≥ 5 scored flights, ≥ 20 min RTK-fixed; FAST-LIO APE < 0.5 m RMS and
  RPE(1 s) < 0.15 m / 1° on non-degenerate segments, every evo-visible jump > 2 m
  matched by a `/lio/health` event within 2 s; anchor drift < 0.3 m / < 1° per
  5 min between logged resets; E1R extrinsic < 5 cm / 0.5°; cuVSLAM's usable
  altitude ceiling and the D555 validity number written down; physical parameters
  fitted with residuals; map-quality baseline recorded.

### Stage 2 — Shadow offboard path with exact labels (same flights continue)

- `shadow_tracker` (rclcpp): samples any reference into `px4_msgs/TrajectorySetpoint`
  (NED, feed-forward velocity/acceleration, yaw/yawspeed) on `/shadow/*` at 50 Hz
  plus `OffboardControlMode` at 10 Hz, with ENU twins for Foxglove.
- **Exact label by loopback:** PX4's own `/fmu/out/trajectory_setpoint` (NED) →
  the bridge's ENU → the tracker's NED must reproduce the original to float
  precision. A pilot-mimic (re-deriving the Position-mode stick mapping) is kept
  as a sign/axis diagnostic only; it cannot be an exit criterion because
  `StickAccelerationXY` carries state no reimplementation matches.
- `obstacle_distance` publisher on `/shadow/`: 72 × 5° BODY_FRD, raw Avia + D555
  depth binned at the aircraft's altitude ± 1 m (so forward descents do not read
  ground as obstacles), UINT16_MAX outside the ~14 covered sectors, a
  covered-sector mask so the scorer can report the protected fraction.
- The `/lio/health` DIVERGED/restart event propagated as a **"map pose invalid"**
  flag to every shadow consumer (a FAST-LIO restart re-anchors `camera_init`; an
  ESDF integrating in `odom` would otherwise misalign every existing block).
- Replay harness and determinism fixture; offline scorer skeleton (pure Python
  over MCAP, no ROS graph).
- Bench only: px4-ros2-interface-lib release/1.17 @ 4a3370f0 vendored and built
  against the pinned `px4_msgs`; a `SHADOW-NOFLY` mode registered against SIH on
  the bench firmware variant, own executor thread, `Restart=` and
  re-registration, its own arming check permanently false.
- **Gate:** zero `/fmu/in` messages in every bag and zero guard events; loopback
  residual < 1 mm / < 1e-4 rad with no systematic axis error in the mimic;
  `/shadow/offboard_control_mode` gaps never > 500 ms; tracker latency p99 < 40 ms;
  obstacle stream gaps never > 300 ms over ≥ 3 flights; live vs replayed
  `/shadow/trajectory_setpoint` within 1 cm / 1 cm/s RMS; SIH: 30 min registered
  without deregistration, kill → restart → re-register < 5 s, the mode never
  enterable; shadow nodes ≤ 0.3 core, total ≤ 4.5 cores unwatched.

### Stage 3 — Shadow free space, Assured Return planner, E1R terrain products (scored by replay)

**Entry gate first — the ESDF build-or-buy decision:** nvblox 4.6 with a *moving*
pose on a bench replay returns finite ESDF distances and gradients from
`get_esdf_and_gradient` (two open, unanswered reports on Jazzy/CUDA 13:
isaac_ros_nvblox #154, nvblox #122). Two days to test. If it fails — or if owning
the component is preferred to living on a frozen Isaac line — build our own
ray-cast occupancy + ESDF on ROG-Map's design (CPU, sliding 30 × 30 × 12 m grid,
~6 ms per frame in the original; 3–4 weeks). With the time budget the owner has
described, the build is a legitimate first choice, not only a fallback. The Avia adapter emits a per-point `t`
field or motion compensation is pinned off (a missing field is a glog CHECK that
aborts). GCOPTER vendored at e0444f6 with the two issue-#16 patches, built with
GCC 13 `-Wall -Werror`, quickhull vertex enumeration unit-tested against upstream
akuukka/quickhull on the target (issue #28 reported illegal polytopes on an Orin NX).

- nvblox LiDAR-only (one PointCloud2 input per node; the E1R is nadir and enters
  only as a camera depth image or not at all), 10 cm, LiDAR grid 1800 × 400,
  integration 20–30 m, `map_clearing_radius_m` 60–80 m (memory is bounded by the
  clearing radius, not distance flown: free space allocates 8³ blocks along every
  ray, ~8–9 GB per km if unbounded), `esdf_mode 3d` plus a 2D slice at flight
  altitude, unobserved = occupied, **no layer markers in flight** (the retired role
  spent 1–2 cores serialising cubes), cleared or re-anchored on a LIO restart.
- Planner (rclcpp): own A*/JPS over the ESDF (unknown blocked, hard clearance
  ≥ 1.5 m), a **thin-obstacle layer** from 1–2 s of raw Avia hits (wires and
  branches: the rosette gives sparse hits a TSDF suppresses and lio_map's stray
  filter deletes by design), FIRI corridors, MINCO with the Stage 1 physical
  parameters, PX4 Return geometry as the no-free-path fallback, 10 Hz replans
  through the Stage 2 tracker; flatness map for feasibility checks only, never
  emitted as attitude/rate setpoints.
- E1R `grid_map` accumulation node (resolution tied to AGL: the footprint is
  ~2h × 3.5h), YAML filter chain (inpaint, normals, slope, roughness, patch size),
  `/shadow/agl`, `clearance_ahead`, landing candidates.
- Scorer: would-have-collided sweep against the post-flight lio_map, time in
  unknown, feasibility vs MPC limits, agreement with PX4's own Return and Mission
  references; pilot-path metrics descriptive only (a planner that disagrees with a
  human may be better).
- Flights: Position-mode and mission legs near real structure inside the fence;
  several PX4 Return activations from different points (the deterministic
  comparison); survey lines at 8/15/25 m over slope and hedge; landings on ≥ 3
  surfaces; a wire survey item on the site card.
- **Gate:** ≥ 10 flights replayed deterministically; zero proposals with clearance
  < 1.5 m or into unobserved space; < 0.1 % of lio_map occupied voxels inside
  ESDF free space beyond the margin; planner p99 < 100 ms at ≥ 8 Hz, ≤ 0.6 core;
  nvblox RSS < 3 GB and GPU < 15 % over a 20 min flight, CUDA cache warm;
  `/shadow/agl` vs H-Flow < 0.1 m where both valid and vs RTK-minus-terrain
  < 0.3 m; pilot touchdown in the top quartile of landing cells on ≥ 80 % of
  landings, no candidate on > 10° slope; stack ≤ 5.5 cores unwatched;
  map-quality metrics unchanged from Stage 1; **a 30 min sealed-airframe bench
  soak with everything running: VDD_IN ≤ 20 W, tj ≤ 80 °C.**

### Stage 4a — First influence by proxy: PX4 Collision Prevention

- Firmware flash re-adding **only** `obstacle_distance` (one auditable commit);
  `SDLOG_PROFILE 147` (bit 7 logs `collision_constraints`); `CP_DIST 6`,
  `CP_DELAY 0.4`, `CP_GUIDE_ANG 10`, `CP_GO_NO_DATA 1`; the publisher promoted
  from `/shadow/` to `/fmu/in` with the guard allow-list widened to exactly that
  topic; props-off bench test of no-data → zero XY → Hold.
- The flight procedure states what CP protects: forward approaches inside the
  Avia/D555 cone only. Sideways drift into a tree, the common Position-mode
  mishap, stays invisible.
- Flights: Position Slow first, then Position; forward approaches to a soft
  obstacle; lateral and backward translations to confirm `CP_GO_NO_DATA 1`; one
  deliberate publisher stop in open space at altitude.
- **Gate:** obstacle stream ≥ 10 bagged flight hours gap-free in shadow
  beforehand; ≥ 5 CP-active flights with zero unintended Holds and zero clips in
  open space, every clip explained by lio_map within `CP_DIST + 1 m`; sector ranges
  within 0.5 m of lio_map for 95 % of populated bins; takeover by any RC mode
  switch or kill ends CP influence within one cycle in the ulog.

### Stage 4b — Bounded Offboard under the pilot's switch

- Firmware flash re-adding `offboard_control_mode`, `trajectory_setpoint`,
  `goto_setpoint`; `RC_MAP_OFFB_SW` on the agreed channel; `COM_RC_OVERRIDE 3`
  (bit 1 enables stick takeover in Offboard: a filtered stick-*rate* test);
  `COM_RC_STICK_OV 30 %`; `COM_OF_LOSS_T 1.0`; `COM_OBL_RC_ACT 0` (knowing v1.17
  substitutes `NAV_RCL_ACT` = Return if RC is also lost); `MPC_XY_VEL_MAX 5` for
  these flights (at 12 m/s a 1 s loss timeout is 12 m plus a 24 m stop).
- The executor publishes to `/fmu/in` only while `nav_state == OFFBOARD` **and** a
  companion monitor (estimator validity, ESDF age, LIO health, geofence margin,
  backup-hover feasibility) says healthy; otherwise it publishes nothing so PX4's
  loss fallback fires within 1 s. The XRCE client has no mode gate; this node is
  the gate.
- Flights, open space at 20 m AGL: 10 s hover-hold ×3 per flight with stick
  takeover each time; then 20–30 m Goto; then the Stage 3 planned return behind
  the Stage 3 tree line.
- **Gate:** SIH rehearsal on the bench firmware of switch-in, stick override,
  stream stop → Position within `COM_OF_LOSS_T`, RC loss → Return, node crash, all
  in a ulog; ≥ 10 switch-ins with 100 % clean disengagement by stick (< 300 ms) and
  switch, kill rehearsed on the bench; hover drift < 0.5 m / 10 s; Goto tracking
  < 1 m RMS; planned return tracking < 1.5 m RMS with clearance ≥ 1.5 m on ≥ 5
  returns; heartbeat gap p99 < 300 ms; zero non-allow-listed `/fmu/in`
  publishers; zero unintended override trips logged.

### Stage 5 — External mode: Assured Return in the failsafe chain

- Firmware flash re-adding the registration and arming-check topics;
  px4-ros2-interface-lib release/1.17 mode `ASSURED-RETURN` (requirements
  local_position + home_position), own process and executor thread, `Restart=`
  and re-registration, its own arming check as the per-flight enable (the mode
  *will* appear in QGC on 1.17: name it unmistakably; `COM_FLTMODE1-6` stay
  internal so RC cannot reach it).
- `replace_internal_mode = RTL` only after ≥ 10 operator-triggered activations
  (QGC Return command or the RC Return slot with sticks live — never by switching
  the transmitter off) and ≥ 3 node-kill fallbacks observed with internal RTL
  taking over < 1.5 s.
- BehaviorTree.CPP executive for takeoff → behaviour → return → land with
  reactive guards; a GSN safety case in `docs/` pointing each gate at its bag or
  ulog (gsn2x), which doubles as the channel's explainer.
- **Gate:** `check-message-compatibility.py` passes against the flashed firmware;
  the Siyi's Android QGC shows and selects the mode (bench check); ≥ 10
  activations arriving within 3 m of home with clearance ≥ 1.5 m; zero "Mode is
  unresponsive" events in flight.

### Stage 6 — Terrain-following survey with avoidance (second external mode)

Lawnmower lines at terrain + AGL from the Stage 3 grid with a 3 s look-ahead and a
climb-first rule, corridor routing around obstacles, speed capped by ESDF
clearance; E1R range into PX4 `distance_sensor` only after EKF2's downward-instance
selection with the H-Flow present is read in source and bench-tested with
`EKF2_RNG_CTRL 0` first. **Gate:** AGL error < 1 m RMS over ≥ 5 polygons with
≥ 3 m terrain steps, obstacles rounded with ≥ 1.5 m clearance, ≥ 95 % line
completion, map-quality metrics at or above the Stage 1 baseline, VDD_IN ≤ 20 W
sustained and tj ≤ 80 °C in the hottest flight.

### Stage 7 — Learned components, each against a measured deficit (no dedicated flights)

Telemetry anomaly row (IsolationForest/LSTM-AE, advisory) after ≥ 10 nominal
flights; SegFormer-B0 terrain classes only if the landing scorer misranks by
surface type; Air-IO-style IMU odometry only if Stage 1 shows LIO/GNSS gaps EKF2
cannot coast across; ESS-light or PromptDA only if the D555 validity number
demands it; NavRL-style policy as a scorer comparison line only. **Gate per node:**
deficit closed by ≥ 50 % on held-out flights, GPU/CPU inside the Stage 6 totals
with lidar_view watched, documented behaviour on confidence collapse, never
heartbeat-critical, never a mode. EKF2 external-vision fusion stays out unless
GNSS-denied flight becomes a stated goal with its own gates.

### Parallel lane — Hadron thermal (from the r2 board's arrival; independent of the authority ladder)

Driver (vendored `flir_boson_usb` + telemetry/sync/FFC), the PPS-disciplined MCU
trigger and frame↔pulse pairing, intrinsics and thermal→Avia extrinsics
(LVT2Calib), Y16 + telemetry recording, then hot-spot thresholding and
georeferencing through the LiDAR map and a YOLO26-class detector at 10–15 Hz, all
in shadow. Details and gates in [hadron-thermal.md](hadron-thermal.md). Its
products enter the autonomy stack as landing vetoes on the approach corridor
(Stage 3+) and as keep-outs for the planner (Stage 5+).

### Sequencing in one line

Trustworthy aircraft and record → estimator truth → exact-label plumbing → one
chain end to end in shadow (free space → return planner → tracker → scorer) →
authority in order of least novel mechanism (PX4 clips the pilot → pilot's switch
→ external mode) → second behaviour → learning where geometry fails.

The merged gates ask for roughly **30–60 flights before an external mode**. That
is the honest count; which gates to relax if the season does not allow it is the
owner's call (§7).

## 4. The compute ledger: how much autonomy does an Orin NX 16 GB buy

The question is measurable and the shadow stages are the instrument: every stage
runs the whole path at full load on real flights with nothing at stake. The
ledger below is a standing table, updated per stage from `jetson_stats` in the
bags (VDD_IN, every thermal zone, fan PWM, per-process CPU), with **tail
latencies, not averages**, next to the capability bought.

Measured baseline, 2026-10-03, MAXN, fan `max` (after this afternoon's change),
lidar_view unwatched: **5.2 of 8 cores busy, GPU 0–5 % (36 % while the LiDAR view
is watched), 4.1 of 15.6 GB, 12.7–15 W, tj 57–64 °C.** The algorithms are cheap:
FAST-LIO2 0.11 core, cuVSLAM ~0.3 + 0.25 (bridge), the whole LIO chain ~0.6.
About **3 of the 5.2 cores are rclpy executor and DDS overhead** across ten Python
nodes; the single largest consumer is the camera node at 1.2 cores (a Python
GStreamer/NVENC pipeline).

| Component | Cores | GPU % | RAM GB | When | Status |
|---|---|---|---|---|---|
| Baseline today | 5.2 | 5 (36 watched) | 4.1 | now | measured |
| Flight recorder while armed (~14 MB/s zstd) | 0.5 | 0 | 0.3 | every flight | estimate, measure in 0b |
| rclpy EventsExecutor on every Python node | −0.5…−1 | 0 | 0 | 0b | x86 figure, A/B here |
| C++/composition of camera, health, px4_bridge, bridges | −0.5…−1 | 0 | −0.2 | 2 | estimate |
| shadow_guard + tracker + frame monitor (rclcpp) | 0.1 | 0 | 0.1 | 2 | estimate |
| obstacle_distance publisher | 0.15 | 0 | 0.05 | 2 shadow, 4a live | estimate |
| nvblox 10 cm ESDF 3D, no markers | 0.3 | 8 | 1.5–1.8 | 3 | bench static: GPU 0–2 %, 1.2 GB |
| Return planner (A*/JPS + FIRI + MINCO, 10 Hz, C++) | 0.5–0.6 | 0 | 0.3 | 3 | SUPER-class 10–20 ms on x86 ×2–3 |
| E1R grid_map terrain + filters (2 Hz) | 0.2 | 0 | 0.1 | 3 | estimate |
| Companion monitor + executive | 0.1 | 0 | 0.1 | 4b–5 | estimate |
| Hadron driver + hot-spot thresholding | 0.3 | 0 | 0.2 | parallel lane | estimate |
| Hadron YOLO26-n/s INT8 at 10–15 Hz | 0.1 | 5–10 | 0.5 | parallel lane | 3.5–4.8 ms/frame on Orin NX (JP6) |
| SegFormer-B0 FP16 at 5 Hz (if gated in) | 0.15 | 8 | 0.6 | 7 | ~12 ms est. |
| lidar_view render + NVENC, while watched | 0.2 | 30 | 0.5 | in baseline | measured |
| **Projected, Stage 3 in flight, unwatched** | **~5.3** | **~15** | **~6.5** | | target ≤ 5.5 / ≤ 15 % / < 7 GB |

Where the Orin NX will run out, so the stages test it rather than assume it:

1. **Memory bandwidth before TOPS.** 102 GB/s is shared by eight cores and the
   GPU; LiDAR mapping, ESDF updates and registration are bandwidth-bound. The
   symptom is the replan period growing jittery when mapper, detector and render
   run together, which is why the ledger carries p99s.
2. **RAM is fine for points, tight for distance fields.** The 4 M-point map is
   64 MB; a hashed TSDF/ESDF at 10 cm is gigabytes per kilometre of free space.
   Local fine, global coarse; the clearing radius is the knob.
3. **The CPU limit is self-inflicted today.** Recover the executor and composition
   cores before any hardware wish; every new node is rclcpp.
4. **GPU is a scheduling problem, not a capacity problem.** ESDF (8–10 %), render
   (30 % when watched), one small CNN at 10–15 Hz (5–10 %) fit; ESS-full (52 %) or
   Depth-Anything-class (50 %) models do not and compete with sensors already
   carried. INT8 CNNs can move to the two DLAs.
5. **Thermal is the real envelope.** The module runs MAXN with clocks pinned; the
   fan now holds its rated 6 000 rpm (profile `max`), which bought 7 °C at the same
   load. An Orin NX 16 GB at full load has been reported at 32 W / 82–85 °C against
   a 99 °C throttle. Rules: VDD_IN and tj in every bag; the budget is set from the
   hottest flight; a 30 min sealed soak with the full stack before it flies
   (Stage 3 gate); the enclosure air path (defined inlet and exhaust through the
   module's fins) is the next thermal lever, not a bigger fan.

## 5. Learned versus classical, function by function

The split is decided per function against a measured deficit, never by default.
On an aircraft with two LiDARs, hardware stereo and RTK, depth, odometry and
mapping are measurements; learning earns a slot where geometry cannot answer the
question (semantics, inertial coasting, anomalies, thermal classification) and
only ever as an advisory layer in this programme. **No learned component holds
authority in any stage of this plan.**

| Function | Approach | Why | When |
|---|---|---|---|
| Flying state estimate | classical (EKF2 + RTK + heading) | best estimator on the aircraft; the failsafes trust it; `EKF2_EV_CTRL` stays 0 | permanent |
| Map pose (LiDAR-inertial, visual) | classical (FAST-LIO2, cuVSLAM) | sub-core, understood failure modes, watchdogged | now; Stage 1 validates |
| Free space / ESDF | classical (nvblox GPU, ROG-Map fallback) | exact geometry with explicit unknown; learned occupancy hallucinates free space | Stage 3 |
| Global route, local trajectory | classical (A*/JPS, FIRI, MINCO) | deterministic, feasibility-checked, explainable on camera | Stage 3 |
| Tracking / inner loops | classical (PX4 only) | a companion controller bypasses PX4's failsafe-aware loops | permanent |
| Collision avoidance layer | classical (PX4 CP, later a CBF filter) | provable; a learned filter has no replay-checkable assurance | 4a, 5 |
| Terrain geometry, landing geometry | classical (grid_map filters) | slope/roughness/clearance are centimetre LiDAR facts | Stage 3 |
| Landing surface semantics (water, crop, road) | **learned** (SegFormer-B0) | geometry cannot tell a pond from a lawn; auto-labelled from LiDAR projections | 7, gated |
| People / animals / vehicles | **learned** (YOLO26-n/s; thermal on the Hadron) | no geometric answer; thermal works at night and through smoke | Hadron lane, 7 |
| Wildfire hot spots | classical (radiometric thresholds + persistence) | T-linear counts are temperature; thresholding is trivial and exact | Hadron lane |
| IMU dead-reckoning across LIO/GNSS gaps | **learned** (Air-IO-style), deferred | the one odometry case where learning clearly wins; needs a measured dropout first | 7, gated |
| Dense depth / stereo networks | defer | compete with sensors carried; decide after the D555 outdoor validity number | likely never |
| Telemetry / sensor anomalies | hybrid (rules first, LSTM-AE/IsolationForest advisory) | rules catch the known faults; the model catches the unnamed ones | 7 |
| Mission executive | classical (BehaviorTree.CPP, lifecycle) | readable, loggable, replayable | 5 |
| VLM task advisor | never in the loop | Qwen3-VL-2B INT4 runs on the Orin at 1–3 s/query; a demo, not a capability | — |

By count that is three or four learned nodes among roughly twenty; by GPU time,
learned perception may use 10–25 % of the GPU when it is all gated in; by
safety-relevance, none.

## 6. Open-source picks (verified 2026-10-03)

| Function | Pick (pin) | Alternative | Note |
|---|---|---|---|
| PX4 message contract | `px4_msgs` release/1.17 @ 86d8239 (already pinned) + XRCE Agent 2.4.3 (frozen upstream) | none | move to release/1.18 with the FC, never main (`vehicle_status` becomes `_v4`) |
| Companion mode plumbing | px4-ros2-interface-lib **release/1.17 @ 4a3370f0**, vendored, C++ only | raw Offboard via `RC_MAP_OFFB_SW` (Stage 4b first) | main/2.x and the Cloudsmith debs are message-incompatible with v1.17 |
| Free space / ESDF | isaac_ros_nvblox 4.6.0 (core 0.0.10), LiDAR-only, 10 cm | ROG-Map in C++ (GPL, fetch-at-build) | Jazzy line frozen at 4.6 (5.0 is Lyrical); ETH fork shows Isaac is removable |
| Trajectory back-end | GCOPTER headers @ e0444f6 + two issue-#16 patches (MIT) | aerostack2/gcopter_trajectory_generator_lib; SUPER as reading | upstream dead since 2023-06; test quickhull on the target |
| Front-end search | own A*/JPS, ~200 lines, unknown = blocked | dyn_small_obs_avoidance-ros2 as a comparison planner | OMPL RRT* in GCOPTER's demo is a placeholder |
| Terrain / landing | ros-jazzy-grid-map 2.2.2 + own E1R accumulation node | elevation_mapping_cupy (rclpy, cupy-cuda12x, open Orin issues) | ROS 2 line has one maintainer in maintenance mode; vendor the tag if apt stalls |
| Record / replay | rosbag2 0.26.11 MCAP (deployed) | pure-Python mcap / rosbags; RoboStack jazzy on the Mac | Jazzy bugs #2383, #2354, #2463 worked around |
| Odometry evaluation | evo 1.37.1 (or ≥ bc52497d for interpolation sync) | rpg_trajectory_evaluation (dead) | reference must carry yaw (PoseWithCovarianceStamped) |
| Collision prevention | PX4 v1.17 CP via `/fmu/in/obstacle_distance` | NTNU composite_cbf re-implemented on px4_msgs | first authority step |
| Executive | BehaviorTree.CPP 4.10 (apt) + nav2 lifecycle/bond pattern | py_trees_ros | frameworks that own the aircraft rejected |
| HIL rehearsal | PX4 SIH on the real 6X (`SYS_HITL 2`, props off) | PX4 SITL + Gazebo Harmonic on a Linux box (Livox plugin) | pre-flight `SYS_HITL == 0` check mandatory |
| Learned (gated) | YOLO26-n/s INT8, SegFormer-B0 FP16, Air-IO algorithm, sklearn anomaly | RT-DETR; ESS-light only on a measured deficit | Isaac ROS 4.6 is the last Jazzy line for Orin |
| Thermal | vendored ctu-vras/flir_boson_usb v3.0.0 + telemetry/sync; LVT2Calib offline | usb_cam (Y16, no telemetry) | see hadron-thermal.md |

**Build list** (own code, C++): the Offboard executor and tracker, the
`/fmu/in` guard and MAVLink-source check, the `obstacle_distance` publisher, the
A*/JPS front-end with the unknown-space gate and thin-obstacle layer, the E1R
grid accumulation node, the companion monitor, the scorer and replay harness, the
ulog→MCAP converter, the frame monitor, the Hadron telemetry/sync pairing and the
trigger MCU firmware; the ESDF if nvblox fails its gate or ownership is preferred.
Everything else above is pinned and vendored. From nav2, pull BehaviorTree.CPP +
BehaviorTree.ROS2, `nav2_lifecycle_manager` and the `nav2_util` LifecycleNode and
pluginlib patterns, as robot_localization was pulled; leave costmaps, planners,
controllers and smac.

Rejected for this aircraft, with reasons recorded in the landscape appendix:
Aerostack2, MRS UAV System, AirStack, aerial-autonomy-stack (patterns borrowed,
code not), DYNUS (Gurobi), MPPI planners (no ROS 2 quadrotor node; CUDA 10–12 era),
the ROS 1 legacy (Fast-Planner, FASTER/MADER/PANTHER, PX4-Avoidance archived),
learned occupancy/scene completion, VLA/VLM in the loop, EKF2 external-vision
fusion for this campaign.

## 7. Decisions only the owner can make

1. **Geofence and Return per site.** Enlarge `GF_MAX_HOR/VER_DIST` and
   `RTL_RETURN_ALT` for the campaign, or fit every pattern inside 50 × 30 m?
2. **Is GNSS-denied flight a goal?** It decides `EKF2_EV_CTRL`, Air-IO, cuVSLAM's
   future and the frame-contract emphasis. If not, LIO stays a map pose only.
   *Decided 2026-10-04: no.* `EKF2_EV_CTRL` stays 0, nothing from the Jetson
   enters the FC's estimator, LIO is a map pose in `camera_init`, the
   `camera_init → px4_local` monitor is what every planner output converts
   through, and cuVSLAM/Air-IO drop to "evaluate only if cheap".
3. **D555 in the bag** — *decided 2026-10-03: LiDAR-first for now.* A throttled
   IR pair or depth copy (a second unicast subscriber on the camera's link,
   measured against the encoder) stays optional; without it cuVSLAM and any
   vision stage are evaluated only from live odometry, never from replay.
4. **QGC Survey missions** during the campaign (Mission is not on an RC slot) as the
   repeatable label?
5. **RC allocation:** which of ch 12/13 is the marker and which the Offboard
   switch; mapped how; reachable without moving a stick?
6. **Flight budget:** packs per session, sessions per week, season end. The merged
   gates need ~30–60 flights before an external mode; which gates to relax?
7. **Storage and replay host:** NVMe capacity, how ~50 GB per flight hour reaches
   the Mac, where the archive lives, whether an x86 Linux (NVIDIA) host exists for
   replaying the rclcpp stack. *Decided 2026-10-04: the owner's "Atomic"
   workstation (x86, NVIDIA, 4 TB NVMe, Isaac Sim installed) is both the archive
   and the replay host; it becomes an Ansible target so the replay stack is the
   same roles as the aircraft.*
8. **Firmware:** which of the four commits on `uav/v1.17.0-pps` fly; how many
   flashes per campaign; maintain a bench variant with the registration topics?
9. **RTK in the field:** correction source, expected fixed rate, antenna lever
   arm measured?
10. **Deliberate RC-off tests:** acceptable at all, or every failsafe path by
    GCS/RC-slot command and SIH only? (Recommended: the latter.)
11. **Collision Prevention on a forward-only suite:** willing to make the companion
    flight-critical for it, knowing it protects forward approaches only?
12. **Recorder pre-roll:** accept losing the first ~2 s after arming, or start the
    bag from the RC switch before every arming?
13. **Which milestones become videos**, which fixes what needs a visible demo
    rather than a scorer number.

Decided 2026-10-03: standard Offboard mode, no custom mode until the end (§2,
Stage 5); build-or-buy per component with ground-up implementation accepted where
nothing maps exactly (§6); nav2's executive and lifecycle pieces pulled in;
LiDAR-first; the Avia's time source is PPS + UTC, not a move to the PTP segment.

## 8. Hardware futures (not for this programme)

- **Jetson T2000 PAB baseboard** (announced 2026-07-15; module Q1 2027; ~50 × 87 mm,
  so a new Holybro carrier): 2 × 10 GbE with hardware timestamping would retire the
  i226-as-grandmaster and `ptpd` software-timestamp arrangements and the 1 GbE
  bandwidth ceiling; 400 FP4 TFLOPS changes the learned-perception question;
  memory bandwidth only 102 → 137 GB/s; 40 W needs a conduction path. The one
  line item to put in front of a carrier designer: **one AON-domain GPIO on a
  header** (the Orin NX baseboard has none, so the GTE/HTE cannot be used, which
  is why the Hadron trigger is an MCU).
- **A PTP/PoE TSN switch** (BotBlox RouterCore / RouterBlox Max, slipped past its
  Q1 2026 ETA): PTP on every port plus a PPS input would retire the i226 card and
  give per-port power cycling for sensor recovery; 10 GbE only pays with a 10 G
  companion. With the Avia on PPS it serves a future second gPTP sensor, not a
  present need. Same single-failure-domain trade as merging power rails.
- **An octorotor airframe (EFT X2300 class)** is what makes "assured landing after a
  propulsion failure" a safety case rather than a convenience; on the X950 a motor
  failure is a flight-termination event and the landing-site machinery serves the
  planned contingencies only. Common-mode failures (CAN bus, battery, a BEC rail)
  are unchanged by rotor count; the PWM backup path for the ESCs is the one
  propulsion redundancy a quad can have (still an open item).

## 9. Risk register (top ten)

| Risk | Mitigation |
|---|---|
| A stray `/fmu/in` publisher steers the aircraft in Position mode | firmware deletion of the subscriptions; guard; MAVLink-source check; replay never with the agent up; bag assertion |
| FMU module / Ethernet mid-flight loss (PAB X2, PPS back-power) | Stage 0a retention and cold-boot gate; TELEM2 keeps the recorder keyed; 4b monitor stops the stream so PX4's fallback fires |
| FAST-LIO restart re-anchors the frame and poisons the ESDF | `/lio/health` → "map pose invalid"; ESDF cleared/re-anchored; planner holds |
| nvblox 3D ESDF bug on Jazzy/CUDA 13 | Stage 3 entry bench test; ROG-Map fallback |
| Thin obstacles invisible to TSDF and stray filter | thin-obstacle layer from raw Avia hits; wire survey on the site card |
| Collision Prevention over-trusted (14 of 72 bins) | covered-sector mask; procedure states the protected fraction; altitude-band filter |
| Thermal in the sealed airframe | VDD_IN/tj every bag; 30 min soak gate; enclosure air path; fan `max` |
| Timesync bias / PX4 local-frame resets corrupt comparisons | PPS-residual correction kept; RTT logged; reset counters tracked and honoured |
| Recorder silently stops (rosbag2 #2463) | stalled-growth alarm with a fault test |
| Gate erosion over a long campaign | gates as scripted checks producing a pass/fail file per flight; the GSN case links them; the channel's audience is the second reviewer |

## 10. What to do this week (before the first campaign flight)

1. Stage 0a hardware and parameter items, geofence per site, RC map, the
   `SYS_HITL`/checksum pre-flight item.
2. The firmware commit (publications at 50 Hz; `/fmu/in` subscriptions deleted;
   bench variant), flashed and verified with `ros2 topic list` — needs the owner's
   go-ahead for the firmware work.
3. Recorder hardening, flight cards, the ulog→MCAP converter, `EventsExecutor`
   A/B, the README corrections.
4. Answers to §7 items 1–3 and 6–7; they decide the shape of Stages 1–3.
