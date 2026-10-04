# PX4 configuration for the X950 maiden flight

State of the Pixhawk 6X on 2026-10-01, firmware `v1.17.0-1-g27174e09f5`
(`uav/v1.17.0-pps`, worktree `PX4-Autopilot-fc`). That branch has since grown:
three Septentrio heading fixes (2026-10-02) and, on 2026-10-04, the
uXRCE-DDS topic change for the autonomy programme (`trajectory_setpoint` and
`vehicle_local_position_setpoint` published at 50 Hz; every `/fmu/in`
subscription except `message_format_request` removed, see
[the autonomy roadmap](autonomy-roadmap.md)). The flashed version is read back
with `ver all` before each campaign and recorded here; the FC may lag the
worktree. The parameter set, applied and
proposed, is in [px4/x950-maiden.params](px4/x950-maiden.params). A full dump of
the 1161 parameters as they were before this pass is on the Jetson at
`/home/john/uav-probes/params.json` (made with `uav-probes/paramdump.py`).

The FC is not managed by this playbook. This note exists so the maiden-flight
decisions are written down next to the companion configuration they interact
with (flight recorder, bridge, time base).

## Applied on 2026-10-01

| Parameter | Was | Now | Why |
| --- | --- | --- | --- |
| `CA_R_REV` | 15 | 0 | All four motors were marked reversible. The allocator then publishes negative per-motor thrust at low collective and the UAVCAN ESC driver casts that straight into a signed `RawCommand`, i.e. a reverse-spin request to the A1C. A standard quad needs 0. |
| `COM_LOW_BAT_ACT` | 0 | 3 | Warning only → Return at the critical level, Land at the emergency level. |
| `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` | 0 / 0 | 50 m / 30 m | Geofence was configured (`GF_ACTION=3`, Return) but had no limits. |
| `MPC_MAN_TILT_MAX` | 35° | 25° | Caps manual acceleration to about 0.47 g for the first hover. |
| `MPC_MAN_Y_MAX` | 150°/s | 60°/s | Yaw authority of a large quad with `KM=0.05` is low; 150°/s saturates the allocator. |
| `MC_YAWRATE_MAX` | 200°/s | 90°/s | Same reason, applies to all modes. |
| `SDLOG_PROFILE` | 1 | 19 | Default + estimator replay + high-rate rate-controller logging for the first flights. |

Verified after an FC reboot: `commander check` → `Preflight check: OK`,
`actuator_motors.reversible_flags = 0`, values persisted.

## Applied on 2026-10-02 (UTC)

| Parameter | Was | Now | Why |
| --- | --- | --- | --- |
| `COM_FLTMODE1..6` | Mission, Altitude, Manual, Hold, Takeoff, Land | Stabilized, Altitude, Position, Position Slow, Land, Return | No Position mode was reachable, and Mission sat on slot 1. The order is the fallback ladder: Stabilized needs only attitude, Altitude adds height, the Position family adds horizontal position and yaw. |
| `COM_RC_IN_MODE` | 2 | 5 | 2 keeps whichever manual-control source it has until that source times out. RC switches (mode slots, arm, kill) are only processed while the source is RC, so a MAVLink joystick that took over during an RC dropout would have kept it, kill switch ignored, after the RC came back. With 5 the RC wins whenever it is valid. |

Applied by the owner the same day (read back from the FC):

| Parameter | Now | Why |
| --- | --- | --- |
| `RC_MAP_AUX3` / `MC_SLOW_MAP_HVEL` | 14 / AUX3 | Dial 1: Position Slow speed limit, linear from `MC_SLOW_MIN_HVEL` (0.3 m/s) to `MPC_VEL_MANUAL` (10 m/s); with a dial mapped, the fixed 3 m/s default no longer applies. |
| `RC_MAP_AUX4` / `MC_SLOW_MAP_YAWR` | 15 / AUX4 | Dial 2: Position Slow yaw-rate limit, 3°/s to `MPC_MAN_Y_MAX` (60°/s). |
| `RC_MAP_AUX1`, `RC_MAP_AUX2`, `MC_SLOW_MAP_PTCH` | 0 | The old gimbal nub passthrough, retired. |
| `PWM_AUX_FUNC4` | 0 | Was 1 (Constant Min) after the CAN-only change; now disabled like AUX1-3. |
| `RC_MAP_ROLL` | 1 | Briefly 10 after these changes: channel 10 is a switch resting at 1049 µs on an uncalibrated 1000/1500/2000 range, so PX4 read roll -0.90 at rest. Armed, that is a hard left bank on lift-off, the X650's crash on this same FC. |
| `RC4_TRIM` | 1499 | Was 1448 against a stick resting at 1499: yaw read +0.10 at rest. Radio recalibrated. |

Channels 11-13 drive the companion: 11 records a bag (flight recorder),
12 takes a photo and 13 starts/stops recording (camera node); see those roles'
READMEs. Return remains on mode slot 6 (`RC_MAP_RETURN_SW=0`).

Before every arming, with the sticks centred, `listener manual_control_setpoint`
(or QGC's radio page) should read roll, pitch and yaw at 0. That check catches
a wrong stick mapping or trim, which nothing else does before lift-off.

How the transmitter interacts with these, from the v1.17 source:

- Mode buttons are edge-triggered (`ManualControl.cpp`): pressing the button of the
  slot already latched does nothing. After a failsafe has changed the mode, take
  over with a *different* button. During the 5 s `COM_FAIL_ACT_T` hold only a mode
  change works; once the action runs, sticks past 30 % also work
  (`COM_RC_OVERRIDE=1`, not during a critical-battery reaction).
- When the FC first sees the RC while disarmed, the mode is taken from the latched
  slot.
- Arming throttle: in Stabilized the stick must be in its bottom 10 %
  (normalised < -0.8); in Altitude/Position anything up to 0.2 (60 % of travel)
  is accepted. Within 5 s of a disarm, switch re-arming skips these checks.
- `MPC_THR_CURVE=0`: in Stabilized a centred throttle commands the hover-thrust
  *estimate*, which starts at `MPC_THR_HOVER` (0.5) until it has learned the real
  value. With a spring-centred throttle, releasing it after arming in
  Stabilized asks for that thrust.
- Manual (0) and Stabilized (8) are the same on a multicopter (identical control
  flags in `control_mode.cpp`); Stabilized is used for the clearer label.

## Found, left for the owner

- **Arming was blocked by a stale `cpuload` topic** ("No CPU and RAM load
  information", health component `system`). `load_mon` only publishes while the
  NuttX idle-task runtime keeps changing; the CPU-load monitor reference count
  had reached 0 during the previous boot (last publish at t=49 s). Restarting
  `load_mon` does not help; a reboot does. Do not run `top` on the FC before a
  flight. Disabling the check (`COM_CPU_MAX=-1`, `COM_RAM_MAX=-1`) is possible
  but was not done.
- **Two motor command paths are live.** `UAVCAN_EC_FUNC1..4=101..104` (the real
  path, telemetry on `esc_status` instance 1) and `PWM_AUX_FUNC1..4=101..104`
  with `PWM_AUX_TIM0=-3` (DShot600) plus `DSHOT_BIDIR_EN=1`. The A1C PWM input
  does not decode DShot (bidirectional telemetry: 650 k frames, zero decoded),
  and the DShot driver publishes `esc_status` instance 0, which is the only
  instance the ESC arming check reads. Either go CAN-only (proposed lines in the
  params file) or make the fallback honest: `PWM_AUX_TIM0=400`, `PWM_AUX_MAX*`
  1940, and prove on the bench, no props, that a motor keeps running when the
  CAN bus is disconnected mid-spin.
- `NAV_DLL_ACT=2` returns on GCS link loss after 10 s. For an RC-flown maiden
  on the Siyi link (RC and MAVLink share the radio) the RC-loss failsafe
  already covers the radio; a GCS-only drop would otherwise trigger a return.
- Return levels off at the geofence ceiling: `RTL_RETURN_ALT` 30 m and
  `GF_MAX_VER_DIST` 30 m (the latter from the 2026-10-01 pass). With
  `GF_PREDICT=0` only an actual crossing trips the fence, but any overshoot at the
  top of the climb is one, raising a `GF_ACTION=3` breach during the Return
  itself. Keep the return altitude a few metres under the ceiling, or raise the
  ceiling, according to the field's obstacles.
- `EKF2_OF_CTRL=1` fuses H-Flow velocity. `SENS_FLOW_ROT=0` matches the URDF
  (`hflow_link` has zero rotation) but has never been checked in motion. Check:
  carry the vehicle 1 m forward/left at under 2 m above the floor and compare
  `estimator_aid_src_optical_flow.innovation` against GNSS velocity.
- `GPS_1_CONFIG=202` and `SEP_PORT1_CFG=202` both claim GPS2. The Septentrio
  driver wins the port today (`gps status` → not running).
- `BAT1_V_CHARGED=4.2`, `BAT1_V_EMPTY=3.6`: confirm against the MAD semi-solid
  pack's datasheet. `UAVCAN_SUB_BAT=2` means PX4's own estimator runs on the
  PM08's voltage/current, so these two numbers define the SOC and the failsafe
  thresholds. The PM08 current read 1.68 A against about 1.4 A on the Rigol.

## Before the first spin with props

1. **Idle thrust.** The no-load motor tests gave RPM ≈ 3168 + 3213 × command,
   i.e. about half of maximum speed at zero command. If the A1C holds that as
   a speed floor under load, a 24-inch prop at ~3200 RPM produces roughly a
   quarter of maximum thrust per motor, and the vehicle could be close to
   weightless at idle. Strap the airframe down, put it on a scale, arm in
   Stabilized with the throttle at minimum, and read the weight loss.
2. **Start threshold.** Map the command below which the ESC stops and the RPM
   it jumps to when it starts (`/home/john/uav-probes/escdeadzone.py`, motor
   test, no props, 12 s per point). Then set `UAVCAN_EC_MIN1..4` above that
   threshold so the allocator can never command a motor into the stop band in
   flight.
3. **RC.** `input_rc` has never been published on this boot (transmitter off).
   Check channel directions, the six mode positions, the arm switch (ch 6) and
   the kill switch (ch 9) before anything spins.

The GNSS heading source, antenna offset and field calibration are handled by
the owner and are deliberately not covered here.
