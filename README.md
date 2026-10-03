# uav_ansible

Repeatable, version-pinned provisioning for a **Jetson Orin NX 16GB** UAV
companion computer on a **Holybro Jetson Baseboard**, flying with **PX4**.

This playbook takes a freshly-flashed Jetson and converges it into a
**PTP-grandmaster, PX4-capable ROS 2 host** — every system-level step automated
so the build is reproducible for filming and for anyone following along on the
channel.

---

## What this does / does not do

This is **Layer 1** provisioning (configure an already-flashed machine). It
deliberately does **not** cover:

- **Layer 0 — flashing JetPack** → manual, documented below. It's a once-per-board
  host-side operation; automating it hides the recovery-mode ritual you actually
  need to learn. The flashing doc is the reproducibility anchor for the OS.
- **The ROS 2 application workspace** → a separate repo. Cloning + `colcon` is
  application code, not provisioning.
- **A golden disk image** → skipped. The playbook is the source of truth; a
  one-time NVMe clone after first reaching golden state is your only "parachute."

**Definition of done:** with the flight controller connected,
`ros2 topic list` shows PX4 topics; the box serves PTP (gPTP) to sensors and NTP
to the LAN; GNSS time + RTK corrections flow at boot independent of any GCS
(`drone-link-connect` enabled at boot).

---

## Hardware target (pinned)

| | |
|---|---|
| Compute | Jetson Orin NX 16GB |
| Carrier | Holybro Jetson Baseboard (integral Ethernet switch → Realtek r8168) |
| OS | JetPack 7.2 · Ubuntu 24.04 (Noble) · L4T `<confirm>` |
| ROS | ROS 2 Jazzy |
| Autopilot | PX4: uXRCE-DDS over Ethernet/UDP; MAVLink over the baseboard-internal UART (TELEM2) |
| Timing NIC | Intel i226 (M.2) — PTP grandmaster PHC |
| LiDAR | Robosense E1R (gPTP / 802.1AS, on the i226 segment) · Livox Avia (drone LAN, `192.168.144.80`) |
| Depth camera | Intel RealSense D555 (native DDS over Ethernet, `192.168.144.70`) |
| GNSS | Septentrio mosaic-G5 (USB serial, SBF) |
| Air link | Siyi UniRC7 (forces `192.168.144.0/24`) |
| Lights | RP2040 over PWM (GPIO12) |

---

## Architecture decisions (the "why", so future-me doesn't relitigate)

- **PX4 link: uXRCE-DDS over Ethernet/UDP; MAVLink over the internal UART.**
  The baseboard's integral switch gives DDS the bandwidth/CPU headroom serial
  (≤92 KB/s) doesn't. MAVLink is small, and on the UART (Pixhawk TELEM2 ↔
  Jetson `/dev/ttyTHS1`) it has exactly one peer: no UDP partner latch to lose
  (next bullet) and no share of the FMU's Ethernet transmit buffers, which
  XRCE needs (see `px4_link`, transmit keepalive).
- **Two Ethernet segments, never bridged.** `192.168.144.0/24` (r8168 + Siyi +
  PX4) for everything; a separate L2 segment on the i226 for PTP. gPTP (802.1AS)
  is link-local and a vanilla Linux bridge is **not** a transparent clock —
  bridging would wreck sync. The Jetson is multi-homed and routes at L3; DDS
  discovery across segments uses a Fast-DDS Discovery Server, not bridged multicast.
- **Addressing.** Jetson `192.168.144.1` (the de-facto L3 gateway — it owns the
  egress, so `.1` is conventional, not a hack), PX4 `192.168.144.2`,
  XRCE agent on UDP `8888`. The 144 interface carries **no default route**. It
  came from the USB wifi dongle until 2026-10-02; since the airframe was sealed
  the dongle is out and the default route is a TUN device (`gcs0`) whose traffic
  leaves through a SOCKS5 server on the Siyi GCS (`gcs_gateway`), with operator
  access over Tailscale (`tailscale`). The home wifi subnet must not overlap
  `192.168.144.0/24` or the i226 `.145.0/30` segment.
- **Time topology** (GPIO PPS lands in the *system-clock* domain, so chrony owns it —
  `ts2phc` is out because the i226 card exposes no SDP/extts pin):
  ```
  mosaic USB2 SBF ──fan-out──uav-gnss-time──SOCK(coarse)──┐
  mosaic 1PPS ──GPIO07──/dev/gnss-pps──PPS(lock GPS)───────┴──chrony──> system clock ──phc2sys──> i226 PHC ──ptp4l(gPTP GM)──> Robosense E1R
                                                                   └──> serves NTP to the LAN
  ```
  The SBF source says which second it is (ReceiverTime, only once the receiver
  clock is synchronised: FINETIME); the PPS says exactly where it begins.
  Exactly one controller per clock (chrony→system, phc2sys→PHC, ptp4l distributes)
  — no contention. RTC = boot/holdover fallback only, and a cold power-off
  loses it on this carrier: the boot clock reads 1970 until chrony steps it.
  phc2sys therefore waits (bounded) for chrony to be synchronised before it
  initialises the PHC, and steps the PHC (`-S 1`) if the system clock is
  stepped later; without both, a PHC set from the 1970 clock stayed 56 years
  behind, slewing at the i226's 6.25 % limit (observed 2026-09-30).
  Chrony's configured NTP sources can supply the common bench clock while GNSS
  is unavailable; this does not establish GNSS-level absolute accuracy.
  `phc2sys -w` obtains `currentUtcOffset` from ptp4l (no forced `-O 0`), keeping
  the system in UTC and the PHC in PTP/TAI, currently UTC + 37 seconds.
  The E1R uses RoboSense's static gPTP master profile: 8 Hz Sync, 802.1AS
  Follow_Up information, fixed master, and suppressed Announce/master delay
  requests. See [E1R manual section 2.4.2](https://robosense-robotics.github.io/product-manual/en/E1R/#242-use-linuxptp-tool-to-verify-time-synchronization).
  Bench verification on 2026-09-25: all 750 DIFOP packets in 15 seconds reported
  mode 3 (gPTP), status 1 (success); all 43,201 MSOP packets reported mode 3,
  with synchronized frame status in inspected samples. The PHC and raw E1R
  timestamps followed PTP/TAI, not UTC. These observations verify acquisition
  and the timescale, not submillisecond sensor accuracy. **ROS integration must
  convert raw PTP/TAI to UTC using the verified UTC offset for cloud headers,
  absolute per-point timestamps, and IMU headers together.** The upstream
  RoboSense driver does not perform that conversion or reject unlocked sensor
  time automatically. The E1R ROS driver is not deployed yet; do not treat
  `use_lidar_clock=true` alone as a synchronization check.
- **GNSS: two USB ports, one owner each, SBF only.** The mosaic's USB2 streams
  SBF out; a tty hands each byte to exactly one reader, so the **fan-out**
  (`uav-gnss-broker`) is its only reader and re-serves the stream, **read-only**,
  on `127.0.0.1:28785` to the time feed, the Septentrio ROS driver (rich data:
  NavSatFix + baseline heading + full covariances, which only SBF carries) and
  diagnostics. USB1, the command port, belongs to **drone-link**, the one RTCM
  writer: under systemd at boot, GCS-independent, enrolled from the station (see
  *RTK corrections*). No gpsd: it cannot decode SBF, and the receiver speaks
  nothing else, so the time comes from SBF too (`uav-gnss-time`).
- **MAVLink goes through one router on the Jetson, never straight to PX4.**
  `mavlink-router` holds the only MAVLink session with PX4 — its TELEM2
  instance on the internal UART (`/dev/ttyTHS1`, 921600 = `SER_TEL2_BAUD`) —
  and fans it out: QGC over TCP 5760 on every interface (any number of GCSs),
  onboard apps on localhost UDP 14540. PX4's Ethernet MAVLink instance is
  retired (`MAV_0_CONFIG=0` on this FC): a UDP MAVLink instance latches the
  *first* peer that talks to it (3 s after start) and unicasts only to that
  address:port until it reboots, so any GCS on the drone LAN can win the race
  and leave the router deaf — which is exactly what the bench showed on
  2026-09-30. `mavlink_router_px4_link: udp` keeps the fixed-port
  server-endpoint answer to that race (a server on `0.0.0.0:14550` that also
  hears PX4's discovery broadcast to `.255`) for a build without the UART; PX4
  then books its own broadcasts as TX errors and throttles every stream to 5 %
  until it latches, so a sparse broadcast is normal, not a fault.
- **Real-time: tune, don't rebuild.** PX4 owns the hard loops on the FMU; the Jetson
  is supervisory/offboard. `rt_tuning` (CPU isolation, SCHED_FIFO, mlock, IRQ
  affinity, governor, `preempt=full` toggle) covers it. A full `rt_kernel` is a
  parked, measurement-gated option — not the default, because it breaks the
  DKMS/apt-kernel reproducibility story.

---

## Layer 0 — flashing JetPack 7.2  (manual, do this first)

> This is the one step the playbook can't do — it runs from an **x86 Ubuntu host**
> over USB with the board in recovery mode. Capture exact versions here so the build
> stays reproducible.

1. **Host:** Ubuntu 22.04/24.04 x86 with NVIDIA SDK Manager (or the L4T
   `flash.sh` / `l4t_initrd_flash` scripts).
2. Put the Orin NX in **Force Recovery** (recovery jumper on the Holybro carrier)
   and connect USB-C to the host. Verify with `lsusb` (NVIDIA Corp APX device).
3. Flash JetPack **7.2** to the **NVMe** (Orin NX has no eMMC). Pin the exact
   L4T/BSP rev once flashed: `cat /etc/nv_tegra_release` → record in
   `group_vars/all.yml` (`l4t_release`).
4. First boot, finish `oem-config` (create the `target_user`), enable SSH.
5. Proceed to **Usage** below.

*(Expand with the on-camera walkthrough / screenshots.)*

---

## Usage

### One command (recommended for viewers — `ansible-pull`)
Runs the playbook locally on the Jetson; no control node needed.
```bash
curl -fsSL https://raw.githubusercontent.com/smyrgl/uav_ansible/main/bootstrap.sh | bash
```

### Manual local run
```bash
ansible-galaxy collection install -r requirements.yml
sudo ansible-playbook -i inventory/localhost.yml site.yml
```

### Run / re-run a single role by tag
```bash
sudo ansible-playbook -i inventory/localhost.yml site.yml --tags time_sync
```

> **Reboots:** `kernel_modules` and `device_tree` may require a reboot to take
> effect — re-run the playbook after rebooting; roles are idempotent.

> **Network cutover:** `networking` moves the drone LAN to its static
> `192.168.144.1` only once the wifi uplink holds a default route; if it never
> does, the play stops with the LAN untouched. An SSH session riding the LAN
> (e.g. the Holybro switch patched into a home network) drops at the cutover,
> so run that converge detached (`tmux`) and reconnect on the uplink address
> the play prints.

### Connecting QGroundControl
QGC talks to the router, never to PX4 directly (see *Architecture decisions*).
In QGC: **Application Settings → Comm Links → Add**, type **TCP**, host = the
Jetson's uplink address, port **5760**, then **Connect**. For push-style
auto-connect (QGC listening on UDP 14550), list a fixed-address GCS in
`mavlink_router_gcs_endpoints`, or, for a bench laptop on DHCP, drop an
unmanaged file on the Jetson that survives converges:
```ini
# /etc/mavlink-router/config.d/50-bench.conf
[UdpEndpoint bench]
Mode = Normal
Address = <laptop IP>
Port = 14550
```

### Secrets
Wi-Fi credentials and anything sensitive live in `group_vars/vault.yml`
(git-ignored). `site.yml` loads it explicitly: `group_vars/` only auto-loads
files named after an inventory group, and there is no `vault` group. Copy
`group_vars/vault.yml.example`, fill it in, and encrypt:
```bash
cp group_vars/vault.yml.example group_vars/vault.yml
ansible-vault encrypt group_vars/vault.yml
# add --ask-vault-pass to your playbook runs
```

### RTK corrections
Corrections reach the receiver through **drone-link**, not through this
playbook. Converge first; then, on your rtk-station's **Connectors** page, *Add
client*, and paste the one line it shows into a shell on the Jetson. That line
installs the drone-link binary, its client certificate and the
`drone-link-connect` unit, enabled at boot, writing RTCM straight into the
receiver's USB1 command port: give it
`serial:///dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_<serial>-if00:115200`
(`if00`; `if02` is USB2, the SBF port the fan-out owns, and the fan-out port
discards anything written to it).

- A converge **never touches `/etc/drone-link` or `drone-link-connect`**;
  `bootstrap.sh` does a `git reset --hard` and a full converge. The two never
  share a port, so a fan-out restart doesn't touch the corrections.
- After a re-flash, *Remove* the client on the station and *Add* it again — the
  old key is on the old disk.
- Why not a role: the enrollment spends a single-use token and mints a private
  key, so a converge cannot replay it, and a YAML copy of the bootstrap would
  drift from the one every other rover uses.
- The receiver's own profile (rover mode, message set, ports) is configured on
  the receiver, not by this playbook: a bench unit that was last a base still
  is one after a converge. Reset it and load the vendor's rover profile first.

---

## Roles

| Role | Purpose |
|---|---|
| `base` | apt baseline, locale/tz/hostname, swap, nvpmodel + jetson_clocks |
| `dev_tools` | git, git-lfs (+`git lfs install`), nano, common CLI tools |
| `networking` | netplan (networkd): wifi uplink first (gated; the dongle is in on the bench, out in flight, and its config stays installed either way), then the two un-bridged segments |
| `gcs_gateway` | egress through the Siyi GCS: hev-socks5-server on its Android (no root; ADB-supervised from the Jetson) + hev-socks5-tunnel (`gcs0`, default route, DNS) |
| `tailscale` | operator access from the tailnet over that egress (`jethawk`); pinned repository key |
| `jtop` | jetson-stats + `jtop` group (non-sudo access; handles PEP 668) |
| `kernel_modules` | **audit-first** DKMS (igc/ch341 likely in-tree; Wi-Fi dongle isn't) |
| `device_tree` | overlays: pps-gpio (GPIO07 → `/dev/gnss-pps`), PWM out (GPIO12) |
| `time_sync` | chrony (PPS + GNSS SOCK + NTP) · phc2sys · ptp4l gPTP grandmaster |
| `gnss` | read-only SBF fan-out + SBF→chrony time feed + udev; RTCM goes in on USB1 via drone-link |
| `ros2` | ROS 2 Jazzy `ros-base`, rosdep, colcon, global sourcing |
| `px4_link` | Micro XRCE-DDS Agent (UDP) as a systemd service · FMU Ethernet transmit keepalive |
| `px4_bridge` | PX4 uORB → ROS-native ENU/FLU topics (IMU, odometry for comparison, battery, status); no GNSS |
| `gnss_ros` | Septentrio ROS 2 driver as a read-only client of the SBF fan-out: NavSatFix (full covariance), GPSFix, pose, twist, attitude |
| `localization` | robot_localization REP-105 dual EKF (`ekf_odom`: odom→base_link, `ekf_map`: map→odom) + navsat_transform (utm→map, GNSS); PX4's own estimate stays in `px4_local`, informational |
| `vslam` | Isaac ROS cuVSLAM on the D555 stereo IR → `/vslam/odometry` (UTC, base_link twist); shadow source, nothing fuses it yet |
| `lio` | FAST-LIO2 on the Avia + its built-in IMU (GPL-2.0, fetched and built at a pinned commit) → `/lio/odometry`; shadow source, primary candidate; voxel map of the Avia and the E1R (registered with FAST-LIO's poses and the nominal extrinsic); a divergence watchdog (`/lio/health`) freezes and rolls back the map, stops the E1R registration and restarts FAST-LIO |
| `lidar_view` | GPU (EGL) chase-camera render of the LIO map, the live Avia and E1R scans and the X950's URDF → NVENC RTSP `:8555/lidar`, the camera's stream 2 in QGC; rendered only while watched |
| `hflow` | H-Flow over the FC's CAN2 bus, read listen-only on the Jetson's can0 → PX4-typed ROS 2 topics + Range |
| `mavlink_router` | MAVLink hub: PX4 (TELEM2 UART) ↔ QGC (TCP 5760) ↔ onboard apps (UDP 14540) |
| `rt_tuning` | soft-RT: isolation, SCHED_FIFO, mlock, IRQ affinity, governor |
| `rt_kernel` | **parked** — opt-in, measurement-gated PREEMPT_RT |
| `isaac_ros` | NVIDIA Isaac ROS apt repository and packages, pinned (4.6); back since 2026-10-02 for `vslam` (cuVSLAM) |
| `nvblox` | **retired** (`nvblox_state: absent`): the September 2026 nvblox POC; findings in the role README |

## D555 camera

The `mavlink_camera` role provides native ROS2 RGB, H.265 RTSP video, photos, and
onboard recording through MAVLink Camera Protocol v2. It runs as a persistent
user service on jethawk. See [camera operation and bench routing](roles/mavlink_camera/README.md).
The same camera component advertises a second stream, the LiDAR map view: a
third-person render of what the Avia and the E1R see
([lidar_view](roles/lidar_view/README.md)). QGC lists it in its stream selector.

## ROS sensor bench and visualization

The X950 description, E1R driver, Foxglove bridge and sensor diagnostics run as persistent systemd services on Jazzy. See [the ROS bench guide](docs/ros-bench.md) for the dashboard, timing limits, service controls and deployment steps.

## Airframe description (`x950_description/`)

The X950 model lives in this repository: `x950_description/x950_linkforge.blend`
(Blender 5 + LinkForge authoring scene, git-lfs), the CAD sources it was built
from (`cad_source_meshes/`, git-lfs) and the exported ROS 2 package (`urdf/`,
`meshes/`, `config/`, `docs/`). The `robot_description` role deploys the
exported package (`robot_description_source_dir`). To change the model, edit
the scene in Blender with the MCP extension running and export through the
package's own scripts from the repository root:

```bash
python3 x950_description/scripts/blender_rpc.py x950_description/scripts/export_linkforge.py
python3 x950_description/scripts/validate_description.py
```

See [x950_description/README.md](x950_description/README.md) for the pose
arguments, the sensor frames and the mesh-optimisation records. Moved here from
`Printables` on 2026-09-30; the H-Flow was the first sensor added in place.

## 3D reconstruction (nvblox) — retired

The nvblox POC ran on the bench in September 2026 (Isaac ROS 4.6 natively on
JetPack 7.2) and is taken down: `nvblox_state` is `absent`, so a converge
removes its units and helpers (the `isaac_ros` repository is back for `vslam`).
Its bandwidth was the deciding cost: every D555 subscriber receives its own
unicast copy of the images, and nvblox plus its colour adapter were two of the
five copies that saturated the drone LAN. What was learned (JIT compile cache,
LiDAR model maths, QoS, time bases) stays in [the nvblox role](roles/nvblox/README.md).
