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
to the LAN; GNSS time + RTK corrections flow at boot independent of any GCS.

---

## Hardware target (pinned)

| | |
|---|---|
| Compute | Jetson Orin NX 16GB |
| Carrier | Holybro Jetson Baseboard (integral Ethernet switch → Realtek r8168) |
| OS | JetPack 7.2 · Ubuntu 24.04 (Noble) · L4T `<confirm>` |
| ROS | ROS 2 Jazzy |
| Autopilot | PX4 over Ethernet (uXRCE-DDS / UDP) |
| Timing NIC | Intel i226 (M.2) — PTP grandmaster PHC |
| LiDAR | Robosense E1R (gPTP / 802.1AS, on the i226 segment) |
| GNSS | Septentrio mosaic-G5 (USB serial, SBF) |
| Air link | Siyi UniRC7 (forces `192.168.144.0/24`) |
| Lights | RP2040 over PWM (GPIO12) |

---

## Architecture decisions (the "why", so future-me doesn't relitigate)

- **PX4 link is Ethernet/UDP, not serial.** The baseboard's integral switch is the
  point; UDP has the bandwidth/CPU headroom serial (≤92 KB/s) doesn't. The serial
  link stays wired as a MAVLink/console debug fallback only.
- **Two Ethernet segments, never bridged.** `192.168.144.0/24` (r8168 + Siyi +
  PX4) for everything; a separate L2 segment on the i226 for PTP. gPTP (802.1AS)
  is link-local and a vanilla Linux bridge is **not** a transparent clock —
  bridging would wreck sync. The Jetson is multi-homed and routes at L3; DDS
  discovery across segments uses a Fast-DDS Discovery Server, not bridged multicast.
- **Time topology** (GPIO PPS lands in the *system-clock* domain, so chrony owns it —
  `ts2phc` is out because the i226 card exposes no SDP/extts pin):
  ```
  Mosaic SBF ──gpsd──SHM(coarse)──┐
  Mosaic 1PPS ──GPIO11──/dev/pps0──┴──chrony──> system clock ──phc2sys──> i226 PHC ──ptp4l(gPTP GM)──> Robosense E1R
                                          └──> serves NTP to the LAN
  ```
  Exactly one controller per clock (chrony→system, phc2sys→PHC, ptp4l distributes)
  — no contention. RTC = boot/holdover fallback only.
- **GNSS is Path B (single serial, broker).** The mosaic-G5 (unlike the X5) exposes
  no virtual COM ports, so one process owns the serial and tees SBF over TCP to two
  read-only consumers — `gpsd` (time + RTK write-back) and the Septentrio ROS driver
  (rich data: NavSatFix + baseline heading + covariances). **gpsd is the only writer**
  (NTRIP→RTCM), runs under systemd at boot, GCS-independent.
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

### Secrets
NTRIP credentials and anything sensitive live in `group_vars/vault.yml`
(git-ignored). Copy `group_vars/vault.yml.example`, fill it in, and encrypt:
```bash
cp group_vars/vault.yml.example group_vars/vault.yml
ansible-vault encrypt group_vars/vault.yml
# add --ask-vault-pass to your playbook runs
```

---

## Roles

| Role | Purpose |
|---|---|
| `base` | apt baseline, locale/tz/hostname, swap, nvpmodel + jetson_clocks |
| `dev_tools` | git, git-lfs (+`git lfs install`), nano, common CLI tools |
| `networking` | netplan (networkd renderer) for the two un-bridged segments |
| `jtop` | jetson-stats + `jtop` group (non-sudo access; handles PEP 668) |
| `kernel_modules` | **audit-first** DKMS (igc/ch341 likely in-tree; Wi-Fi dongle isn't) |
| `device_tree` | overlays: pps-gpio (GPIO11), PWM out (GPIO12) |
| `time_sync` | gpsd · chrony (PPS+SHM+NTP) · phc2sys · ptp4l gPTP grandmaster |
| `gnss` | mosaic serial broker (Path B) + udev + gpsd NTRIP write path |
| `ros2` | ROS 2 Jazzy `ros-base`, rosdep, colcon, global sourcing |
| `px4_link` | Micro XRCE-DDS Agent (UDP) as a systemd service |
| `rt_tuning` | soft-RT: isolation, SCHED_FIFO, mlock, IRQ affinity, governor |
| `rt_kernel` | **parked** — opt-in, measurement-gated PREEMPT_RT |
| `isaac_ros` | **parked** — deferred until Isaac ROS supports JetPack 7.2 |
