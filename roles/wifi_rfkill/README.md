# Wi-Fi radio off while armed (`wifi_rfkill`)

The uplink dongle (Cudy AC650, `kernel_modules` + `networking`) is a permanent
fixture on the airframe, and the UniRC 7 Pro link that carries RC and MAVLink
uses 2.400–2.476 and 5.725–5.829 GHz, the same bands. So the Wi-Fi radio is
soft-blocked for the whole armed interval and released after landing.

`uav-wifi-rfkill.service` runs `/usr/local/lib/uav/uav-wifi-rfkill.py` as root.
It reads the autopilot's HEARTBEAT from mavlink-router's TCP server (the same
read-only source the flight recorder uses) and decodes nothing but HEARTBEAT,
with the frame checksum verified, from system 1 component 1 only. No pymavlink.

| Event | Action |
| --- | --- |
| HEARTBEAT with `MAV_MODE_FLAG_SAFETY_ARMED` | `rfkill block wlan` at once |
| disarmed for `wifi_rfkill_unblock_delay_s` (5 s) | `rfkill unblock wlan` |
| no autopilot HEARTBEAT for `wifi_rfkill_stale_s` (10 s) | `rfkill unblock wlan` |
| service stopped | `ExecStopPost` unblocks |
| service (re)started | radio untouched until the first HEARTBEAT decides |

The stale rule is the bench guarantee: with the FC off or the router down
nothing can leave the Jetson behind a dark radio. While blocked, the default
route via the dongle disappears and the GCS tunnel's route (`gcs_gateway`,
metric 100) carries the Jetson's egress and the operator's Tailscale path, as
in a sealed-airframe flight before the dongle became permanent.

`wpa_supplicant` follows rfkill on its own: the interface drops on block and
re-associates on unblock, and networkd renews the lease, so no netplan or
service restart is involved.

## Verify

```bash
systemctl status uav-wifi-rfkill            # "connected to tcp:127.0.0.1:5760"
cat /run/uav/wifi-rfkill.json               # armed / blocked / heartbeat age
journalctl -u uav-wifi-rfkill -f            # "block wlan (autopilot armed, ...)"
rfkill list                                 # Soft blocked: yes while armed
```

Bench test without props: arm from QGC, the journal shows `block wlan` within a
second and the wifi default route vanishes; disarm, and `unblock wlan` follows
five seconds later with the lease back. Operator escape hatch at any time:
`sudo rfkill unblock wlan` (the agent re-blocks only on the next armed
HEARTBEAT) or `systemctl stop uav-wifi-rfkill`.

`wifi_rfkill_enabled: false` stops and disables the unit and unblocks the radio.
