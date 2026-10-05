# ArduRemoteID build for the interim Remote ID module

ArduRemoteID tag v1.14, target `ESP32C3_DEV` (Seeed XIAO ESP32-C3: module RX GPIO2 = D0,
TX GPIO3 = D1), plus one patch, `px4-arming.patch`:

- the module's HEARTBEAT `system_status` carries its own arming check (ACTIVE / CRITICAL).
  Upstream sends 0 (UNINIT), which PX4 1.15+ treats as "Open Drone ID system not ready",
  making `COM_ARM_ODID` useless with PX4;
- the arming check no longer demands Self ID and Operator ID messages from a GCS every
  22 s (optional in F3411, not required of a Part 89 broadcast module; PX4 never sends them);
- System / System-Update are accepted from the autopilot component only, so a GCS without a
  GPS fix cannot overwrite the take-off location with 0/0 every second.

Rationale and the observed symptoms are in `docs/remote-id.md`.

`build.sh` reproduces the upstream Makefile build (arduino-cli 0.27.1, esp32 core 2.0.3, all
under `.work/`, nothing in `~`), applies the patch and leaves
`.work/ArduRemoteID/RemoteIDModule/ArduRemoteID-ESP32C3_DEV.bin` (flash at 0x0) and the
`_OTA.bin` (web update; unsigned, so only with `LOCK_LEVEL -1`). About five minutes on an
Apple-silicon Mac with Rosetta (the pinned core ships x86_64 macOS toolchains).

Flash: `esptool --chip esp32c3 --port /dev/cu.usbmodem* erase-flash` then
`write-flash 0x0 <bin>`. Setup, wiring and PX4 parameters: `docs/remote-id.md`;
module configuration: `tools/remoteid_setup.py`.
