# ArduRemoteID build for the interim Remote ID module

ArduRemoteID tag v1.14, target `ESP32C3_DEV` (Seeed XIAO ESP32-C3: module RX GPIO2 = D0,
TX GPIO3 = D1), plus one patch:

- `heartbeat-state.patch`: the module's HEARTBEAT `system_status` becomes ACTIVE while a
  Location and a System message from the flight controller are fresher than 5 s and encode
  into valid F3411 messages, CRITICAL otherwise. Upstream sends 0 (UNINIT), which PX4 1.15+
  treats as "Open Drone ID system not ready", making `COM_ARM_ODID` useless with PX4.

`build.sh` reproduces the upstream Makefile build (arduino-cli 0.27.1, esp32 core 2.0.3, all
under `.work/`, nothing in `~`), applies the patch and leaves
`.work/ArduRemoteID/RemoteIDModule/ArduRemoteID-ESP32C3_DEV.bin` (flash at 0x0) and the
`_OTA.bin` (web update; unsigned, so only with `LOCK_LEVEL -1`). About five minutes on an
Apple-silicon Mac with Rosetta (the pinned core ships x86_64 macOS toolchains).

Flash: `esptool --chip esp32c3 --port /dev/cu.usbmodem* erase-flash` then
`write-flash 0x0 <bin>`. Setup, wiring and PX4 parameters: `docs/remote-id.md`;
module configuration: `tools/remoteid_setup.py`.
