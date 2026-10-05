# Remote ID: ArduRemoteID on a XIAO ESP32-C3, on the FC's TELEM3

Status 2026-10-04 (night before the first campaign flights): the NewBeeDrone BeeID
module stopped working. Interim broadcast module: a Seeed XIAO ESP32-C3 running
ArduRemoteID, fed by PX4 over MAVLink on TELEM3. Flashed (patched v1.14) from the
Mac; the wiring, the FC parameters and the end-to-end phone check are the next
steps (see "Bring-up" at the end).

Regulatory position, stated once: under 14 CFR Part 89 the broadcast-module path
requires a module listed on an FAA Declaration of Compliance, and a homebrew
ArduRemoteID board is not one, so this does not make the aircraft Part 89
compliant. It does put a standards-conformant ASTM F3411 broadcast in the air
(Basic ID, Location, System with the take-off location, which is what Part 89 asks
of a broadcast module) carrying the serial number already on the aircraft's FAA
registration, so anyone with a Remote ID receiver app identifies the aircraft and
operator correctly. The owner made that call; the serial and registration numbers
are never committed here (public repository).

## Hardware

| Item | Detail |
| --- | --- |
| Board | Seeed Studio XIAO ESP32-C3 (ESP32-C3 rev 0.4, 4 MB flash, native USB-Serial/JTAG). No onboard antenna: the U.FL antenna must be fitted or the radio is badly mismatched. No user LED. |
| Firmware | ArduRemoteID, tag v1.14, target `ESP32C3_DEV` (same pins as Holybro's Remote ID module, which runs this firmware). Built from `firmware/remoteid/` with the heartbeat patch below; the unpatched release binary is `ArduRemoteID-ESP32C3_DEV.bin` from the v1.14 GitHub release. |
| FC port | Pixhawk 6X TELEM3 (`/dev/ttyS1`), PX4 MAVLink instance 0. TELEM1 carries the SIYI UART backup link (460800), TELEM2 the Jetson router (921600). |
| Power | TELEM3 pin 1, 5 V. The ESP32-C3 draws ~75 mA with Wi-Fi up, ~30 mA Bluetooth-only, peaks ~300 mA in Wi-Fi bursts. Holybro: TELEM1 has its own 1.5 A limiter, every other port shares one 1.5 A limiter (with GPS2's Septentrio, CAN, ...). If the peripheral-5V fault reappears after adding the module, take its 5 V from TELEM1 pin 1 instead. |

### Wiring (TELEM3 JST-GH 6-pin to the XIAO)

| TELEM3 pin | Signal | XIAO pin | ESP32-C3 GPIO | Note |
| --- | --- | --- | --- | --- |
| 1 | VCC 5 V out | 5V | | not while the XIAO is also on USB |
| 2 | TX (FC out) | D0 | GPIO2 | module RX |
| 3 | RX (FC in) | D1 | GPIO3 | module TX |
| 4, 5 | CTS, RTS | | | leave open; PX4 auto-detects the missing flow control |
| 6 | GND | GND | | |

GPIO2 is a strapping pin on the C3; the datasheet says it does not actually select
the boot mode ("recommended to pull up due to glitches"), and the FC's TX line idles
high anyway. A WS2812 on D8 (GPIO8) would show the firmware's ready (green) / fail
(red) status; the bare XIAO has none. Pins D6/D7 (UART0) carry the firmware's debug
prints and a second MAVLink parser; nothing goes to the USB connector after boot.

## Firmware

Flash from a Mac (esptool 5.x; the native USB resets the chip into the bootloader by
itself, no BOOT button):

```bash
esptool --port /dev/cu.usbmodem101 --chip esp32c3 erase-flash
```

```bash
esptool --port /dev/cu.usbmodem101 --chip esp32c3 --baud 921600 write-flash 0x0 ArduRemoteID-ESP32C3_DEV.bin
```

`erase-flash` first: the parameter store is an NVS partition, and leftovers from
another firmware make `nvs_flash_init` fail silently (parameters then never persist).

Never set `LOCK_LEVEL` to 2. At that level the firmware burns eFuses that disable
USB download mode and USB-JTAG permanently; the board could then only be updated
with ArduPilot-signed OTA images. `LOCK_LEVEL` 0 (default) is fine: parameters
stay writable and the web update accepts only signed official images.

### The heartbeat patch (`firmware/remoteid/heartbeat-state.patch`)

PX4 1.15+ derives the Remote ID health that `COM_ARM_ODID` gates on from the
module's HEARTBEAT `system_status`: STANDBY or ACTIVE means healthy. Stock
ArduRemoteID sends 0 (UNINIT), so with the stock firmware `COM_ARM_ODID=2` can
never arm and `COM_ARM_ODID=1` prints "Preflight Fail: Open Drone ID system not
ready" at every health report even though the module works. PX4 does not read the
module's OPEN_DRONE_ID_ARM_STATUS at all (no handler in 1.17). The patch reports
ACTIVE only while a Location and a System message from the FC arrived within the
last 5 s and the data encodes into valid F3411 messages, CRITICAL otherwise. So
`COM_ARM_ODID=2` then means: no arming unless the module is alive and has a
position and a take-off location to broadcast. PX4 sends System only with a 3D fix
and a valid home, so indoors the module stays CRITICAL, which is the point.
Nothing in PX4 1.17 acts on Remote ID health in flight; losing the module only
produces the "Remote ID system lost" message.

Build: `firmware/remoteid/build.sh` (arduino-cli 0.27.1, esp32 core 2.0.3 as pinned by
upstream, everything under `firmware/remoteid/.work/`). Built on the Mac 2026-10-04
in about five minutes; binaries in `.work/` (gitignored).

## Module configuration

The module's web page (join `RID_<mac>`, password `ArduRemoteID`, open
`http://192.168.4.1`) is status and OTA only; it cannot set parameters. Over
MAVLink only numeric parameters are settable, and the one string that matters,
`UAS_ID`, has exactly one MAVLink path: a module whose `UAS_ID_TYPE` and `UAS_TYPE`
are still 0 persists the first OPEN_DRONE_ID_BASIC_ID it receives into its
parameters. `tools/remoteid_setup.py` does all of this through the Jetson's
mavlink-router: PX4 forwards messages addressed to component 236 from TELEM2 onto
TELEM3 and the replies back (both instances have `MAV_x_FORWARD=1`, and
`MAV_HB_FORW_EN=1` forwards the module's heartbeat too). PX4 itself never sends
BASIC_ID in Normal mode, so the module cannot pick up PX4's GUID by accident.

```bash
tools/remoteid_setup.py --link tcp:<jethawk>:5760 set-id --uas-id <serial on the FAA registration> --id-type 1 --ua-type 2
```

| Parameter | Value | Why |
| --- | --- | --- |
| `UAS_ID` / `UAS_ID_TYPE` 1 / `UAS_TYPE` 2 | registered serial, serial-number type, helicopter-or-multirotor | the Basic ID message; FAA registration lookup key |
| `BT4_RATE` 1, `BT5_RATE` 1 (defaults) | Bluetooth 4 legacy + Bluetooth 5 long range | phones (iOS: BT4 only) and Android receivers |
| `WIFI_NAN_RATE` 0, `WIFI_BCN_RATE` 0 (defaults) | no Wi-Fi broadcast | nothing on the SIYI's 2.4 GHz band but a few BLE advertisements per second |
| `WEBSERVER_EN` 0 before flight | no soft-AP | with it at 0 and the Wi-Fi rates at 0 the Wi-Fi radio is never started; set 1 again (over MAVLink) when the status page is wanted on the bench |
| `BAUDRATE` 57600 (default) | matches `SER_TEL3_BAUD` | a change takes effect at the module's next power-up; an FC reboot power-cycles the TELEM rail |
| `BT4_POWER`/`BT5_POWER` 18 dBm (default) | | the C3's BLE range is -24..+20 dBm |
| `LOCK_LEVEL` 0 | | see above |

The Operator ID message (where the FAA registration number could go) is not a
parameter: it is broadcast only while something keeps sending
OPEN_DRONE_ID_OPERATOR_ID, which PX4 does not. Part 89 does not need it; a small
sender on the Jetson could add it later.

## PX4 side

PX4 streams OPEN_DRONE_ID_LOCATION and OPEN_DRONE_ID_SYSTEM at 1 Hz in Normal mode
(also Onboard and Onboard-low-bandwidth). Location: GPS fix, course, speed, height
over take-off, UTC seconds within the hour; System: the home position as the
operator location (`MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF`, the broadcast-module
rule), only once there is a 3D fix and a valid home. Known PX4 quirk: the geodetic
altitude field carries the MSL altitude.

| Parameter | Was | Now | Why |
| --- | --- | --- | --- |
| `SER_TEL3_BAUD` | 115200 | 57600 | module default; reboot |
| `MAV_0_RATE` | 5760 B/s | 0 | 0 = half the link, 2880 B/s; `mavlink status` showed the Normal stream set at ~2.6 kB/s, so the 1 Hz Remote ID streams keep rate multiplier 1.0 (they are not exempt from rate scaling) |
| `MAV_0_MODE`, `MAV_0_CONFIG`, `MAV_0_FORWARD` | Normal, 103, 1 | unchanged | |
| `COM_ARM_ODID` | 0 | 2 with the patched firmware (1 = warn only, 0 = off) | deny arming without a healthy module; with the stock firmware leave it at 0 |

`tools/remoteid_setup.py fc-params` shows the current values against these;
`--apply --reboot` sets them and reboots the FC (the reboot power-cycles TELEM3, so
the module restarts on the new baud).

## Verification

1. `tools/remoteid_setup.py watch`: the module's heartbeat (type 36, ODID) with
   `system_status` 5 (CRITICAL, no FC data yet) indoors, 4 (ACTIVE) outside with a
   fix, and its arm status. "Remote ID system lost" from PX4 means the heartbeat
   stopped for 3 s.
2. A phone with a Remote ID receiver app (Drone Scanner on iOS/Android, OpenDroneID
   on Android): the aircraft appears with the serial number, position and take-off
   point. iPhones receive the Bluetooth 4 legacy advertisements only.
3. The module's status page on the bench (`WEBSERVER_EN` 1): BasicID 1, Location,
   System fields filled from PX4; Location Status `REMOTE_ID_SYSTEM_FAILURE` while
   no FC data has arrived in 5 s.

## Bring-up checklist

- [x] XIAO flashed with the patched v1.14 build (`erase-flash`, stock, then `write-flash 0x0` of `.work/*-hbstate.bin`), 2026-10-04. QGC auto-connects to the XIAO's USB port and blocks esptool; quit it before flashing.
- [x] FC: instance 0 confirmed free on TELEM3 (`mavlink status`: rx 0 B/s)
- [ ] Fit the antenna, wire TELEM3 pins 1, 2, 3, 6 to 5V, D0, D1, GND
- [ ] `fc-params --apply --reboot` (SER_TEL3_BAUD 57600, MAV_0_RATE 0, COM_ARM_ODID 2)
- [ ] `set-id --uas-id <serial>` once the module heartbeat shows in `watch`
- [ ] Outside with a fix: `watch` shows ACTIVE, phone app shows the aircraft; then `set-param WEBSERVER_EN 0`

Follow-ups: feed OPEN_DRONE_ID_OPERATOR_ID / SELF_ID from the Jetson (needs nothing
new on the FC: TELEM2 forwarding is on); diagnose the BeeID (it has its own
battery and GPS; a dead cell looks like "nothing at all"); replace the interim
module with a DOC-listed one (Holybro Remote ID, BlueMark db201/db202mav, Dronetag)
and keep the registration's serial current.
