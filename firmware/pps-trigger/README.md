# pps-trigger: the Hadron's PPS-disciplined trigger train (RP2040-Zero)

Firmware for the Waveshare RP2040-Zero that turns the receiver's 1PPS into the
60 Hz one-shot slave-sync train the Hadron 640R+ needs, and tells the Jetson
which pulse of which second each edge was. The design and its reasons are in
`docs/hadron-thermal.md` ("Timestamping"); the short version: the Jetson's
hardware timestamp engine only sees AON-domain GPIOs the baseboard does not
expose, so the trigger comes from the one MCU left on the aircraft.

## Wiring

| RP2040-Zero | Signal | Goes to |
|---|---|---|
| GPIO2 | PPS in, rising edge, 3.3 V | the receiver's PPS fan-out (a 3.3 V push-pull leg) |
| GPIO3 | TRIG out, 60 Hz, 100 µs high | 100 Ω in series to the r2 board's **J3 pin 1** (VSYNC_3V3); **J3 pin 2** is GND; pin 3 spare |
| GPIO4 | EVT in, rising edge | spare: the chopper-wheel slot sensor for the bench verification |
| GPIO5 | TEST out, 1 Hz 5 ms (only after `TEST 1`) | jumper to GPIO2 for a self-test |
| USB-C | power, reports and commands | the Jetson baseboard's **Orin USB 2.0 header**, JST-GH 5-pin (Holybro pinout: 1 VBUS 5 V out, 2 DM, 3 DP, 4 GND, 5 shield) |

USB pigtail, USB-C plug at the Zero to JST-GH 5-pin at the baseboard, standard
USB colours: pin 1 red (VBUS), pin 2 white (D−), pin 3 green (D+), pin 4 black
(GND), pin 5 the cable's drain wire or empty. The header supplies VBUS
unconditionally like any USB 2.0 host, so no USB-C CC handling is involved; the
Zero draws about 50 mA. Everything is one ground domain: the Jetson's USB ground
is the Zero's ground, the r2 board's (its USB) and the receiver's (its USB), so
the PPS tap and the J3 lead need no isolation; the FC's isolated PPS leg stays
separate and nothing here connects to the FMU. Put the 100 Ω in series at the
Zero end of both signal leads and run PPS and TRIG as twisted pairs with their
GND, kept short. GPIO5 is high-impedance unless `TEST 1` is active, so a bench
unit's GPIO5–GPIO2 solder link never drives the receiver's PPS net while the
self-test is off; remove it on the flight unit all the same.

The r2 board shifts VSYNC_3V3 to the camera's 1.8 V (SN74AXC1T45) and feeds
Hadron pin 43. Keep the harness short and shielded. The RP2040-Zero's crystal
is an ordinary ±30 ppm part; a 0.5 ppm TCXO on XIN remains the one component
worth specifying for a dedicated board (hold-over quality only; with PPS the
period is remeasured every second).

## What it does

- **Train**: a PIO state machine emits pulses with cycle-exact spacing (8 ns at
  125 MHz) from a queue the TRIG-edge interrupt keeps four pulses deep, so the
  main loop can never starve it. A second is 60 intervals of P/60 with the
  remainder spread Bresenham-style. Intervals are clamped to the camera's
  59.75–60.25 Hz window; a queue underrun can only make a pulse late.
- **Discipline**: each PPS interval measures the crystal; the median of the
  last five is P. Every pulse is labelled on the PPS grid (second N, pulse k,
  k = 0 on the edge), its offset from the grid is measured with the 1 µs
  hardware timer, and the next interval is corrected by minus that offset,
  bounded to 0.3 % (about 50 µs per pulse, 3 ms per second): slewed, never
  stepped. Lock = 60 consecutive pulses within 50 µs.
- **No PPS**: before the first edge the train runs at the crystal's nominal
  60 Hz (status `N`); after a loss it holds the last period (`H`) and the
  labels extrapolate, flagged. When PPS returns it re-acquires by slewing.
- **Reports** over USB CDC, NMEA-style with an XOR checksum, only while a host
  has the port open: `$HELLO` (boot id, firmware, pins), `$PPS` per edge
  (N, measured period, ppm, current offset, status, missed count), `$TRG` per
  pulse (`boot,seq,N,k,offset_ns,status`: edge time = UTC(N) + k/60 s +
  offset), `$EVT` per event edge, `$STAT` every 10 s. The Jetson stamps the
  same PPS edge itself and knows which UTC second N is, so the 1 s grid labels
  every pulse unambiguously (`hadron-thermal.md`). A new `boot` id tells the
  consumer to re-bootstrap its frame pairing.
- **Dead time**: the queue is four pulses deep, so a correction shows up in the
  measurement four pulses later. The controller subtracts the corrections
  already queued from the measured offset before asking for the next one
  (a dead-time predictor). Without it the loop limit-cycled at ±4 × the bound,
  ±200 µs, and never locked; with it, lock takes two seconds.
- **Commands**: `TEST 1` / `TEST 0` (self-test pulse on GPIO5), `TESTW <ms>`
  (its width, default 5; 500 makes it visible on a meter), `PINS` (pin levels,
  test-pulse and PPS-interrupt counters: the contact check), `STATUS`, `RESET`.
- **LED**: red no PPS, amber acquiring, green locked, magenta hold-over, a blue
  blink on every PPS edge.

## Build and flash

```sh
cmake -S firmware/pps-trigger -B firmware/pps-trigger/build -G Ninja -DPICO_SDK_PATH=$HOME/pico-sdk
ninja -C firmware/pps-trigger/build
picotool load -f firmware/pps-trigger/build/pps_trigger.uf2 && picotool reboot
```

pico-sdk 2.2.0, `PICO_BOARD=waveshare_rp2040_zero`. The build directory is
ignored by git.

## Bench check

`tools/trigger_monitor.py` reads the port, verifies checksums, prints the
`$PPS`/`$STAT` lines and a per-second summary of the `$TRG` offsets. Self-test:
jumper GPIO5 to GPIO2 and send `TEST 1` (`--test on`): the status should go
N → A → L within a few seconds with offsets of a few µs (same crystal, so the
period reads 1 000 000 µs). Pull the jumper: status H after 1.3 s, train
unchanged. The real check against the receiver's PPS, and the chopper-wheel
verification of the frame-to-pulse pairing, are in `hadron-thermal.md`.

## Measured (loopback self-test, 2026-10-05)

GPIO5 soldered to GPIO2, `TESTW 5`, `TEST 1`: lock (`L`) two seconds after the
first edge; thereafter per-second mean offsets within ±1 µs with a spread of
1–3 µs over 60 pulses, 60.0 pulses per second, zero underruns, measured period
1 000 000 ± 7 µs (the ±7 is the edge timestamps' interrupt jitter on the same
crystal). `TEST 0`: hold-over (`H`) after 1.3 s, train and labels continue,
offsets ±0.8 µs against the extrapolated grid. `TEST 1` again: the first
interval is flagged as a bad period (an arbitrary restart phase, as a lost PPS
would not be), the missed seconds are counted, re-lock in two seconds.

Known artefact: pulse 0 of each second shows up to ±10 µs where the rest sit
within ±1 µs. It lands on the PPS edge by design, so the two edge interrupts
fire together and one timestamp waits for the other's handler. The train is
not affected (the pulse-to-pulse spacing is PIO-exact); a single bank-level
timestamp per interrupt would remove it from the report. The RP2040-Zero's
crystal is used as is; a 0.5 ppm TCXO remains the hold-over upgrade.
