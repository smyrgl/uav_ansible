# Hadron 640R+ on the X950: integration notes for the autonomy programme

Written 2026-10-03 from a sourced research pass (FLIR datasheets and app notes,
the Boson SDK 4.1 API, Linux `uvcvideo` source, ROS 2 driver repositories read on
that date). Companion to [the autonomy roadmap](autonomy-roadmap.md). Numbers
marked *est.* are estimates; everything else is from a primary document.

## What is on the aircraft

- Teledyne FLIR **Hadron 640R+**, part **70640AS32-6PARXP**: Boson+ 640×512
  LWIR core (12 µm, < 20 mK, radiometric, EFL 13.6 mm, **32° HFOV**, f/1.0) plus
  a 64 MP visible camera (67° HFOV) in one 56 g module. 5 V, < 1.8 W typical,
  < 2.9 W peak during the shutter (FFC) event; −20 to +60 °C; IP54 with the rear
  sealed. SN P006301 is in hand.
- **Mount** (x950_description URDF): `hadron_mount` at base_link (0.164, 0, 0.012) m,
  pitched **15° nose-down**, upright (thermal lens above the visible). Thermal
  optical origin +0.043 m forward / +0.008 m up of the mount.
- **Electrical path** (owner's r2 flight interface board, PCBWay, submitted
  2026-09-30; project `/Users/john/Code/KiCAD/FLIR`): **USB only, thermal only.**
  The IR core's USB 3 SuperSpeed pair and USB 2 pair go through TPD4E05U06 ESD
  arrays and 100 nF AC coupling to a USB-C receptacle, then to the Jetson's spare
  USB 3.1 port (Holybro: 1.5 A per port). Power from VBUS (~360 mA typical,
  < 630 mA at FFC). The EO camera's MIPI, the IR MIPI and the module's IMU
  (ICM-20602) are deliberately unconnected: "the D555 owns visible".
- **EXT_VSYNC** (Hadron pin 43) is an *input* to the camera, fed from the J3
  JST-GH header through a fixed-direction SN74AXC1T45 (3.3 V aircraft side →
  1.8 V camera side; VIH 2.0 V, so the source must be a true 3.3 V push-pull).
- Mission role: **wildfire detection**. Not a lynchpin of autonomous flight.

## Geometry: it shares the Avia's view, not the E1R's

- Thermal VFOV ≈ 25.8° (32° HFOV at 640:512). Pitched 15° down, the cone spans
  **~2° to ~28° below the horizon** on a level aircraft, ±16° laterally. That is
  entirely inside the Avia's forward 70.4° × 77.2° cone (vertical half-angle
  38.6°) and never reaches the downward E1R, whose forward edge is 45° below the
  horizon.
- Ground on a level aircraft: the axis meets the ground **3.73 × AGL** ahead
  (37 / 112 / 224 m at 10 / 30 / 60 m AGL); the lower image edge 1.9 × AGL
  (19 / 57 / 113 m); the upper edge is effectively the horizon. Swath at the axis
  intercept ≈ 2.2 × AGL. IFOV 0.88 mrad: 10 cm across-track at 30 m AGL,
  stretched ~3.9× along-track by the grazing angle. A standing person on-axis is
  ~5 × 16 px at 30 m AGL and ~2.5 × 8 px at 60 m (*est.*): detection is realistic
  to ~40–50 m AGL on-axis, recognition less.
- Consequences: (a) **thermal-to-LiDAR extrinsics are calibrated against the
  Avia** (LVT2Calib supports Livox Avia + thermal: four-hole acrylic board with a
  heating pad, offline on a laptop from recorded data; intrinsics first with a
  sun- or lamp-heated pattern); (b) **landing-site vetoes act on the approach
  corridor**, not the hover footprint: a site is vetted while it sits 2–4 × AGL
  ahead, or on a descent slope no steeper than ~28°; the vertical terminal descent
  is thermally blind and needs a time-to-stale rule on the last clear look;
  (c) in the forward view the Hadron adds *semantics* (warm, moving things),
  night and smoke penetration and a warm-wire cue; it cannot range, so every
  detection becomes 3D by ray-casting into the Avia cloud / FAST-LIO map.

## Driver path (Linux 6.8 / JetPack 7.2, in-tree only)

- The Boson enumerates as **UVC** (video) plus **CDC-ACM** (control, FSLP at
  921600 baud on `/dev/ttyACMn`). No UVC extension unit, no out-of-tree driver:
  that is the point of the board's USB decision.
- Video formats: **Y16** 640×512, or **640×514 with the two telemetry rows**
  (16-bit tap selectable IR16/T-stable or T-linear via
  `sysctrlSetUsbVideoIR16Mode`), or post-AGC I420/NV12. One format at a time.
  Stock `uvcvideo` maps the Y16 GUID; FLIR's own `BosonUSB` example uses plain
  V4L2 RAW16.
- **Bandwidth**: 640×514×2 B at 60 Hz = 39.5 MB/s = 316 Mbit/s. USB 2 high-speed
  isochronous is capped at 24.6 MB/s, so **60 Hz Y16 needs SuperSpeed**; 30 Hz
  just fits at 480M. The board's acceptance test (`lsusb -t` shows 5000M) is a
  functional requirement, not a nicety.
- **Radiometry**: in T-linear mode the 16-bit value is **K × 100 in high gain,
  K × 50 in low gain** (a 20 °C blackbody reads 29315). The gain state is in the
  telemetry status bits of the same frame; parse it before converting. Calibrated
  scene range to ~140 °C in high gain; flames saturate even low gain, so treat
  saturation as the "fire" class and use T-linear for smouldering ground.
  Accuracy ±3–5 °C under steady state (dT_fpa/dt < 0.1 °C/min, T_fpa within
  0.2 °C of the last FFC).
- **FFC** (shutter): the image freezes ~0.5 s while the **telemetry line keeps
  updating**; status bits 0–1 give imminent / in progress / complete, bits 2–4 the
  gain mode, word 42 the rolling frame counter, word 44 the counter at the last
  FFC, word 47 the camera temperature. Gate detections on the FFC bits, not on
  counter stalls. Run **manual FFC** and command it by flight phase (cruise legs,
  never final approach), honouring "FFC desired" within a bounded delay. A gain
  switch changes the scale 2× and may itself trigger an FFC.
- **ROS 2 Jazzy**: vendor `ctu-vras/flir_boson_usb` v3.0.0 (RAW16 → mono16,
  telemetry-row crop, radiometric image when built with the SDK; stamps = V4L2
  buffer time + the realtime offset per frame) and add a telemetry/sync message,
  FFC gating and one diagnostics row (USB speed, fps, counter gaps, FFC state,
  gain, pairing residual). `usb_cam` does Y16 but no telemetry; `v4l2_camera` has
  no Y16. The SDK is login-walled at FLIR; a complete 4.1 copy is bundled in
  `VideologyInc/flir_boson_kernel_module` under FLIR's SDK licence.
- **Recording**: raw Y16 at 60 Hz is 142 GB/h (71 at 30 Hz); zstd in MCAP gains
  ~1.5–2.5× (*est.*). Record Y16 + telemetry losslessly (that is the labelling and
  radiometric ground truth) plus an 8-bit AGC H.265 review stream; the Orin's
  encoders take 8/10-bit YUV only, so normalise on the CPU or CUDA first.

## Timestamping: one-shot slave sync from a PPS-disciplined MCU

The Jetson's hardware timestamping engine (HTE/GTE) is **not an option**: it
timestamps AON-domain GPIOs only and the Holybro baseboard exposes none (the PPS
edge is already a software-IRQ stamp on an ordinary GPIO). The trigger therefore
comes from an **RP2040-class MCU disciplined to the GNSS PPS**; with the lights
RP2040 gone (lights are DroneCAN), it is the only MCU on the aircraft.

- **Camera side** (Boson ext-sync app note): in slave mode "the camera will
  generate a new frame for each pulse … if no pulses are received, the camera will
  not output a video signal". Readout starts **0.5 ms** after the rising edge,
  rows follow at **27.8 µs** (14.2 ms per frame); validated train 59.75–60.25 Hz,
  highest 60.4 Hz, pulses closer than 16.56 ms are ignored (silent decimation);
  rates far below 60 Hz degrade uniformity; bursts and gaps degrade the image
  "for several seconds". One-shot output mode gives exactly one frame per pulse.
  30 Hz frames: use the averager with a 60 Hz train (EXT_SYNC at twice the output
  rate). Pulse width > 90 ns (use ~100 µs). Pipeline latency ≈ 24–25 ms; the
  bolometer time constant ≈ 8 ms smears fast hot targets.
- **Do not persist slave mode to flash**: pre-2.1 firmware reset-looped when booted
  without a signal, and current firmware shows no video without pulses. Power up
  free-running, start the stream, start the train, then `bosonSetExtSyncMode(2)`
  over CDC-ACM; revert to 0 if the train is lost (a DQBUF watchdog: a stopped train
  looks like a hung camera).
- **MCU design**: count system clocks between PPS edges with a PIO state machine
  (8–16 ns resolution); emit the 60 Hz train from a Bresenham remainder, not a
  fractional divider (first-order delta-sigma jitter); correct phase by **slewing**
  the period a few cycles so pulse 0 of every second lands on the PPS edge, never
  by stepping. Guard the period so a bad filter can never exceed 60.25 Hz. On PPS
  loss hold the last frequency and flag it (a ±30 ppm crystal reaches ~1 ms of
  phase error after minutes; **a 0.5 ppm TCXO on XIN is the one component worth
  specifying**). 3.3 V push-pull output, ~100 Ω series, short shielded harness.
- **Time report**: the MCU never needs UTC. `$TRG,<boot_id>,<seq>,<pps_count>,<k>,<offset_ns>,<status>*CS`
  over USB-CDC (~3 kB/s) says "pulse *k* of the second with PPS count *N*"; the
  Jetson already stamps the same PPS edge (tens of µs, software IRQ) and knows the
  UTC second from PTP, so the 1 s grid labels it unambiguously. Edge time =
  UTC(N) + k/60 + offset. Per-row time = edge + 0.5 ms + row × 27.8 µs.
- **Pairing**: keep the offset D = frame_counter − seq; bootstrap by matching
  arrival − L_host (L ≈ 26–30 ms, *est.*) to the nearest edge for N frames; USB
  drops leave gaps in the counter and in `uvcvideo`'s `buf.sequence` but the
  pairing stays exact because the counter is camera-side. Re-bootstrap on MCU
  `boot_id` change or camera restart.
- **Bench verification**: a chopper wheel over a warm target spun at a rate
  incommensurate with 60 Hz (e.g. 7.3 Hz), its slot sensor fed to a spare MCU
  input and reported as `$EVT`, pins the frame↔pulse integer and verifies the
  27.8 µs/row model over a few hundred events; histogram L_host over 10⁴ frames;
  confirm no reset loop when the train stops; confirm the re-lock slews after a
  10 min PPS outage.

## Autonomy uses, in order

1. **Hot-spot detection and georeferencing** (classical): T-linear thresholds
   (absolute, e.g. > 80–100 °C, plus > 30 K over the local background, *est.*),
   temporal persistence, clustering; pixel → ray (intrinsics) → base_link
   (Avia-refined extrinsics) → odom at the pulse time (LIO pose interpolated) →
   ray-cast into the FAST-LIO map → UTM. Error budget at 30 m AGL / 112 m ahead:
   0.3° angular → ~0.6 m cross-track, ~2.3 m along-track (1/sin 15°). **1–3 m at
   100 m standoff is realistic, and only because the ray meets the real LiDAR
   terrain, not a flat-earth plane.** Negligible compute.
2. **People, animals, vehicles** (learned): YOLO26-n/s INT8 at 3.5–4.8 ms on the
   Orin NX (JetPack 6 figures; re-measure on 7.2), run at **10–15 Hz** (~5–10 %
   GPU), calibrated INT8 on thermal frames, trained "rect" at 640×512. Fine-tune
   from HIT-UAV + WiSARD + FLIR ADAS v2, then on in-house labelled frames: the
   15° oblique view from 10–60 m sits between HIT-UAV (near-nadir, 60–130 m) and
   ADAS (ground level), so in-house labels matter more than volume.
3. **Night obstacle cues**: geometry stays with the Avia; thermal adds
   classification and a warm-wire cue (thermal YOLOv8n reached 98.4 % mAP50 on
   power lines, Sensors 2024).

**Stage-1 shadow programme**: record Y16 + telemetry + `$TRG`/`$PPS` + arrival
times + LIO poses; run hot-spot thresholding and a detector in shadow, logging
map/UTM points with uncertainty; label 1–2 k frames; evaluate sync (L stability,
counter gaps), radiometry (a water pan or black plate at a measured temperature,
after 30 min warm-up and right after power-up), geolocation (an RTK-surveyed
charcoal pan from 30/60 m AGL), detector precision/recall vs range, FFC gating.

## Datasets to fetch now

FLAME 3 (radiometric 640×512 prescribed-burn pairs, IEEE Dataport, the closest
match to this sensor and mission), FLAME 2 (~155 GB), FLAME, HIT-UAV (CC BY 4.0),
WiSARD (MIT, people in wilderness, 40 GB), BIRDSAI/Conservation Drones (animals,
CDLA-Permissive), FLIR ADAS v2 (14-bit TIFFs, FLIR dataset terms), LLVIP
(non-commercial), CART (Caltech aerial RGB-T terrain classes, incl. water).
Weights: Ultralytics YOLO26 n/s COCO checkpoints as the fine-tuning base.

## Sources (primary)

Hadron 640 Series datasheet 24-0531 rev 2024-09; Hadron 640R Engineering
Datasheet R100 and Integration Guide 1.12 (the project holds R170); Boson
datasheet 102-2013-40 Rev 340; Boson+ datasheet 23-0404 (2025-01); Boson External
Sync App Note R200; Boson FFC/NUC Control App Doc; Boson SDK 4.1
`Client_API.py`; torvalds/linux `drivers/media/common/uvc.c`,
`drivers/media/usb/uvc/uvc_video.c`, `drivers/hte/hte-tegra194.c`,
`Documentation/devicetree/bindings/timestamp/nvidia,tegra194-hte.yaml`; RP2040
datasheet; ctu-vras/flir_boson_usb; Clothooo/lvt2calib; Ultralytics Jetson guide;
the dataset pages named above.
