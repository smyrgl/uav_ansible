# GNSS receiver plumbing

The `gnss` role owns the Jetson side of the Septentrio mosaic-G5 P6:

- `uav-gnss-broker`: the only reader of the receiver's USB2 data port
  (`/dev/gnss`). It re-serves every byte, read-only, on `127.0.0.1:28785`.
- `uav-gnss-time`: SBF ReceiverTime from the fan-out to chrony's SOCK source.
  That source numbers the PPS edges (see `time_sync`).
- The udev symlink for the data port, and `cdc_acm` loaded at boot, so
  sandboxed units can open the port if the receiver appears later.

The receiver's USB1 port is its command port. drone-link holds it and writes
the RTCM corrections. Nothing in this role writes to the receiver.

## Receiver configuration the system depends on

The receiver keeps its configuration in its own Boot file, and Ansible does
not manage it. The lines below are what the Jetson and the flight controller
need. If the receiver is reprogrammed, for example from RxTools, check that
they are still in Boot. `lcf, Boot` lists everything that differs from the
receiver defaults.

The Jetson fan-out, ROS driver, chrony time feed and health node read USB2:

```text
setDataInOut, USB2, RTCMv3, SBF
setSBFOutput, Stream5, USB2, AttEuler+PVTGeodetic+PosCovGeodetic+VelCovGeodetic+AttCovEuler, msec20
setSBFOutput, Stream6, USB2, AuxAntPositions+GALAuthStatus+BaseVectorGeod+DOP+ReceiverTime+ReceiverStatus+ReceiverSetup+QualityInd+RFStatus, sec1
setSBFOutput, Stream7, USB2, MeasEpoch+ChannelStatus, sec1
```

Stream5 runs at 50 Hz because that is the P6's maximum with attitude on
(datasheet: position 100 Hz, but "RTK + attitude" and "Standalone, DGNSS +
attitude" 50 Hz). Until 2026-10-02 it ran at 10 ms: the attitude engine then
included no satellites at all (AttEuler NrSV Do-Not-Use, "not enough
measurements" for the Main-Aux1 baseline) and the error log carried "Bad Sbf
order" entries, an attitude block dispatched ahead of its PVT block.

USB2 input is RTCMv3 so that stray bytes on the data port can never run as
commands. Without these streams the fan-out logs "no data from /dev/gnss",
chrony loses its GNSS time source and the PPS lock, and every `/gnss` topic
stops. That happened on 2026-10-01 when the Boot file was rewritten.

PX4 reads COM1, on the FC's GPS2 port:

```text
setDataInOut, COM1, auto, SBF
setSBFOutput, Stream1, COM1, GALAuthStatus+AttEuler+PVTGeodetic+VelCovGeodetic+DOP+EndOfPVT+ReceiverStatus+AttCovEuler+QualityInd+RFStatus, msec100
```

`SEP_AUTO_CONFIG` is 0 on the FC, so PX4 no longer writes this stream at
boot. At boot PX4 only sends the ten-"S" escape and a `gecm` ping to find the
port. Its driver reports unhealthy without DOP, PVTGeodetic, VelCovGeodetic,
AttEuler and AttCovEuler. With autoconfig on and `SEP_HARDW_SETUP=1`, PX4 sent
`setGNSSAttitude, MovingBase` at every boot. On a single dual-antenna receiver
that reports the bearing of the RTK baseline to the correction base station,
and EKF2 fused it as yaw.

Timing and antennas:

```text
setFrontendMode, DualAnt
```

- **PPS width: 5 ms, the receiver default** (so Boot carries no
  `setPPSParameters` line). The Jetson GPIO PPS and the FC's `pps_capture`
  latch only the rising edge, so the width does not affect them. It was 100 ms
  from 2026-10-01 for the Livox Avia, which needs a 20-200 ms high time, and
  went back to 5 ms on 2026-10-02: the Avia syncs over PTP, and the FC's
  Ethernet failed to link at boot with the PPS on its capture pin, the same
  fault as 2026-09-30. A longer pulse keeps that pin high 20 times longer.
  If the Avia moves to PPS, give it the G5's second PPS output at 100 ms, or
  buffer the FC's capture input first.
- **Frontend mode.** `Nominal`, the default, resolves to SingleAnt on this
  product, which leaves the Aux1 front end off: no Aux1 measurements and no
  multi-antenna attitude. `setFrontendMode` takes effect only after a reset,
  so it must be in Boot.

The attitude offset, PVT mode, receiver dynamics and the constellation and
signal selections are owner settings and are deliberately not listed here.
Two of them silently disable the heading, found by bisection from the factory
defaults on 2026-10-02 (backyard, 0.673 m lateral baseline):

| Setting | Effect on multi-antenna attitude |
| --- | --- |
| `PPP` in the rover modes of `setPVTMode` | the attitude engine drops to about 8 satellites (18 without), and its float solution never converged: pitch 60-72° on a level baseline, heading wandering by tens of degrees |
| `setReceiverDynamics, High, ...` | Septentrio: "high-frequency motion becomes visible at the expense of an increase in the noise"; recommended level Moderate. Each dynamics change also restarts the attitude filter |
| no GLONASS | 3-4 fewer satellites in the attitude solution (owner's choice) |
| the extra signals (GPS L1C, Galileo AltBOC, BeiDou B2I/B2b, SBAS L5) | no measurable cost |

Since then the Boot file has the rover modes without PPP and the dynamics at
`Moderate, UAV`, and the attitude fixes: AttEuler mode 2 on 13 satellites,
heading 99.4° (owner-confirmed), 1-sigma 0.62° (AttCovEuler), pitch within ±1°,
and the AuxAntPositions baseline 0.667 m against the URDF's 0.673 m.
`setGNSSAttitude` stays at its default `MultiAntenna, Fixed`, so only fixed
headings leave the receiver: float solutions here swung by tens of degrees.

Under trees or near the house the Aux1 antenna loses enough measurements for
"not enough measurements" even when its satellites are tracked: the helical
antennas have no ground plane and see reflections from below. A fix comes
fastest under open sky, and the attitude needs the whole sky more than the
position does.

## Changing the receiver from the Jetson

1. Stop drone-link, which holds USB1: `sudo systemctl stop drone-link-connect`.
   A trap that restarts it on exit is safer for scripted sessions.
2. Send the escape sequence first. An auto-mode port blocks user commands once
   it has seen RTCM, until it receives ten "S" characters within 5 s.
3. Make the changes, then persist them with `eccf, Current, Boot`.
4. Start drone-link again.

To use RxTools from another machine, bridge the ports with socat for the
session, one client at a time, and stop the fan-out and drone-link first.

## Checks

The fan-out should carry about 30 kB/s, with the Stream5 blocks at 50 Hz.
Attitude: AttEuler Mode 2 (fixed ambiguities), Error 0, and in AuxAntPositions
(Stream6) an Aux1 baseline near 0.67 m with ambiguity type 0 (fixed).
MeasEpoch should list measurements from both antennas: antenna ID is bits 5-7
of each sub-block's Type byte. ReceiverStatus should list AGC entries for the
Aux1 front ends as well as the Main ones.

The broker regression tests run anywhere, with pyserial stubbed:

```sh
cd roles/gnss/test && python3 -m unittest test_broker -v
```
