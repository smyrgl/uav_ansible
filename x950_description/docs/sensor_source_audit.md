# Sensor geometry and mounting source audit

This records the source geometry, frame conversions, and remaining uncertainties
used for the first X950 ROS 2 description. It is a reconstruction of the supplied
CAD and bracket records, not calibrated sensor extrinsics or a flightworthiness
assessment. Original design files are left unchanged.

The user confirmed **ROS 2 Jazzy** and sensor masses during this build:
Avia **0.498 kg**, D555 **0.337 kg**, E1R **0.330 kg**. Use these measured/provided
mass values instead of estimates from the tessellated solid volumes. Inertia
tensors remain approximate unless separately measured or supplied.

## Coordinate systems and units

The bracket design records use an **aircraft CAD frame in millimetres**:

- `+X`: starboard/right.
- `-Y`: forward.
- `+Z`: down.

The ROS body convention used by the description is FLU: `+X` forward, `+Y` left,
`+Z` up. For an aircraft CAD point `(Xa, Ya, Za)`, the ROS coordinates in metres are
`(-Ya, -Xa, -Za) / 1000`. This is a proper rotation, not a reflection.

The current Blender aircraft scene was separately inspected in this build session;
its axes are `+X` starboard, `+Y` up, `+Z` aft, in metres. Its ROS conversion is
`(-Zb, -Xb, +Yb)`. Do not apply the bracket CAD transform directly to the original
Blender scene.

`meshes/source_*.stl` intentionally retain the STEP assembly coordinates and
millimetre units. These are intermediate CAD meshes, not meshes ready to load into
ROS without a transform. Final robot meshes must apply each transform exactly once.

## Files to use

| Assembly | Physical CAD source | Mounted bracket source |
| --- | --- | --- |
| Livox Avia | `Avia/Livox_Avia_shell_FOV.stp`, shell only | `Avia/avia_mount_bracket.stl` and `Avia/avia_backing_plate.stl` |
| D555 | `D555/D555_SOLID_SOC.STEP`, all 14 physical solids | `D555/D555_belly_bracket_v03.stl` |
| RoboSense E1R | `E1R/E1R 3D数模.STEP`, 23 physical solids | `E1R/E1R_belly_bracket_v01.stl` |

Exclude `Avia/avia_drill_template.stl`, which is temporary tooling. Exclude the
Avia FOV solid and both E1R FOV sheet models from physical geometry. D555 v02 is
superseded by v03. The source tessellation script inventories included and excluded
geometry in [cad_tessellation.json](cad_tessellation.json).

## Transforms into the aircraft CAD frame

The following equations have millimetre inputs and outputs. `Xs,Ys,Zs` are
coordinates in the relevant source STEP or STL, after STEP assembly placements
have been applied.

| Source | `Xa` | `Ya` | `Za` |
| --- | --- | --- | --- |
| Avia STEP | `-Zs - 35.975` | `Xs - 281.960` | `-Ys - 62.100` |
| Avia bracket STL | `Xs` | `Ys` | `Zs` |
| Avia backing plate STL | `Xs` | `Ys` | `Zs` |
| D555 STEP | `Xs` | `-Zs - 150.000` | `Ys + 69.351` |
| D555 v03 bracket STL | `-Xs` | `-Ys - 92.000` | `Zs + 26.500` |
| E1R STEP | `-Ys` | `84.000 - Xs` | `84.900 - Zs` |
| E1R bracket STL | `Xs` | `Ys + 84.000` | `Zs + 24.500` |

The D555 build script rounds the bottom-to-midline distance to 20.85 mm, so its
envelope uses `Ys + 69.350`. The extraction record states the STEP bottom datum
as `Ys=-20.851`; seating that plane at aircraft `Za=48.5` gives `69.351` above.
This 0.001 mm difference has no practical mounting consequence.

## Avia: front spinal mount

The current authority is
[AVIA_MOUNT_extraction_report.md](../../Avia/AVIA_MOUNT_extraction_report.md),
especially lines 12–22 and 25–32. It supersedes the old root
`AVIA_SESSION_HANDOFF_1.md` where the two disagree.

- Zero pitch, bottom mounting face at aircraft `z=-78.0`.
- Body rear face `y=-171.16`; front face `y=-262.16`.
- Housing/fin envelope centred laterally, fins at approximately `x=±33.125`.
- Body top `z=-142.8`.
- M12 exits to port/left; tip approximately `x=-42.4`.
- Mount pattern is **65 × 46 mm**, not the handoff's 46 × 34 mm.
- Four mounting centres are `x={-20.475,+25.525}` and
  `y={-177.16,-242.16}`. Pattern centroid is `(2.525,-209.66,-78)`.
- A useful mechanical link origin is that mounting-pattern centroid, whose ROS
  position is `(0.209660,-0.002525,0.078000)` m. This is not a claimed lidar origin.

The STEP shell uses identity assembly placement. Its BREP is `#52900`; the
separate `LIVOX_AVIA_FOV` product uses solid `#53741` and must be excluded.
The current Open CASCADE reader splits the body BREP into two physical solids:
the housing and the M12 connector. Both are retained in the exported mesh.
The shell's vertex range is approximately `(19.8,15.9,-69.1)` to
`(110.8,80.7,6.4500)` mm. The exact Avia transform above was reconstructed by:

1. Mapping STEP `X=19.8/110.8` to aircraft front/rear `Y=-262.16/-171.16`.
2. Mapping STEP bottom `Y=15.9` to aircraft `Z=-78`.
3. Following the report's `+Z_step -> -X_aircraft` orientation.
4. Matching STEP bottom M3 centres `X={39.8,104.8}`, `Z={-61.5,-15.5}` to all four
   reported bracket-hole centres. These appear near STEP lines 19035–19449.

The bracket/backing STLs already use aircraft CAD coordinates. Their measured
bounds are respectively `[-56,-246.5,-104.5]..[56,-147.66,-58]` and
`[-54,-146.16,-103]..[54,-142.16,-83]` mm.

The report states that the Avia upper edge stands 24.4 mm above the canopy and
requires a physical propeller-clearance check. It also flags simplified cover
wall geometry and unverified physical pin/hole details. Those caveats do not
change the documented geometric placement.

## D555: inverted front belly mount

Authorities are
[D555_v03_extraction.json](../../D555/D555_v03_extraction.json),
[build_bracket_v03.py](../../D555/build_bracket_v03.py), and the v03 section of
[D555_bracket_NOTES.md](../../D555/D555_bracket_NOTES.md).

- Camera is forward looking and inverted, with its port/bottom face upward.
- Rear housing datum/web face `y=-102.0`; front face `y=-150.0`.
- Port mounting face `z=48.5`; main housing lower extent approximately `z=90.21`.
- Current STEP body width approximately **167.10 mm** and depth 48 mm. The older
  extraction record says 165.31 mm; current direct CAD import takes precedence.
- A raised rear strip reaches approximately `z=46.19` near the aft edge.
- Tripod bolt axis at aircraft `(0,-118.995)` is the vertical mounting datum.
- A useful mechanical link origin is the housing front-face centre
  `(0,-150,69.351)` mm, ROS `(0.150,0,-0.069351)` m. It is not an imager centre.
- Relative to the native upright sensor, the installed unit has a 180° roll;
  sensor forward/right/down axes may be retained as the sensor body convention.

The STEP convention and bottom/rear datums are stated in extraction JSON line 4;
body/port details are in lines 65–96. Build-script lines 32–41 and 101–106 record
the mounted planes; render-script lines 11–13 explicitly provide the STL-to-aircraft
transform. Measured v03 STL bounds are `[-69,-5,0]..[69,38,52.1]` mm.

v03 moves the sensor 10 mm forward compared with v02. The source notes document
a remaining CAD-versus-caliper rear M4 height discrepancy (20.85 versus 22.1 mm);
the bracket slots accommodate both and the tripod pillar fixes the height. The
old extraction width was smaller than the physical caliper measurement. Direct
import of the supplied STEP now yields about 167.10 mm, consistent with the
reported 167 mm caliper width; the source geometry is preserved. No source here
establishes D555 colour/depth optical
centres, stereo baseline, or driver TF frame offsets.

## E1R: downward looking rear belly mount

Authorities are
[E1R_extraction.json](../../E1R/E1R_extraction.json),
[build_e1r_bracket_v01.py](../../E1R/build_e1r_bracket_v01.py), and
[E1R_bracket_NOTES.md](../../E1R/E1R_bracket_NOTES.md).

- Orientation B is locked in the notes: **120° wide axis fore–aft**, 90° lateral,
  viewing downward, connector starboard.
- Rear plate mount face `z=24.5`; fin tips `z=44.5`, giving the documented 20 mm gap.
- Glass outer plane `z=86.9`; mounting pad plane `z=59.52`.
- Native STEP datum `(0,0,0)` is the inner-glass plane at the optic-axis X/Y
  location. It maps to aircraft `(0,84,84.9)`, ROS `(-0.084,0,-0.0849)` m.
- The combined STEP-to-aircraft-to-ROS rotation is identity: native `+X` maps
  forward, `+Y` maps left, `+Z` maps up. Its native viewing direction is `-Z`.
- A sensible mechanical body link therefore uses the STEP datum, with a separate
  nominal optical-centre child if useful.
- Documented model optical centre `(0,0,5.588)` maps to aircraft `(0,84,79.312)`,
  ROS `(-0.084,0,-0.079312)` m. This is CAD-derived, not calibrated extrinsics.
- Documented COM `(0,-6.35,18.88)` maps to aircraft `(6.35,84,66.02)` mm.
- Notes state a sensor mass of 330 g. No measured inertia tensor is supplied.

The exact sensor transform is in build-script lines 10–12 and the current notes
lines 16–21. The bracket STL offset is explicit in build-script lines 263–268 and
render-script lines 10–12. Measured STL bounds are
`[-59.5,-50.4,0]..[59.5,50.3,57]` mm.

The STEP imports as 25 solids and two nonphysical FOV sheet models. Of those
solids, two are approximately 1 mm datum markers, explicitly named `光心`
(optical centre, STEP product line 5648) and `质心` (centre of mass, product line
300835). Their centres agree with the documented optical and COM datums. These
two markers are excluded, leaving **23 physical solids**, with both FOV sheets
also excluded. The notes
report a successful first-print fit; post-anneal dimensions still require the
listed checks. The 1.33% occlusion figure is a prior placement-study result,
not a new measurement or guarantee made by this robot description.

## Modelling limits and next inputs

The geometry and fixed mounting transforms are sufficiently specified for a
first visual/TF model. The following inputs are absent or remain provisional:

- Measured aircraft mass, centre of mass, and inertia tensor; actual battery and
  payload configuration.
- Authoritative driver TF conventions and calibrated lidar/camera extrinsics.
- D555 individual optical-frame origins and baselines.
- Confirmation that the current physical Avia and D555 mounts match the latest
  design revisions; E1R post-anneal dimensions.

Use clearly identified mechanical frames and nominal CAD optical frames until
those values are supplied. Do not infer trustworthy flight dynamics from the
solid tessellation, printed-part volumes, or visual collision approximations.

## H-Flow (optical flow + ToF rangefinder) — added 2026-09-30

Source: Holybro `H-FLOW.step` (kept as `cad_source_meshes/hflow.step`, joined
full-resolution mesh `cad_source_meshes/hflow_joined.stl`, provenance in
`hflow.source.json`). The 33 STEP solids (housing, top plate, PCB, PAA3905E1
flow sensor and lens, AFBR-S50LV85D ToF module, IR LED, JST-GH connector) are
joined into one visual, `hflow_body_visual`, on `hflow_link`.

Placement: the user placed the STEP at the belly mount; `hflow_link` keeps
that position with an identity rotation (FLU: +X forward, +Y left, +Z up). The
module arrow points forward (connector aft), matching the flight controller's
`SENS_FLOW_ROT = 0`. Optics face −Z.

Frames (nominal, from STEP part positions; not calibrated):

| Frame | Origin (mm, in `hflow_link`) | Axes |
| --- | --- | --- |
| `hflow_nominal_flow_optical_frame` | (2.89, −7.95, −2.84), PAA3905 lens centre | X image-right, Y image-down (aft), Z viewing (down) |
| `hflow_nominal_range_frame` | (1.21, 8.70, −8.80), AFBR-S50 window | +X along the beam (down), +Y left, +Z forward |
| `hflow_nominal_frd_frame` | (0, 0, 0) | +X forward, +Y right, +Z down: the PX4/DroneCAN sensor axes of the flow and gyro integrals |

Coverage guides (hidden curves): 42° cone for the flow sensor and a
12.4° × 6.2° receiver pyramid with the 2° × 2° emitter beam for the ToF, both
0.6 m long. The wide ToF axis is assumed along the module's long side
(vehicle X). Mass 15.2 g with casing (manufacturer), stored as metadata.
