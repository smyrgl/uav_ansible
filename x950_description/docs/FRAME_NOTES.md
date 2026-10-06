# Frame choices for the first draft

All ROS translations below are metres. These are **mechanical/CAD transforms**,
not measured extrinsic calibration. The aircraft and all propellers are static
visual geometry under `base_link`.

| Frame | Position in base_link | Axes / physical datum |
|---|---|---|
| base_link | 0, 0, 0 | Original aircraft CAD datum; X forward, Y left, Z up. Not known to be the flight-controller IMU or centre of mass. |
| avia_mount_link | 0.147660, 0, 0.093250 | Nominal centre of the sandwich bolt field; axes aligned with base. |
| avia_link | 0.183367, 0, 0.053776; pitch +45° (nose-down) | Centroid of four bottom mounting holes in the A-S+ cage v2 (avia_joint xyz 0.035707 0 -0.039474, rpy 0 0.785398 0 from Printables/Pitch_Study/build/urdf_poses.json, 2026-10-05). The runtime `avia_lidar_*` child (bezel plane, +0.0525 X +0.0324 Z) is the ranging-origin proxy; calibrate. |
| d555_mount_link | 0.104904, 0, -0.026500 | Front belly-plate bolt-pattern centre; axes aligned with base. |
| d555_link | 0.143367, 0, -0.094827; roll pi, pitch +40° | Front housing-plane centre in saddle v0.4: inverted and 40° nose-down (d555_joint xyz 0.038463 0 -0.068327, rpy 3.141592 0.698132 0, urdf_poses.json 2026-10-05). X forward-down, Y right, Z down-aft. |
| d555_nominal_camera_link | 0.145350, -0.047500, -0.069350 | Provisional left IR/depth origin: lateral offset confirmed in D555 CAD, axial -4.65 mm borrowed from D455. |
| e1r_mount_link | -0.084000, 0, -0.024500 | Rear belly-plate bolt-pattern centre; axes aligned with base. |
| e1r_link | -0.084000, 0, -0.084900 | Original E1R CAD datum; X wide axis forward, Y left, Z up; sensor looks along -Z. |
| e1r_nominal_lidar_frame | -0.084000, 0, -0.079312 | Optical centre modeled in source extraction; X looks down, Y left, Z forward. Sensor driver axes/origin still need verification. |

Every joint is fixed. Mount origins are organisational datums; changing a mount
joint moves its bracket and sensor together. Changing a sensor joint adjusts the
sensor relative to its bracket. Nominal frame names intentionally avoid claiming
ownership of sensor-driver TF frames. The supplied launch starts no sensor drivers.
Do not relabel incoming data into these nominal frames without reconciling the
physical measurement origin and axes.

## Coordinate chain

The open `x950.blend` scene is already in metres, with +X starboard, +Y up, +Z aft.
It becomes ROS FLU by the proper rotation:

```
ROS = (-Blender.Z, -Blender.X, Blender.Y)
```

The bracket design sources use millimetres, +X starboard, -Y forward, +Z down:

```
ROS = (-aircraft.Y, -aircraft.X, -aircraft.Z) / 1000
```

The CAD origin is retained; no guessed centre-of-mass translation is applied.
The original Blender meshes are transformed with their full world matrices,
including existing object/parent poses. Imported STL scale is applied once to
vertices; exported visual scales remain unit scale.

## Source decisions

- **Avia:** cage v2 at 45° nose-down (`Avia/avia_cage_v2_*_aircraft.stl`: main,
  pad strut, top shim t1.6, jack foot v2, backing plate v2) replaced the level
  bracket and backing plate on 2026-10-05 (`build/avia_envelope.json`). Hardware
  heads, keep-outs and the M12 cable envelopes are not modelled.
- **D555:** `D555/D555_saddle_v04_aircraft.stl` (saddle v0.4, 40° nose-down,
  inverted camera) replaced belly bracket v03 on 2026-10-05; the joint and the
  saddle come from the Pitch_Study integration (`build/urdf_poses.json`,
  `build/d555_envelope.json`). Fasteners and cable envelopes are not modelled.
- **E1R:** use `E1R_belly_bracket_v01.stl` and orientation B in
  `build_e1r_bracket_v01.py`: wide field fore-aft, connector starboard,
  20 mm fin-tip gap. Keep 23 physical solids; omit two CAD datum-marker cubes (optical centre and centre of mass) and two FOV sheet models.

Machine-readable matrices, link datums, mesh bounds, CAD source hashes, and
validation results are alongside this file. `sensor_source_audit.md` provides
source references and original mounting uncertainties.

## Remaining integration decisions

Target ROS 2 Jazzy. Confirm the relationship between the aircraft
CAD datum and flight-controller IMU, and the actual sensor-driver frame names,
origins, and calibration. A physical mounting check is still needed. Dynamics
would require measured masses, centres of mass, inertia tensors, collision
geometry, rotor joints and actuator models; these are outside this first
visualization model. LinkForge's stock export inserts default masses, so use
`scripts/export_linkforge.py` for this model until physical properties are added.

## Supplied sensor masses

The project owner supplied Avia **0.498 kg**, D555 **0.337 kg**, and E1R **0.330 kg**. These are saved in `config/sensor_specs.json` and Blender sensor-link metadata. Mass alone does not determine inertia. The visualization URDF therefore omits inertials instead of asserting guessed physical properties. E1R has a CAD centre-of-mass marker, retained as metadata, not a measured mass distribution.

## D555 internal optical frames

The user confirmed that D555 uses D455 optics. The D555 CAD lens axes at
X = +47.5, -47.5 and -11.5 mm corroborate the manufacturer D455 frame layout.
The original housing-centre optical placeholder is replaced by
`d555_nominal_camera_link` with depth, infra1, infra2 and color body/optical
children. Depth and infra1 share the nominal camera origin; infra2 is
(0,-0.095,0) m and color is (0,-0.059,0) m relative to it. Each optical child
uses rpy (-pi/2,0,-pi/2). All inherit the installed housing's roll pi.

The camera root is (-0.00465,+0.0475,0) m relative to `d555_link`. The axial
-4.65 mm is a **D455-derived estimate**, not a confirmed D555 depth zero plane.
Override `d555_nominal_camera_xyz` and `d555_nominal_camera_rpy` after calibration.
All names contain `nominal` and do not compete with the RealSense driver's
factory-calibrated TF names. Intrinsics remain owned by the device and driver.
No D455 IMU position is borrowed. See REALSENSE_INTEGRATION.md and
d555_optics_cad.json for evidence and driver integration details.

## Battery, avionics, GNSS and Hadron

| Frame | Position in base_link (m) | Datum / confidence |
| --- | --- | --- |
| battery_support_link | −0.026230, 0, −0.010000 | Floor beneath pack center; final cradle source geometry. |
| battery_link | −0.026230, 0, 0.034100 | Measured pack envelope geometric center, nominal uncompressed cradle fit; not measured CoM. |
| avionics_carrier_link | 0.065000, 0, 0.075000 | Estimated PCB footprint center at bottom of existing CAD standoffs. Native board +X provisionally forward. |
| fmu_housing_link | 0.085600, 0, 0.093323 | Pixhawk CAD housing center; NOT an IMU datum or established flight-controller axes. |
| gnss_main_mount_link | −0.334278, +0.336696, 0.092500 | Port v02 bracket foot-pad plane; yaw 172° from aircraft frame. |
| gnss_aux_mount_link | −0.334278, −0.336696, 0.092500 | Starboard v02 bracket foot-pad plane; yaw −90°. |
| gnss_main_link / gnss_aux_link | Same XY as mounts, Z=0.122980 | Antenna mounting-seat centers, +Z up; phase centers unknown. Names identify mechanical sides, not receiver assignment. |
| hadron_mount_link | 0.166639, 0, 0; rpy 0, 0.261799, 0 | Rear housing face centre (lip-top plane) on base v2 + carrier v1 (15° variant) bolted to the front plate's lower slope; A-S+ integration 2026-10-05 (urdf_poses.json). 20°/25° carriers exist: xyz 0.167922/0.169411, pitch 0.349066/0.436332. |
| hadron_link | Same as mount | Upright, roll 0: thermal lens above the visible lens. +X forward, +Y left, +Z up, pitched 15° nose-down with the mount. Base v2, the two backing blocks v2, carrier v1 and plate v2 are visuals of hadron_mount_link. |
| hadron_nominal_thermal_optical_frame | 0.209855, 0, −0.003504 | Proxy thermal lens face, not calibrated optical centre. Optical Z along the lens axis (15° below aircraft forward), X image-right, Y image-down (upright sensor). |
| hadron_nominal_visible_optical_frame | 0.203712, 0, −0.022564 | Approximate visible lens face; optical axes as above. |

The Holybro model includes its own four short standoffs and one Pixhawk 6X.
No duplicate hardware or extra risers were added. Its installation is a deliberately
adjustable estimate. The Hadron pose is the decided mount of 2026-09-30, still
adjustable through the launch arguments and pending the physical offer-up. Hadron
optical locations are mechanical illustrations; lens entrance pupils are not
established. Field-of-view clearance at this pose was ray-cast in the model
([map](hadron_fov_overlap_2026-09-30.png), [numbers](hadron_fov_overlap_2026-09-30.json)):
the thermal frame is clear of airframe and swept prop discs, the visible frame's
upper corners see the front prop discs. Not physically verified.

Component evidence is in [battery notes](BATTERY_NOTES.md),
[Holybro source](HOLYBRO_SOURCE.md), [antenna notes](ANTENNA_NOTES.md), and
[Hadron source](HADRON_SOURCE.md). The GNSS brackets replace the complete stock
antenna/bracket assemblies; the source scene remains intact. Printed bracket
materials and appearance confidence are recorded in [materials](MATERIALS.md).
