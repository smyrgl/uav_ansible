# FLIR Hadron 640R+ source and provisional geometry

Verified 2026-09-24. The exact product is **Hadron 640R+**, part
**70640AS32-6PARXP**. [FLIR's product page](https://oem.flir.com/products/hadron-640/?model=70640AS32-6PARXP)
confirms the variant, nominal **56 g** mass, **32 degree thermal HFOV**,
and **67 degree visible HFOV**. The visible imager is not needed for this
aircraft's current use, but its housing remains in the visual envelope.

## Official CAD availability

The [official FLIR OEM support matrix](https://flir.custhelp.com/app/answers/detail/a_id/6003/~/flir-oem---cores-support)
lists Hadron mechanical drawings and 3D files as **"Available upon Request"**.
The linked request page is [FLIR technical support](https://flir.custhelp.com/app/ask).
No public Hadron STEP download was located, no account was created, and no
support request was sent. Ask for the **mechanical IDD and STEP for Hadron
640R+, 70640AS32-6PARXP** to replace this proxy later.

The general product page's statement that IDD/STEP files are available is not
a direct public CAD download. It presently links public datasheets and an
integration guide in its documentation list.

## Drawing used

The public [Hadron 640 Series engineering datasheet, 102-2053-40 R170](https://flir.netx.net/file/asset/52567/original/attachment/)
explicitly includes the 640R+ in its part-number table. Figures 3 and 4 on PDF
pages 9-10 were visually inspected. The downloaded original is
`cad_source_meshes/hadron_640_series_engineering_R170.pdf`.

The dimensioned drawing gives **36 mm width, 50 mm height, 42.65 mm depth**,
plus a **0.5 mm rear gasket**. These dimensions were used in preference to the
rounded sales-envelope dimensions. The nominal COG is **19.125 mm forward
of the rear datum and 6.82 mm below the thermal axis**; its negligible
0.019 mm lateral offset is omitted here. The thermal axis lies 17.2 mm below
the upright top, derived from the mounting-row dimensions. The roughly
20 mm thermal-to-visible separation is estimated from the drawing, not an
explicit dimension or an optical calibration.

## Mesh convention and use

`scripts/tessellate_hadron.py` creates the four
`cad_source_meshes/hadron_640r_plus_proxy_*.stl` files using the standard
Python library. The body, thermal barrel and visible barrel are also supplied
separately for material assignment. All coordinates are **millimetres**.
The complete mesh has 872 triangles. Regenerate with:

```sh
python3 scripts/tessellate_hadron.py
```

This project-defined frame is upright **X forward, Y left, Z up**, with origin
at the centre of the rear mounting plane, excluding the gasket. These are
not asserted to be FLIR's native CAD axes. Convert millimetres to metres once
when importing into the robot model. The installed pose (decided 2026-09-30)
is **upright, roll 0** (thermal lens above the visible lens), pitched **15°
nose-down** about the rear housing face centre at base_link (0.164, 0, 0.012) m:
`hadron_mount_xyz` 0.164 0 0.012, `hadron_mount_rpy` 0 0.261799 0,
`hadron_rpy` 0 0 0. The earlier provisional pose (x 0.151, roll π, thermal
below visible) is superseded.

In the upright local frame, the thermal visual lens-face reference is
`[42.65, 0, 7.8] mm`; the visible reference is `[41.65, 0, -12.2] mm`.
These are provisional visual references, **not calibrated optical centres**.
Depths, barrel radii, taper and corner rounding are illustrative. The
manufacturer nominal COG becomes `[19.125, 0, 0.98] mm` in this frame.

## Mount (2026-09-30)

The camera sits in the printed backing plate v2 (36 × 50 rear-face tray with
the interface-board pocket), which bolts through the wedge v1 adapter to the
X950 front plate's lower slope; two backing blocks in the wiring channel seat
the four M3 heads. The designs live in the FLIR board repository
(`mount/plate_v2.py`, `mount/wedge_v1.py`, CadQuery; plate v1 is void after the 2026-09-29 review). Their STLs are imported
as `hadron_plate_v2_visual`, `hadron_wedge_v1_visual` and
`hadron_wedge_backing_{lower,upper}_visual` under `hadron_mount_link`, with the
plate frame mapped by lx = 0.93 − Z, ly = 4.828 − X, lz = Y + 7.8 (mm) into the
mount frame and the wedge already in aircraft coordinates. None of the mount
parts has been printed as v1 or offered up to the airframe yet.

Field-of-view clearance at this pose (pinhole rays from the lens-face
references, swept prop discs from the motor CAD, 2026-09-30) is recorded in
`docs/hadron_fov_overlap_2026-09-30.{png,json}`.

`cad_source_meshes/hadron_640r_plus_proxy.source.json` contains the dimensions,
provenance, omissions, frame convention, bounds, checksums and FOV values.
Its vertical FOV values are pinhole estimates from horizontal FOV and full
sensor aspect ratio; they are not manufacturer vertical-FOV specifications.

The meshes contain no invented internal solids, mounting holes, calibrated
entrance pupils, aircraft bracket, cable mass or inertia tensor. A box
collision/inertia approximation is appropriate for this first visual/TF pass;
the component placement, optical extrinsics and final mass properties remain
subject to measurement.
