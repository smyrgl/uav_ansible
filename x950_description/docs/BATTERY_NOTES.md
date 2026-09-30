# Battery and printed retention parts

The description includes the MAD Components semi-solid **12S 20Ah battery** and
the user's **printed TPU lattice retention parts**. The source STLs describe the
outer envelopes of the cradle and end block; the printed internal lattice/infill
is represented by those envelopes.

The accepted battery mass is **3.060 kg**. The source record states that mass and
labels the **103.6 W × 72.6 H × 191.5 L mm** dimensions as measured; it does not
say whether the mass came from a scale or a specification. No exact battery SKU
is recorded. See [battery identification and dimensions](/Users/john/Code/Printables/Battery/x950-battery-bay-inserts-handoff.md:55).

## Assembly included in the model

| Link or part | Geometry and placement |
| --- | --- |
| `battery_support_link` | Fixed to the aircraft, ROS position `(-0.02623, 0, -0.01000)` m; contains the cradle and end-block meshes |
| `battery_link` | Child of `battery_support_link`, local position `(0, 0, 0.04410)` m; ROS aircraft position `(-0.02623, 0, 0.03410)` m |
| Battery envelope | Box with ROS X/Y/Z dimensions `(0.1915, 0.1036, 0.0726)` m |
| Cradle | `Battery/stl/cradle.stl` |
| Rear end block | `Battery/stl/end_block.stl` |

All listed link orientations are aligned with the ROS aircraft frame: X forward,
Y left, Z up. The support-link origin is a modelling reference at the bay floor
under the battery centre. It is **not** an asserted centre of mass.

`Battery/stl/coupon_floor.stl` and `Battery/stl/coupon_side.stl` are test pieces,
so they are excluded from the aircraft. The current cradle supersedes the older
handoff's separate side/top pads and battery resting directly on the bay floor.
See [adopted design](/Users/john/Code/Printables/Battery/bay-geometry-verified.md:76)
and [superseded handoff items](/Users/john/Code/Printables/Battery/bay-geometry-verified.md:148).

## Coordinates and nominal fit

Battery CAD uses **millimetres**, with **+X right/starboard, +Y up, +Z aft**.
The STLs retain their assembly positions; they are not centred print-layout files.
This is the original Blender axis convention. It differs from the sensor-bracket
aircraft CAD frame. See [source-frame definition](/Users/john/Code/Printables/Battery/x950-battery-bay-inserts-handoff.md:35).

For a point `(X, Y, Z)` in a Battery source STL:

```text
Sensor-bracket aircraft CAD, mm: (X, Z, -Y)
ROS aircraft frame, m:           (-Z, -X, Y) / 1000
Original Blender frame, m:      (X, Y, Z) / 1000
```

The nominal battery source bounds are X `-51.8…51.8`, Y `-2.20…70.40`,
Z `-69.52…121.98` mm. Its geometric centre is `(0, 34.10, 26.23)` mm, which gives
the ROS battery position above. A 12 mm front plenum separates the pack from the
bay's front wall. These values follow
[cradle parameters and placement equations](/Users/john/Code/Printables/Battery/cradle.py:25)
and the [verified placement table](/Users/john/Code/Printables/Battery/bay-geometry-verified.md:95).

The model retains the source's **nominal uncompressed** printed envelopes:

- **0.40 mm vertical preload:** the battery's nominal top is above the ceiling-rib
  contact plane by this amount.
- **0.75 mm lateral preload per side:** the free cradle cavity is narrower than
  the battery.
- **1.20 mm end-block bumper preload:** the free bumpers extend into the closed
  lid's contact plane.
- **0.50 mm free end-block-to-pack gap:** lid preload closes this gap.

These small overlaps represent intended TPU compression. The description does not
simulate deformation. See [pack fit check](/Users/john/Code/Printables/Battery/validate.py:48)
and [end-block preload](/Users/john/Code/Printables/Battery/bay-geometry-verified.md:174).

The current end-block geometry uses a fore–aft balance-connector slot, as defined
in [end_block.py](/Users/john/Code/Printables/Battery/end_block.py:48); an earlier
table in the notes describes a superseded transverse slot.

## Mass properties and confidence

The accepted battery and user-provided sensor masses sum to **4.225 kg**:

| Component | Accepted mass |
| --- | ---: |
| Battery | 3.060 kg |
| Livox Avia | 0.498 kg |
| D555 | 0.337 kg |
| RoboSense E1R | 0.330 kg |

This is a **partial known subtotal**, excluding the aircraft, motors/propellers,
electronics, other mounting hardware, TPU retention parts, and leads. The aircraft's
total mass and assembled centre of mass remain unknown; no numeric total or
aircraft centre of mass is asserted.

The source estimates approximately **0.205 kg for the cradle** and **0.060 kg for
the end block**, depending on printing. These are not weighed values and are not
included in the known subtotal. See [printed-part estimates](/Users/john/Code/Printables/Battery/bay-geometry-verified.md:227)
and the [density-based estimation code](/Users/john/Code/Printables/Battery/cradle.py:177).
Weighing the printed parts would replace these estimates without changing geometry.

[mass_properties.json](/Users/john/Code/Printables/x950_description/config/mass_properties.json)
stores the provenance, accepted masses, and candidate properties. For the battery,
it assumes a uniform box with its centre of mass at its geometric centre. The
resulting principal moments about that centre, aligned with ROS X/Y/Z, are:

```text
Ixx = 0.00408094860 kg m^2
Iyy = 0.01069546755 kg m^2
Izz = 0.01208832855 kg m^2
Ixy = Ixz = Iyz = 0
```

These are **estimates**, not measured battery inertia. The candidates are stored
in configuration and are **not yet physical inertial elements in the URDF**.
The printed-part centroid candidates in the same file use uniform effective
density over each STL envelope; actual lattice, skins, and compression can shift
their mass distribution. Leads and straps are not assigned separate mass properties.
