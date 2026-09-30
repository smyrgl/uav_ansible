# MAN1216Q50 antenna and v02 bracket source audit

The project owner confirmed there is no antenna STEP model and approved a
primitive-based visual model from the datasheet, with measurements to refine it
later. Use two MAN1216Q50 antennas on two identical v02 printed brackets, replacing
the factory GNSS assemblies on the port and starboard rear arms.

## Sources and precedence

- `Antenna/RTK_mount_NOTES.md`: latest bracket decisions and the v02 fit correction.
- `Antenna/RTK_extraction.json`: original factory geometry and mount datums.
- `Antenna/build_rtk_antenna_mount_v02.py`: build and print-frame definitions.
- `Antenna/rtk_antenna_mount_v02.stl`: selected finished bracket, in millimetres
  and print orientation; 18,966 triangles.
- `Antenna/rtk_bracket_stock.stl`: repaired stock assembly in the bracket build
  frame; 14,546 triangles.
- `Antenna/rtk_full_left_aircraft.stl`: the same repaired stock assembly in the
  aircraft coordinate frame; used to recover placement.
- Original `Scene` objects `gnss_antenna_main` and `gnss_antenna_alt` in
  `x950.blend`: verified against their actual mesh vertices and world matrices.
- [Manufacturer MAN1216Q50 datasheet hosted by SparkFun](https://cdn.sparkfun.com/assets/c/4/1/f/c/MAN1216Q50-Updated-Dec2025.pdf),
  also supplied as `Antenna/MAN1216Q50-Updated-Dec2025.pdf`.

The linked PDF was opened online and the local four-page copy was extracted.
Its complete page 4 mechanical drawing was rendered and inspected visually.
Although the filename says updated December 2025, the drawing's revision table
says Rev0, Preliminary, 2025/10/17.

## Antenna dimensions and limitations

The datasheet specifies a **50.00 mm maximum diameter, 42.16 mm overall height,
and 25 g mass**. Its drawing locates three mounting holes at
`(0, +13.50)`, `(-11.69, -6.75)`, and `(+11.69, -6.75)` mm: a nominal 27 mm
bolt circle. The central recess is 19 mm diameter. Exact recess depth, shoulder
height, taper diameters, thread size, insert depth, and phase-centre coordinates
are not dimensioned. The silhouette shows a tapered upper portion and a wider
lower band; those undimensioned details must be marked approximate.

The electrical interface carries RF and DC through one SMA connection. The
datasheet lists 50 ohms, RHCP, 360-degree coverage, IP67, a 28 dB LNA, 3-16 V,
and 15 mA typical/20 mA maximum current. No phase-centre offset or variation,
mass centre, or inertia tensor is provided.

The local mount notes provide the additional physical-part evidence: M2.5
inserts were test-fitted, and the antenna connector was confirmed as a recessed
SMA **male** jack. The older extraction JSON calls it female; the later notes
explicitly resolve that contradiction and take precedence. Thread engagement
depth remains an open measurement. These details are source-note evidence,
not additional dimensions read from the PDF.

For the first visual model, the 50 mm by 42.16 mm envelope and flat mounting
seat are established. A shoulder/taper may be approximated without changing
that envelope. Do not assign a GNSS phase centre at the geometric centre or
radome top. The model's antenna frame should identify the mechanical mounting
seat centre; receiver measurement-frame calibration remains unknown.

## What is replaced

The stock source is one fused cosmetic assembly: bracket, collar, sleeve, and
old antenna/mast. It is not a separate reusable pole plus antenna mesh. The v02
bracket already preserves the factory lower airframe interface, saddle, feet,
pins, bell, and arm-screw ears, while replacing the old collar/mast with the
new retention plate.

Remove each complete factory `gnss_antenna_main`/`gnss_antenna_alt` visual from
the generated ROS assembly and substitute the v02 bracket plus new antenna.
Retaining the old whole mesh or adding `rtk_full_left_aircraft.stl` would leave
duplicate factory bracket/mast geometry. Keep the original source scene intact.

v02 supersedes v01: the four factory arm-screw bores were opened to 3.4 mm
clearance after a physical fit check. The remaining interface is unchanged.
The measured STL volume is approximately 20,271.536 cubic millimetres; this is
not a measured printed mass. The v01 STL and 3MF are not the selected bracket.
The notes establish that the part is symmetric and two identical prints fit;
both aircraft placements below are proper rotations, not reflected geometry.

## Frame definitions

All STL coordinates are millimetres. The bracket build frame has its origin on
the mast/seat axis at the foot-pad plane, with +Z upwards. Arm contact is at
build Z = 9.9 mm; antenna seat is at Z = 30.48 mm. The v02 STL is exported
upside-down for printing, with its antenna seat on the print bed.

Undo print orientation before applying a build-frame placement:

```text
build_mm = (print.X, -print.Y, 30.48 - print.Z)
```

The aircraft coordinate frame (ACF) has +X starboard, +Y aft, +Z down. Original
Blender coordinates are `(ACF.X, -ACF.Z, ACF.Y) / 1000`. ROS FLU coordinates are
`(-ACF.Y, -ACF.X, -ACF.Z) / 1000`, equivalently
`(-Blender.Z, -Blender.X, Blender.Y)`.

## Verified placement

A rigid least-squares fit between all 43,638 corresponding triangle vertices
of the stock-build and full-left-aircraft STLs gives a maximum residual of
0.0000222 mm. The other instance follows the two actual factory glTF placements.
Both were then checked against the original Blender objects' actual vertex
clouds and world matrices: 7,266 of the 7,267 unique repaired-STL vertices match
original mesh vertices, within 0.0000115 mm for port and 0.0000072 mm for
starboard. The repaired STL has one additional vertex absent from the original
mesh. Rotated object bounding-box corners should not be substituted for actual
mesh extrema in this comparison.

Build-frame to aircraft transforms, with translations in millimetres:

```text
port / original gnss_antenna_main:
[-0.139173096302  +0.990268069396   0  -336.696488345]
[+0.990268069396  +0.139173096302   0  +334.278190058]
[ 0               0              -1   -92.500000245]
[ 0               0               0     1            ]

starboard / original gnss_antenna_alt:
[+1   0   0  +336.696472709]
[ 0  -1   0  +334.278208045]
[ 0   0  -1   -92.500000245]
[ 0   0   0     1            ]
```

Residual rotation components below 5e-9 are omitted. Both rotation determinants
are +1. The tiny port/starboard position asymmetry is retained from source CAD.

Suggested mechanical frame placements in ROS metres, with mount origins at the
foot-pad plane:

| Frame purpose | ROS translation xyz | ROS roll-pitch-yaw |
| --- | --- | --- |
| Port bracket build origin | `-0.334278190058 +0.336696488345 +0.092500000245` | `0 0 3.00196631` (172 degrees yaw) |
| Starboard bracket build origin | `-0.334278208045 -0.336696472709 +0.092500000245` | `0 0 -1.57079633` (-90 degrees yaw) |
| Each antenna seat, relative to its bracket build origin | `0 0 0.03048` | `0 0 0` |

Both antennas extend upwards (+ROS Z) from their seat, to approximately
ROS Z = 0.165140000245 m. Mount-seat positions in `base_link` are port
`(-0.334278190029, +0.336696488404, +0.122980000245)` m and starboard
`(-0.334278208107, -0.336696472689, +0.122980000245)` m. Lateral seat-to-seat
separation is approximately 0.673392961 m. These are mechanical datums, not
an established GNSS phase-centre baseline.

## Direct v02 STL transforms

The following homogeneous matrices act **directly on print-STL millimetre
vertices**, including print-orientation reversal and millimetre-to-metre scale.
The translation columns are metres; do not apply a second 0.001 mesh scale.
Numerical terms below 3e-12 are suppressed.

```text
v02 print STL -> original Blender metres, port:
[-0.000139173096  -0.000990268069   0       -0.336696488404]
[ 0                0              -0.001   +0.122980000245]
[+0.000990268069  -0.000139173096   0       +0.334278190029]
[ 0                0               0        1             ]

v02 print STL -> original Blender metres, starboard:
[+0.001   0        0       +0.336696472689]
[ 0      0       -0.001   +0.122980000245]
[ 0     +0.001    0       +0.334278208107]
[ 0      0        0        1             ]

v02 print STL -> ROS metres, port:
[-0.000990268069  +0.000139173096   0       -0.334278190029]
[+0.000139173096  +0.000990268069   0       +0.336696488404]
[ 0                0              -0.001   +0.122980000245]
[ 0                0               0        1             ]

v02 print STL -> ROS metres, starboard:
[ 0      -0.001    0       -0.334278208107]
[-0.001   0       0       -0.336696472689]
[ 0       0      -0.001   +0.122980000245]
[ 0       0       0        1             ]
```

If mesh vertices are first baked into the bracket build frame in metres,
instead use the unit-scale bracket frames in the preceding table. These are
equivalent workflows; mixing them would double-transform the model.

Cable routing, connector recess depth, exact radome profile, screw engagement,
physical final fit, receiver identity of main versus alternate, and phase-centre
calibration still require hardware evidence. Preserve port/starboard mechanical
names without assigning unsupported receiver roles.
