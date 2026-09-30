# X950 description

First-pass **ROS 2 Jazzy** mechanical model, authored in Blender 5.1.2 with
LinkForge 1.5.3. Open `x950_linkforge.blend` and select **X950_ROS_LinkForge**.
The original `x950.blend` is unchanged; its original scene is also retained in
the new file. ROS loads `urdf/x950.urdf.xacro`; `urdf/x950.urdf` contains the same
default assembly in plain URDF.

Included:

- X950 airframe and fixed propeller visuals.
- Livox Avia, D555 and E1R manufacturer geometry with the supplied printed brackets.
- Measured battery envelope and final TPU cradle/end-block envelopes.
- Holybro Jetson/Pixhawk 6X assembly, approximately located in the front avionics bay.
- Two MAN1216Q50 antenna envelopes on the v02 brackets, replacing both stock GNSS assemblies.
- Holybro H-Flow optical-flow/ToF landing sensor under the belly (manufacturer STEP, joined into one visual; nominal flow-optical, range-beam and PX4 FRD frames with coverage guides; added 2026-09-30).
- Drawing-derived Hadron 640R+ approximation on the front plate's lower slope: upright, 15° nose-down, in the printed backing plate v2 on the wedge v1 adapter (mount parts included as visuals; decided 2026-09-30).
- Confirmed bracket, battery, TPU and frame material identities, with provisional appearance choices identified separately.

This model supports visualization and mounting-frame review. The avionics
installation pose remains an estimate and the Hadron pose awaits its physical
offer-up; other placements follow the supplied CAD
and bracket design sources. It has **no collision model, complete inertias,
rotor dynamics, sensor plugins, or calibrated aircraft-to-sensor extrinsics**.
The Hadron is a documented approximation because official CAD is available
from FLIR upon request. No claim of a measured aircraft CoM or IMU datum is made.

## Build and view on Jazzy

Place this folder at `src/x950_description` in a ROS workspace, source Jazzy,
then run from the workspace root:

```bash
rosdep install --from-paths src/x950_description --ignore-src -y
colcon build --symlink-install --packages-select x950_description
source install/setup.bash
ros2 launch x950_description display.launch.py
```

The launch file expands Xacro, starts `robot_state_publisher` and opens RViz.
All joints are fixed, so no joint-state publisher is needed. RViz uses
`base_link` as its fixed frame. Publish without opening RViz with
`rviz:=false`. To expand manually:

```bash
xacro "$(ros2 pkg prefix --share x950_description)/urdf/x950.urdf.xacro" \
  -o /tmp/x950.urdf
```

The authoring computer has no ROS 2 installation. Xacro expansion has been
validated locally; `colcon`, `robot_state_publisher` and RViz still need a
runtime check on the Jazzy host.

## Adjust poses

Every listed stem has two launch/Xacro arguments, `<stem>_xyz` and
`<stem>_rpy`. Each replaces the complete joint origin **relative to its parent**;
it is not an additive correction. Units are metres and roll-pitch-yaw radians.

| Assembly | Move entire assembly | Move component relative to its mount |
| --- | --- | --- |
| Avia | `avia_mount` | `avia` |
| D555 | `d555_mount` | `d555` |
| E1R | `e1r_mount` | `e1r` |
| Battery and supports | `battery_support` | `battery` |
| Holybro assembly | `avionics_carrier` | `fmu_housing` (frame only, not an IMU) |
| Port GNSS antenna | `gnss_main_mount` | `gnss_main` |
| Starboard GNSS antenna | `gnss_aux_mount` | `gnss_aux` |
| Hadron | `hadron_mount` | `hadron` |
| H-Flow | — (direct to `base_link`) | `hflow` |

Additional independent frame adjustments are `d555_nominal_camera`,
`hadron_nominal_thermal_optical_frame` and
`hadron_nominal_visible_optical_frame`. For example:

```bash
ros2 launch x950_description display.launch.py \
  avia_mount_xyz:="0.150 0 0.093250"
```

Inspect defaults with `ros2 launch x950_description display.launch.py --show-args`.
Launch overrides do not edit Blender or the plain URDF. Record accepted poses
in the authoring scene and re-export. Preserve calibrated values outside the
rebuild scripts before rebuilding the initial model.

## Frames, calibration and mass

ROS axes are **X forward, Y left, Z up**. `base_link` preserves the aircraft CAD
origin. Meshes are in metres with unit scale and package-relative resource paths.
[Frame notes](docs/FRAME_NOTES.md) describe all datums and source transformations.

The D555 retains its own supplied housing CAD. The examined upstream RealSense
package has no D555 component, and its D455 macro also inserts the wrong housing,
collision and inertial data. We reuse the documented D455 optical layout instead:
D555 CAD corroborates the lateral lens spacing, while the axial origin remains
provisional. The names contain `nominal` so they do not take ownership of the
live driver's factory-calibrated internal frames. No D455 IMU location is assumed.
Keep factory intrinsics/internal extrinsics in the RealSense driver and calibrate
the aircraft-to-camera transform. See [RealSense integration](docs/REALSENSE_INTEGRATION.md).

D555 depth/RGB coverage guides are available in a hidden Blender collection.
They use the owner-supplied 87×58° and 90×65° fields, each ±3°, at nominal lens
origins. Their 0.6 m length is for drawing only. Guides are excluded from ROS.
Hadron lens-face frames are approximate; GNSS frames identify mechanical seats,
not antenna phase centers. The launch file starts no sensor drivers.

The recorded component masses total **4.331 kg**: battery 3.060 kg, Avia 0.498 kg,
D555 0.337 kg, E1R 0.330 kg, antennas 2×0.025 kg and Hadron 0.056 kg. This subtotal
excludes the airframe, motors, avionics, mounts and cables; it is **not aircraft
mass**. Source quality and tentative battery/CoM calculations are retained in
`config/mass_properties.json`; none are silently exported as physical inertials.
TPU meshes describe the external printed-part envelopes, not slicer lattice/infill.

## Materials and visuals

Black PAHT-CF brackets, black TPU95 battery supports, silver plastic battery wrap,
carbon-fiber tubes and composite frame panels follow owner guidance. Frame panel
colors follow the supplied product photo. D555 black metal, Avia silver metal and black plastic GNSS radomes also follow
owner confirmation. Exact resin formulations, alloys, finishes and numerical
surface roughness remain unmeasured. [Materials notes](docs/MATERIALS.md)
and `config/materials.json` distinguish confirmed identity from chosen appearance.
Blender retains shading properties; standard URDF/RViz receives per-visual colors.
No material choice assigns density or changes the mass model.

Preview the [whole aircraft](docs/x950_overview.png),
[sensor assembly](docs/x950_sensors.png), or
[internal components](docs/x950_internals.png). The Hadron's ray-cast
field-of-view obstruction map is [here](docs/hadron_fov_overlap_2026-09-30.png). The latter two are illustrative
cutaways: exterior parts are temporarily hidden for the render only.

## Edit and export

Use **`scripts/export_linkforge.py`**, rather than LinkForge's stock export
button: the stock exporter introduces default mass/inertia values. With the
generated scene active and the Blender MCP extension running, execute from the
original `Printables` folder:

```bash
python3 x950_description/scripts/blender_rpc.py \
  x950_description/scripts/export_linkforge.py
```

This refreshes the assembly manifest, exports meshes/URDF/parameterized Xacro,
and validates LinkForge conversion and all pose overrides. Save the Blender file
after accepted edits. For a complete initial rebuild, open the original
`x950.blend`, select its original scene and run:

```bash
python3 x950_description/scripts/blender_rpc.py \
  x950_description/scripts/build_blender_scene.py
python3 x950_description/scripts/blender_rpc.py \
  x950_description/scripts/export_linkforge.py
```

The builder requires the original source subfolders and prepared assets under
`cad_source_meshes/`. It refuses to replace an existing generated scene. It
recreates initial placements, materials and the lighter avionics visual; it does
not preserve manual calibration. Source CAD and native-millimetre intermediates
remain separate from the metre-scale `meshes/` installed into ROS.

## Verification and limits

Validation compares every exported visual's world bounds and every fixed frame
against the Blender manifest, and checks installation directions. The final tree
has **29 links, 28 fixed joints, 42 visuals and 38 pose arguments**. Native
LinkForge and official ROS Xacro 2.0.13 both expand defaults to the plain URDF and
exercise every pose override independently. Machine-readable results are in
`docs/geometry_validation.json`, `docs/linkforge_validation.json` and
`docs/xacro_validation.json`. On a host with NumPy and Xacro, repeat:

```bash
python3 scripts/validate_description.py
python3 scripts/validate_xacro.py
```

CAD meshes retain some topology warnings (unwelded edges, slivers and open or
non-manifold geometry); expected missing-physics warnings also remain. These
visual assets have not been approved for collision or inertia computation.
The complete display assembly was reduced from **1,508,256 to 357,973 triangles**
(76.3% fewer; about 17.9 MB of binary STL). The Holybro assembly uses 78,206
triangles instead of 521,374. Every accepted reduction kept bounds differences
under 0.25 mm and sampled bidirectional surface differences under 0.5 mm, with
all link poses unchanged. These are sampled geometric checks, not a global
maximum-error guarantee or a Foxglove performance benchmark. Reports are
`docs/avionics_mesh_optimization.json` and `docs/display_mesh_optimization.json`;
detailed source geometry is retained.

Source-specific documentation: [primary sensors](docs/sensor_source_audit.md),
[battery](docs/BATTERY_NOTES.md), [Holybro](docs/HOLYBRO_SOURCE.md),
[antennas](docs/ANTENNA_NOTES.md), [Hadron](docs/HADRON_SOURCE.md).
Package metadata uses a placeholder maintainer address and `Proprietary` marker;
choose project-approved metadata and confirm CAD redistribution rights before
publishing the package.
