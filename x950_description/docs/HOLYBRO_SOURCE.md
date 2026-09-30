# Holybro Jetson baseboard assembly source

The avionics visual uses Holybro's published Jetson baseboard STEP assembly. The user confirmed a Holybro Jetson Baseboard with Pixhawk 6X and authorized approximate placement in the front avionics bay; placement is not a caliper measurement. The supplied assembly also contains an Orin module and its heatsink/fan. The installed Orin SKU and exact hardware revision have not been confirmed.

## Provenance and included parts

- Official landing page: [Holybro baseboard downloads](https://docs.holybro.com/autopilot/pixhawk-baseboards/download), **3D CAD File**, `Pixhawk-Jetson-Baseboard-3D.stp`.
- Retrieved 2026-09-24 as `cad_source_meshes/holybro_jetson_baseboard.stp`, 78,172,053 bytes. Exact public media URL is preserved in the adjacent `.source.json`.
- Source SHA-256: `dfee72717b735c4b10089435ae1019482878f6607051f1a3b8397da7721ed6ee`.
- STEP header names `ORIN_NX-RC02_BOARDANDM-0603_ASM`, exported from Creo on 2024-06-19.
- The assembly contains the populated baseboard, `P3767-GROGU` Orin module geometry, heatsink/fan, `PIXHAWK6X-REV8-SIMPLE` flight-controller assembly, four nylon standoffs and nuts. Detailed source names and bounds are recorded in `.inventory.json`.
- The separately downloadable `Jetson_Base_Case.stp` alloy enclosure is **not included**. No second Pixhawk model is added on top of the one already in the assembly.

The official assembly references the Orin module through two CAD branches at the same physical placement. The conversion removes repeated instances of the same topology with numerically equivalent transforms, plus exactly coincident STL triangles. It does not delete different parts merely because their bounds match. Null solder-mask/reference geometry and non-solid curves/sheets are omitted.

## Native geometry and placement convention

The source STL preserves the STEP coordinates in **millimeters**. Its PCB footprint is 126 mm along native X and 80 mm along native Y, with native Z pointing upward. A ROS mesh import must convert millimeters to meters exactly once.

| Item | Native bounds or center, mm |
| --- | --- |
| Entire physical assembly | X 0 to 126; Y −1 to 80.512502; Z −8.0254 to 30.89832 |
| Entire assembly dimensions | 126 × 81.512502 × 38.92372 |
| PCB footprint | X 0 to 126; Y 0 to 80 |
| PCB footprint center | X 63; Y 40 |
| Orin/heatsink/fan envelope | X 8.99 to 54; Y 5.3 to 74.9; Z 7.99832 to 30.89832 |
| Pixhawk 6X housing envelope | X 64.2 to 103; Y 24.1 to 55.9; Z 2.02232 to 18.57232 |
| Four standoff axes | (3, 3), (3, 77), (123, 3), (123, 77) in XY |
| Standoff spacing | 120 × 74 |
| CAD corner bore diameter | 3.2 |

The mounting-axis positions come directly from the four nylon support assemblies and were independently confirmed by the board's cylindrical faces (radius 1.6 mm), not from a guessed inset in the 126 × 80 mm outline. These are CAD dimensions; verify actual hardware before fabrication.

For a provisional link with its origin at the footprint center and the lowest existing hardware plane, a direct proper-axis mapping is:

```text
link_x = (step_x - 63) / 1000
link_y = (step_y - 40) / 1000
link_z = (step_z + 8.0254) / 1000
```

Native +X may be aligned with vehicle +X to put the long board direction fore/aft. This is an **installation choice**, not proof of the Pixhawk's configured forward direction. No unambiguous FC forward-arrow or physical IMU origin was established from this CAD. The 6X housing center must not be advertised as a measured IMU datum.

The original STEP already has supports extending to Z = −8.0254 mm, approximately 8 mm below the board. Anchor that lowest hardware plane at the chosen bay floor unless an intentional additional spacer is desired. Do not silently add another full set of provisional risers to the supplied supports. Later caliper measurements should replace the approximate vehicle-to-assembly pose and establish the actual FC orientation and IMU datum.

## Reference dimensions and masses

Holybro's [Dimension & Weight](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-jetson-baseboard/dimension-and-weight) page lists the assembly with Orin NX, heatsink/fan and FC as 126 × 80 × 38.6 mm. The exact CAD envelope above is slightly larger because it includes protruding components and hardware; neither value should overwrite the other.

The same manufacturer page lists 85 g for the baseboard without Jetson or FC, 175 g with Jetson and heatsink but no FC, 185 g with Jetson/heatsink/Pixhawk 6X, and 190 g when M.2 SSD and Wi-Fi are also fitted. The optional alloy case is separately listed as 90 g. These describe published configurations, not a measurement of this aircraft. No inertia tensor or center of mass is inferred from these values.

## Reproduction and checks

`scripts/tessellate_holybro.py` uses the existing OpenCascade/OCP runtime, preserving the native assembly frame. It tessellates closed physical solids with 0.2 mm chordal deflection and 0.4 rad angular deflection, removes zero-area float32 triangles, and records counts, bounding boxes, duplicate removal and SHA-256 hashes in `cad_source_meshes/holybro_jetson_baseboard.inventory.json`.

This conversion retains 1,776 physical solid placements after removing 303 repeated placements, then removes 3,919 exactly coincident triangles and 225 zero-area triangles. The resulting STL has 521,374 triangles and SHA-256 `1bbf985b275e5960268d9ebf36e48e34fccd10f2f946a6a1e967a2b1e92c62c1`.

The STL is a visual asset. It has no assigned density, inertial model, calibrated FC frame, or claimed hardware configuration beyond the named manufacturer assembly.

## IMU datum follow-up

No dimensioned Rev8 sensing-origin offsets relative to this housing were verified.
Keep `fmu_housing_link` as a mechanical housing center, not an IMU origin. The
installed Pixhawk revision itself also remains to be confirmed.

Holybro's [technical specification](https://docs.holybro.com/autopilot/pixhawk-6x/technical-specification)
identifies three ICM-45686 IMUs in Rev8. The official
[DS-012 Pixhawk v6X standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-012%20Pixhawk%20Autopilot%20v6X%20Standard.pdf),
v0.4.0 dated December 2022, covers older sensor sets through Rev5. Its page-10
sensor-location drawing was inspected: it shows component layouts and direction
arrows, but no XYZ sensing-origin dimensions. It cannot establish Rev8 offsets.
[PX4's board sensor configuration](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6x/init/rc.board_sensors)
provides sensor rotations, not mechanical translations relative to the housing.
The image host for Holybro's
[Rev8 dimension drawings](https://docs.holybro.com/autopilot/pixhawk-6x/dimensions/rev-8-current)
returned HTTP 403, so those drawings were not visually verified in this pass.
A dimensioned sensor-assembly drawing or manufacturer-confirmed datum is still
needed before adding physical IMU frames.
