# RealSense D555 integration

The first-pass model retains the supplied D555 CAD geometry and uses the D455
optical-frame definitions as a **provisional nominal template**. The user
identified D555 as sharing the D455 optics and D585 vision-processing hardware
and approved that basis for the first pass. This does not import the D455/D585
housing, collision geometry, mass, or inertia. Nominal visualization frames
remain separate from the live camera's factory-calibrated frames.

## Upstream checked

Reviewed on 2026-09-24: `realsenseai/realsense-ros`, branch `ros2-master`, commit
`9a11121700cb4780e273e34141f6402fe184321d` (2026-08-30, release 4.58.4).
Its [URDF directory](https://github.com/realsenseai/realsense-ros/tree/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description/urdf)
and [mesh directory](https://github.com/realsenseai/realsense-ros/tree/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description/meshes)
contain no D555 description or mesh. Driver support for D555 does not imply
availability of a D555 model in `realsense2_description`.

The nearest names are different products:

| Model | Declared width × height × depth | Reason not imported |
| --- | --- | --- |
| D455 | 124 × 29 × 26 mm | Different housing and mechanical datum |
| D585 | 182 × 43 × 93.1 mm | Different housing; several datum and IMU values explicitly marked provisional |

See [D455 model, lines 29–86](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description/urdf/_d455.urdf.xacro#L29-L86)
and [D585 model, lines 29–81](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description/urdf/_d585.urdf.xacro#L29-L81).
Neither file declares full D555 compatibility. The D455 macro has no
optics-only switch: `use_mesh:=false` substitutes a box visual, while its
collision and inertial blocks remain unconditional. Reusing its optical
transforms separately avoids introducing a different camera's housing and
physics into this model.

The official [D555 datasheet v1.1](https://realsenseai.com/wp-content/uploads/2025/08/D555-Datasheet-v1.1.pdf)
identifies the D450 optical module and a nominal 95 mm stereo baseline. This
supports the shared-optics basis. The selected axial datum remains provisional;
IMU frames are omitted because their D555 placement has not been established.

## Reused D455 optical mathematics

All translations below are meters in the nominal camera body axes, before
the drone's inverted-mount rotation. The camera root coincides with the
depth/left-IR origin.

| Child frame | Parent | Translation xyz | Rotation rpy |
| --- | --- | --- | --- |
| Depth frame | Camera root | `0 0 0` | `0 0 0` |
| Infra1 frame | Camera root | `0 0 0` | `0 0 0` |
| Infra2 frame | Camera root | `0 -0.095 0` | `0 0 0` |
| Color frame | Camera root | `0 -0.059 0` | `0 0 0` |
| Each corresponding optical frame | Its body frame | `0 0 0` | `-pi/2 0 -pi/2` |

Source: [D455 optical definitions, lines 106–166](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description/urdf/_d455.urdf.xacro#L106-L166).
The upstream D455 also supplies IMU offsets, but those are not used here:
shared optical hardware does not establish identical IMU placement in D555.

The D455 template places the depth-zero plane 4.55 mm behind its glass and
its glass 0.1 mm behind its front plate. Its front-plate midpoint to camera-root
translation is therefore `(-0.00465, +0.0475, 0)` m. This model uses that
translation as its **provisional camera-root default**, with no relative
rotation. The 0.1 mm recess is a D455 housing assumption; the complete 4.65 mm
axial offset is explicitly an estimate for D555. No D455 tripod-mount offset
is used. See [datum definitions, lines 43–79](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description/urdf/_d455.urdf.xacro#L43-L79).

## Current mechanical frames

`d555_link` is a **housing datum**, at the front-face center. Its default position
in `base_link` is `(0.150, 0, -0.06935)` m with roll π, representing the inverted
mount. The source CAD conversion is:

```text
CAD (X, Y, Z) mm → base_link (Z + 150, -X, -Y - 69.35) mm
CAD (X, Y, Z) mm → d555_link local (Z, X, Y) mm
```

The D555 CAD lens axes were checked at source `(X,Y)` positions
`(-47.5,0)`, `(-11.5,0)`, and `(+47.5,0)` mm. These match the proposed right IR,
color, and left IR/depth lateral positions. Front barrel rings occur at source
`Z=-1.45` mm, but the CAD does not establish that this ring plane is the
optical depth-zero plane. It is not used as one.

`d555_nominal_camera_link` is attached to `d555_link` at
`xyz="-0.00465 0.0475 0"`, `rpy="0 0 0"`. Its Xacro arguments are
`d555_nominal_camera_xyz` and `d555_nominal_camera_rpy`, allowing the planned
extrinsic calibration to refine that attachment.

The camera root has children `d555_nominal_depth_frame`,
`d555_nominal_infra1_frame`, `d555_nominal_infra2_frame`, and
`d555_nominal_color_frame`, each with its corresponding `*_optical_frame`.
These replace the earlier convention-only frame at the housing center.
All `d555_nominal_*` frames are provisional; they do not override or claim to
reproduce the device's factory calibration. No nominal IMU frame is added.
The model does not change focal lengths, distortion, camera-info data, or any
other factory intrinsics; the planned calibration concerns extrinsic placement.

## Intended bridge — not implemented

Keep the robot description responsible for the drone, bracket, housing, and
one future fixed attachment to the camera's calibrated root. Keep the camera
driver responsible for its internal sensor transforms:

```text
base_link → d555_mount_link → d555_link
                              ├─ d555_nominal_*              [provisional visualization]
                              └─ d555_camera_link            [future calibrated bridge]
                                   ├─ d555_depth_frame → d555_depth_optical_frame
                                   ├─ d555_infra1_frame → d555_infra1_optical_frame
                                   ├─ d555_infra2_frame → d555_infra2_optical_frame
                                   ├─ d555_color_frame → d555_color_optical_frame
                                   └─ enabled IMU frames
```

The manufacturer's camera root is the left IR/depth origin, not the housing
center. The wrapper obtains internal transforms from the device's extrinsics
and applies the standard optical-axis rotation, rather than hard-coding one
camera's color or IMU offsets. See [driver transform implementation, lines
116–224](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_camera/src/tfs.cpp#L116-L224).

For the live-camera bridge, replace or verify the provisional D555
left-IR/depth-zero datum relative to the housing, including its orientation.
A device-specific dimensioned drawing or external calibration can provide
that refinement. The reviewed sources do not establish the complete exact
transform. This does not prevent the approved nominal visualization model;
it separates its assumptions from measured hardware calibration. Internal
factory extrinsics alone cannot locate the camera inside the drone frame.

## Wrapper naming and TF ownership

For a future `realsense2_camera` launch, these values produce the names above:

```text
camera_name:=d555
base_frame_id:=camera_link
tf_prefix:=''
publish_tf:=true
tf_publish_rate:=0.0
```

`base_frame_id` is a **suffix**: the wrapper prepends `camera_name` and an
underscore. Thus `camera_link` becomes `d555_camera_link`, keeping the root
distinct from this package's housing `d555_link`. Its default suffix `link`
would instead produce `d555_link` and incorrectly equate those two datums.
The driver's prefix parameter is `tf_prefix`; it prepends a literal string to
all its frame IDs. See [parameter construction, lines 27–92](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_camera/src/parameters.cpp#L27-L92)
and [frame-name definitions, lines 85–90](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_camera/include/base_realsense_node.h#L85-L90).

The wrapper publishes internal static transforms by default. Leave these
enabled for hardware and do not also publish the same child frames from the
robot description. `publish_tf:=false` disables both static and dynamic TF;
`tf_publish_rate:=0.0` alone does **not** disable static TF. See [upstream TF
parameters](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/README.md#L530-L553).

## Native D555 firmware alternative

D555 can also publish factory-calibrated internal TF directly from firmware.
In that mode the wrapper parameters above do not configure the device. Inspect
the actual TF message frame names and attach the future bridge to its actual
root. The manufacturer's native interface documents a device-specific
`/realsense/D555_<Serial>/tf_static` topic; consumers must receive those
transforms through the appropriate TF topic integration. Avoid running a
second publisher for the same internal child frames. See [RealSenseNativeROS
documentation](https://github.com/realsenseai/RealSenseNativeROS/blob/d2f6d190f2099e2f2a6b752d397c2e11c0ea7b02/README.md#tf-from-coordinate-a-to-coordinate-b)
(reviewed commit `d2f6d190f2099e2f2a6b752d397c2e11c0ea7b02`).

No camera driver, firmware, or network configuration was changed as part of
this first-pass model.
