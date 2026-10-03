# lidar_view

A live third-person view of what the aircraft's LiDARs see, as a video stream
in QGC next to the D555 RGB. FAST-LIO's voxel map, the live Avia and E1R scans,
both sensors' fields of view, the flight trail and the X950 itself (its URDF
meshes) are seen from a chase camera behind the aircraft. The frames are
rendered headless on the Orin's GPU and encoded by NVENC. They reach QGC as the
MAVLink camera's second stream.

| | |
|---|---|
| Service | `uav-lidar-view` (`uav-ros`, groups `video render`), `/usr/local/lib/uav/uav_lidar_view.py` |
| Stream | `rtsp://192.168.144.1:8555/lidar`, H.265 1280×720 at 30 fps, 4 Mbit/s |
| MAVLink | camera component 100, `VIDEO_STREAM_INFORMATION` stream 2 of 2, "LiDAR map" |
| Inputs | `/lio/map/updates`, `/cloud_registered`, `/lio/registered/e1r`, `/Odometry` (all in FAST-LIO's `camera_init`); `/lio/map/epoch`, `/lio/health` |
| Diagnostics | the `lidar_view` row: clients, fps, render time, map points, AGL and its source, boom length, cutaway |

## What is drawn

- **The map**: lio_map's 5 cm voxels, which hold both the Avia and the E1R
  returns. Points are coloured by height with Turbo, using the 2nd to 98th
  percentile of the map's heights within 80 m of the aircraft, and shaded by
  reflectivity. They are drawn as round sprites sized in world units: 6.5 cm,
  clamped to 1.6–9 px.
  - Stray returns are kept out. Indoors, the Avia returns about 0.3 % stray
    points, at random ranges along real beams out to ~430 m, and doesn't
    flag them in the Livox tag's noise bits. Each lands in a voxel of its
    own. On the bench on 2026-10-02 they were 45.5k of lio_map's 125.5k
    points, a dust over the whole view that also stretched the colour range
    to ±150 m. Since 2026-10-03, lio_map filters them at the source: a point
    joins the map only once its 0.5 × 0.5 × 0.25 m cell holds three voxels
    (lio README, *Stray returns*), so the view shows exactly what a save
    would write. The view has the same filter (`--promote`). The role turns
    it on only if lio_map's is off (`lidar_view_promote`), so one of the two
    is always active.
  - When `/lio/map/epoch` changes, the view drops the map it holds and the
    whole map follows on `/lio/map/updates`. lio_map bumps it when it takes
    points back after a divergence and when it starts a new map.
- **The live scans**: the current Avia scan in white and the current E1R scan
  in magenta, at 8 cm and at least 2.4 px. You can see at a glance what each
  sensor covers right now, including the gap between the E1R's forward edge
  (+h) and the Avia's lower edge (~1.25 h ahead at level flight). Magenta that
  doesn't sit on the map is an E1R extrinsic or timing error. The scans and
  the overlay lines are pulled 0.4 % of the way toward the camera. They lie on
  the very surfaces the map already holds, and without the pull they would
  lose the depth test to the map's own points.
- **Fields of view**: the Avia as an elliptical cone, 70.4° × 77.2°, its
  rosette's envelope. The E1R as its 120° × 90° frustum, pointing down, with
  the 120° across the aircraft. The E1R frustum reaches the ground, and a plumb
  line with a ring marks the point straight below.
- **The trail**: one point every 10 cm. When it fills, the older half is
  thinned. It survives viewers coming and going. It is cleared when the pose
  leaps, meaning more than 25 m and faster than 60 m/s, or when its time goes
  back or skips 5 s; that is FAST-LIO restarting with a new origin. Three
  leaps in 10 s show "FAST-LIO DIVERGED".
- **The X950**: about 390k triangles from `x950.urdf`, at FAST-LIO's pose. The
  pose is interpolated (see *Timing*) and moved from the IMU to `base_link`
  through the lio bridge's lever arm. The carbon finish is lifted toward a
  warm grey and rim-lit so the airframe reads against a dark map.
  - The URDF has motors but no propellers. Two rings per motor stand in for
    the rotor discs (`--rotor-radius`, 0.25 m; decorative). They are what
    makes the airframe read as a quadcopter from a long boom; its 1–2 px
    carbon arms don't.
  - The airframe is never drawn narrower than 60 px (`--model-min-px`), at
    most 3× true size. That is reached only from a long boom, where the camera
    is far from any obstacle. On the ground it is true size.
- **Eye-dome lighting**: a full-screen pass over linear depth. It darkens each
  pixel by `exp(-s Σ max(0, log2 d − log2 dₙ))` over 8 neighbours at
  1.4 px. Points have no normals, and this is what turns a point cloud from
  coloured dust into surfaces. `lidar_view_edl_strength` sets *s*.
- **The overlay**: AGL, altitude since start, speed, map size, the two scan
  rates and the UTC time on a panel at the top left, and a legend on its own
  panel at the top right. Both sit inside `lidar_view_safe_area`: by default
  10 % of the frame at the top, 8 % at the sides and 16 % at the bottom. Those
  are the bands where QGC draws its own toolbar, tool strip, camera panel,
  instruments and map thumbnail over the video. One warning at a time,
  centred above the bottom band:
  - "NO FAST-LIO POSE" when odometry has stopped;
  - "FAST-LIO DIVERGED · MAP FROZEN" once the lio watchdog has declared it
    on `/lio/health` (lio README, *Divergence*), with the countdown to its
    restart of FAST-LIO, or "AUTO-RESTART EXHAUSTED" once it has given up;
  - otherwise the view's own evidence: "FAST-LIO DIVERGED (…): RESTART
    uav-lio" after three pose leaps in 10 s, or above 40 m/s
    (`--diverged-speed`), which no X950 flies.
- **The plumb line** is amber, so it can't be mistaken for the cyan trail when
  both run down the middle of the frame.

## The chase camera

The boom points backward along the aircraft's smoothed heading, and the
horizon stays level. Heading, position and height above ground are lagged
(τ 0.8, 0.25 and 1 s), so turns and climbs swing the view instead of jerking
it. The framing is solved rather than tuned. With *v* the vertical screen
position (−1 top, +1 bottom):

- the aircraft sits at *v* = −0.1, just above the centre. The camera pitch
  follows from that: pitch = φ − atan(−0.1 · tan(fovy/2));
- the ground straight below, the middle of the E1R's footprint, sits no lower
  than *v* = 0.8. A boom of length *L* at elevation φ sees that point
  atan((h + L sin φ) / (L cos φ)) below the horizon, for a height *h* above
  ground. That gives the boom length
  *L* = max(7 m, h / (tan(pitch + atan(0.8 tan(fovy/2))) cos φ − sin φ)),
  capped at 90 m;
- φ steepens from 25° on the ground to 45° at 30 m AGL. That shows more of the
  E1R's footprint (±h fore-aft, ±1.7h across) and less sky. The Avia's forward
  view fills the top of the frame.

On the ground the boom is 7 m. At 12 m AGL it is 17 m, and at 35 m it is
33 m. The test suite checks the framing at 0.3–50 m.

**Height above ground** is the median height of the E1R's returns within
0.5 m + 0.25 h of the point straight below. It is filtered with τ 0.6 s. The
radius doubles, up to 12 m, until at least 20 returns fall inside it, because
an E1R frame (~28k points over a footprint of 2h × 3.5h) thins out with
height. With no E1R, it falls back to the takeoff height (the first pose) less
the URDF's lowest vertex: the gear's feet, 0.276 m below `base_link`.

**Occlusion**:

- Map points inside a cone from the camera to the aircraft are not drawn. The
  cone is 0.25 m wide at the camera, 0.7 m at the aircraft, and stops 0.9 m
  short of it. Trees and walls never hide the aircraft.
- Sometimes the aircraft is *enclosed*, under a ceiling or a canopy. Then
  everything more than `lidar_view_cutaway_above_m` above the aircraft is cut
  away, giving a dollhouse view. Either of two tests detects enclosure:
  - **The map**, on an occupancy grid of 0.5 m columns and 0.25 m levels: at
    least 12 columns within 5 m, and half of those that hold data, have a
    floating surface overhead. That means empty 0.1–0.6 m above the aircraft
    and occupied above that, up to 8 m. A wall, a trunk or a facade fills the
    empty band, so it doesn't count; the floor's level can't reach the band.
  - **The live Avia scan** looks like a room: 90 % of returns within 12 m,
    and at least 15 % more than 1 m above the aircraft. Outdoors, the far
    ground and the trees push the 90th percentile out to tens of metres. This
    test needs no map, which is what makes it work on the bench: there the
    Avia, 38.6° up at most, never sees the ceiling over the aircraft.

  The cut switches in after ~0.6 s and out after ~1 s. Outdoors in the open
  nothing is cut.

## Cost

Nothing heavy runs while nobody watches. Idle, the node follows only
`/Odometry`, for the trail.

The first RTSP client creates the media pipeline. The node then subscribes to
the map, the scans and the E1R. lio_map notices the new subscriber and sends
the whole fine map once. That message goes to every subscriber, so the view
deduplicates by voxel key, in a sorted int64 array at 8 B a voxel. The last
client's departure unsubscribes and frees the GPU map.

Per frame, the GPU draws up to 4 M points and 390k triangles plus the EDL
pass. The 3.7 MB RGBA frame is read back through two pixel-pack buffers: each
frame returns the previous one, so `glReadPixels` never stalls the pipeline.
nvvidconv converts it to NV12 and flips it, since OpenGL rows run bottom-up,
on the VIC. NVENC encodes it. The CPU only copies.

The stream is shared between clients. Every PLAY forces an IDR, so a second
viewer starts at once.

Measured 2026-10-02 with the map full (4 M points): an RTSP client on the
Jetson received 29.7 fps on average, and a frame took 6.5 ms to render, read
back and push, against a 33 ms budget.

## Timing

FAST-LIO's poses are stamped in the Avia's PTP clock and arrive at 10 Hz. The
view is drawn at the stamp *now − latency − 1.3 × interval − 20 ms*. The
latency is the smallest arrival-minus-stamp seen lately. Showing one interval
late means the pose after the drawn instant has almost always arrived, so the
model and camera interpolate (slerp) instead of stepping at 10 Hz. It also
needs no agreement between the Jetson's clock and the PTP timescale.

## QGC

The camera advertises two streams. QGC requests stream 0, meaning all of them,
and lists "D555 RGB" and "LiDAR map" in the camera's stream selector. Choosing
one sends `VIDEO_START_STREAMING <id>`, which the camera acknowledges without
touching the RGB pipeline. QGC's player then opens the stream's URI. `RUNNING`
in the stream's flags means this server accepts connections.

Set `lidar_view_qgc_overlay: true` to flag the stream as *thermal* instead.
QGC then leaves it out of the selector and overlays it on the RGB stream, both
at once, in the camera's thermal view mode: picture-in-picture, blend or full.
It is a different kind of camera, but that is the only way QGC shows two
streams together.

## Checks

```bash
systemctl status uav-lidar-view
gst-launch-1.0 rtspsrc location=rtsp://127.0.0.1:8555/lidar latency=0 ! rtph265depay ! h265parse ! fakesink -v
```

`--demo AGL` renders a synthetic site with no ROS at all: rolling ground, a
road, trees and a building, with the aircraft AGL metres up. The Avia and E1R
scans are cut from the site by their fields of view, with the nearest return
per 0.25° bin as a crude z-buffer. Use it to tune the look and check the
framing at any height:

```bash
PYOPENGL_PLATFORM=egl python3 /usr/local/lib/uav/uav_lidar_view.py --demo 30 --snapshot /tmp/demo.png \
  --urdf /opt/uav/ros/install/share/x950_description/urdf/x950.urdf \
  --package-dir x950_description=/opt/uav/ros/install/share/x950_description
```

A one-frame render of the live map to a PNG doesn't need RTSP or a client.
Run it as any user with the ROS environment:

```bash
PYOPENGL_PLATFORM=egl python3 /usr/local/lib/uav/uav_lidar_view.py --snapshot /tmp/view.png \
  --imu-lever-arm 0.22035 -0.02626 0.1384 \
  --urdf /opt/uav/ros/install/share/x950_description/urdf/x950.urdf \
  --package-dir x950_description=/opt/uav/ros/install/share/x950_description
```

## Caveats

- The view is only as good as FAST-LIO. A diverged FAST-LIO shows up here as
  kilometres of trail and ALT in the thousands. On 2026-10-02 the Avia on the
  bench faced something about 1.1 m away. That is inside its blind zone, and
  FAST-LIO got almost no usable returns from the first scan; the map filled
  to its 4 M cap with the runaway. Since 2026-10-03 the lio watchdog freezes
  the map, takes the runaway back out of it, and restarts `uav-lio` (lio
  README, *Divergence*); the map starts over with it. FAST-LIO does not
  recover by itself, and a restart helps only once the Avia has a view: with
  the aircraft turned toward the room, it restarted cleanly.
- The map is in FAST-LIO's start frame. Heights are coloured along its z,
  which is only as level as the aircraft was when FAST-LIO started.
- The field-of-view mounts are the nominal ones (`lidar_view_*_mount`). The
  E1R's points are placed with TF's extrinsic by lio_register, not with these.
