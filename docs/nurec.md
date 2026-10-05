# NuRec: reconstructing a flight into an Isaac Sim asset

Status 2026-10-05: first shakedown on the tuning flight of the same morning,
flown with level cameras and not as a survey. The purpose was to exercise the
chain end to end and learn what a proper capture needs; the results section at
the end records what came out. Everything runs on `atomic` (RTX 5090, 32 GB).

## Which NVIDIA pipeline is which

- **NuRec robotics workflows** ([mono](https://docs.nvidia.com/nurec/robotics/neural_reconstruction_mono.html),
  [stereo](https://docs.nvidia.com/nurec/robotics/neural_reconstruction_stereo.html)):
  poses from COLMAP (mono) or cuSFM (stereo), optionally FoundationStereo depth and
  an nvblox mesh as the initial point cloud, then **3DGRUT** training
  ([nv-tlabs/3dgrut](https://github.com/nv-tlabs/3dgrut)) and a USDZ export for
  Isaac Sim. This is the path for a robot's own camera data and the one used here.
- **NRE container** (`nvcr.io/nvidia/nre/nre-ga`, pulled without an NGC login, 31 GB):
  the AV product. It consumes NCore v4 datasets (zarr: poses, intrinsics, camera
  frames, lidar ray bundles) plus "auxiliary data" (object tracks, dynamic masks)
  and trains the same 3DGUT representation with lidar supervision. Its prod configs
  are Hyperion car rigs. Worth revisiting once the E1R is back, since it is the one
  that takes LiDAR; it is not what the robotics docs run.
- Isaac Sim renders either output: the NuRec USDZ (5.x) or the standard USD
  ParticleField (6.0, where the custom USDZ is being deprecated).

## The chain as built

1. **Frames and poses out of the bag**: `tools/bag_to_frames.py` writes every
   N-th colour frame (YUY2 decoded) and the IR pair at the same stamps as PNG, with
   metric camera-to-world poses: PX4's ENU odometry interpolated at each frame's UTC
   capture stamp (slerp for attitude) composed with the URDF's static transforms
   from `/tf_static`. Output `frames.json` and `cameras.json` are pipeline-neutral.
2. **COLMAP-format dataset**: `tools/frames_to_colmap.py` writes
   `sparse/0/{cameras,images,points3D}.bin` with a FULL_OPENCV camera straight from
   `camera_info`, world-to-camera poses, relative image links, and seeds `points3D`
   with the flight's LIO map (`lio_map.pcd`) carried from `camera_init` into the
   odometry frame by an SE(3) fit of the two odometries (0.41 m rms on this flight,
   the LIO's own tilt and jump). It prints the capture's height, speed and pitch
   statistics.
3. **Propeller mask**: `tools/nurec/prop_mask.py` writes one static `_mask.png`
   beside every image (3DGRUT multiplies the loss by it). On these level frames the
   derived mask ends up excluding the whole sky half, which is where the blades
   sweep; fine for a ground reconstruction, and moot once the cameras pitch down.
4. **3DGUT** in the project's Docker image (`docker build --build-arg
   CUDA_VERSION=12.8.1 -t 3dgrut:cuda12 .`, 14.6 GB, Blackwell through CUDA 12.8;
   the first run compiles the CUDA kernels for sm_120 in about three minutes).
   `tools/nurec/run_3dgut.sh` runs `apps/colmap_3dgut_mcmc.yaml` with the venv's
   `bin` on PATH (the Slang compiler lives there and the launcher does not add it),
   exporting the NuRec USDZ and a PLY. Every 8th image is held out as the test split.
5. **Pose refinement, the mono workflow's way**: `tools/nurec/run_colmap_sfm.sh`
   runs COLMAP SfM (CPU build from apt is enough) on the same frames with the
   calibrated intrinsics held fixed and the masks applied, sequential matching with
   vocabulary-tree loop detection; `tools/nurec/align_colmap_to_metric.sh` fits the
   result onto the PX4 camera positions with `colmap model_aligner` so the second
   training run is metric too, and reports the residual, which is also a measure of
   the odometry's accuracy against the photogrammetric solution.

Working directory on atomic: `/srv/flights/nurec/<bag>/{frames,colmap}` and
`/srv/flights/nurec/runs/<experiment>/`; outputs are private (the flight was over
the owner's neighbourhood) and are not in this repository.

## Capture gotchas from the first flight

- **Camera pitch.** Both cameras and the Avia were level: half of every frame was
  sky, the horizon was in all of them, and the ground was seen only obliquely from
  up to 84 m. Brackets at 45° (Avia) and 40° (D555) are being made; the URDF mount
  transforms must follow, and the camera-to-LIO and E1R floor checks get redone.
- **Props in the frame.** With a level camera the propeller tips cross the top of
  the field of view in every frame; pitching the camera down clears them, a mask
  handles the archive.
- **Flight pattern.** A tuning flight hovers, yaws in place and flies legs; a
  reconstruction wants a double grid at 25 to 40 m with 70 % overlap and more than
  one heading, flown at the speeds the global shutter can take (5 m/s was blur-free
  here).
- **Pose accuracy.** PX4's odometry is at 0.12 m against RTK, which at 20 to 50 m
  range is several pixels of reprojection error: enough to train, not enough for
  sharp splats, hence the SfM refinement step.
- **LIO frame.** FAST-LIO's camera_init was 5.4° off level; the map seed is placed
  through the SE(3) fit, so the tilt does not reach the reconstruction, but the fit
  residual carries the LIO's drift.
- **The E1R was unpowered and the FC clock is GPS time**: see
  [flight-2026-10-05.md](flight-2026-10-05.md). Neither touched this chain, which
  uses the Jetson's UTC throughout.

## Results

(filled in below as the runs finish)
