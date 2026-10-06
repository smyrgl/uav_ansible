# X950 description deployment

Copies runtime files from the authored `x950_description` package on the Ansible
controller into `/opt/uav/ros/src/x950_description`, then builds only that package
in the common merged workspace. The source `.blend` files and CAD scripts stay
on the workstation. Set `robot_description_source_dir` if the controller uses a
different checkout layout.

`uav-description.service` runs `robot_state_publisher` as `uav-ros`. It publishes
the transient-local `/robot_description` and `/tf_static` topics without RViz or
joint-state traffic: all modeled joints are fixed. Model meshes retain their
`package://x950_description/meshes/...stl` URIs for Foxglove asset retrieval.

The deployment launch adds identity aliases for native D555 names (`camera_link`,
`camera_depth_frame`, `camera_depth_optical_frame`, and equivalent color/infra1/
infra2 names). Their parents are the corresponding `d555_nominal_*` model frames.
These are **nominal mechanical estimates**, not a claim of measured calibration.
The authored geometry and Xacro are not modified. Disable the aliases before
deploying another publisher that owns the same native frame IDs.

The launch also adds two Avia frames from `/etc/uav/ros/description.yaml`:
`avia_nominal_lidar_frame` under `avia_link` (`avia_lidar_xyz/rpy`, the ranging
origin) and `avia_imu_frame` under it (`avia_imu_xyz/rpy`, Livox's factory IMU
offset). Nothing else in the repository carries sensor geometry: FAST-LIO's
IMU-to-LiDAR extrinsic, the lio bridge, the E1R registration, the LiDAR map view
and the bag tools all look these frames up in TF (live) or in the bag's
`/tf_static` (offline), so a calibration written here reaches all of them at
their next start.

Pose overrides can be supplied through `robot_description_xacro_mappings`, using
the existing Xacro argument names and space-separated metre/radian values. They
are checked against the exported model at launch. E1R data should use
`e1r_nominal_lidar_frame` until a calibrated frame is supplied.

The role and E1R role share a merged workspace: run them sequentially. Service
launchers source that workspace and `/opt/ros/jazzy`; the systemd environment
file supplies common DDS settings. The unit restarts on exit and starts at boot.
