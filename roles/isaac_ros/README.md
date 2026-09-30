# Isaac ROS apt repository

Isaac ROS 4.6 ships native ROS 2 Jazzy Debian packages for JetPack 7.2
(`noble-jetpack`) from `https://isaac.download.nvidia.com/isaac-ros/<release>`.
There is no Docker in this path: `nvblox_node` is an ordinary executable under
`/opt/ros/jazzy/lib/nvblox_ros/`, linked against JetPack's CUDA 13.2, VPI 4 and
TensorRT 10.

This role pins three things so the install is reproducible:

| Pin | Variable | Why |
| --- | --- | --- |
| Release | `isaac_ros_release` (repository path) | NVIDIA publishes one repository per release; there is no channel that moves. |
| Signing key | `isaac_ros_key_fingerprints` | `repos.key` is fetched over HTTPS every converge, but the keyring is only installed when it carries the pinned fingerprints. |
| Package version | `isaac_ros_package_version` | `apt` installs `pkg=version`; a re-published release fails loudly instead of drifting under the config. |

`repos.key` currently carries two keys: the 2026 *Isaac ROS Buildfarm Repository
Signing* key (`428F 5D2A CFBC 9AA1 F8EA C846 58F4 DA02 3E69 1207`) that signs
`release-4.6`, and the older 2023 buildfarm key. Both are pinned.

JetPack itself ships `/etc/apt/preferences.d/nvidia-repo-pin`, which raises every
`*.nvidia.com` origin to priority 600. The Isaac repository does not currently
ship packages that shadow `packages.ros.org`, so this has no effect on the ROS
base install, but it is why `apt-cache policy` shows 600 for `ros-jazzy-nvblox-*`.

Only the nvblox packages are installed here; roles that use Isaac ROS list their
own runtime dependencies. Adding another Isaac package is a one-line change to
`isaac_ros_packages` (same version string, same repository).

Note on GPU code: the Jetson debs embed SASS and PTX for `sm_75` only. The Orin
(`sm_87`) JIT-compiles every kernel from PTX on first use and caches the result
under the invoking user's `~/.nv/ComputeCache` (`CUDA_CACHE_PATH`). Services that
run Isaac ROS nodes must give that account a persistent, writable cache, or every
start pays the compilation again; see `roles/nvblox`.
