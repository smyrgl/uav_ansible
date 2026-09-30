# Jetson platform stats (isaac_ros_jetson_stats)

NVIDIA's ROS 2 wrapper around jetson-stats (`jtop`), vendored from
[NVIDIA-ISAAC-ROS/isaac_ros_jetson](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_jetson)
branch `release-4.6` (the Jazzy release line), commit
`b282ff3415ddcdcbec5701a523de7042933b43f4`, MIT (`files/LICENSE.upstream`). The only
local changes remove upstream's `isaac_ros_common` version-info hooks (a
build-time dependency that only embeds build metadata) from
`isaac_ros_jetson_stats_services/CMakeLists.txt` and
`isaac_ros_jetson_stats/setup.py`, so the packages build without Isaac ROS
installed. Update by replacing `files/` from a
newer tag and bumping the commit in `defaults/main.yml`.

`uav-jetson-stats.service` runs the `jtop` node as the `uav-ros` user (a member
of the `jtop` group, so no root) and publishes on `/diagnostics` once a second.
The node runs from `/opt/uav/jtop-venv`, a venv holding the `jtop` client library
pinned to `jetson_stats_version`: jetson-stats 7.x installs itself into the
installing user's `~/.local/share/jtop` venv and points `jtop.service` at it, so
no system interpreter can import it, and the client must match the service
version.

| Row | Content |
| --- | --- |
| `jetson_stats/board/Status` | nvpmodel, jetson_clocks state, uptime |
| `jetson_stats/board/Config` | module, L4T/JetPack, library versions |
| `jetson_stats/cpu/N`, `gpu/<name>`, `engine/<name>` | load, frequency, governor |
| `jetson_stats/mem/RAM`, `SWAP`, `EMC` | usage |
| `jetson_stats/temp/<zone>` | every thermal zone; WARN at 84 °C, ERROR at 100 °C |
| `jetson_stats/power/<rail>` | INA3221 rails (VDD_IN is the module's total draw) |
| `jetson_stats/fan/<name>` | speed, profile |

The health node folds these into one "Jetson Orin NX" summary row so the
Foxglove Diagnostics Summary stays readable; the detail rows remain available
in the Diagnostics Detail panel. The upstream launch file also starts a
`diagnostic_aggregator` for rqt_robot_monitor; it is not used here.

The fan, nvpmodel and jetson_clocks services exist on the node but the Foxglove
bridge's service whitelist does not expose them; use `ros2 service call` on the
Jetson if needed.
