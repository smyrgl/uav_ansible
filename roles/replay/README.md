# Replay and archive host

Autonomy roadmap §7.7 and Stage 0b/1: an x86 NVIDIA workstation (`atomic`,
`inventory/replay.yml`) that archives the flight bags and replays them through
the aircraft's own LIO stack, so a planner or a map can be scored against what
the aircraft actually computed in flight.

```sh
ansible-playbook -i inventory/replay.yml site.yml --tags replay
uav-replay /srv/flights/flight_<UTC>            # on the host
```

## What the role builds

The same source builds as the aircraft, from the same roles and pins, by
including their build task files: `ros_health/px4_messages.yml` (px4_msgs at
the firmware's commit), `avia/messages.yml` (the Livox message package alone),
`lio/build.yml` and `lio/scripts.yml` (FAST-LIO in its C++17 tree, the lio
bridge, voxel map, E1R registration), `px4_bridge/build.yml`. Their restart
handlers are disabled here (`<role>_manage_services: false` in the play): the
host runs no aircraft unit. On top: a ZFS dataset for the bags
(`replay_zfs_dataset` at `replay_flights_dir`), a scoring environment
(`replay_venv`: evo, mcap), the launcher and the scorer.

DDS isolation: the host gets its own domain (`replay_domain_id`, 42) and a
Cyclone configuration bound to loopback, so a replay can never join the
aircraft's graph. (The aircraft's Cyclone is bound to the drone LAN interface
only, so the two cannot meet even on the shared home LAN; the domain and
loopback binding are the second lock.)

## uav-replay

`uav-replay <bag> [--rate R] [--out DIR] [--no-score] [--view]` starts
FAST-LIO, the lio bridge, the voxel map (saving into the output directory), the
E1R registration, the PX4 bridge and a rosbag2 recorder of their outputs, all
with `use_sim_time`; plays the bag with `--clock-topics-all` (a /clock update
before every message; it cannot be combined with `--clock`) and an exclusion
that replays **only sensor inputs**: nothing from `/fmu/in`, and none of the
live stack's outputs (FAST-LIO's `/Odometry` and clouds, `/lio/*`, `/px4/*`,
dynamic `/tf`, diagnostics), which would otherwise be echoed by the replayed
nodes and score as a perfect match. rosbag2 full-matches the exclusion regex,
so it is written as `^/(lio/.*|...)$`. `/tf_static` is replayed: the URDF's
static frames are an input this host does not produce. After playback the map
is saved (`/lio/map/save`), everything is stopped, and the scorer runs.

Lessons from the first runs (2026-10-04): a replay killed half-way leaves its
nodes on the domain, and a second FAST-LIO on the same bag doubles every pose
(the launcher now clears leftovers first); FAST-LIO opens a position log under
its source tree's `Log/` at start and its destructor closes that pointer
unconditionally, so the directory must exist (the role creates it) and
FAST-LIO is ended with SIGTERM rather than SIGINT.

## Scoring

`replay_score.py` reads `/lio/odometry` from the live bag and the replay,
matches poses by header stamp (both stamp with the LiDAR frame's time), aligns
the replayed trajectory to the live one with one SE(3) Umeyama fit (the two
runs have different map origins: FAST-LIO's starts at boot, the replay's at
the bag's first scan) and reports the APE median, p95, max and RMS. The maps
(the live `lio_map.pcd` the recorder's post-flight step saved, the replayed
one) are compared as voxel sets after the same transform: Jaccard, meaningful
only when the live map covers the bag's interval (the recorder resets the live
map when a bag starts, `flight_recorder_reset_map_on_start`), and recall, the
fraction of replayed voxels the live map also holds, meaningful regardless.
Gates: `replay_ape_median_m` (0.02), `replay_map_jaccard_min` (0.98) or
`replay_map_recall_min` when set. Exit 0 on pass, 3 on fail; `score.json`
beside the replay.

A bench bag of a stationary aircraft cannot validate fidelity: the trajectory
has no extent, so the alignment's rotation is unobservable. Replay-against-
replay of the same bag (a common origin) is the determinism check; a flight bag
is the fidelity check.

## Offload and archive

The aircraft pushes signed-off bags here itself (flight_recorder role,
`uav-offload-bags.timer`): its key, fetched by the aircraft play into the
control machine's gitignored `.offload-keys/`, is installed for `replay_user`
restricted to `rrsync -wo` into the flights dataset, so it can write bags and
nothing else. Run the aircraft play before this one when the key changes.

Behind the NVMe sits the TrueNAS share (`replay_archive_share`, user
`replay_archive_user`): with `truenas_uav_password` in the vault the role
installs a root-only credentials file, a systemd mount/automount at
`replay_archive_dir`, and `uav-archive-bags.timer`, which mirrors the dataset
to the share one way every `replay_archive_interval` (replay outputs and
offload stamps excluded, nothing deleted on either side). Without the vault
entry those steps are skipped and reported.
