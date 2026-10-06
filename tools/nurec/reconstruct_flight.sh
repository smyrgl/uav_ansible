#!/bin/bash
# One flight bag to a 3DGUT reconstruction and an Isaac Sim asset, on the replay host.
#
#   reconstruct_flight.sh <bag dir> [--every N] [--min-height M] [--refine] [--no-train] [--prop-mask]
#
# Steps (each idempotent, outputs under /srv/flights/nurec/<bag name>/):
#   frames/        bag_to_frames.py: colour + IR frames, metric camera-to-world poses (PX4 odometry + URDF)
#   colmap/        frames_to_colmap.py: COLMAP-format dataset, LIO map as the seed; with --prop-mask, propeller
#                  masks (prop_mask.py), for bags from the level camera only: since the A-S+ mount (2026-10-06)
#                  no propeller is in any camera's view (integration report: 0 % at CAD radius and at +20 mm),
#                  and the mask's dark-marks-against-sky search would start masking ground texture instead.
#   runs/<bag>_3dgut_mcmc        3DGUT training in the 3dgrut:cuda12 container, NuRec USDZ + PLY exported
#   --refine: COLMAP features and matches (CPU), then refine_poses_ba.sh (triangulate with the odometry poses,
#             bundle-adjust with fixed intrinsics, sim(3) back onto the odometry) and a second training run on colmap/ba
# Prerequisites on the host: the replay role's venv (/opt/uav/replay-venv), docker image 3dgrut:cuda12, colmap
# for --sfm. The tools are looked up beside this script.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=${NUREC_ROOT:-/srv/flights/nurec}
PY=${REPLAY_PY:-/opt/uav/replay-venv/bin/python}
BAG=$1; shift
EVERY=10; MINH=1.0; REFINE=0; TRAIN=1; PROPMASK=0
while [ $# -gt 0 ]; do
  case "$1" in
    --every) EVERY=$2; shift 2 ;;
    --min-height) MINH=$2; shift 2 ;;
    --refine|--sfm) REFINE=1; shift ;;
    --no-train) TRAIN=0; shift ;;
    --prop-mask) PROPMASK=1; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
NAME=$(basename "${BAG%/}"); OUT=$ROOT/$NAME; mkdir -p "$OUT"
log() { echo "$(date -u +%FT%TZ) $*"; }

if [ ! -f "$OUT/frames/frames.json" ]; then
  log "frames: every $EVERY-th colour frame with poses"
  $PY "$HERE/../bag_to_frames.py" "$BAG" "$OUT/frames" --every "$EVERY" > "$OUT/frames.log" 2>&1
  tail -1 "$OUT/frames.log"
fi
if [ ! -f "$OUT/colmap/sparse/0/images.bin" ]; then
  log "colmap dataset: metric poses, LIO seed, height gate $MINH m"
  $PY "$HERE/../frames_to_colmap.py" "$OUT/frames" "$OUT/colmap" --min-height-m "$MINH" | tee "$OUT/colmap.log"
  if [ "$PROPMASK" = 1 ]; then
    log "propeller masks"
    (cd "$OUT/colmap/images" && $PY "$HERE/prop_mask.py" . | tail -1)
  fi
fi
if [ "$TRAIN" = 1 ] && [ ! -f "$OUT/.trained" ]; then
  log "3DGUT on the odometry poses"
  "$HERE/run_3dgut.sh" "$NAME/colmap" "${NAME}_3dgut_mcmc" > "$OUT/train.log" 2>&1 && date -u +%FT%TZ > "$OUT/.trained"
  ls -d "$ROOT/runs/${NAME}_3dgut_mcmc"/*/ | tail -1 | xargs -I{} sh -c 'echo "run: {}"; cat {}/metrics.json; echo'
fi
if [ "$REFINE" = 1 ]; then
  if [ ! -f "$OUT/colmap/sfm/database.db" ]; then
    log "COLMAP features and matches (CPU, fixed intrinsics, masks)"; "$HERE/run_colmap_sfm.sh" "$OUT/colmap" > "$OUT/sfm.log" 2>&1; tail -2 "$OUT/sfm.log"
  fi
  if [ ! -f "$OUT/colmap/ba/sparse/0/images.bin" ]; then
    log "pose refinement: triangulate with the odometry poses, bundle-adjust, re-align"
    PATH=$(dirname "$PY"):$PATH "$HERE/refine_poses_ba.sh" "$OUT/colmap" 2>&1 | tee "$OUT/refine.log" | grep -E "sim\(3\)|Mean reprojection|done" | cut -c1-200
  fi
  if [ "$TRAIN" = 1 ] && [ ! -f "$OUT/.trained_refined" ]; then
    log "3DGUT on the refined poses"
    "$HERE/run_3dgut.sh" "$NAME/colmap/ba" "${NAME}_3dgut_mcmc_refined" > "$OUT/train_refined.log" 2>&1 && date -u +%FT%TZ > "$OUT/.trained_refined"
    ls -d "$ROOT/runs/${NAME}_3dgut_mcmc_refined"/*/ | tail -1 | xargs -I{} sh -c 'echo "run: {}"; cat {}/metrics.json; echo'
  fi
fi
log "done: $OUT"
