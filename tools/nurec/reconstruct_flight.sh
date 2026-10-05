#!/bin/bash
# One flight bag to a 3DGUT reconstruction and an Isaac Sim asset, on the replay host.
#
#   reconstruct_flight.sh <bag dir> [--every N] [--min-height M] [--sfm] [--no-train]
#
# Steps (each idempotent, outputs under /srv/flights/nurec/<bag name>/):
#   frames/        bag_to_frames.py: colour + IR frames, metric camera-to-world poses (PX4 odometry + URDF)
#   colmap/        frames_to_colmap.py: COLMAP-format dataset, LIO map as the seed, propeller masks (prop_mask.py)
#   runs/<bag>_3dgut_mcmc        3DGUT training in the 3dgrut:cuda12 container, NuRec USDZ + PLY exported
#   --sfm: colmap/sfm (COLMAP SfM, CPU), colmap/sfm_metric (aligned to the odometry), a second training run
# Prerequisites on the host: the replay role's venv (/opt/uav/replay-venv), docker image 3dgrut:cuda12, colmap
# for --sfm. The tools are looked up beside this script.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=${NUREC_ROOT:-/srv/flights/nurec}
PY=${REPLAY_PY:-/opt/uav/replay-venv/bin/python}
BAG=$1; shift
EVERY=10; MINH=1.0; SFM=0; TRAIN=1
while [ $# -gt 0 ]; do
  case "$1" in
    --every) EVERY=$2; shift 2 ;;
    --min-height) MINH=$2; shift 2 ;;
    --sfm) SFM=1; shift ;;
    --no-train) TRAIN=0; shift ;;
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
  log "propeller masks"
  (cd "$OUT/colmap/images" && $PY "$HERE/prop_mask.py" . | tail -1)
fi
if [ "$TRAIN" = 1 ] && [ ! -f "$ROOT/runs/${NAME}_3dgut_mcmc/.done" ]; then
  log "3DGUT on the odometry poses"
  "$HERE/run_3dgut.sh" "$NAME/colmap" "${NAME}_3dgut_mcmc" > "$OUT/train.log" 2>&1 && date -u +%FT%TZ > "$ROOT/runs/${NAME}_3dgut_mcmc/.done"
  ls -d "$ROOT/runs/${NAME}_3dgut_mcmc"/*/ | tail -1 | xargs -I{} sh -c 'echo "run: {}"; cat {}/metrics.json; echo'
fi
if [ "$SFM" = 1 ]; then
  if [ ! -f "$OUT/colmap/sfm/sparse/0/images.bin" ]; then
    log "COLMAP SfM (CPU, fixed intrinsics, masks)"; "$HERE/run_colmap_sfm.sh" "$OUT/colmap" > "$OUT/sfm.log" 2>&1; tail -4 "$OUT/sfm.log"
  fi
  if [ ! -f "$OUT/colmap/sfm_metric/sparse/0/images.bin" ]; then
    log "align the SfM model to the odometry"; "$HERE/align_colmap_to_metric.sh" "$OUT/colmap" | tee "$OUT/align.log"
  fi
  if [ "$TRAIN" = 1 ] && [ ! -f "$ROOT/runs/${NAME}_3dgut_mcmc_sfm/.done" ]; then
    log "3DGUT on the SfM poses"
    "$HERE/run_3dgut.sh" "$NAME/colmap/sfm_metric" "${NAME}_3dgut_mcmc_sfm" > "$OUT/train_sfm.log" 2>&1 && date -u +%FT%TZ > "$ROOT/runs/${NAME}_3dgut_mcmc_sfm/.done"
    ls -d "$ROOT/runs/${NAME}_3dgut_mcmc_sfm"/*/ | tail -1 | xargs -I{} sh -c 'echo "run: {}"; cat {}/metrics.json; echo'
  fi
fi
log "done: $OUT"
