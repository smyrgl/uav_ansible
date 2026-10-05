#!/bin/bash
# COLMAP structure-from-motion (CPU build is fine) on a frames_to_colmap.py export: the calibrated
# FULL_OPENCV intrinsics are held fixed, the propeller masks beside the images are applied, matching is
# sequential with vocabulary-tree loop detection. Output: <dataset>/sfm/sparse/0 in COLMAP's own frame;
# scale and orientation come from a later alignment to the metric PX4 poses (colmap model_aligner).
# Usage: run_colmap_sfm.sh <dataset dir> [--map]   (--map also runs the incremental mapper: on the 2026-10-05
# aerial sequence it drifted in scale by a factor of two along the flight; refine_poses_ba.sh needs only the
# features and matches this script always produces)
set -eo pipefail
DS=$1; MAP=0; [ "${2:-}" = "--map" ] && MAP=1
OUT=$DS/sfm; mkdir -p "$OUT/masks" "$OUT/sparse"
PARAMS=$(python3 - "$DS/export.json" <<'PY'
import json, sys
print(",".join("%.6f" % v for v in json.load(open(sys.argv[1]))["params"]))
PY
)
# COLMAP looks for <mask_path>/<image name>.png: for 000239.png that is 000239.png.png
for m in "$DS"/images/*_mask.png; do
  b=$(basename "$m" _mask.png); ln -sfn "$(realpath --relative-to="$OUT/masks" "$m")" "$OUT/masks/$b.png.png"
done
VOCAB=$(dirname "$DS")/../vocab_tree_flickr100K_words32K.bin
[ -f "$VOCAB" ] || curl -sfL --max-time 900 -o "$VOCAB" https://demuc.de/colmap/vocab_tree_flickr100K_words32K.bin || echo "no vocab tree: loop detection off"
LOOP=1; [ -f "$VOCAB" ] || LOOP=0
echo "$(date -u +%T) features"
colmap feature_extractor --database_path "$OUT/database.db" --image_path "$DS/images" \
  --ImageReader.single_camera 1 --ImageReader.camera_model FULL_OPENCV --ImageReader.camera_params "$PARAMS" \
  --ImageReader.mask_path "$OUT/masks" --SiftExtraction.use_gpu 0 --SiftExtraction.num_threads 22 \
  --SiftExtraction.max_image_size 1600 --SiftExtraction.estimate_affine_shape 1 --SiftExtraction.domain_size_pooling 1 \
  > "$OUT/features.log" 2>&1
echo "$(date -u +%T) matching"
colmap sequential_matcher --database_path "$OUT/database.db" --SiftMatching.use_gpu 0 --SiftMatching.num_threads 22 \
  --SequentialMatching.overlap 20 --SequentialMatching.quadratic_overlap 1 \
  --SequentialMatching.loop_detection $LOOP --SequentialMatching.vocab_tree_path "$VOCAB" \
  --SequentialMatching.loop_detection_num_images 30 > "$OUT/matching.log" 2>&1
[ "$MAP" = 1 ] || { echo "$(date -u +%T) done (features and matches in $OUT/database.db)"; exit 0; }
echo "$(date -u +%T) mapping"
colmap mapper --database_path "$OUT/database.db" --image_path "$DS/images" --output_path "$OUT/sparse" \
  --Mapper.ba_refine_focal_length 0 --Mapper.ba_refine_principal_point 0 --Mapper.ba_refine_extra_params 0 \
  --Mapper.num_threads 22 > "$OUT/mapper.log" 2>&1
echo "$(date -u +%T) done"
for m in "$OUT"/sparse/*/; do
  echo "model $m"; colmap model_analyzer --path "$m" 2>&1 | grep -E "Registered|Points|Observations|Mean reprojection|Mean track"
done
