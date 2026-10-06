#!/bin/bash
# Refine the odometry camera poses with the matched features: triangulate with the metric poses held as
# the start, bundle-adjust with intrinsics fixed, then re-align the result to the odometry centres with a
# sim(3) so the bundle adjustment's gauge freedom cannot move the scene. Needs the feature database of
# run_colmap_sfm.sh. Output: <dataset>/ba/sparse/0 (+ images link), ready for 3DGRUT.
# Usage: refine_poses_ba.sh <dataset dir>
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
DS=$1; DB=$DS/sfm/database.db; OUT=$DS/ba; mkdir -p "$OUT/init" "$OUT/tri" "$OUT/adj" "$OUT/sparse/0"
python3 "$HERE/colmap_model_from_db.py" "$DB" "$DS/sparse/0" "$OUT/init"
echo "$(date -u +%T) triangulating with the odometry poses"
colmap point_triangulator --database_path "$DB" --image_path "$DS/images" --input_path "$OUT/init" --output_path "$OUT/tri" \
  --Mapper.ba_refine_focal_length 0 --Mapper.ba_refine_principal_point 0 --Mapper.ba_refine_extra_params 0 > "$OUT/triangulate.log" 2>&1
colmap model_analyzer --path "$OUT/tri" 2>&1 | grep -E "Registered|Points|Mean reprojection"
echo "$(date -u +%T) bundle adjustment, intrinsics fixed"
colmap bundle_adjuster --input_path "$OUT/tri" --output_path "$OUT/adj" \
  --BundleAdjustment.refine_focal_length 0 --BundleAdjustment.refine_principal_point 0 --BundleAdjustment.refine_extra_params 0 \
  --BundleAdjustment.max_num_iterations 100 > "$OUT/ba.log" 2>&1
colmap model_analyzer --path "$OUT/adj" 2>&1 | grep -E "Registered|Points|Mean reprojection"
# the adjusted model's images.bin may be text or binary depending on the input; normalise to binary
[ -f "$OUT/adj/images.bin" ] || colmap model_converter --input_path "$OUT/adj" --output_path "$OUT/adj" --output_type BIN > /dev/null 2>&1
python3 - "$DS/sparse/0/images.bin" "$OUT/ref_positions.txt" <<'PY'
import struct, sys, numpy as np
with open(sys.argv[1], "rb") as f, open(sys.argv[2], "w") as o:
    n = struct.unpack("<Q", f.read(8))[0]
    for _ in range(n):
        f.read(4); q = struct.unpack("<4d", f.read(32)); t = np.array(struct.unpack("<3d", f.read(24))); f.read(4); nm = b""
        while True:
            c = f.read(1)
            if c == b"\x00": break
            nm += c
        k = struct.unpack("<Q", f.read(8))[0]; f.read(24 * k)
        w, x, y, z = q
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        C = -R.T @ t; o.write("%s %.6f %.6f %.6f\n" % (nm.decode(), *C))
PY
python3 "$HERE/align_model_sim3.py" "$OUT/adj" "$OUT/ref_positions.txt" "$OUT/sparse/0"
ln -sfn ../images "$OUT/images"
# masks beside the images for 3DGRUT
for m in "$DS"/images/*_mask.png; do [ -e "$m" ] || continue; b=$(basename "$m"); ln -sfn "../../images/$b" "$OUT/images/$b" 2>/dev/null || true; done
echo "$(date -u +%T) done: $OUT/sparse/0"
