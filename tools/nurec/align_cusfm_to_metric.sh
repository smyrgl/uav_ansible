#!/bin/bash
# Bring cuSFM's COLMAP export into the metric odometry frame: colmap model_aligner from the SfM camera centres
# onto the camera centres rosbag_to_mapping_data derived from the PX4 odometry (its frames_meta.json
# camera_to_world, written when the bag is also given as --pose_bag_file). Output: <out>/sparse/0 plus an
# images link, ready for 3DGRUT. The reference may be a keyframe subset (a mapping dir converted with
# --pose_bag_file) while the images come from the full conversion cuSFM ran on.
# Usage: align_cusfm_to_metric.sh <reference mapping dir> <images mapping dir> <cusfm colmap model dir> <out dir>
set -eo pipefail
REF=$1; MAP=$2; MODEL=$3; OUT=$4; mkdir -p "$OUT/sparse/0"
python3 - "$REF/frames_meta.json" "$OUT/ref_positions.txt" <<'PY'
import json, sys
j = json.load(open(sys.argv[1]))
n = 0
with open(sys.argv[2], "w") as f:
    for k in j["keyframes_metadata"]:
        t = k.get("camera_to_world", {}).get("translation")
        if t:
            f.write("%s %.6f %.6f %.6f\n" % (k["image_name"], t["x"], t["y"], t["z"])); n += 1
print("%d reference camera positions from the odometry" % n)
PY
colmap model_aligner --input_path "$MODEL" --output_path "$OUT/sparse/0" --ref_images_path "$OUT/ref_positions.txt" \
  --ref_is_gps 0 --alignment_type custom --alignment_max_error 1.0 --min_common_images 3 2>&1 | grep -v "^$" | tail -4
ln -sfn "$(realpath --relative-to="$OUT" "$MAP")" "$OUT/images"
colmap model_analyzer --path "$OUT/sparse/0" 2>&1 | grep -E "Registered|Points|Mean reprojection"
python3 - "$OUT" <<'PY'
import os, sys, struct, numpy as np
out = sys.argv[1]
ref = {l.split()[0]: np.array(list(map(float, l.split()[1:4]))) for l in open(os.path.join(out, "ref_positions.txt"))}
names = {}
with open(os.path.join(out, "sparse", "0", "images.bin"), "rb") as f:
    n = struct.unpack("<Q", f.read(8))[0]
    for _ in range(n):
        f.read(4); q = struct.unpack("<4d", f.read(32)); t = np.array(struct.unpack("<3d", f.read(24))); f.read(4); name = b""
        while True:
            c = f.read(1)
            if c == b"\x00": break
            name += c
        npts = struct.unpack("<Q", f.read(8))[0]; f.read(24 * npts)
        w, x, y, z = q
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        names[name.decode()] = -R.T @ t
d = np.array([np.linalg.norm(names[k] - ref[k]) for k in names if k in ref])
print("aligned cuSFM centres vs odometry centres over %d images: median %.3f m, p95 %.3f m, max %.3f m" % (len(d), np.median(d), np.percentile(d, 95), d.max()))
PY
