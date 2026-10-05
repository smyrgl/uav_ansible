#!/bin/bash
# Bring a COLMAP SfM model (its own frame and scale) into the metric ENU frame of the frames_to_colmap.py
# export: a similarity transform fitted by colmap model_aligner from the SfM camera centres to the PX4-derived
# camera centres of the same images. Output: <dataset>/sfm_metric/sparse/0 (+ images link) ready for 3DGRUT.
# Usage: align_colmap_to_metric.sh <dataset dir> [sfm model dir, default <dataset>/sfm/sparse/0]
set -eo pipefail
DS=$1; MODEL=${2:-$DS/sfm/sparse/0}; OUT=$DS/sfm_metric; mkdir -p "$OUT/sparse/0"
python3 - "$DS" "$OUT/ref_positions.txt" <<'PY'
import json, os, sys, struct
import numpy as np
ds, out = sys.argv[1], sys.argv[2]
# camera centres of the metric export, per image name, from images.bin (world-to-camera -> centre = -R^T t)
def read_images_bin(p):
    names = {}
    with open(p, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            iid = struct.unpack("<i", f.read(4))[0]; q = struct.unpack("<4d", f.read(32)); t = np.array(struct.unpack("<3d", f.read(24)))
            cid = struct.unpack("<i", f.read(4))[0]; name = b""
            while True:
                c = f.read(1)
                if c == b"\x00": break
                name += c
            npts = struct.unpack("<Q", f.read(8))[0]; f.read(24 * npts)
            w, x, y, z = q
            R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            names[name.decode()] = -R.T @ t
    return names
centres = read_images_bin(os.path.join(ds, "sparse", "0", "images.bin"))
with open(out, "w") as f:
    for name, c in sorted(centres.items()):
        f.write("%s %.6f %.6f %.6f\n" % (name, c[0], c[1], c[2]))
print("%d reference camera positions" % len(centres))
PY
colmap model_aligner --input_path "$MODEL" --output_path "$OUT/sparse/0" --ref_images_path "$OUT/ref_positions.txt" \
  --ref_is_gps 0 --alignment_type custom --alignment_max_error 1.0 --min_common_images 3 2>&1 | grep -v "^$" | tail -5
ln -sfn ../images "$OUT/images"
colmap model_analyzer --path "$OUT/sparse/0" 2>&1 | grep -E "Registered|Points|Mean reprojection"
# residual of the alignment: SfM centres after alignment vs the PX4 centres
python3 - "$OUT" <<'PY'
import os, sys, struct, numpy as np
out = sys.argv[1]
ref = {l.split()[0]: np.array(list(map(float, l.split()[1:4]))) for l in open(os.path.join(out, "ref_positions.txt"))}
def centres(p):
    names = {}
    with open(p, "rb") as f:
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
    return names
al = centres(os.path.join(out, "sparse", "0", "images.bin"))
d = np.array([np.linalg.norm(al[n] - ref[n]) for n in al if n in ref])
print("aligned SfM centres vs PX4 centres over %d images: median %.3f m, p95 %.3f m, max %.3f m" % (len(d), np.median(d), np.percentile(d, 95), d.max()))
PY
