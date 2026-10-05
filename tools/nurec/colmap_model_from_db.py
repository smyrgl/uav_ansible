#!/usr/bin/env python3
"""Write a COLMAP text model whose image ids match a feature database, with poses taken from a
metric model (frames_to_colmap.py output) by image name: the starting point for point_triangulator
and bundle_adjuster, so that odometry poses get refined by the matched features instead of being
re-estimated from scratch. Usage: colmap_model_from_db.py <database.db> <metric sparse dir> <out dir>
"""
import os
import sqlite3
import struct
import sys


def read_images_bin(path):
    out = {}
    with open(path, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            f.read(4); q = struct.unpack("<4d", f.read(32)); t = struct.unpack("<3d", f.read(24)); f.read(4); name = b""
            while True:
                c = f.read(1)
                if c == b"\x00": break
                name += c
            k = struct.unpack("<Q", f.read(8))[0]; f.read(24 * k)
            out[name.decode()] = (q, t)
    return out


def main(argv=None):
    db, metric, out = argv or sys.argv[1:4]
    poses = read_images_bin(os.path.join(metric, "images.bin"))
    con = sqlite3.connect(db)
    cams = con.execute("SELECT camera_id, model, width, height, params FROM cameras").fetchall()
    imgs = con.execute("SELECT image_id, name, camera_id FROM images").fetchall()
    os.makedirs(out, exist_ok=True)
    models = {0: "SIMPLE_PINHOLE", 1: "PINHOLE", 2: "SIMPLE_RADIAL", 3: "RADIAL", 4: "OPENCV", 5: "OPENCV_FISHEYE", 6: "FULL_OPENCV"}
    with open(os.path.join(out, "cameras.txt"), "w") as f:
        for cid, model, w, h, params in cams:
            p = struct.unpack("<%dd" % (len(params) // 8), params)
            f.write("%d %s %d %d %s\n" % (cid, models[model], w, h, " ".join("%.10g" % v for v in p)))
    n = 0
    with open(os.path.join(out, "images.txt"), "w") as f:
        for iid, name, cid in imgs:
            if name not in poses:
                continue
            q, t = poses[name]
            f.write("%d %.12g %.12g %.12g %.12g %.12g %.12g %.12g %d %s\n\n" % (iid, *q, *t, cid, name)); n += 1
    open(os.path.join(out, "points3D.txt"), "w").close()
    print("%d images posed from the metric model, %d cameras, ids as in %s" % (n, len(cams), db))
    return 0


if __name__ == "__main__":
    sys.exit(main())
