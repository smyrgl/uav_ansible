#!/usr/bin/env python3
"""Align a COLMAP model to reference camera positions with a similarity transform (sim(3), Umeyama with
scale, two rounds of inlier rejection) and write the transformed model. Replaces `colmap model_aligner
--alignment_type custom`, which on COLMAP 3.9 returned an unaligned model here (54 m median residual).

Usage: align_model_sim3.py <in sparse dir> <ref_positions.txt: name x y z per line> <out sparse dir> [--max-error-m 1.0]
"""
import argparse
import math
import os
import struct
import sys


def read_images(path):
    out = []
    with open(path, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            iid = struct.unpack("<i", f.read(4))[0]; q = struct.unpack("<4d", f.read(32)); t = struct.unpack("<3d", f.read(24))
            cid = struct.unpack("<i", f.read(4))[0]; name = b""
            while True:
                c = f.read(1)
                if c == b"\x00": break
                name += c
            npts = struct.unpack("<Q", f.read(8))[0]; pts = f.read(24 * npts)
            out.append([iid, list(q), list(t), cid, name.decode(), pts, npts])
    return out


def write_images(path, images):
    with open(path, "wb") as f:
        f.write(struct.pack("<Q", len(images)))
        for iid, q, t, cid, name, pts, npts in images:
            f.write(struct.pack("<i", iid)); f.write(struct.pack("<4d", *q)); f.write(struct.pack("<3d", *t))
            f.write(struct.pack("<i", cid)); f.write(name.encode() + b"\x00"); f.write(struct.pack("<Q", npts)); f.write(pts)


def read_points(path):
    out = []
    with open(path, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            pid = struct.unpack("<Q", f.read(8))[0]; xyz = list(struct.unpack("<3d", f.read(24))); rgb = f.read(3); err = f.read(8)
            tl = struct.unpack("<Q", f.read(8))[0]; track = f.read(8 * tl)
            out.append([pid, xyz, rgb, err, tl, track])
    return out


def write_points(path, points):
    with open(path, "wb") as f:
        f.write(struct.pack("<Q", len(points)))
        for pid, xyz, rgb, err, tl, track in points:
            f.write(struct.pack("<Q", pid)); f.write(struct.pack("<3d", *xyz)); f.write(rgb); f.write(err); f.write(struct.pack("<Q", tl)); f.write(track)


def qvec_to_mat(q):
    import numpy as np
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_qvec(r):
    import numpy as np
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    i = int(np.argmax(np.diag(r))); j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(1e-12, 1.0 + r[i, i] - r[j, j] - r[k, k])) * 2
    q = [0.0, 0.0, 0.0]; q[i] = 0.25 * s; q[j] = (r[j, i] + r[i, j]) / s; q[k] = (r[k, i] + r[i, k]) / s
    return [(r[k, j] - r[j, k]) / s] + q


def umeyama_sim3(src, dst):
    import numpy as np
    ms, md = src.mean(0), dst.mean(0)
    a, b = src - ms, dst - md
    u, s, vt = np.linalg.svd(a.T @ b / len(src))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    scale = float((s * np.array([1.0, 1.0, d])).sum() / (a ** 2).sum() * len(src))
    return scale, r, md - scale * r @ ms


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("inp"); ap.add_argument("ref"); ap.add_argument("out")
    ap.add_argument("--max-error-m", type=float, default=1.0, help="inlier threshold for the second fit")
    args = ap.parse_args(argv)
    import numpy as np
    ref = {}
    for line in open(args.ref):
        p = line.split()
        if len(p) >= 4:
            ref[p[0]] = np.array(list(map(float, p[1:4])))
    images = read_images(os.path.join(args.inp, "images.bin"))
    src, dst = [], []
    for _iid, q, t, _cid, name, _p, _n in images:
        if name in ref:
            src.append(-qvec_to_mat(q).T @ np.array(t)); dst.append(ref[name])
    src, dst = np.array(src), np.array(dst)
    if len(src) < 3:
        sys.exit("fewer than 3 common images")
    scale, r, tr = umeyama_sim3(src, dst)
    res = np.linalg.norm(src @ (scale * r).T + tr - dst, axis=1)
    inl = res < max(args.max_error_m, 2 * np.median(res))
    scale, r, tr = umeyama_sim3(src[inl], dst[inl])
    res = np.linalg.norm(src @ (scale * r).T + tr - dst, axis=1)
    print("sim(3) from %d of %d common images: scale %.4f, residual median %.3f m, p95 %.3f m, max %.3f m" % (
        inl.sum(), len(src), scale, np.median(res), np.percentile(res, 95), res.max()))
    os.makedirs(args.out, exist_ok=True)
    for img in images:
        q, t = img[1], img[2]
        R_cw = qvec_to_mat(q); C = -R_cw.T @ np.array(t)
        C2 = scale * r @ C + tr; R_cw2 = R_cw @ r.T
        img[1] = mat_to_qvec(R_cw2); img[2] = (-R_cw2 @ C2).tolist()
    write_images(os.path.join(args.out, "images.bin"), images)
    pts = read_points(os.path.join(args.inp, "points3D.bin"))
    for p in pts:
        p[1] = (scale * r @ np.array(p[1]) + tr).tolist()
    write_points(os.path.join(args.out, "points3D.bin"), pts)
    with open(os.path.join(args.inp, "cameras.bin"), "rb") as a, open(os.path.join(args.out, "cameras.bin"), "wb") as b:
        b.write(a.read())
    print("wrote %s: %d images, %d points" % (args.out, len(images), len(pts)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
