#!/usr/bin/env python3
"""Carry cuSFM-refined IR-left poses onto the colour camera and write a COLMAP dataset for 3DGRUT.

Inputs: an aligned cuSFM COLMAP model (metric, images named d555_ir_left/<stamp ns>.jpg), the
bag_to_frames.py export (colour frames with stamps; cameras.json with the URDF extrinsics of both
cameras). For every colour frame inside the refined trajectory's time span the IR-left pose is
interpolated at the colour stamp (translation linear, rotation slerp) and composed with the rig's
IR-left-to-colour transform; the model's points3D are kept as the seed. Output: <out>/sparse/0 and
<out>/images -> the colour frames.
"""
import argparse
import json
import math
import os
import struct
import sys

NS = 1_000_000_000


def quat_to_mat(w, x, y, z):
    import numpy as np
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


def slerp(q0, q1, f):
    import numpy as np
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + f * (q1 - q0); return q / np.linalg.norm(q)
    th = math.acos(min(1.0, d))
    return (math.sin((1 - f) * th) * q0 + math.sin(f * th) * q1) / math.sin(th)


def read_images_bin(path):
    """{name: (qvec wxyz world-to-camera, tvec)}"""
    out = {}
    with open(path, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            f.read(4); q = struct.unpack("<4d", f.read(32)); t = struct.unpack("<3d", f.read(24)); f.read(4); name = b""
            while True:
                c = f.read(1)
                if c == b"\x00": break
                name += c
            npts = struct.unpack("<Q", f.read(8))[0]; f.read(24 * npts)
            out[name.decode()] = (q, t)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("model", help="aligned cuSFM model dir (sparse/0 with images.bin, points3D.bin)")
    ap.add_argument("frames_dir", help="bag_to_frames.py export")
    ap.add_argument("out")
    ap.add_argument("--ir-camera", default="infra1")
    ap.add_argument("--max-gap-s", type=float, default=1.0, help="skip colour frames between keyframes further apart than this")
    args = ap.parse_args(argv)
    import numpy as np
    cams = json.load(open(os.path.join(args.frames_dir, "cameras.json")))["cameras"]
    fr = json.load(open(os.path.join(args.frames_dir, "frames.json")))["frames"]
    T_b_ir = np.array(cams[args.ir_camera]["T_base_camera"]); T_b_c = np.array(cams["color"]["T_base_camera"])
    T_ir_c = np.linalg.inv(T_b_ir) @ T_b_c
    imgs = read_images_bin(os.path.join(args.model, "images.bin"))
    traj = []
    for name, (q, t) in imgs.items():
        stamp = int(os.path.splitext(os.path.basename(name))[0])
        R = quat_to_mat(*q); C = -R.T @ np.array(t)           # camera-to-world: rotation R^T, centre C
        traj.append((stamp, C, np.array([q[1], q[2], q[3], q[0]])))   # xyzw of world-to-camera for slerp
    traj.sort(key=lambda r: r[0])
    ts = np.array([r[0] for r in traj], dtype=np.int64)
    os.makedirs(os.path.join(args.out, "sparse", "0"), exist_ok=True); os.makedirs(os.path.join(args.out, "images"), exist_ok=True)
    cam = cams["color"]; k = cam["K"]; d = list(cam["D"]) + [0.0] * 5
    params = [k[0], k[4], k[2], k[5], d[0], d[1], d[2], d[3], d[4], 0.0, 0.0, 0.0]
    with open(os.path.join(args.out, "sparse", "0", "cameras.bin"), "wb") as f:
        f.write(struct.pack("<Q", 1)); f.write(struct.pack("<iiQQ", 1, 6, cam["width"], cam["height"])); f.write(struct.pack("<12d", *params))
    written = 0; skipped = 0
    with open(os.path.join(args.out, "sparse", "0", "images.bin"), "wb") as f:
        entries = []
        for e in fr:
            t = e["stamp_utc_ns"]
            i = int(np.searchsorted(ts, t))
            if i <= 0 or i >= len(ts) or ts[i] - ts[i - 1] > args.max_gap_s * NS:
                skipped += 1; continue
            a, b = traj[i - 1], traj[i]; fct = (t - ts[i - 1]) / float(ts[i] - ts[i - 1])
            C = a[1] + fct * (b[1] - a[1])
            qwc = slerp(a[2], b[2], fct)                        # world-to-camera rotation (xyzw)
            Rwc = quat_to_mat(qwc[3], qwc[0], qwc[1], qwc[2])
            T_w_ir = np.eye(4); T_w_ir[:3, :3] = Rwc.T; T_w_ir[:3, 3] = C
            T_w_c = T_w_ir @ T_ir_c; T_c_w = np.linalg.inv(T_w_c)
            name = "%s.png" % e["index"]
            src = os.path.abspath(os.path.join(args.frames_dir, e["files"]["color"])); dst = os.path.join(args.out, "images", name)
            if not os.path.lexists(dst):
                os.symlink(os.path.relpath(src, os.path.dirname(os.path.abspath(dst))), dst)
            entries.append((mat_to_qvec(T_c_w[:3, :3]), T_c_w[:3, 3].tolist(), name))
        f.write(struct.pack("<Q", len(entries)))
        for iid, (qv, tv, name) in enumerate(entries, 1):
            f.write(struct.pack("<i", iid)); f.write(struct.pack("<4d", *qv)); f.write(struct.pack("<3d", *tv)); f.write(struct.pack("<i", 1))
            f.write(name.encode() + b"\x00"); f.write(struct.pack("<Q", 0))
        written = len(entries)
    src = os.path.join(args.model, "points3D.bin")
    if os.path.exists(src):
        with open(src, "rb") as a, open(os.path.join(args.out, "sparse", "0", "points3D.bin"), "wb") as b:
            b.write(a.read())
    print("%d colour frames posed from %d refined IR keyframes (%d skipped outside or across gaps); rig offset IR->colour %s m" % (
        written, len(traj), skipped, [round(float(v), 4) for v in T_ir_c[:3, 3]]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
