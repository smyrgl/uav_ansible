#!/usr/bin/env python3
"""Turn a bag_to_frames.py export into a COLMAP-format dataset for 3DGRUT.

Writes <out>/sparse/0/{cameras.bin,images.bin,points3D.bin} and links the colour
images as <out>/images/<index>.png. Poses are the export's metric camera-to-world
transforms (PX4 ENU odometry through the URDF), inverted into COLMAP's
world-to-camera qvec/tvec. The camera is COLMAP's OPENCV model (fx fy cx cy k1 k2
p1 p2 k3) from camera_info, as COLMAP's FULL_OPENCV. points3D are an initial Gaussian seed: the flight's LIO
map (lio_map.pcd, FAST-LIO camera_init frame) carried into the odometry frame
with an SE(3) fit of /lio/odometry onto /px4/odometry over the bag, subsampled.

Also prints what the camera saw: height, speed and pitch statistics, so a
capture's coverage can be judged before training.
"""
import argparse
import json
import math
import os
import struct
import sys

NS = 1_000_000_000


def rot_to_qvec(r):
    """COLMAP quaternion (w, x, y, z) of a rotation matrix."""
    import numpy as np
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    i = int(np.argmax(np.diag(r))); j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(1e-12, 1.0 + r[i, i] - r[j, j] - r[k, k])) * 2
    q = [0.0, 0.0, 0.0]; q[i] = 0.25 * s; q[j] = (r[j, i] + r[i, j]) / s; q[k] = (r[k, i] + r[i, k]) / s
    return [(r[k, j] - r[j, k]) / s] + q


def write_cameras(path, cams):
    with open(path, "wb") as f:
        f.write(struct.pack("<Q", len(cams)))
        for cid, (model_id, w, h, params) in cams.items():
            f.write(struct.pack("<iiQQ", cid, model_id, w, h))
            f.write(struct.pack("<%dd" % len(params), *params))


def write_images(path, images):
    with open(path, "wb") as f:
        f.write(struct.pack("<Q", len(images)))
        for iid, (qvec, tvec, cid, name) in images.items():
            f.write(struct.pack("<i", iid)); f.write(struct.pack("<4d", *qvec)); f.write(struct.pack("<3d", *tvec))
            f.write(struct.pack("<i", cid)); f.write(name.encode() + b"\x00"); f.write(struct.pack("<Q", 0))


def write_points3d(path, xyz, rgb):
    with open(path, "wb") as f:
        f.write(struct.pack("<Q", len(xyz)))
        for i, (p, c) in enumerate(zip(xyz, rgb)):
            f.write(struct.pack("<Q", i + 1)); f.write(struct.pack("<3d", *p)); f.write(struct.pack("<3B", *c))
            f.write(struct.pack("<d", 0.0)); f.write(struct.pack("<Q", 0))


def read_pcd(path):
    import numpy as np
    with open(path, "rb") as f:
        fields = sizes = types = None; count = 0
        while True:
            line = f.readline()
            key, _, rest = line.decode("ascii", "replace").strip().partition(" ")
            if key == "FIELDS": fields = rest.split()
            elif key == "SIZE": sizes = [int(v) for v in rest.split()]
            elif key == "TYPE": types = rest.split()
            elif key == "POINTS": count = int(rest)
            elif key == "DATA":
                assert rest.strip() == "binary", "binary PCD only"; break
        dtype = np.dtype([(n, {"F": "<f", "I": "<i", "U": "<u"}[t] + str(s)) for n, s, t in zip(fields, sizes, types)])
        d = np.frombuffer(f.read(count * dtype.itemsize), dtype=dtype, count=count)
    xyz = np.stack([d["x"], d["y"], d["z"]], 1).astype(float)
    inten = d["intensity"].astype(float) if "intensity" in fields else None
    return xyz, inten


def lio_to_pose_frame(bag, pose_topic, lio_topic):
    """SE(3) mapping FAST-LIO's camera_init onto the pose topic's frame, from the two odometries in the bag."""
    import numpy as np
    sys.path.insert(0, "/usr/local/lib/uav"); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import estimator_truth as et
    data = et.read_bag(bag, [pose_topic, lio_topic])
    tl, pl, _q, _ = et.odometry_arrays(data[lio_topic]); tp, pp, _q2, _ = et.odometry_arrays(data[pose_topic])
    inside = (tl >= tp[0]) & (tl <= tp[-1])
    ppi = np.stack([np.interp(tl[inside], tp, pp[:, i]) for i in range(3)], -1)
    r, t = et.umeyama(pl[inside], ppi)
    resid = float(np.sqrt(((pl[inside] @ r.T + t - ppi) ** 2).sum(1).mean()))
    m = np.eye(4); m[:3, :3] = r; m[:3, 3] = t
    return m, resid, int(inside.sum())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="\n".join(__doc__.splitlines()[2:]))
    ap.add_argument("frames_dir", help="bag_to_frames.py output")
    ap.add_argument("out")
    ap.add_argument("--camera", default="color")
    ap.add_argument("--bag", default="", help="the bag (for the LIO-to-odometry fit); default: frames.json's bag")
    ap.add_argument("--pcd", default="", help="LIO map PCD; default <bag>/lio_map.pcd")
    ap.add_argument("--max-points", type=int, default=500_000)
    ap.add_argument("--lio-topic", default="/lio/odometry")
    ap.add_argument("--min-height-m", type=float, default=None, help="drop frames below this height in the odometry frame")
    args = ap.parse_args(argv)
    import numpy as np
    fr = json.load(open(os.path.join(args.frames_dir, "frames.json")))
    cams = json.load(open(os.path.join(args.frames_dir, "cameras.json")))["cameras"]
    cam = cams[args.camera]
    k = cam["K"]; d = list(cam["D"]) + [0.0] * 5
    # FULL_OPENCV (COLMAP model 6): fx fy cx cy k1 k2 p1 p2 k3 k4 k5 k6; 3DGRUT reads all six radial terms
    model_id, params = 6, [k[0], k[4], k[2], k[5], d[0], d[1], d[2], d[3], d[4], 0.0, 0.0, 0.0]
    os.makedirs(os.path.join(args.out, "sparse", "0"), exist_ok=True); os.makedirs(os.path.join(args.out, "images"), exist_ok=True)
    images = {}; heights = []; speeds = []; pitches = []; last = None
    for i, e in enumerate(fr["frames"]):
        T_wc = np.array(e["T_world_camera"][args.camera]); T_wb = np.array(e["T_world_base"])
        if args.min_height_m is not None and T_wb[2, 3] < args.min_height_m:
            continue
        T_cw = np.linalg.inv(T_wc)
        name = "%s.png" % e["index"]
        src = os.path.abspath(os.path.join(args.frames_dir, e["files"][args.camera])); dst = os.path.join(args.out, "images", name)
        if not os.path.exists(dst):
            os.symlink(src, dst)
        images[len(images) + 1] = (rot_to_qvec(T_cw[:3, :3]), T_cw[:3, 3].tolist(), 1, name)
        heights.append(T_wb[2, 3])
        look = T_wc[:3, 2]                                   # optical axis in the world (ENU)
        pitches.append(math.degrees(math.asin(max(-1.0, min(1.0, -look[2])))))   # positive = looking down
        if last is not None:
            dt = (e["stamp_utc_ns"] - last[0]) / NS
            if dt > 0:
                speeds.append(float(np.linalg.norm(T_wb[:3, 3] - last[1])) / dt)
        last = (e["stamp_utc_ns"], T_wb[:3, 3])
    write_cameras(os.path.join(args.out, "sparse", "0", "cameras.bin"), {1: (model_id, cam["width"], cam["height"], params)})
    write_images(os.path.join(args.out, "sparse", "0", "images.bin"), images)
    print("%d images; height %.1f..%.1f m (median %.1f); speed median %.1f max %.1f m/s; camera pitch down median %.1f deg (%.0f%% of frames looking down more than 20 deg)" % (
        len(images), min(heights), max(heights), float(np.median(heights)), float(np.median(speeds)) if speeds else 0, max(speeds) if speeds else 0,
        float(np.median(pitches)), 100 * float(np.mean(np.array(pitches) > 20))))
    bag = args.bag or fr["meta"]["bag"]; pcd = args.pcd or os.path.join(bag, "lio_map.pcd")
    if os.path.isfile(pcd):
        m, resid, n = lio_to_pose_frame(bag, fr["meta"]["pose_topic"], args.lio_topic)
        xyz, inten = read_pcd(pcd)
        xyz = xyz @ m[:3, :3].T + m[:3, 3]
        if len(xyz) > args.max_points:
            sel = np.random.default_rng(0).choice(len(xyz), args.max_points, replace=False); xyz = xyz[sel]; inten = None if inten is None else inten[sel]
        grey = np.full(len(xyz), 128, np.uint8) if inten is None else np.clip(inten / max(1.0, float(np.percentile(inten, 99))) * 255, 0, 255).astype(np.uint8)
        write_points3d(os.path.join(args.out, "sparse", "0", "points3D.bin"), xyz, np.stack([grey] * 3, 1))
        print("points3D: %d LIO map points carried camera_init -> %s (fit over %d poses, residual %.3f m rms)" % (len(xyz), fr["meta"]["world_frame"], n, resid))
    else:
        write_points3d(os.path.join(args.out, "sparse", "0", "points3D.bin"), np.zeros((0, 3)), np.zeros((0, 3), np.uint8))
        print("no LIO map at %s: empty points3D (3DGRUT will seed randomly)" % pcd)
    json.dump({"frames_dir": args.frames_dir, "camera": args.camera, "images": len(images), "camera_model": "FULL_OPENCV", "params": params},
              open(os.path.join(args.out, "export.json"), "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
