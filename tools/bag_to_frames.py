#!/usr/bin/env python3
"""Export camera frames with metric camera-to-world poses from a flight bag.

For every selected colour frame (every --every-th message on --color-topic) the
tool writes a PNG (YUY2 decoded to RGB), finds the IR pair recorded within
--pair-tolerance-ms of it, and interpolates the aircraft pose on --pose-topic
(PX4 ENU odometry, px4_local -> base_link) at the frame's capture stamp:
position linearly, attitude by slerp. Camera extrinsics come from /tf_static
in the bag (the URDF), composed from each camera's optical frame up to the
odometry's child frame. Output (one folder per bag):

  images/<camera>/<index>.png                 colour RGB, infra1/infra2 grey
  frames.json                                 per frame: stamp_utc_ns, files, T_world_camera (4x4,
                                              OpenCV optical convention: x right, y down, z forward)
                                              for every camera, aircraft pose, odometry age
  cameras.json                                per camera: width, height, K, distortion, T_base_camera

The output is the common input for a COLMAP model, an NCore dataset or a
cuSFM frames_meta.json; it is not tied to any of them. The bag is read once.
Needs: mcap, mcap-ros2-support, numpy, pillow (the replay host's scoring venv).
"""
import argparse
import glob
import json
import math
import os
import sys

NS = 1_000_000_000


def quat_to_mat(q):
    import numpy as np
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(m):
    import numpy as np
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s; q[j] = (m[j, i] + m[i, j]) / s; q[k] = (m[k, i] + m[i, k]) / s; q[3] = (m[k, j] - m[j, k]) / s
    return np.array(q)


def transform(t, q):
    import numpy as np
    m = np.eye(4)
    m[:3, :3] = quat_to_mat(q)
    m[:3, 3] = t
    return m


def slerp(q0, q1, f):
    import numpy as np
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + f * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(min(1.0, d))
    return (math.sin((1 - f) * th) * q0 + math.sin(f * th) * q1) / math.sin(th)


def yuy2_to_rgb(buf, width, height):
    """YUY2 (Y0 U Y1 V, BT.601 limited range) to RGB uint8."""
    import numpy as np
    a = np.frombuffer(buf, dtype=np.uint8).reshape(height, width // 2, 4).astype(np.float32)
    y = np.empty((height, width), np.float32)
    y[:, 0::2] = a[:, :, 0]; y[:, 1::2] = a[:, :, 2]
    u = np.repeat(a[:, :, 1], 2, axis=1) - 128.0
    v = np.repeat(a[:, :, 3], 2, axis=1) - 128.0
    c = 1.164 * (y - 16.0)
    rgb = np.stack([c + 1.596 * v, c - 0.392 * u - 0.813 * v, c + 2.017 * u], -1)
    return np.clip(rgb, 0, 255).astype(np.uint8)


def stamp_ns(header):
    return header.stamp.sec * NS + header.stamp.nanosec


class StaticTf:
    def __init__(self):
        self.parent = {}      # child -> (parent, 4x4 parent_T_child)

    def add(self, tr):
        self.parent[tr.child_frame_id] = (tr.header.frame_id, transform(
            [tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z],
            [tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w]))

    def chain(self, frame, root):
        """root_T_frame through the static tree, or None."""
        import numpy as np
        m = np.eye(4); f = frame; hops = 0
        while f != root:
            if f not in self.parent or hops > 32:
                return None
            p, t = self.parent[f]
            m = t @ m; f = p; hops += 1
        return m


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="\n".join(__doc__.splitlines()[2:]))
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--every", type=int, default=10, help="keep every N-th colour frame")
    ap.add_argument("--color-topic", default="/d555/color/image")
    ap.add_argument("--ir-topics", default="/d555/infra1/image,/d555/infra2/image")
    ap.add_argument("--info-suffix", default="camera_info", help="camera_info topic beside each image topic")
    ap.add_argument("--pose-topic", default="/px4/odometry")
    ap.add_argument("--pair-tolerance-ms", type=float, default=20.0)
    ap.add_argument("--pose-max-age-ms", type=float, default=50.0, help="skip frames with no odometry sample this close")
    ap.add_argument("--start-s", type=float, default=0.0, help="skip this much from the bag start")
    ap.add_argument("--min-height-m", type=float, default=None, help="keep frames above this height in the pose frame")
    args = ap.parse_args(argv)

    import numpy as np
    from PIL import Image
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    ir_topics = [t for t in args.ir_topics.split(",") if t]
    image_topics = [args.color_topic] + ir_topics
    info_topics = [t.rsplit("/", 1)[0] + "/" + args.info_suffix for t in image_topics]
    files = sorted(glob.glob(os.path.join(args.bag, "*.mcap"))) if os.path.isdir(args.bag) else [args.bag]
    os.makedirs(args.out, exist_ok=True)
    cam_name = {args.color_topic: "color"}
    for i, t in enumerate(ir_topics):
        cam_name[t] = "infra%d" % (i + 1)
    for n in cam_name.values():
        os.makedirs(os.path.join(args.out, "images", n), exist_ok=True)

    # pass 1: poses, static tf, camera infos (small messages)
    tf = StaticTf(); infos = {}; odom = []
    for path in files:
        with open(path, "rb") as f:
            r = make_reader(f, decoder_factories=[DecoderFactory()])
            for _s, ch, _m, ros in r.iter_decoded_messages(topics=["/tf_static", args.pose_topic] + info_topics):
                if ch.topic == "/tf_static":
                    for tr in ros.transforms:
                        tf.add(tr)
                elif ch.topic == args.pose_topic:
                    p, q = ros.pose.pose.position, ros.pose.pose.orientation
                    odom.append((stamp_ns(ros.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w, ros.header.frame_id, ros.child_frame_id))
                elif ch.topic not in infos:
                    infos[ch.topic] = ros
    if len(odom) < 2:
        sys.exit("no poses on %s" % args.pose_topic)
    odom.sort(key=lambda r: r[0])
    ot = np.array([r[0] for r in odom], dtype=np.int64)
    opos = np.array([r[1:4] for r in odom]); oq = np.array([r[4:8] for r in odom])
    world_frame, body_frame = odom[0][8], odom[0][9]
    cameras = {}
    for img_topic, info_topic in zip(image_topics, info_topics):
        info = infos.get(info_topic)
        if info is None:
            sys.exit("no camera_info on %s" % info_topic)
        base_T_cam = tf.chain(info.header.frame_id, body_frame)
        if base_T_cam is None:
            sys.exit("no static tf chain from %s to %s" % (info.header.frame_id, body_frame))
        cameras[cam_name[img_topic]] = {"topic": img_topic, "frame_id": info.header.frame_id, "width": info.width, "height": info.height,
                                        "K": [float(v) for v in info.k], "distortion_model": info.distortion_model,
                                        "D": [float(v) for v in info.d], "T_base_camera": base_T_cam.tolist()}
    meta = {"bag": args.bag, "pose_topic": args.pose_topic, "world_frame": world_frame, "body_frame": body_frame,
            "convention": "T_world_camera maps camera-optical coordinates (x right, y down, z forward) to the pose topic's "
                          "world frame (ENU for /px4/odometry); poses interpolated at the frame capture stamp (UTC)"}
    with open(os.path.join(args.out, "cameras.json"), "w") as f:
        json.dump({"meta": meta, "cameras": cameras}, f, indent=1)

    def pose_at(t):
        i = int(np.searchsorted(ot, t))
        lo, hi = max(0, i - 1), min(len(ot) - 1, i)
        age = min(abs(int(ot[lo]) - t), abs(int(ot[hi]) - t))
        if age > args.pose_max_age_ms * 1e6:
            return None, age
        if hi == lo or ot[hi] == ot[lo]:
            return transform(opos[lo], oq[lo]), age
        f = (t - int(ot[lo])) / float(int(ot[hi]) - int(ot[lo]))
        return transform(opos[lo] + f * (opos[hi] - opos[lo]), slerp(oq[lo], oq[hi], f)), age

    # pass 2: images. The colour stream selects frames; IR messages are buffered briefly for pairing.
    frames = []; n_color = 0; kept = 0; ir_buf = {t: [] for t in ir_topics}; stats = {"no_pose": 0, "no_pair": 0, "below_height": 0}
    t_begin = None
    for path in files:
        with open(path, "rb") as f:
            r = make_reader(f, decoder_factories=[DecoderFactory()])
            pending = []
            for _s, ch, _m, ros in r.iter_decoded_messages(topics=image_topics):
                t = stamp_ns(ros.header)
                if t_begin is None:
                    t_begin = t
                if ch.topic in ir_buf:
                    ir_buf[ch.topic].append((t, ros))
                    ir_buf[ch.topic] = [x for x in ir_buf[ch.topic] if t - x[0] < 2 * NS]
                    continue
                n_color += 1
                if (n_color - 1) % args.every or t - t_begin < args.start_s * NS:
                    continue
                pending.append((t, ros))
                # pair once the IR streams have caught up past this stamp (they arrive interleaved)
                while pending and all(ir_buf[k] and ir_buf[k][-1][0] > pending[0][0] + int(args.pair_tolerance_ms * 1e6) for k in ir_buf):
                    tc, msg = pending.pop(0)
                    T_wb, age = pose_at(tc)
                    if T_wb is None:
                        stats["no_pose"] += 1; continue
                    if args.min_height_m is not None and T_wb[2, 3] < args.min_height_m:
                        stats["below_height"] += 1; continue
                    pair = {}
                    for k in ir_buf:
                        best = min(ir_buf[k], key=lambda x: abs(x[0] - tc))
                        if abs(best[0] - tc) <= args.pair_tolerance_ms * 1e6:
                            pair[k] = best
                    if len(pair) != len(ir_buf):
                        stats["no_pair"] += 1
                    idx = "%06d" % kept; kept += 1
                    entry = {"index": idx, "stamp_utc_ns": tc, "pose_age_ms": round(age / 1e6, 2), "files": {}, "T_world_camera": {},
                             "T_world_base": T_wb.tolist()}
                    rgb = yuy2_to_rgb(bytes(msg.data), msg.width, msg.height) if msg.encoding == "yuv422_yuy2" else \
                        np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, -1)
                    rel = os.path.join("images", "color", idx + ".png")
                    Image.fromarray(rgb).save(os.path.join(args.out, rel), compress_level=1)
                    entry["files"]["color"] = rel
                    entry["T_world_camera"]["color"] = (T_wb @ np.array(cameras["color"]["T_base_camera"])).tolist()
                    for k, (ti, im) in pair.items():
                        name = cam_name[k]
                        grey = np.frombuffer(bytes(im.data), np.uint8).reshape(im.height, im.width)
                        rel = os.path.join("images", name, idx + ".png")
                        Image.fromarray(grey).save(os.path.join(args.out, rel), compress_level=1)
                        T_wb_ir, _ = pose_at(ti)
                        entry["files"][name] = rel
                        entry["stamp_%s_ns" % name] = ti
                        entry["T_world_camera"][name] = ((T_wb_ir if T_wb_ir is not None else T_wb) @ np.array(cameras[name]["T_base_camera"])).tolist()
                    frames.append(entry)
                    if kept % 50 == 0:
                        print("kept %d frames (%d colour seen)" % (kept, n_color)); sys.stdout.flush()
    with open(os.path.join(args.out, "frames.json"), "w") as f:
        json.dump({"meta": meta, "every": args.every, "frames": frames, "stats": stats}, f)
    print("done: %d colour frames seen, %d written to %s; %s" % (n_color, kept, args.out, stats))
    return 0


if __name__ == "__main__":
    sys.exit(main())
