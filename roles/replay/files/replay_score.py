#!/usr/bin/env python3
"""Score a replayed flight against the live run recorded in the same bag.

APE: every replayed /lio/odometry pose against the live pose with the nearest
header stamp (within --window-ms); both stamp with the LiDAR frame's time, so no
alignment is needed, the frames are the same camera_init. Map: the voxel sets of
the live PCD (postflight's lio_map.pcd) and the replayed one, Jaccard at the
map's fine voxel and at 0.2 m. Exit 0 when both gates pass, 3 otherwise.
"""
import argparse
import bisect
import json
import math
import os
import sys


def read_odometry(bag_dir, topic):
    """[(stamp_ns, x, y, z)] of nav_msgs/Odometry on `topic`, sorted by stamp."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        name, data, _log_time = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        rows.append((msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec, p.x, p.y, p.z))
    rows.sort()
    return rows


def match(live, replay, window_ns):
    """Pairs (replay xyz, live xyz) matched by header stamp; the unmatched count."""
    stamps = [r[0] for r in live]
    pairs, unmatched = [], 0
    for t, x, y, z in replay:
        i = bisect.bisect_left(stamps, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(live) and abs(live[j][0] - t) <= window_ns:
                if best is None or abs(live[j][0] - t) < abs(live[best][0] - t):
                    best = j
        if best is None:
            unmatched += 1
        else:
            pairs.append(((x, y, z), live[best][1:]))
    return pairs, unmatched


def umeyama(src, dst):
    """SE(3) (R, t) that maps src onto dst in the least-squares sense (no scale):
    the alignment between the replay's camera_init (origin at the bag's first
    scan) and the live one (origin at FAST-LIO's start, earlier)."""
    import numpy as np
    a, b = np.asarray(src, dtype=float), np.asarray(dst, dtype=float)
    ma, mb = a.mean(axis=0), b.mean(axis=0)
    h = (a - ma).T @ (b - mb)
    u, _s, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return r, mb - r @ ma


def ape(live, replay, window_ns):
    """Position errors of replayed poses against live ones, after one SE(3)
    alignment of the whole trajectory (the two runs have different map origins)."""
    import numpy as np
    pairs, unmatched = match(live, replay, window_ns)
    if len(pairs) < 3:
        return {"matched": len(pairs), "unmatched": unmatched}, None
    src = np.array([p[0] for p in pairs]); dst = np.array([p[1] for p in pairs])
    r, t = umeyama(src, dst)
    aligned = src @ r.T + t
    errors = np.sort(np.linalg.norm(aligned - dst, axis=1))
    def pct(p):
        return float(errors[min(len(errors) - 1, int(p * len(errors)))])
    raw = float(np.median(np.linalg.norm(src - dst, axis=1)))
    return {"matched": len(pairs), "unmatched": unmatched, "median_m": pct(0.5), "p95_m": pct(0.95),
            "max_m": float(errors[-1]), "rms_m": float(math.sqrt(float((errors ** 2).mean()))),
            "origin_offset_m": raw, "alignment_translation_m": [float(v) for v in t]}, (r, t)


def read_pcd(path):
    """(n, 3) float32 xyz of a binary PCD with float32 x y z [intensity]."""
    import numpy as np
    with open(path, "rb") as f:
        fields, sizes, types, count = None, None, None, None
        while True:
            line = f.readline()
            if not line:
                raise ValueError("no DATA line in %s" % path)
            key, _, rest = line.decode("ascii", "replace").strip().partition(" ")
            if key == "FIELDS": fields = rest.split()
            elif key == "SIZE": sizes = [int(v) for v in rest.split()]
            elif key == "TYPE": types = rest.split()
            elif key == "POINTS": count = int(rest)
            elif key == "DATA":
                if rest.strip() != "binary":
                    raise ValueError("only binary PCD is supported")
                break
        dtype = np.dtype([(n, {"F": "<f", "I": "<i", "U": "<u"}[t] + str(s)) for n, s, t in zip(fields, sizes, types)])
        data = np.frombuffer(f.read(count * dtype.itemsize), dtype=dtype, count=count)
    return np.stack([data["x"], data["y"], data["z"]], axis=1).astype("<f4")


def jaccard(a, b, voxel):
    """Voxel-set overlap of the live map (a) and the replayed map (b): Jaccard
    (meaningful when both maps cover the same interval, i.e. the live map was
    reset when the bag started) and recall, the fraction of replayed voxels the
    live map also holds (meaningful even when the live map is older and larger)."""
    import numpy as np
    def keys(points):
        ijk = np.floor(points / voxel).astype("<i8")
        return set(map(tuple, ijk.tolist()))
    ka, kb = keys(a), keys(b)
    union, inter = len(ka | kb), len(ka & kb)
    return {"voxel_m": voxel, "live_voxels": len(ka), "replay_voxels": len(kb),
            "jaccard": (inter / union) if union else None, "recall": (inter / len(kb)) if kb else None}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--live", required=True); ap.add_argument("--replay", required=True)
    ap.add_argument("--live-map", default=""); ap.add_argument("--replay-map", default="")
    ap.add_argument("--odometry-topic", default="/lio/odometry")
    ap.add_argument("--window-ms", type=float, default=20.0)
    ap.add_argument("--ape-median-m", type=float, default=0.02)
    ap.add_argument("--jaccard-min", type=float, default=0.98)
    ap.add_argument("--voxel", type=float, default=0.05)
    ap.add_argument("--out", default="")
    ap.add_argument("--recall-min", type=float, default=0.0, help="gate on recall instead of Jaccard when > 0")
    ap.add_argument("--ulog-summary", default="", help="ulog2mcap summary JSON: adds the ulog-to-bag alignment gate")
    ap.add_argument("--ulog-align-median-ms", type=float, default=2.0)
    args = ap.parse_args(argv)
    live = read_odometry(args.live, args.odometry_topic)
    replay = read_odometry(args.replay, args.odometry_topic)
    stats, transform = ape(live, replay, args.window_ms * 1e6)
    result = {"live_poses": len(live), "replay_poses": len(replay), "ape": stats, "maps": [], "reasons": []}
    if args.live_map and args.replay_map and os.path.isfile(args.live_map) and os.path.isfile(args.replay_map):
        import numpy as np
        a, b = read_pcd(args.live_map), read_pcd(args.replay_map)
        if transform is not None:
            r, t = transform
            b = (b.astype(float) @ r.T + t).astype("<f4")      # the replayed map into the live frame
        result["maps"] = [jaccard(a, b, args.voxel), jaccard(a, b, 0.2)]
    else:
        result["reasons"].append("map comparison skipped (missing PCD)")
    a = result["ape"]
    if not a.get("matched"):
        result["reasons"].append("no replayed poses matched a live pose within %.0f ms" % args.window_ms)
    elif a["median_m"] > args.ape_median_m:
        result["reasons"].append("APE median %.3f m over %.3f m" % (a["median_m"], args.ape_median_m))
    if result["maps"]:
        fine = result["maps"][0]
        if args.recall_min > 0:
            if (fine["recall"] or 0) < args.recall_min:
                result["reasons"].append("map recall %.3f under %.2f at %.2f m voxels" % (fine["recall"] or 0, args.recall_min, args.voxel))
        elif (fine["jaccard"] or 0) < args.jaccard_min:
            result["reasons"].append("map Jaccard %.3f under %.2f at %.2f m voxels" % (fine["jaccard"] or 0, args.jaccard_min, args.voxel))
    if args.ulog_summary and os.path.isfile(args.ulog_summary):
        with open(args.ulog_summary) as f:
            ulog = json.load(f)
        pps = (ulog.get("cross_checks") or {}).get("pps_capture") or {}
        result["ulog"] = {"offset_source": ulog.get("offset_source"), "pps": pps,
                          "gps": {k: v for k, v in (ulog.get("cross_checks") or {}).items() if k.startswith("vehicle_gps")}}
        if not ulog.get("offset_samples"):
            result["reasons"].append("ulog has no timesync offset: no UTC alignment")
        elif not pps.get("count"):
            result["reasons"].append("ulog has no PPS captures to check the alignment")
        elif pps["median_us"] > args.ulog_align_median_ms * 1000:
            result["reasons"].append("ulog-to-bag alignment %.2f ms median over %.1f ms" % (pps["median_us"] / 1000, args.ulog_align_median_ms))
    result["pass"] = not result["reasons"]
    if args.out:
        with open(args.out, "w") as f:
            json.dump(result, f, indent=2)
    print("poses live %d replay %d; APE matched %d median %s m p95 %s m max %s m" % (
        len(live), len(replay), a.get("matched", 0), a.get("median_m"), a.get("p95_m"), a.get("max_m")))
    if a.get("origin_offset_m") is not None:
        print("map origins %.2f m apart before alignment" % a["origin_offset_m"])
    for m in result["maps"]:
        print("map at %.2f m voxels: Jaccard %s, recall %s (live %d / replay %d voxels)" % (
            m["voxel_m"], m["jaccard"], m["recall"], m["live_voxels"], m["replay_voxels"]))
    if "ulog" in result:
        pps = result["ulog"]["pps"]
        print("ulog alignment: %s; PPS residual median %s us over %s edges" % (result["ulog"]["offset_source"], pps.get("median_us"), pps.get("count")))
    print("REPLAY " + ("PASS" if result["pass"] else "FAIL: " + "; ".join(result["reasons"])))
    return 0 if result["pass"] else 3


if __name__ == "__main__":
    sys.exit(main())
