#!/usr/bin/env python3
"""Stage 1 estimator truth: odometry against the RTK + dual-antenna heading reference.

Scores every odometry source recorded in a flight bag (FAST-LIO `/lio/odometry`,
PX4 `/px4/odometry`, cuVSLAM `/vslam/odometry` when present) against the
receiver's own solution recorded in the same bag: `/gnss/pvtgeodetic` (RTK
position of the main antenna, 50 Hz, with the solution mode) and
`/gnss/atteuler` (dual-antenna heading). Everything is on the one UTC time base,
so association is by stamp.

Geometry. The receiver positions the main antenna, the estimators position
`base_link`; each estimator pose is carried to the antenna with the URDF lever
arm before comparing (so the comparison is exact under any attitude). The
reference heading is used as the receiver reports it: the owner configures the
antenna-to-vehicle offset in the receiver, this tool applies none unless told.

Alignment. The estimator worlds (FAST-LIO's camera_init, PX4's local origin)
and local ENU are all gravity-aligned, so the frames differ by a yaw and a
translation: a 4-DoF least-squares alignment, well conditioned even on a
straight line, is the model (the full SE(3) Umeyama is reported alongside as a
tilt check when the motion allows). APE is the position error after that
alignment; RPE(1 s) compares 1 s displacement vectors; yaw is compared sample by
sample after the alignment yaw (offset, spread, drift).

Gates (autonomy roadmap, Stage 1): FAST-LIO APE RMS < 0.5 m, RPE(1 s) < 0.15 m
and < 1 deg on RTK-fixed segments. Exit 0 pass, 3 fail, 2 when the bag holds no
usable reference (no RTK-fixed samples, or too little motion to align).
"""

import argparse
import json
import math
import os
import sys

DNU = -1e10                 # Septentrio "do not use": -2e10 in every float field
RTK_FIXED, RTK_FLOAT = 4, 5  # PVTGeodetic mode, low nibble
MOVING_BASE_FIXED, MOVING_BASE_FLOAT = 7, 8
A_WGS84, F_WGS84 = 6378137.0, 1 / 298.257223563
NS = 1_000_000_000
# gnss_main_link in base_link from x950_description (mount at (-0.334278, 0.336696, 0.0925), antenna 0.03048 above)
DEFAULT_LEVER_ARM = (-0.334278, 0.336696, 0.12298)


def wrap(angle):
    import numpy as np
    return (np.asarray(angle, dtype=float) + math.pi) % (2 * math.pi) - math.pi


def geodetic_to_enu(lat, lon, h, lat0, lon0, h0):
    """WGS84 geodetic (radians, metres) -> local ENU metres about (lat0, lon0, h0)."""
    import numpy as np
    e2 = F_WGS84 * (2 - F_WGS84)
    def ecef(la, lo, hh):
        n = A_WGS84 / np.sqrt(1 - e2 * np.sin(la) ** 2)
        return np.stack([(n + hh) * np.cos(la) * np.cos(lo), (n + hh) * np.cos(la) * np.sin(lo),
                         (n * (1 - e2) + hh) * np.sin(la)], axis=-1)
    d = ecef(np.asarray(lat, float), np.asarray(lon, float), np.asarray(h, float)) - ecef(lat0, lon0, h0)
    sl, cl, sp, cp = math.sin(lon0), math.cos(lon0), math.sin(lat0), math.cos(lat0)
    r = np.array([[-sl, cl, 0.0], [-sp * cl, -sp * sl, cp], [cp * cl, cp * sl, sp]])
    return d @ r.T


def heading_to_yaw(heading_deg, offset_deg=0.0):
    """Compass heading (degrees clockwise from north) -> ENU yaw (radians, counter-clockwise from east)."""
    import numpy as np
    return wrap(np.radians(90.0 - (np.asarray(heading_deg, float) + offset_deg)))


def quat_to_matrix(q):
    """(N, 4) xyzw -> (N, 3, 3)."""
    import numpy as np
    x, y, z, w = (q[:, i] for i in range(4))
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], axis=1)


def quat_to_yaw(q):
    import numpy as np
    x, y, z, w = (q[:, i] for i in range(4))
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rotz(yaw):
    import numpy as np
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def align_yaw(src, dst):
    """Yaw + translation (4-DoF) least squares mapping src onto dst: (R, t, yaw)."""
    import numpy as np
    a = src[:, :2] - src[:, :2].mean(0)
    b = dst[:, :2] - dst[:, :2].mean(0)
    yaw = math.atan2(float((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum()), float((a * b).sum()))
    r = rotz(yaw)
    return r, dst.mean(0) - r @ src.mean(0), yaw


def umeyama(src, dst):
    """SE(3) (R, t) mapping src onto dst, no scale (replay_score.py has the same)."""
    import numpy as np
    ma, mb = src.mean(0), dst.mean(0)
    u, _s, vt = np.linalg.svd((src - ma).T @ (dst - mb))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return r, mb - r @ ma


def interpolate_reference(ref_t, ref_xyz, ref_yaw, t, max_gap_ns):
    """Reference position and yaw at stamps t (linear; yaw through the unwrapped
    angle); samples farther than max_gap_ns from both neighbours are masked."""
    import numpy as np
    idx = np.searchsorted(ref_t, t)
    lo = np.clip(idx - 1, 0, len(ref_t) - 1)
    hi = np.clip(idx, 0, len(ref_t) - 1)
    ok = (t >= ref_t[0]) & (t <= ref_t[-1]) & ((ref_t[hi] - ref_t[lo]) <= max_gap_ns)
    xyz = np.stack([np.interp(t, ref_t, ref_xyz[:, i]) for i in range(3)], -1)
    yaw = None
    if ref_yaw is not None:
        valid = ~np.isnan(ref_yaw)
        if valid.sum() >= 2:
            unwrapped = np.unwrap(ref_yaw[valid])
            yaw = wrap(np.interp(t, ref_t[valid], unwrapped))
            # no heading where the reference has none within the gap
            vi = np.searchsorted(ref_t[valid], t)
            vlo = np.clip(vi - 1, 0, valid.sum() - 1); vhi = np.clip(vi, 0, valid.sum() - 1)
            yaw_ok = ok & ((ref_t[valid][vhi] - ref_t[valid][vlo]) <= max_gap_ns) & (t >= ref_t[valid][0]) & (t <= ref_t[valid][-1])
            yaw = np.where(yaw_ok, yaw, np.nan)
    return xyz, yaw, ok


def percentiles(errors):
    import numpy as np
    e = np.sort(np.asarray(errors, float))
    if len(e) == 0:
        return {}
    def pct(p):
        return float(e[min(len(e) - 1, int(p * len(e)))])
    return {"n": int(len(e)), "rms": float(math.sqrt(float((e ** 2).mean()))), "median": pct(0.5),
            "p95": pct(0.95), "max": float(e[-1])}


def relative_errors(t, est, ref, delta_ns, tol_ns):
    """Errors of displacement over delta between est and ref (both (N,3) on stamps t)."""
    import numpy as np
    j = np.searchsorted(t, t + delta_ns)
    j = np.clip(j, 0, len(t) - 1)
    ok = np.abs(t[j] - (t + delta_ns)) <= tol_ns
    ok &= j > np.arange(len(t))
    i = np.nonzero(ok)[0]
    if len(i) == 0:
        return np.zeros(0), i, j[i]
    return np.linalg.norm((est[j[i]] - est[i]) - (ref[j[i]] - ref[i]), axis=1), i, j[i]


def score_estimator(t, p, q, lever, ref_t, ref_xyz, ref_yaw, *, min_extent_m=2.0, max_gap_ns=100_000_000,
                    rpe_delta_s=1.0, rpe_tol_s=0.05):
    """All metrics of one estimator (stamps t ns, base_link positions p (N,3), quaternions q (N,4) xyzw)."""
    import numpy as np
    out = {"poses": int(len(t))}
    if len(t) < 10:
        out["skipped"] = "fewer than 10 poses"
        return out
    rot = quat_to_matrix(q)
    antenna = p + np.einsum("nij,j->ni", rot, np.asarray(lever, float))
    ref_at, ref_yaw_at, ok = interpolate_reference(ref_t, ref_xyz, ref_yaw, t, max_gap_ns)
    out["matched"] = int(ok.sum())
    if ok.sum() < 10:
        out["skipped"] = "fewer than 10 poses inside RTK-fixed reference coverage"
        return out
    t, antenna, ref_at, yaw_est = t[ok], antenna[ok], ref_at[ok], quat_to_yaw(q[ok])
    ref_yaw_at = ref_yaw_at[ok] if ref_yaw_at is not None else None
    extent = float(np.linalg.norm(ref_at[:, :2] - ref_at[:, :2].mean(0), axis=1).max() * 2)
    out["reference_extent_m"] = round(extent, 2)
    out["duration_s"] = round(float((t[-1] - t[0]) / NS), 1)
    if extent < min_extent_m:
        out["skipped"] = "reference motion %.1f m is below %.1f m: alignment not defined" % (extent, min_extent_m)
        return out
    r, tr, yaw_align = align_yaw(antenna, ref_at)
    aligned = antenna @ r.T + tr
    err = aligned - ref_at
    out["alignment"] = {"yaw_deg": round(math.degrees(yaw_align), 3), "translation_m": [round(float(v), 3) for v in tr]}
    sv = np.linalg.svd(ref_at - ref_at.mean(0), compute_uv=False)
    if sv[1] > 0.5 and sv[2] > 0.25:                    # motion in three directions: the SE(3) fit is meaningful
        r6, _t6 = umeyama(antenna, ref_at)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(r6[2, 2])))))
        out["alignment"]["se3_tilt_deg"] = round(tilt, 3)
    out["ape"] = percentiles(np.linalg.norm(err, axis=1))
    out["ape_horizontal"] = percentiles(np.linalg.norm(err[:, :2], axis=1))
    out["ape_vertical"] = percentiles(np.abs(err[:, 2]))
    big = np.nonzero(np.linalg.norm(err, axis=1) > 2.0)[0]
    out["jumps_over_2m"] = [round(float((t[i] - t[0]) / NS), 1) for i in big[:: max(1, len(big) // 20)]][:20]
    rpe, i, j = relative_errors(t, aligned, ref_at, int(rpe_delta_s * NS), int(rpe_tol_s * NS))
    out["rpe_%gs" % rpe_delta_s] = percentiles(rpe)
    if ref_yaw_at is not None and (~np.isnan(ref_yaw_at)).sum() >= 10:
        yaw_err = wrap(yaw_est + yaw_align - ref_yaw_at)
        valid = ~np.isnan(yaw_err)
        ye = np.degrees(yaw_err[valid]); tt = (t[valid] - t[valid][0]) / NS
        med = float(np.median(ye)); mad = float(np.median(np.abs(ye - med)) * 1.4826)
        slope = float(np.polyfit(tt, ye, 1)[0]) if len(tt) > 20 and tt[-1] > 30 else None
        out["yaw"] = {"n": int(valid.sum()), "offset_deg_median": round(med, 3), "spread_deg_mad": round(mad, 3),
                      "drift_deg_per_5min": None if slope is None else round(slope * 300, 3)}
        if len(i):
            dy = wrap((yaw_est[j] - yaw_est[i]) - (ref_yaw_at[j] - ref_yaw_at[i]))
            dy = np.degrees(np.abs(dy[~np.isnan(dy)]))
            out["rpe_%gs" % rpe_delta_s]["yaw_deg"] = percentiles(dy)
    return out


def read_bag(bag_dir, topics):
    """{topic: [decoded ros messages]} via mcap-ros2-support (schemas come from the bag)."""
    import glob
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    files = sorted(glob.glob(os.path.join(bag_dir, "*.mcap"))) if os.path.isdir(bag_dir) else [bag_dir]
    out = {t: [] for t in topics}
    for path in files:
        with open(path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _schema, channel, _message, ros in reader.iter_decoded_messages(topics=topics):
                out[channel.topic].append(ros)
    return out


def stamp_ns(msg):
    return msg.header.stamp.sec * NS + msg.header.stamp.nanosec


def reference_from_bag(bag, pvt_topic, att_topic, heading_offset_deg, allow_float):
    """(t, enu xyz, yaw, info) of the main antenna in a local ENU frame from the receiver's own solution."""
    import numpy as np
    pvt = sorted(bag.get(pvt_topic, []), key=stamp_ns)
    info = {"pvt_samples": len(pvt), "modes": {}}
    good_modes = {RTK_FIXED, MOVING_BASE_FIXED} | ({RTK_FLOAT, MOVING_BASE_FLOAT} if allow_float else set())
    rows = []
    for m in pvt:
        mode = int(m.mode) & 0x0F
        info["modes"][str(mode)] = info["modes"].get(str(mode), 0) + 1
        if mode in good_modes and m.latitude > DNU and m.longitude > DNU and m.height > DNU:
            rows.append((stamp_ns(m), float(m.latitude), float(m.longitude), float(m.height),
                         float(m.h_accuracy) * 0.01 if m.h_accuracy < 65535 else None))
    info["reference_samples"] = len(rows)
    if len(rows) < 10:
        return None, None, None, info
    t = np.array([r[0] for r in rows], dtype=np.int64)
    lat, lon, h = (np.array([r[k] for r in rows]) for k in (1, 2, 3))
    xyz = geodetic_to_enu(lat, lon, h, float(lat[0]), float(lon[0]), float(h[0]))
    acc = [r[4] for r in rows if r[4] is not None]
    info["h_accuracy_m_median"] = round(float(np.median(acc)), 3) if acc else None
    info["duration_s"] = round(float((t[-1] - t[0]) / NS), 1)
    yaw = np.full(len(t), np.nan)
    att = sorted(bag.get(att_topic, []), key=stamp_ns)
    if att:
        at = np.array([stamp_ns(m) for m in att], dtype=np.int64)
        heading = np.array([float(m.heading) for m in att])
        valid = np.array([int(m.error) == 0 and int(m.mode) in (1, 2, 3) for m in att]) & (heading > DNU)
        info["heading_samples"] = int(valid.sum())
        info["heading_fixed_fraction"] = round(float(np.mean([int(m.mode) in (2, 3) for m in att])), 3)
        if valid.sum() >= 2:
            yaw_valid = heading_to_yaw(heading[valid], heading_offset_deg)
            # nearest heading sample within 60 ms of each reference sample
            idx = np.clip(np.searchsorted(at[valid], t), 0, valid.sum() - 1)
            near = np.abs(at[valid][idx] - t) <= 60_000_000
            yaw[near] = yaw_valid[idx[near]]
    info["heading_coverage"] = round(float((~np.isnan(yaw)).mean()), 3)
    return t, xyz, yaw, info


def odometry_arrays(msgs):
    import numpy as np
    msgs = sorted(msgs, key=stamp_ns)
    t = np.array([stamp_ns(m) for m in msgs], dtype=np.int64)
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in msgs], float)
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
                   m.pose.pose.orientation.w] for m in msgs], float)
    frames = {(m.header.frame_id, m.child_frame_id) for m in msgs}
    return t, p, q, sorted(frames)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="\n".join(__doc__.splitlines()[2:]))
    ap.add_argument("bag", help="bag directory (its .mcap files) or one .mcap file")
    ap.add_argument("--estimators", default="/lio/odometry,/px4/odometry,/vslam/odometry")
    ap.add_argument("--gate-estimator", default="/lio/odometry", help="the estimator the gates apply to")
    ap.add_argument("--pvt-topic", default="/gnss/pvtgeodetic")
    ap.add_argument("--heading-topic", default="/gnss/atteuler")
    ap.add_argument("--lever-arm", default=",".join(str(v) for v in DEFAULT_LEVER_ARM),
                    help="main antenna in base_link, metres (x,y,z)")
    ap.add_argument("--heading-offset-deg", type=float, default=0.0,
                    help="added to the receiver's heading; 0: the receiver already reports vehicle heading")
    ap.add_argument("--allow-float", action="store_true", help="accept RTK-float reference samples too")
    ap.add_argument("--min-extent-m", type=float, default=2.0)
    ap.add_argument("--ape-rms-max-m", type=float, default=0.5)
    ap.add_argument("--rpe-max-m", type=float, default=0.15)
    ap.add_argument("--rpe-yaw-max-deg", type=float, default=1.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args(argv)
    lever = tuple(float(v) for v in args.lever_arm.split(","))
    if len(lever) != 3:
        ap.error("lever-arm needs x,y,z")
    estimators = [e for e in args.estimators.split(",") if e]
    bag = read_bag(args.bag, estimators + [args.pvt_topic, args.heading_topic])
    ref_t, ref_xyz, ref_yaw, info = reference_from_bag(bag, args.pvt_topic, args.heading_topic,
                                                       args.heading_offset_deg, args.allow_float)
    result = {"bag": args.bag, "reference": info, "lever_arm_m": list(lever), "heading_offset_deg": args.heading_offset_deg,
              "estimators": {}, "gates": {}, "reasons": []}
    if ref_t is None:
        result["reasons"].append("no usable reference: %d RTK-fixed PVT samples" % info["reference_samples"])
        result["pass"] = None
        _write(result, args.out)
        print("TRUTH SKIPPED: " + result["reasons"][0])
        return 2
    for topic in estimators:
        msgs = bag.get(topic, [])
        if not msgs:
            result["estimators"][topic] = {"poses": 0, "skipped": "not in the bag"}
            continue
        t, p, q, frames = odometry_arrays(msgs)
        scored = score_estimator(t, p, q, lever, ref_t, ref_xyz, ref_yaw, min_extent_m=args.min_extent_m)
        scored["frames"] = ["%s -> %s" % f for f in frames]
        result["estimators"][topic] = scored
    gate = result["estimators"].get(args.gate_estimator, {})
    if "ape" not in gate:
        result["reasons"].append("%s: %s" % (args.gate_estimator, gate.get("skipped", "not scored")))
        result["pass"] = None
    else:
        rpe = gate.get("rpe_1s", {})
        result["gates"] = {"ape_rms_m": [gate["ape"]["rms"], args.ape_rms_max_m],
                           "rpe_1s_m": [rpe.get("rms"), args.rpe_max_m],
                           "rpe_1s_yaw_deg": [rpe.get("yaw_deg", {}).get("rms"), args.rpe_yaw_max_deg]}
        if gate["ape"]["rms"] > args.ape_rms_max_m:
            result["reasons"].append("APE RMS %.3f m over %.2f m" % (gate["ape"]["rms"], args.ape_rms_max_m))
        if rpe.get("rms") is not None and rpe["rms"] > args.rpe_max_m:
            result["reasons"].append("RPE(1 s) RMS %.3f m over %.2f m" % (rpe["rms"], args.rpe_max_m))
        yaw_rms = rpe.get("yaw_deg", {}).get("rms")
        if yaw_rms is not None and yaw_rms > args.rpe_yaw_max_deg:
            result["reasons"].append("RPE(1 s) yaw RMS %.2f deg over %.1f deg" % (yaw_rms, args.rpe_yaw_max_deg))
        result["pass"] = not result["reasons"]
    _write(result, args.out)
    print("reference: %d RTK samples over %s s, heading coverage %s, h-accuracy median %s m" % (
        info["reference_samples"], info.get("duration_s"), info.get("heading_coverage"), info.get("h_accuracy_m_median")))
    for topic, s in result["estimators"].items():
        if "ape" in s:
            rpe = s.get("rpe_1s", {})
            print("%-16s APE rms %.3f median %.3f max %.3f m | RPE(1 s) rms %s m, yaw rms %s deg | yaw offset %s deg, drift %s deg/5 min | align yaw %.2f deg%s" % (
                topic, s["ape"]["rms"], s["ape"]["median"], s["ape"]["max"], rpe.get("rms"),
                rpe.get("yaw_deg", {}).get("rms"), s.get("yaw", {}).get("offset_deg_median"),
                s.get("yaw", {}).get("drift_deg_per_5min"), s["alignment"]["yaw_deg"],
                "" if "se3_tilt_deg" not in s["alignment"] else ", tilt %.2f deg" % s["alignment"]["se3_tilt_deg"]))
        else:
            print("%-16s %s" % (topic, s.get("skipped", "not scored")))
    if result["pass"] is None:
        print("TRUTH SKIPPED: " + "; ".join(result["reasons"]))
        return 2
    print("TRUTH " + ("PASS" if result["pass"] else "FAIL: " + "; ".join(result["reasons"])))
    return 0 if result["pass"] else 3


def _write(result, path):
    if path:
        with open(path, "w") as f:
            json.dump(result, f, indent=2)


if __name__ == "__main__":
    sys.exit(main())
