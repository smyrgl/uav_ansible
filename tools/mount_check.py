#!/usr/bin/env python3
"""Check the sensor mounts against the URDF: every sensor's view of the floor, in base_link.

With the aircraft on its skids on a flat floor, every depth sensor sees the floor. Each
sensor's points go into base_link through that sensor's mount in the URDF (the frame_id
of its own messages, looked up in /tf_static) and a plane is fitted to the points below
the aircraft. If every mount is right, every sensor reports the same plane: the same
normal and the same height of base_link above it, with the normal along gravity (a floor
is level to a fraction of a degree; PX4's attitude gives gravity in base_link). A mount
pitched or rolled wrong by d tilts that sensor's normal by d; a mount at the wrong height
or position moves its plane. Accelerometers give a second, floor-free check: the specific
force each IMU measures, carried into base_link through its URDF mount, is "up" and must
agree with PX4's (to the accelerometer's bias, typically a fraction of a degree).

    mount_check.py [--seconds 10]                      # live, on the Jetson (ROS environment)
    mount_check.py --bag BAG [--start S] [--seconds 10]  # offline, from a recorded bag

Nothing here knows where a sensor is: every pose comes from the URDF's TF tree, so the
check covers whatever the description says, after any re-mount or calibration. Put the
aircraft on the floor, not a bench (each sensor must see the same surface), with nothing
else within a couple of metres below its sensors.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

try:                                       # installed beside it in /usr/local/lib/uav
    import lio_extrinsic as lx
except ImportError:                        # the repository layout
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "roles", "lio", "files"))
    import lio_extrinsic as lx

CLOUD_TOPICS = ["/avia/points", "/e1r/points"]
IMU_TOPICS = ["/avia/imu", "/d555/imu"]
ATTITUDE_TOPIC = "/px4/imu/data"
DEPTH_INFO_SUFFIX = "/camera_info"


# ----- geometry ---------------------------------------------------------------------------

def fit_plane(points, up, tol=0.03, iterations=400, max_tilt_deg=30.0, seed=0):
    """Dominant plane within max_tilt_deg of up: (normal along up, height of the origin above
    the plane, inliers, rms) or None. RANSAC on three points, then least squares on the inliers."""
    if len(points) < 50:
        return None
    rng = np.random.default_rng(seed)
    up = np.asarray(up, float) / np.linalg.norm(up)
    cos_max = math.cos(math.radians(max_tilt_deg))
    best_count, best = 0, None
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n @ up < 0:
            n = -n
        if n @ up < cos_max:
            continue
        inliers = np.abs((points - a) @ n) < tol
        count = int(inliers.sum())
        if count > best_count:
            best_count, best = count, inliers
    if best is None:
        return None
    p = points[best]
    centre = p.mean(0)
    n = np.linalg.svd(p - centre, full_matrices=False)[2][2]
    if n @ up < 0:
        n = -n
    residual = (p - centre) @ n
    return n, float(-(n @ centre)), int(best.sum()), float(np.sqrt(np.mean(residual ** 2)))


def mount_error(v, reference):
    """The mount error a measured 'up' implies: (angle, pitch, roll) in degrees.

    A sensor whose true mount is its URDF mount rotated by E (in base_link) sees the floor,
    and gravity, through the URDF mount as the true ones rotated by E^-1, so v = E^T reference
    and E's rotation vector is v x reference. Pitch is its component about base_link y (+:
    the sensor more nose-down than the URDF says), roll about x (+: rolled right). Yaw is
    invisible to a floor and to gravity."""
    v, reference = (np.asarray(x, float) / np.linalg.norm(x) for x in (v, reference))
    angle = math.acos(max(-1.0, min(1.0, float(v @ reference))))
    axis = np.cross(v, reference)
    w = axis / np.linalg.norm(axis) * angle if np.linalg.norm(axis) > 1e-12 else np.zeros(3)
    return math.degrees(angle), math.degrees(w[1]), math.degrees(w[0])


def quaternion_matrix(q):
    return lx.quaternion_matrix(*q)


def up_from_attitude(quaternions, r_base_frame):
    """'Up' in base_link from orientations of frame F in an ENU world (x, y, z, w): R_base_F R_world_F^T z."""
    ups = [r_base_frame @ (quaternion_matrix(q).T @ np.array([0.0, 0.0, 1.0])) for q in quaternions]
    u = np.mean(ups, axis=0)
    return u / np.linalg.norm(u)


def up_from_accelerometer(accelerations, r_base_frame):
    """'Up' in base_link from an IMU at rest: the mean specific force, through the IMU's mount."""
    f = np.mean(np.asarray(accelerations, float), axis=0)
    u = r_base_frame @ (f / np.linalg.norm(f))
    return u / np.linalg.norm(u)


# ----- messages ---------------------------------------------------------------------------

POINT_TYPES = {7: "<f4", 8: "<f8"}


def cloud_xyz(message):
    fields = {f.name: f for f in message.fields}
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": [POINT_TYPES[fields[n].datatype] for n in "xyz"],
                      "offsets": [fields[n].offset for n in "xyz"], "itemsize": message.point_step})
    raw = np.frombuffer(bytes(message.data), dtype=dtype, count=message.width * message.height)
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)]


def depth_xyz(image, info, stride=8):
    """A depth image (16UC1 mm or 32FC1 m) back-projected through its camera_info K: points in
    the image's optical frame (x right, y down, z forward)."""
    h, w = image.height, image.width
    if image.encoding in ("16UC1", "mono16"):
        depth = np.frombuffer(bytes(image.data), dtype="<u2").reshape(h, image.step // 2)[:, :w] / 1000.0
    elif image.encoding == "32FC1":
        depth = np.frombuffer(bytes(image.data), dtype="<f4").reshape(h, image.step // 4)[:, :w].astype(np.float64)
    else:
        raise ValueError(f"depth encoding {image.encoding}")
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    keep = np.isfinite(z) & (z > 0.1)
    z, u, v = z[keep], u[keep], v[keep]
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)


# ----- the check -----------------------------------------------------------------------------

class Collector:
    def __init__(self):
        self.tree = lx.StaticTree()
        self.clouds = {}              # topic -> (frame, [xyz arrays])
        self.depth = {}               # topic -> (frame, [xyz arrays])
        self.imus = {}                # topic -> (frame, [accelerations])
        self.attitude = []            # (frame, quaternion)

    def cloud(self, topic, message, limit=200_000):
        frame, chunks = self.clouds.setdefault(topic, (message.header.frame_id, []))
        if sum(len(c) for c in chunks) < limit:
            chunks.append(cloud_xyz(message))

    def depth_image(self, topic, image, info):
        frame, chunks = self.depth.setdefault(topic, (image.header.frame_id, []))
        if len(chunks) < 10:
            chunks.append(depth_xyz(image, info))

    def imu(self, topic, message):
        a, q = message.linear_acceleration, message.orientation
        frame, acc = self.imus.setdefault(topic, (message.header.frame_id, []))
        acc.append((a.x, a.y, a.z))
        if topic == ATTITUDE_TOPIC:
            self.attitude.append((message.header.frame_id, (q.x, q.y, q.z, q.w)))


def mount(tree, frame, base="base_link"):
    t = tree.lookup(base, frame.lstrip("/"))
    return (t[:3, :3], t[:3, 3]) if t is not None else None


def evaluate(c, base="base_link", below=-0.05, radius=8.0, tol=0.03):
    report = {"reference": None, "planes": {}, "imus": {}, "problems": []}
    reference = None
    if c.attitude:
        frame = c.attitude[0][0]
        m = mount(c.tree, frame, base)
        if m is None:
            report["problems"].append(f"no {base} -> {frame} for {ATTITUDE_TOPIC}")
        else:
            reference = up_from_attitude([q for _, q in c.attitude], m[0])
            report["reference"] = {"source": f"{ATTITUDE_TOPIC} (PX4 attitude, {frame})", "up": [round(float(v), 4) for v in reference]}
    if reference is None:
        reference = np.array([0.0, 0.0, 1.0])
        report["problems"].append("no PX4 attitude: base_link's z taken as up (the floor's tilt then includes the aircraft's)")
    for kind, sources in (("cloud", c.clouds), ("depth", c.depth)):
        for topic, (frame, chunks) in sources.items():
            m = mount(c.tree, frame, base)
            if m is None:
                report["problems"].append(f"no {base} -> {frame} ({topic}): not in the URDF's TF")
                continue
            if not chunks:
                continue
            p = np.concatenate(chunks) @ m[0].T + m[1]
            sel = p[(p[:, 2] < below) & (np.hypot(p[:, 0], p[:, 1]) < radius)]
            fit = fit_plane(sel, reference, tol=tol)
            if fit is None:
                report["problems"].append(f"{topic}: no floor plane in {len(sel)} points below base_link")
                continue
            n, height, inliers, rms = fit
            angle, pitch, roll = mount_error(n, reference)
            report["planes"][topic] = {"frame": frame, "kind": kind, "points": int(len(sel)), "inliers": inliers,
                                       "rms_m": round(rms, 4), "height_m": round(height, 4),
                                       "tilt_deg": round(angle, 2), "pitch_deg": round(pitch, 2), "roll_deg": round(roll, 2),
                                       "normal": [round(float(v), 4) for v in n]}
    for topic, (frame, acc) in c.imus.items():
        if topic == ATTITUDE_TOPIC or len(acc) < 20:
            continue
        m = mount(c.tree, frame, base)
        if m is None:
            report["problems"].append(f"no {base} -> {frame} ({topic}): not in the URDF's TF")
            continue
        up = up_from_accelerometer(acc, m[0])
        angle, pitch, roll = mount_error(up, reference)
        report["imus"][topic] = {"frame": frame, "samples": len(acc), "tilt_deg": round(angle, 2),
                                 "pitch_deg": round(pitch, 2), "roll_deg": round(roll, 2)}
    for topic in CLOUD_TOPICS + IMU_TOPICS:
        if topic not in c.clouds and topic not in c.imus:
            report["problems"].append(f"{topic}: no messages (sensor off or not publishing)")
    if not c.depth:
        report["problems"].append("no depth images (the D555's native depth topic, or --depth-topic)")
    heights = [v["height_m"] for v in report["planes"].values()]
    if heights:
        median = float(np.median(heights))
        for v in report["planes"].values():
            v["height_vs_median_m"] = round(v["height_m"] - median, 4)
    return report


def print_report(report):
    ref = report["reference"]
    print("gravity reference:", ref["source"] if ref else "none (base_link z)")
    if report["planes"]:
        print(f"\n{'floor seen by':22} {'frame':28} {'points':>7} {'rms':>7} {'height':>8} {'Δ median':>9} {'tilt':>6} {'pitch':>6} {'roll':>6}")
        for topic, v in sorted(report["planes"].items()):
            print(f"{topic:22} {v['frame']:28} {v['inliers']:7d} {v['rms_m']:7.3f} {v['height_m']:8.3f} "
                  f"{v.get('height_vs_median_m', 0.0):+9.3f} {v['tilt_deg']:6.2f} {v['pitch_deg']:+6.2f} {v['roll_deg']:+6.2f}")
    if report["imus"]:
        print(f"\n{'gravity seen by':22} {'frame':28} {'samples':>7} {'tilt':>6} {'pitch':>6} {'roll':>6}")
        for topic, v in sorted(report["imus"].items()):
            print(f"{topic:22} {v['frame']:28} {v['samples']:7d} {v['tilt_deg']:6.2f} {v['pitch_deg']:+6.2f} {v['roll_deg']:+6.2f}")
    print("\nheight: base_link above the plane, m; the same for every sensor if every mount is right.")
    print("tilt: the plane's normal, or the IMU's 'up', against PX4's gravity, deg. pitch/roll: the mount error that implies,")
    print("true minus URDF, about base_link y and x: pitch + = more nose-down than the URDF, roll + = rolled right.")
    print("A floor's own slope and an accelerometer's bias add to it; yaw cannot be seen this way.")
    for problem in report["problems"]:
        print("!", problem)


# ----- sources ---------------------------------------------------------------------------------

def collect_bag(bag, start_s, seconds, depth_topics):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    paths = [bag] if os.path.isfile(bag) else sorted(os.path.join(bag, n) for n in os.listdir(bag) if n.endswith(".mcap"))
    c = Collector()
    infos, t0 = {}, None
    topics = ["/tf_static", ATTITUDE_TOPIC] + CLOUD_TOPICS + IMU_TOPICS + depth_topics + [d.rsplit("/", 1)[0] + DEPTH_INFO_SUFFIX for d in depth_topics]
    for path in paths:
        with open(path, "rb") as f:
            for _s, channel, message, decoded in make_reader(f, decoder_factories=[DecoderFactory()]).iter_decoded_messages(topics=topics):
                if channel.topic == "/tf_static":
                    c.tree.add_message(decoded)
                    continue
                t0 = message.log_time if t0 is None else t0
                t = (message.log_time - t0) / 1e9
                if t < start_s:
                    continue
                if t > start_s + seconds:
                    break
                topic = channel.topic
                if topic in CLOUD_TOPICS:
                    c.cloud(topic, decoded)
                elif topic in IMU_TOPICS or topic == ATTITUDE_TOPIC:
                    c.imu(topic, decoded)
                elif topic.endswith(DEPTH_INFO_SUFFIX):
                    infos[topic] = decoded
                elif topic in depth_topics:
                    info = infos.get(topic.rsplit("/", 1)[0] + DEPTH_INFO_SUFFIX)
                    if info is not None:
                        c.depth_image(topic, decoded, info)
    return c


def collect_live(seconds, depth_topic):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = rclpy.create_node("mount_check")
    c = Collector()
    latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TFMessage, "/tf_static", c.tree.add_message, latched)
    for topic in CLOUD_TOPICS:
        node.create_subscription(PointCloud2, topic, lambda m, t=topic: c.cloud(t, m), qos_profile_sensor_data)
    for topic in IMU_TOPICS + [ATTITUDE_TOPIC]:
        node.create_subscription(Imu, topic, lambda m, t=topic: c.imu(t, m), qos_profile_sensor_data)
    if depth_topic == "auto":           # the D555's native depth image: its name carries the serial
        time.sleep(2.0)
        names = [n for n, types in node.get_topic_names_and_types() if "sensor_msgs/msg/Image" in types and n.endswith("_Depth")]
        depth_topic = names[0] if names else ""
    info = {}
    if depth_topic:
        # The frame-validated relay: the camera's own camera_info interleaves colour intrinsics.
        node.create_subscription(CameraInfo, "/d555/depth/camera_info", lambda m: info.update(k=m), qos_profile_sensor_data)
        node.create_subscription(Image, depth_topic,
                                 lambda m: c.depth_image(depth_topic, m, info["k"]) if "k" in info else None, qos_profile_sensor_data)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return c


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="\n".join(__doc__.splitlines()[2:]))
    ap.add_argument("--bag", default="", help="read a recorded bag (directory or .mcap) instead of live topics")
    ap.add_argument("--start", type=float, default=0.0, help="bag: seconds from its start")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--depth-topic", default="auto", help="live: a depth Image topic ('auto': the D555's native depth; '' none)")
    ap.add_argument("--below", type=float, default=-0.05, help="consider points this far below base_link and lower, m")
    ap.add_argument("--radius", type=float, default=8.0, help="and within this horizontal distance, m")
    ap.add_argument("--tolerance", type=float, default=0.03, help="RANSAC inlier distance, m")
    ap.add_argument("--json", default="", help="also write the report here")
    args = ap.parse_args()
    if args.bag:
        c = collect_bag(args.bag, args.start, args.seconds, ["/d555/depth/throttled"])
    else:
        c = collect_live(args.seconds, args.depth_topic)
    report = evaluate(c, below=args.below, radius=args.radius, tol=args.tolerance)
    print_report(report)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=1)


if __name__ == "__main__":
    main()
