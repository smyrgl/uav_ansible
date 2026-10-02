#!/usr/bin/env python3
"""Register a second LiDAR into FAST-LIO's frame with FAST-LIO's poses.

For the down-looking E1R, whose field of view never overlaps the Avia's: every
point is placed in camera_init with the pose of the Avia's IMU ("body") at that
point's own firing time, interpolated between FAST-LIO's poses (/Odometry: the
IMU in camera_init at each scan's end, 10 Hz; linear in position, slerp in
attitude), through the sensor's mount:

    p_ci = R_ci_body(t) (R_base_s p_s + t_base_s - r_imu) + p_ci_body(t)

with base_link -> sensor from /tf_static (the URDF's nominal extrinsic until it
is calibrated, re-read every few seconds; only the static tree is read, since
following /tf at ~70 Hz cost more CPU than the registration) and r_imu the
IMU's lever arm in base_link (axes aligned, as in the lio bridge). The pose is
interpolated once per firing time (the E1R fires 64 points at once) and
composed with the mount there, so each point is transformed once. A frame
waits for the first
pose at or after its last point; FAST-LIO publishes about 20 ms after each
scan ends. Points whose bracketing poses lie further apart than the maximum
pose gap (FAST-LIO stalled or restarted, with a new origin) are dropped, never
extrapolated, and so are points outside the range gate (no returns, the near
field).

Output: the registered cloud in FAST-LIO's frame (x, y, z, intensity float32,
stamped with the frame's first point), which the lio map adds to the same voxel
map. Diagnostics: the "lio/<name>" row.
"""
import argparse
import math
import time
from collections import deque

import numpy as np

NUMPY_TYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}   # PointField datatypes


def quaternion_matrices(q):
    """(n, 3, 3) rotation matrices of (n, 4) unit quaternions (x, y, z, w)."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], axis=-1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], axis=-1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], axis=-1)], axis=-2)


def slerp(q0, q1, u):
    """Spherical linear interpolation, row by row, of (n, 4) unit quaternions at fractions u (n,)."""
    dot = np.sum(q0 * q1, axis=1)
    q1 = np.where((dot < 0)[:, None], -q1, q1)            # the short way round
    theta = np.arccos(np.clip(np.abs(dot), 0.0, 1.0))
    small = theta < 1e-6
    sin_theta = np.where(small, 1.0, np.sin(theta))
    w0 = np.where(small, 1 - u, np.sin((1 - u) * theta) / sin_theta)
    w1 = np.where(small, u, np.sin(u * theta) / sin_theta)
    q = w0[:, None] * q0 + w1[:, None] * q1
    return q / np.linalg.norm(q, axis=1, keepdims=True)


class PoseBuffer:
    """FAST-LIO's recent poses, oldest first: int ns, position (3), quaternion (4)."""

    def __init__(self, keep_s=5.0, max_gap_s=0.25):
        self.keep_ns = int(keep_s * 1e9)
        self.max_gap_ns = int(max_gap_s * 1e9)
        self.clear()

    def clear(self):
        self.t = np.zeros(0, np.int64)
        self.p = np.zeros((0, 3))
        self.q = np.zeros((0, 4))

    def add(self, t_ns, position, quaternion):
        """False when the time did not advance (a restart, a replay): the buffer starts over."""
        restarted = bool(len(self.t)) and t_ns <= self.t[-1]
        if restarted:
            self.clear()
        keep = self.t >= t_ns - self.keep_ns
        self.t = np.append(self.t[keep], np.int64(t_ns))
        self.p = np.vstack([self.p[keep], position])
        self.q = np.vstack([self.q[keep], quaternion])
        return not restarted

    def latest(self):
        return int(self.t[-1]) if len(self.t) else None

    def interpolate(self, times_ns):
        """Poses at times_ns (m,): (valid (m,) bool, positions (k, 3), rotations (k, 3, 3)) for the
        k valid times, those between two poses at most max_gap apart."""
        times_ns = np.asarray(times_ns, np.int64)
        i1 = np.searchsorted(self.t, times_ns, side="left")
        i0 = i1 - 1
        valid = (i0 >= 0) & (i1 < len(self.t))
        valid[valid] = self.t[i1[valid]] - self.t[i0[valid]] <= self.max_gap_ns
        i0, i1 = i0[valid], i1[valid]
        u = (times_ns[valid] - self.t[i0]) / (self.t[i1] - self.t[i0])
        positions = self.p[i0] + u[:, None] * (self.p[i1] - self.p[i0])
        return valid, positions, quaternion_matrices(slerp(self.q[i0], self.q[i1], u))


def register(points, times_ns, poses, rotation, translation):
    """Sensor points (n, 3), firing times (n,) -> (world (k, 3), kept (n,) bool). rotation and translation
    place the sensor in the IMU (body) frame; they are composed with the pose at each distinct firing time."""
    unique, inverse = np.unique(times_ns, return_inverse=True)
    valid, positions, rotations = poses.interpolate(unique)
    kept = valid[inverse]
    row = (np.cumsum(valid) - 1)[inverse[kept]]
    r_world = (rotations @ rotation)[row]                       # sensor -> world at each point's firing time
    t_world = (rotations @ translation + positions)[row]
    p = points[kept]
    world = r_world[:, :, 0] * p[:, 0:1] + r_world[:, :, 1] * p[:, 1:2] + r_world[:, :, 2] * p[:, 2:3] + t_world
    return world, kept


def range_gate(xyz, min_range, max_range):
    """Finite points with min_range <= |p| (<= max_range when it is positive)."""
    r = np.linalg.norm(xyz, axis=1)
    keep = np.isfinite(r) & (r >= min_range)
    if max_range > 0:
        keep &= r <= max_range
    return keep


def decode(message, time_field):
    """x, y, z (n, 3) float64, intensity (n,) float32 and the time field (n,) float64 of a PointCloud2."""
    fields = {f.name: f for f in message.fields}
    names = ["x", "y", "z", "intensity", time_field]
    order = ">" if message.is_bigendian else "<"
    dtype = np.dtype({"names": names, "formats": [order + NUMPY_TYPES[fields[n].datatype] for n in names],
                      "offsets": [fields[n].offset for n in names], "itemsize": message.point_step})
    raw = np.frombuffer(bytes(message.data), dtype=dtype, count=message.width * message.height)
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
    return xyz, raw["intensity"].astype(np.float32), raw[time_field].astype(np.float64)


def rpy_degrees(rotation):
    return (math.degrees(math.atan2(rotation[2, 1], rotation[2, 2])),
            math.degrees(math.asin(max(-1.0, min(1.0, -rotation[2, 0])))),
            math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])))


def create_register_node(options):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException

    lever_arm = np.array(options.imu_lever_arm, float)

    class Register(Node):
        def __init__(self):
            super().__init__(f"lio_register_{options.name}")
            self.poses = PoseBuffer(max_gap_s=options.max_pose_gap)
            self.pending = deque()
            self.extrinsics = {}            # frame -> (rotation, translation, monotonic when read)
            self.frame = None               # FAST-LIO's world frame, from its odometry
            self.counts = {k: 0 for k in ("frames_in", "frames_out", "frames_no_pose", "frames_no_extrinsic",
                                          "frames_empty", "frames_overflow", "points_in", "points_out",
                                          "points_range_gated", "points_unbracketed", "pose_restarts")}
            self.last_input = self.last_pose = None
            self.outputs, self.waits, self.latencies = [], [], []
            self.tf_buffer = Buffer()                # static transforms only (the mount)
            latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(TFMessage, "/tf_static", self._tf_static, latched)
            self.pub = self.create_publisher(PointCloud2, options.output_topic, 5)
            self.create_subscription(Odometry, options.odometry_topic, self._odom, 50)
            self.create_subscription(PointCloud2, options.input_topic, self._cloud, qos_profile_sensor_data)
            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._types = (PointCloud2, PointField, DiagnosticArray, DiagnosticStatus, KeyValue)
            self._Time, self._TransformException = Time, TransformException
            self.create_timer(0.25, self._flush)     # frames are flushed on each pose; this drops stale ones
            self.create_timer(1.0, self._diagnose)
            self.get_logger().info(f"{options.input_topic} -> {options.output_topic} with {options.odometry_topic} poses "
                                   f"(IMU at {tuple(lever_arm)} m in {options.base_frame})")

        def _odom(self, message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            t_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if not self.poses.add(t_ns, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)):
                self.counts["pose_restarts"] += 1
            self.frame, self.last_pose = message.header.frame_id, time.monotonic()
            self._flush()

        def _tf_static(self, message):
            for transform in message.transforms:
                self.tf_buffer.set_transform_static(transform, "lio_register")

        def _extrinsic(self, frame):
            """base_link -> frame from TF (rotation, translation), re-read every extrinsic_refresh s."""
            now = time.monotonic()
            cached = self.extrinsics.get(frame)
            if cached is not None and now - cached[2] < options.extrinsic_refresh:
                return cached[:2]
            try:
                tf = self.tf_buffer.lookup_transform(options.base_frame, frame, self._Time())
            except self._TransformException:
                return cached[:2] if cached else None
            t, r = tf.transform.translation, tf.transform.rotation
            rotation = quaternion_matrices(np.array([[r.x, r.y, r.z, r.w]]))[0]
            self.extrinsics[frame] = (rotation, np.array([t.x, t.y, t.z]), now)
            return self.extrinsics[frame][:2]

        def _cloud(self, message):
            arrival = self.last_input = time.monotonic()
            self.counts["frames_in"] += 1
            try:
                xyz, intensity, seconds = decode(message, options.time_field)
            except (KeyError, ValueError) as exc:
                self.get_logger().warning(f"cloud skipped: {exc}", throttle_duration_sec=10)
                return
            self.counts["points_in"] += len(xyz)
            keep = range_gate(xyz, options.min_range, options.max_range)
            self.counts["points_range_gated"] += int(len(keep) - keep.sum())
            extrinsic = self._extrinsic(message.header.frame_id)
            if extrinsic is None:
                self.counts["frames_no_extrinsic"] += 1
                self.get_logger().warning(f"no transform {options.base_frame} -> {message.header.frame_id} yet",
                                          throttle_duration_sec=10)
                return
            if not keep.any():
                self.counts["frames_empty"] += 1
                return
            rotation, translation = extrinsic
            mount = (rotation, translation - lever_arm)                   # the sensor in the IMU frame
            times = np.round(seconds[keep] * 1e9).astype(np.int64)
            if len(self.pending) >= options.max_pending:
                self.pending.popleft()
                self.counts["frames_overflow"] += 1
            self.pending.append((arrival, message.header.stamp, int(times.max()), xyz[keep], intensity[keep], times, mount))
            self._flush()

        def _flush(self):
            now = time.monotonic()
            while self.pending:
                arrival, stamp, last, points, intensity, times, mount = self.pending[0]
                latest = self.poses.latest()
                if latest is not None and latest >= last and self.frame is not None:
                    self.pending.popleft()
                    self._register(arrival, stamp, last, points, intensity, times, mount)
                elif now - arrival > options.max_wait:
                    self.pending.popleft()
                    self.counts["frames_no_pose"] += 1
                else:
                    break

        def _register(self, arrival, stamp, last, points, intensity, times, mount):
            world, kept = register(points, times, self.poses, *mount)
            self.counts["points_unbracketed"] += int(len(kept) - kept.sum())
            if not len(world):
                self.counts["frames_no_pose"] += 1
                return
            PointCloud2_, PointField_ = self._types[:2]
            msg = PointCloud2_()
            msg.header.stamp, msg.header.frame_id = stamp, self.frame
            msg.height, msg.width = 1, len(world)
            msg.fields = [PointField_(name=n, offset=4 * i, datatype=PointField_.FLOAT32, count=1)
                          for i, n in enumerate(("x", "y", "z", "intensity"))]
            msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 16, 16 * len(world), True
            msg.data = np.column_stack([world, intensity[kept]]).astype(np.float32).tobytes()
            self.pub.publish(msg)
            mono = time.monotonic()
            self.counts["frames_out"] += 1
            self.counts["points_out"] += len(world)
            self.outputs.append(mono)
            self.waits.append(mono - arrival)
            self.latencies.append(time.time() - last / 1e9)

        def _diagnose(self):
            _, _, DiagnosticArray_, DiagnosticStatus_, KeyValue_ = self._types
            mono = time.monotonic()
            self.outputs = [t for t in self.outputs if mono - t <= 5.0]
            rate = len(self.outputs) / 5.0
            waits, self.waits = sorted(self.waits[-50:]), []
            latencies, self.latencies = sorted(self.latencies[-50:]), []
            if self.last_input is None or mono - self.last_input > 2.0:
                level, text = DiagnosticStatus_.WARN, f"No {options.name} frames on {options.input_topic}"
            elif not self.extrinsics:
                level, text = DiagnosticStatus_.WARN, f"No transform {options.base_frame} -> {options.name} frame"
            elif self.last_pose is None or mono - self.last_pose > 2.0:
                level, text = DiagnosticStatus_.WARN, "No FAST-LIO poses: nothing registered"
            elif rate < options.min_rate_hz:
                level, text = DiagnosticStatus_.WARN, f"Registration degraded: {rate:.1f} Hz"
            else:
                level, text = DiagnosticStatus_.OK, f"Registered at {rate:.1f} Hz with FAST-LIO poses"
            values = {"output_hz": round(rate, 1),
                      "pose_wait_ms": round(waits[len(waits) // 2] * 1e3, 1) if waits else None,
                      "latency_ms": round(latencies[len(latencies) // 2] * 1e3, 1) if latencies else None,
                      **self.counts}
            for frame, (rotation, translation, _) in self.extrinsics.items():
                values[f"{options.base_frame} -> {frame}"] = (
                    "t (%.4f, %.4f, %.4f) m, rpy (%.2f, %.2f, %.2f) deg" % (*translation, *rpy_degrees(rotation)))
            out = DiagnosticArray_()
            out.header.stamp = self.get_clock().now().to_msg()
            out.status = [DiagnosticStatus_(level=level, name=f"lio/{options.name}",
                                            hardware_id=f"{options.input_topic} with FAST-LIO poses", message=text,
                                            values=[KeyValue_(key=k, value=str(v)) for k, v in values.items()])]
            self.diag_pub.publish(out)

    return Register()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--name", default="e1r", help="diagnostic row lio/<name>")
    parser.add_argument("--input-topic", default="/e1r/points")
    parser.add_argument("--time-field", default="timestamp", help="per-point absolute time, s (the E1R: UTC float64)")
    parser.add_argument("--output-topic", default="/lio/registered/e1r")
    parser.add_argument("--odometry-topic", default="/Odometry", help="FAST-LIO's IMU pose")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--imu-lever-arm", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                        help="the IMU's position in base_link, m")
    parser.add_argument("--min-range", type=float, default=0.1, help="m")
    parser.add_argument("--max-range", type=float, default=0.0, help="m; 0 = no limit")
    parser.add_argument("--max-pose-gap", type=float, default=0.25, help="s between bracketing poses")
    parser.add_argument("--max-wait", type=float, default=0.5, help="s a frame waits for the pose after it")
    parser.add_argument("--max-pending", type=int, default=10, help="frames waiting at once")
    parser.add_argument("--extrinsic-refresh", type=float, default=10.0, help="s between TF re-reads")
    parser.add_argument("--min-rate-hz", type=float, default=8.0)
    options, ros_args = parser.parse_known_args()
    import rclpy
    node = None
    rclpy.init(args=ros_args)
    try:
        node = create_register_node(options)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
