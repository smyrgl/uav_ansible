#!/usr/bin/env python3
"""Bridge from FAST-LIO to the rest of the aircraft.

FAST-LIO publishes the pose of the Avia's built-in IMU ("body") in its own
start frame ("camera_init"), stamped in the Avia's PTP-locked UTC, with no
twist. This republishes it as base_link odometry on /lio/odometry for
robot_localization (and later PX4):
- every pose is moved from the IMU to base_link through the IMU's lever arm
  (nominal URDF mount + the Avia's factory IMU offset, axes aligned), so
  differencing base_link positions accounts for rotation about base_link;
- the twist is a central difference of three consecutive base_link poses,
  rotated into the middle pose's body frame and stamped with its time;
- the twist covariance is a diagonal floor (FAST-LIO publishes none);
- FAST-LIO's frames (camera_init -> body) form their own TF tree, so the
  bridge anchors camera_init under odom with a static transform, computed
  when the first pose arrives (and again after a reset) so that FAST-LIO's
  base_link pose coincides with the EKF's at that instant.
Diagnostics: the "lio" row.
"""
import argparse
import math
import time
from collections import deque


def rotate(q, v):
    """R(q) v for the unit quaternion q = (x, y, z, w)."""
    ux, uy, uz, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (uy * vz - uz * vy), 2 * (uz * vx - ux * vz), 2 * (ux * vy - uy * vx)
    return (vx + w * tx + (uy * tz - uz * ty), vy + w * ty + (uz * tx - ux * tz), vz + w * tz + (ux * ty - uy * tx))


def rotate_inverse(q, v):
    return rotate((-q[0], -q[1], -q[2], q[3]), v)


def base_from_imu(position, quaternion, lever_arm):
    """base_link position for an IMU pose, the IMU sitting at lever_arm in
    base_link with aligned axes: p_base = p_imu - R(q) r."""
    r = rotate(quaternion, lever_arm)
    return tuple(p - d for p, d in zip(position, r))


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def anchor(odom_base, source_base):
    """The static transform odom -> source frame that makes the source's
    base_link pose coincide with odom's at one instant:
    T_odom_source = T_odom_base * T_source_base^-1. Poses are (position, (x, y, z, w))."""
    (p_ob, q_ob), (p_sb, q_sb) = odom_base, source_base
    q_sb_inv = (-q_sb[0], -q_sb[1], -q_sb[2], q_sb[3])
    q = quat_multiply(q_ob, q_sb_inv)
    shifted = rotate(q, p_sb)
    return tuple(a - b for a, b in zip(p_ob, shifted)), q


def twist_covariance(stddev):
    out = [0.0] * 36
    for i in range(3):
        out[i * 7] = stddev * stddev
    for i in range(3, 6):
        out[i * 7] = 1e6
    return out


class CentralVelocity:
    """Body-frame velocity at the middle of three consecutive poses (the
    same estimator as the vslam bridge): world displacement first to third
    over their span, rotated into the middle pose's frame. A gap over
    max_gap_s or an implausible speed starts a new window (counted)."""

    def __init__(self, max_gap_s=0.3, max_speed_mps=30.0):
        self.max_gap_ns = int(max_gap_s * 1e9)
        self.max_speed = float(max_speed_mps)
        self.samples = deque(maxlen=3)
        self.resets = 0

    def add(self, t_ns, position, quaternion, payload=None):
        if self.samples and not 0 < t_ns - self.samples[-1][0] <= self.max_gap_ns:
            self.samples.clear()
            self.resets += 1
        self.samples.append((t_ns, position, quaternion, payload))
        if len(self.samples) < 3:
            return None
        (t0, p0, _, _), (t1, p1, q1, payload1), (t2, p2, _, _) = self.samples
        span = (t2 - t0) * 1e-9
        v_world = tuple((b - a) / span for a, b in zip(p0, p2))
        if math.sqrt(sum(c * c for c in v_world)) > self.max_speed:
            self.samples.clear()
            self.resets += 1
            return None
        return t1, p1, q1, rotate_inverse(q1, v_world), payload1


def create_bridge_node(options):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

    lever_arm = tuple(options.imu_lever_arm)

    class Bridge(Node):
        def __init__(self):
            super().__init__("lio_bridge")
            self.velocity = CentralVelocity()
            self.counts = {"odom_in": 0, "odom_out": 0, "anchors": 0}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.static_tf = StaticTransformBroadcaster(self)
            self.anchored_after_reset = None    # velocity.resets when last anchored
            self._TransformStamped, self._Time, self._TransformException = TransformStamped, Time, TransformException
            self.arrivals, self.latencies, self.last_odom = [], [], None
            self.pub = self.create_publisher(Odometry, options.odometry_topic, 10)
            self.create_subscription(Odometry, options.input_topic, self._odom, 20)
            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._diag = (DiagnosticArray, DiagnosticStatus, KeyValue)
            self.create_timer(1.0, self._diagnose)
            self.get_logger().info(f"lio bridge: {options.input_topic} (IMU at {lever_arm} m in base_link) -> {options.odometry_topic}")

        def _odom(self, message):
            self.counts["odom_in"] += 1
            p, q = message.pose.pose.position, message.pose.pose.orientation
            quaternion = (q.x, q.y, q.z, q.w)
            base = base_from_imu((p.x, p.y, p.z), quaternion, lever_arm)
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            result = self.velocity.add(stamp, base, quaternion, message)
            if result is None:
                return
            t_ns, position, quaternion, (vx, vy, vz), message = result
            now = time.time_ns()
            if self.anchored_after_reset != self.velocity.resets:
                self._anchor(message.header.frame_id, t_ns, position, quaternion)
            message.header.frame_id = options.odom_frame
            message.child_frame_id = options.base_frame
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
            linear, angular = message.twist.twist.linear, message.twist.twist.angular
            linear.x, linear.y, linear.z = vx, vy, vz
            angular.x = angular.y = angular.z = 0.0
            message.twist.covariance = twist_covariance(options.twist_stddev)
            self.pub.publish(message)
            self.counts["odom_out"] += 1
            mono = time.monotonic()
            self.last_odom = mono
            self.arrivals.append(mono)
            self.latencies.append((now - t_ns) / 1e9)

        def _anchor(self, frame, t_ns, position, quaternion):
            """Publish odom -> FAST-LIO's frame once odom -> base_link is known at t."""
            try:
                tf = self.tf_buffer.lookup_transform(options.anchor_parent, options.base_frame, self._Time(nanoseconds=t_ns))
            except self._TransformException:
                return      # retried with the next pose
            t, r = tf.transform.translation, tf.transform.rotation
            p, q = anchor(((t.x, t.y, t.z), (r.x, r.y, r.z, r.w)), (position, quaternion))
            out = self._TransformStamped()
            out.header.stamp = tf.header.stamp
            out.header.frame_id, out.child_frame_id = options.anchor_parent, frame
            out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = p
            out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = q
            self.static_tf.sendTransform(out)
            self.anchored_after_reset = self.velocity.resets
            self.counts["anchors"] += 1
            self.get_logger().info(f"anchored {frame} under {options.anchor_parent}")

        def _diagnose(self):
            DiagnosticArray_, DiagnosticStatus_, KeyValue_ = self._diag
            mono = time.monotonic()
            self.arrivals = [t for t in self.arrivals if mono - t <= 5.0]
            rate = len(self.arrivals) / 5.0
            latencies, self.latencies = self.latencies[-50:], []
            latency = sorted(latencies)[len(latencies) // 2] if latencies else None
            if self.last_odom is None or mono - self.last_odom > 1.0:
                level = DiagnosticStatus_.ERROR if self.last_odom else DiagnosticStatus_.WARN
                text = "No LiDAR-inertial odometry"
            elif rate < options.min_rate_hz or (latency is not None and latency > options.max_latency_s):
                level, text = DiagnosticStatus_.WARN, f"LiDAR-inertial odometry degraded: {rate:.1f} Hz"
            else:
                level, text = DiagnosticStatus_.OK, f"Odometry at {rate:.1f} Hz, Avia UTC (PTP)"
            values = {"odometry_hz": round(rate, 1), "latency_ms": None if latency is None else round(latency * 1e3, 1),
                      "velocity_resets": self.velocity.resets, **self.counts}
            out = DiagnosticArray_()
            out.header.stamp = self.get_clock().now().to_msg()
            out.status = [DiagnosticStatus_(level=level, name="lio", hardware_id="FAST-LIO2 (Livox Avia + IMU)",
                                            message=text, values=[KeyValue_(key=k, value=str(v)) for k, v in values.items()])]
            self.diag_pub.publish(out)

    return Bridge()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--input-topic", default="/Odometry")
    parser.add_argument("--odometry-topic", default="/lio/odometry")
    parser.add_argument("--odom-frame", default="camera_init", help="FAST-LIO's own world frame (it is that frame)")
    parser.add_argument("--anchor-parent", default="odom", help="the TF frame FAST-LIO's frame is anchored under")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--imu-lever-arm", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                        help="the IMU's position in base_link, m")
    parser.add_argument("--twist-stddev", type=float, default=0.05)
    parser.add_argument("--min-rate-hz", type=float, default=8.0)
    parser.add_argument("--max-latency-s", type=float, default=0.35)   # one frame of central difference + processing
    options, ros_args = parser.parse_known_args()
    import rclpy
    node = None
    rclpy.init(args=ros_args)
    try:
        node = create_bridge_node(options)
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
