#!/usr/bin/env python3
"""Bridge between Isaac ROS cuVSLAM and the rest of the aircraft.

- Relays the D555 IR camera_info to cuVSLAM with the camera's own stamps (its
  device clock, the images' clock, which cuVSLAM pairs them by), dropping the
  occasional message the camera labels with another stream's frame.
- Republishes cuVSLAM's odometry on the host UTC time base for
  robot_localization, through the clock model the D555 adapter publishes on
  /d555/clock: nav_msgs/Odometry, pose in the cuVSLAM odom frame, twist in
  base_link. Nothing is published while the model is unavailable: a wrong time
  base would corrupt the EKFs more than a gap.
- Replaces cuVSLAM's twist with a central difference of its poses (see
  CentralVelocity): its own averages over a ten-pose cache, in the oldest
  pose's frame, with the spread of ten velocities as "covariance".
- Reports tracking, rate and latency as the "vslam" diagnostic.
- Anchors the cuVSLAM odom frame under odom with a static transform, computed
  when the first pose arrives (and again after a reset, when cuVSLAM may have
  restarted its coordinate system) so that its base_link pose coincides with
  the EKF's at that instant; otherwise that frame has no place in the TF tree.
"""
import os
import argparse
import json
import math
import time
from collections import deque

VO_STATES = {0: "unknown", 1: "tracking", 2: "failed"}


def unwrap_near(device_ns, reference_ns, wrap_ns):
    """The representation of a possibly wrapped device time nearest reference."""
    if not wrap_ns:
        return device_ns
    return device_ns + round((reference_ns - device_ns) / wrap_ns) * wrap_ns


class ClockMapping:
    """Device clock -> UTC from /d555/clock (the same model uav_camera applies)."""

    def __init__(self, max_age_s=5.0):
        self.max_age_ns = int(max_age_s * 1e9)
        self.model = None
        self.reason = "no /d555/clock model received"

    def update(self, text):
        try:
            model = json.loads(text)
        except ValueError:
            self.model, self.reason = None, "unreadable /d555/clock message"
            return
        if model.get("valid"):
            self.model, self.reason = model, "ok"
        else:
            self.model, self.reason = None, str(model.get("reason", "model not valid"))

    def to_utc_ns(self, device_ns, now_ns=None):
        model = self.model
        if not model or not device_ns:
            return None
        now = time.time_ns() if now_ns is None else now_ns
        if now - int(model["computed_utc_ns"]) > self.max_age_ns:
            self.reason = "clock model stale"
            return None
        ref = int(model["device_ref_ns"])
        d = unwrap_near(int(device_ns), ref, int(model.get("wrap_ns", 0)))
        return d + int(model["offset_ref_ns"]) + round(float(model["skew_ppm"]) * 1e-6 * (d - ref))


def frame_label(frame_id):
    # The D555's native CameraInfo can carry its frame name NUL-padded in a fixed buffer.
    return frame_id.split("\x00", 1)[0].strip()


def twist_covariance(stddev):
    """Diagonal twist covariance for the EKFs: stddev^2 on the linear axes,
    angular unknown (1e6). cuVSLAM's own (~3e-10 (m/s)^2 standing still, the
    spread of its last ten velocities) says nothing about lever-arm, alignment
    or scale errors and would make robot_localization treat VSLAM as exact."""
    out = [0.0] * 36
    for i in range(3):
        out[i * 7] = stddev * stddev
    for i in range(3, 6):
        out[i * 7] = 1e6
    return out


def rotate_inverse(q, v):
    """R(q)^T v: v (a world-frame vector) in the body frame of the unit quaternion q = (x, y, z, w)."""
    ux, uy, uz, w = -q[0], -q[1], -q[2], q[3]
    vx, vy, vz = v
    tx, ty, tz = 2 * (uy * vz - uz * vy), 2 * (uz * vx - ux * vz), 2 * (ux * vy - uy * vx)
    return (vx + w * tx + (uy * tz - uz * ty), vy + w * ty + (uz * tx - ux * tz), vz + w * tz + (ux * ty - uy * tx))


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
    shifted = rotate_inverse((-q[0], -q[1], -q[2], q[3]), p_sb)
    return tuple(a - b for a, b in zip(p_ob, shifted)), q


class CentralVelocity:
    """Body-frame velocity at the middle of three consecutive poses: the
    world-frame displacement from the first to the third over their time span,
    rotated into the middle pose's body frame and stamped with its time, so it
    is neither late nor mis-rotated (one frame of delay before it is known).
    cuVSLAM's own twist is the displacement across its ten-pose cache in the
    oldest pose's frame (pose_cache.cpp, release-4.6): ~0.15 s late at 30 Hz and
    rotated by whatever the vehicle turned since. A gap over max_gap_s (a lost
    frame run, after which cuVSLAM may restart its coordinate system) or an
    implausible speed starts a new window; `resets` counts them."""

    def __init__(self, max_gap_s=0.1, max_speed_mps=30.0):
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
        (t0, p0, _, _), (t1, _, q1, payload1), (t2, p2, _, _) = self.samples
        span = (t2 - t0) * 1e-9
        v_world = tuple((b - a) / span for a, b in zip(p0, p2))
        if math.sqrt(sum(c * c for c in v_world)) > self.max_speed:
            self.samples.clear()
            self.resets += 1
            return None
        return t1, rotate_inverse(q1, v_world), payload1


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def create_bridge_node(options):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import TransformStamped
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from rclpy.time import Time
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import String

    class Bridge(Node):
        def __init__(self):
            super().__init__("vslam_bridge")
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.counts = {"info_relayed": 0, "info_dropped": 0, "odom_in": 0, "odom_out": 0, "odom_unmapped": 0}
            self.velocity = CentralVelocity()
            self.counts["anchors"] = 0
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.static_tf = StaticTransformBroadcaster(self)
            self.anchored_after_reset = None
            self._TransformStamped, self._Time, self._TransformException = TransformStamped, Time, TransformException
            self.clock = ClockMapping()
            self.create_subscription(String, "/d555/clock", lambda m: self.clock.update(m.data), latched)
            for source, target, expected in ((options.left_info_in, options.left_info_out, "camera_infra1_optical_frame"),
                                             (options.right_info_in, options.right_info_out, "camera_infra2_optical_frame")):
                publisher = self.create_publisher(CameraInfo, target, qos_profile_sensor_data)
                self.create_subscription(
                    CameraInfo, source,
                    lambda m, publisher=publisher, expected=expected: self._info(m, publisher, expected),
                    qos_profile_sensor_data)
            self.odom_pub = self.create_publisher(Odometry, options.odometry_topic, 10)
            self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self._odom, qos_profile_sensor_data)
            self.create_subscription(VisualSlamStatus, "/visual_slam/status", self._status, qos_profile_sensor_data)
            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._diag_types = (DiagnosticArray, DiagnosticStatus, KeyValue)
            self.vo_state, self.track_mean_s, self.last_status = None, None, None
            self.arrivals, self.latencies = [], []
            self.last_odom = None
            self.create_timer(1.0, self._diagnose)
            self.get_logger().info(f"vslam bridge: cuVSLAM odometry -> {options.odometry_topic} (UTC via /d555/clock)")

        def _info(self, message, publisher, expected):
            label = frame_label(message.header.frame_id)
            if label != expected:
                self.counts["info_dropped"] += 1
                return
            message.header.frame_id = label
            publisher.publish(message)
            self.counts["info_relayed"] += 1

        def _odom(self, message):
            self.counts["odom_in"] += 1
            p, q = message.pose.pose.position, message.pose.pose.orientation
            result = self.velocity.add(stamp_ns(message.header.stamp), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), message)
            if result is None:
                return
            device_ns, (vx, vy, vz), message = result
            now = time.time_ns()
            utc = self.clock.to_utc_ns(device_ns, now)
            if utc is None:
                self.counts["odom_unmapped"] += 1
                return
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(utc, 1_000_000_000)
            if self.anchored_after_reset != self.velocity.resets:
                pose = message.pose.pose
                self._anchor(message.header.frame_id, utc, (pose.position.x, pose.position.y, pose.position.z),
                             (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            linear, angular = message.twist.twist.linear, message.twist.twist.angular
            linear.x, linear.y, linear.z = vx, vy, vz
            angular.x = angular.y = angular.z = 0.0
            message.twist.covariance = twist_covariance(options.twist_stddev)
            self.odom_pub.publish(message)
            self.counts["odom_out"] += 1
            mono = time.monotonic()
            self.last_odom = mono
            self.arrivals.append(mono)
            self.latencies.append((now - utc) / 1e9)

        def _anchor(self, frame, t_ns, position, quaternion):
            """Publish odom -> cuVSLAM's odom frame once odom -> base_link is known at t."""
            try:
                tf = self.tf_buffer.lookup_transform(options.anchor_parent, "base_link", self._Time(nanoseconds=t_ns))
            except self._TransformException:
                return      # retried with the next pose
            t, r = tf.transform.translation, tf.transform.rotation
            p, q = anchor(((t.x, t.y, t.z), (r.x, r.y, r.z, r.w)), (position, quaternion))
            out = self._TransformStamped()
            out.header.stamp = tf.header.stamp
            out.header.frame_id, out.child_frame_id = options.anchor_parent, frame
            out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = p
            out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = q
            # With SLAM off cuVSLAM's map frame coincides with its odom frame, but
            # it still stamps tracking/slam_path and vis/slam_odometry in it.
            same = self._TransformStamped()
            same.header.stamp = tf.header.stamp
            same.header.frame_id, same.child_frame_id = frame, options.map_frame
            same.transform.rotation.w = 1.0
            self.static_tf.sendTransform([out, same])
            self.anchored_after_reset = self.velocity.resets
            self.counts["anchors"] += 1
            self.get_logger().info(f"anchored {frame} (and {options.map_frame}, identity) under {options.anchor_parent}")

        def _status(self, message):
            self.vo_state, self.track_mean_s = int(message.vo_state), float(message.track_execution_time_mean)
            self.last_status = time.monotonic()

        def _diagnose(self):
            DiagnosticArray_, DiagnosticStatus_, KeyValue_ = self._diag_types
            mono = time.monotonic()
            self.arrivals = [t for t in self.arrivals if mono - t <= 5.0]
            rate = len(self.arrivals) / 5.0
            latencies, self.latencies = self.latencies[-150:], []
            latency = sorted(latencies)[len(latencies) // 2] if latencies else None
            state = VO_STATES.get(self.vo_state, "no status") if self.last_status and mono - self.last_status < 3 else "no status"
            if self.clock.model is None:
                level, text = DiagnosticStatus_.WARN, f"Waiting for the D555 clock model ({self.clock.reason})"
            elif state == "failed":
                level, text = DiagnosticStatus_.ERROR, "Visual odometry lost tracking"
            elif self.last_odom is None or mono - self.last_odom > 2.0:
                level, text = DiagnosticStatus_.ERROR if self.last_odom else DiagnosticStatus_.WARN, "No visual odometry"
            elif rate < options.min_rate_hz or (latency is not None and latency > options.max_latency_s):
                level, text = DiagnosticStatus_.WARN, f"Visual odometry degraded: {rate:.1f} Hz, latency {latency * 1e3:.0f} ms" \
                    if latency is not None else f"Visual odometry degraded: {rate:.1f} Hz"
            else:
                level, text = DiagnosticStatus_.OK, f"Tracking at {rate:.1f} Hz, UTC stamps"
            values = {"vo_state": state, "odometry_hz": round(rate, 1),
                      "latency_ms": None if latency is None else round(latency * 1e3, 1),
                      "track_mean_ms": None if self.track_mean_s is None else round(self.track_mean_s * 1e3, 2),
                      "clock_model": self.clock.reason, "velocity_resets": self.velocity.resets, **self.counts}
            out = DiagnosticArray_()
            out.header.stamp = self.get_clock().now().to_msg()
            out.status = [DiagnosticStatus_(level=level, name="vslam", hardware_id="cuVSLAM (D555 stereo IR)",
                                            message=text, values=[KeyValue_(key=k, value=str(v)) for k, v in values.items()])]
            self.diag_pub.publish(out)

    return rclpy, Bridge()


def _spin(node):
    """rclpy's default executor, or the experimental EventsExecutor when
    UAV_EVENTS_EXECUTOR=1 (uav_ansible: ros_events_executor), the A/B of the
    autonomy roadmap's compute-recovery item."""
    import rclpy     # some nodes import it inside main()
    if os.environ.get("UAV_EVENTS_EXECUTOR", "0") == "1":
        try:
            from rclpy.experimental.events_executor import EventsExecutor
        except ImportError:
            EventsExecutor = None
        if EventsExecutor is not None:
            executor = EventsExecutor()
            executor.add_node(node)
            try:
                executor.spin()
            finally:
                executor.remove_node(node)
                executor.shutdown()
            return
    rclpy.spin(node)

def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--odometry-topic", default="/vslam/odometry")
    parser.add_argument("--anchor-parent", default="odom", help="the TF frame cuVSLAM's odom frame is anchored under")
    parser.add_argument("--map-frame", default="vslam_map", help="cuVSLAM's map frame (identity to its odom frame with SLAM off)")
    parser.add_argument("--left-info-in", required=True)
    parser.add_argument("--right-info-in", required=True)
    parser.add_argument("--left-info-out", default="/vslam/infra1/camera_info")
    parser.add_argument("--right-info-out", default="/vslam/infra2/camera_info")
    parser.add_argument("--twist-stddev", type=float, default=0.05, help="m/s, when cuVSLAM gives no twist covariance")
    parser.add_argument("--max-latency-s", type=float, default=0.25)
    parser.add_argument("--min-rate-hz", type=float, default=20.0)
    options, ros_args = parser.parse_known_args()
    rclpy, node = None, None
    try:
        import rclpy
        rclpy.init(args=ros_args)
        rclpy, node = create_bridge_node(options)
        _spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
