"""PX4 uORB (/fmu/out, px4_msgs) -> ROS-native messages in ROS frames.

Republishes what the ROS side needs from the flight controller, converted from
PX4's NED/FRD conventions to ENU/FLU (REP-103), stamped with PX4's XRCE-synced
epoch timestamps. GNSS data from PX4 is deliberately not republished: the
Jetson has its own receiver with full covariances (gnss_ros role).
"""
import collections
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from px4_msgs import msg as px4
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool

from .convert import (ARMING_STATE, NAV_STATE, battery_fields, imu_fields, local_position_fields,
                      odometry_fields, stamp_from_px4)
from .frames import finite_or_nan


def _versioned(name, msg_type):
    version = getattr(msg_type, "MESSAGE_VERSION", 0)
    return "/fmu/out/" + name + (f"_v{version}" if version else "")


class Px4Bridge(Node):
    def __init__(self):
        super().__init__("uav_px4_bridge")
        defaults = {
            "prefix": "/px4", "imu_frame_id": "fmu_housing_link", "odom_frame_id": "odom",
            "base_frame_id": "base_link", "gyro_stddev_rad_s": 0.01, "accel_stddev_m_s2": 0.1,
            "attitude_max_age_sec": 0.5, "stale_sec": 3.0,
        }
        self.declare_parameters("", list(defaults.items()))
        self.p = {k: self.get_parameter(k).value for k in defaults}
        prefix = str(self.p["prefix"]).rstrip("/")
        out = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = {
            "imu": self.create_publisher(Imu, prefix + "/imu/data", out),
            "imu_raw": self.create_publisher(Imu, prefix + "/imu/data_raw", out),
            "odom": self.create_publisher(Odometry, prefix + "/odometry", out),
            "lp_pose": self.create_publisher(PoseWithCovarianceStamped, prefix + "/local_position/pose", out),
            "lp_twist": self.create_publisher(TwistWithCovarianceStamped, prefix + "/local_position/twist", out),
            "lp_origin": self.create_publisher(NavSatFix, prefix + "/local_position/origin", out),
            "battery": self.create_publisher(BatteryState, prefix + "/battery", out),
            "home": self.create_publisher(GeoPointStamped, prefix + "/home", out),
            "landed": self.create_publisher(Bool, prefix + "/landed", out),
            "diag": self.create_publisher(DiagnosticArray, "/diagnostics", 10),
        }
        self.attitude = None          # (q, monotonic receipt)
        self.orientation_variance = None
        self.rates = collections.defaultdict(lambda: collections.deque(maxlen=512))
        self.state = {"synced": None, "arming": None, "nav": None, "failsafe": None, "landed": None,
                      "timesync_rtt_us": None, "estimator": {}, "last_error": None}
        subs = [("vehicle_attitude", px4.VehicleAttitude, self._attitude),
                ("sensor_combined", px4.SensorCombined, self._sensor_combined),
                ("vehicle_odometry", px4.VehicleOdometry, self._odometry),
                ("vehicle_local_position", px4.VehicleLocalPosition, self._local_position),
                ("battery_status", px4.BatteryStatus, self._battery),
                ("vehicle_status", px4.VehicleStatus, self._status),
                ("vehicle_land_detected", px4.VehicleLandDetected, self._landed),
                ("home_position", px4.HomePosition, self._home),
                ("estimator_status_flags", px4.EstimatorStatusFlags, self._estimator),
                ("timesync_status", px4.TimesyncStatus, self._timesync)]
        self.topics = {}
        for name, msg_type, cb in subs:
            topic = _versioned(name, msg_type)
            self.topics[name] = topic
            self.create_subscription(msg_type, topic, cb, qos_profile_sensor_data)
        self.started = time.monotonic()
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info("PX4 bridge: %d uORB topics -> %s/* (ENU/FLU)" % (len(subs), prefix))

    # --- helpers -----------------------------------------------------------
    def _stamp(self, timestamp_us):
        ns, synced = stamp_from_px4(timestamp_us, self.get_clock().now().nanoseconds)
        self.state["synced"] = synced
        s = rclpy.time.Time(nanoseconds=ns).to_msg()
        return s

    def _tick(self, name):
        self.rates[name].append(time.monotonic())

    def _rate(self, name, now, window=5.0):
        recent = [t for t in self.rates[name] if t >= now - window]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return round((len(recent) - 1) / span, 1) if span > 0 else 0.0

    # --- callbacks ---------------------------------------------------------
    def _attitude(self, msg):
        self._tick("vehicle_attitude")
        if all(math.isfinite(v) for v in msg.q):
            self.attitude = (tuple(float(v) for v in msg.q), time.monotonic())

    def _sensor_combined(self, msg):
        self._tick("sensor_combined")
        stamp = self._stamp(msg.timestamp)
        raw = imu_fields(msg.gyro_rad, msg.accelerometer_m_s2, None, None,
                         float(self.p["gyro_stddev_rad_s"]), float(self.p["accel_stddev_m_s2"]))
        self.pub["imu_raw"].publish(self._imu_msg(stamp, raw))
        att = self.attitude
        if att is not None and time.monotonic() - att[1] <= float(self.p["attitude_max_age_sec"]):
            full = imu_fields(msg.gyro_rad, msg.accelerometer_m_s2, att[0], self.orientation_variance,
                              float(self.p["gyro_stddev_rad_s"]), float(self.p["accel_stddev_m_s2"]))
            self.pub["imu"].publish(self._imu_msg(stamp, full))

    def _imu_msg(self, stamp, f):
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = str(self.p["imu_frame_id"])
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = f["orientation"]
        m.orientation_covariance = f["orientation_covariance"]
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = f["angular_velocity"]
        m.angular_velocity_covariance = f["angular_velocity_covariance"]
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = f["linear_acceleration"]
        m.linear_acceleration_covariance = f["linear_acceleration_covariance"]
        return m

    def _odometry(self, msg):
        self._tick("vehicle_odometry")
        if not all(math.isfinite(v) for v in msg.q):
            return
        self.orientation_variance = list(msg.orientation_variance)
        try:
            f = odometry_fields(finite_or_nan(msg.position), msg.q, finite_or_nan(msg.velocity), int(msg.velocity_frame),
                                finite_or_nan(msg.angular_velocity), msg.position_variance, msg.orientation_variance,
                                msg.velocity_variance, int(msg.pose_frame))
        except ValueError as exc:
            self.state["last_error"] = str(exc)
            return
        m = Odometry()
        m.header.stamp = self._stamp(msg.timestamp_sample or msg.timestamp)
        m.header.frame_id = str(self.p["odom_frame_id"])
        m.child_frame_id = str(self.p["base_frame_id"])
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = f["position"]
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = f["orientation"]
        m.pose.covariance = f["pose_covariance"]
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = f["linear"]
        m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = f["angular"]
        m.twist.covariance = f["twist_covariance"]
        self.pub["odom"].publish(m)

    def _local_position(self, msg):
        self._tick("vehicle_local_position")
        f = local_position_fields(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz,
                                  msg.xy_valid, msg.z_valid, msg.v_xy_valid, msg.v_z_valid)
        stamp = self._stamp(msg.timestamp_sample or msg.timestamp)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = str(self.p["odom_frame_id"])
        pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = f["position"]
        att = self.attitude
        if att is not None:
            from .frames import px4_to_ros_orientation
            (pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
             pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) = px4_to_ros_orientation(att[0])
        cov = [0.0] * 36
        cov[0] = cov[7] = float(msg.eph) ** 2 if hasattr(msg, "eph") else 0.0
        cov[14] = float(msg.epv) ** 2 if hasattr(msg, "epv") else 0.0
        cov[35] = float(msg.heading_var)
        pose.pose.covariance = cov
        self.pub["lp_pose"].publish(pose)
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = str(self.p["odom_frame_id"])
        twist.twist.twist.linear.x, twist.twist.twist.linear.y, twist.twist.twist.linear.z = f["linear"]
        self.pub["lp_twist"].publish(twist)
        if msg.xy_global and msg.z_global:
            origin = NavSatFix()
            origin.header.stamp = stamp
            origin.header.frame_id = str(self.p["odom_frame_id"])
            origin.status.status = NavSatStatus.STATUS_FIX
            origin.status.service = NavSatStatus.SERVICE_GPS
            origin.latitude, origin.longitude, origin.altitude = float(msg.ref_lat), float(msg.ref_lon), float(msg.ref_alt)
            origin.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub["lp_origin"].publish(origin)

    def _battery(self, msg):
        self._tick("battery_status")
        f = battery_fields(msg.voltage_v, msg.current_a, msg.remaining, msg.capacity, msg.discharged_mah,
                           msg.temperature, msg.cell_count, list(msg.voltage_cell_v), msg.connected, msg.warning,
                           msg.time_remaining_s)
        m = BatteryState()
        m.header.stamp = self._stamp(msg.timestamp)
        m.header.frame_id = str(self.p["base_frame_id"])
        m.voltage, m.current, m.charge, m.capacity, m.design_capacity, m.percentage = (
            f["voltage"], f["current"], f["charge"], f["capacity"], f["design_capacity"], f["percentage"])
        m.power_supply_status, m.power_supply_health, m.power_supply_technology = (
            f["power_supply_status"], f["power_supply_health"], f["power_supply_technology"])
        m.present = f["present"]
        m.cell_voltage = f["cell_voltage"]
        m.temperature = f["temperature"]
        m.location = "px4 battery %d" % int(msg.id)
        m.serial_number = ""
        self.pub["battery"].publish(m)
        self.state["battery"] = {"warning": f["warning"], "percentage": f["percentage"], "voltage": f["voltage"],
                                 "time_remaining_s": f["time_remaining_s"]}

    def _status(self, msg):
        self._tick("vehicle_status")
        self.state["arming"] = ARMING_STATE.get(int(msg.arming_state), str(msg.arming_state))
        self.state["nav"] = NAV_STATE.get(int(msg.nav_state), str(msg.nav_state))
        self.state["failsafe"] = bool(msg.failsafe)
        self.state["gcs_connection_lost"] = bool(msg.gcs_connection_lost)

    def _landed(self, msg):
        self._tick("vehicle_land_detected")
        self.state["landed"] = bool(msg.landed)
        self.pub["landed"].publish(Bool(data=bool(msg.landed)))

    def _home(self, msg):
        self._tick("home_position")
        if not (msg.valid_hpos and msg.valid_alt):
            return
        m = GeoPointStamped()
        m.header.stamp = self._stamp(msg.timestamp)
        m.header.frame_id = "wgs84"
        m.position.latitude, m.position.longitude, m.position.altitude = float(msg.lat), float(msg.lon), float(msg.alt)
        self.pub["home"].publish(m)

    def _estimator(self, msg):
        self._tick("estimator_status_flags")
        self.state["estimator"] = {k: bool(getattr(msg, k)) for k in (
            "cs_tilt_align", "cs_yaw_align", "cs_gnss_pos", "cs_opt_flow", "cs_rng_hgt", "cs_baro_hgt",
            "cs_in_air", "cs_mag_fault", "cs_inertial_dead_reckoning", "cs_vehicle_at_rest") if hasattr(msg, k)}

    def _timesync(self, msg):
        self._tick("timesync_status")
        self.state["timesync_rtt_us"] = int(msg.round_trip_time)

    # --- diagnostics -------------------------------------------------------
    def _diagnostics(self):
        now = time.monotonic()
        rates = {name: self._rate(name, now) for name in self.topics}
        live = {name: rate > 0 for name, rate in rates.items()}
        values = {("rate_hz/" + n): r for n, r in rates.items()}
        values.update({"stamps_epoch_synced": self.state["synced"], "arming": self.state["arming"], "nav_state": self.state["nav"],
                       "failsafe": self.state["failsafe"], "landed": self.state["landed"],
                       "timesync_rtt_us": self.state["timesync_rtt_us"], "last_error": self.state["last_error"]})
        values.update({"estimator/" + k: v for k, v in self.state["estimator"].items()})
        values.update({"battery/" + k: v for k, v in self.state.get("battery", {}).items()})
        n_live = sum(live.values())
        if n_live == 0:
            level, msg = (DiagnosticStatus.WARN, "Waiting for PX4 uORB samples") if now - self.started < 10 else (DiagnosticStatus.ERROR, "No PX4 uORB samples")
        elif not live.get("sensor_combined") or not live.get("vehicle_attitude"):
            level, msg = DiagnosticStatus.WARN, "IMU or attitude stream missing (%d/%d topics live)" % (n_live, len(live))
        elif self.state["synced"] is False:
            level, msg = DiagnosticStatus.WARN, "PX4 timestamps are boot time, not epoch (XRCE timesync not applied)"
        else:
            level, msg = DiagnosticStatus.OK, "%d/%d uORB topics live; ENU/FLU republished" % (n_live, len(live))
        d = DiagnosticArray()
        d.header.stamp = self.get_clock().now().to_msg()
        d.status = [DiagnosticStatus(level=level, name="px4/bridge", message=msg, hardware_id="PX4 via XRCE-DDS",
                                     values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])]
        self.pub["diag"].publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Px4Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
