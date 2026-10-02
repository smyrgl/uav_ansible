"""One current diagnostic summary per sensor, with grouped supporting evidence."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, PointCloud2, Range
from px4_msgs import msg as px4_messages
from .dds import TOPICS, topic_name, dds_health

from .core import ImageMetadata, StreamMonitor, OK
from .health import Sample, check, d555_timing, grouped, lidar_health, px4_health, hflow_health, jetson_health, jetson_identity
from .observers import Observers, gnss_health


class SensorHealth(Node):
    def __init__(self):
        super().__init__("uav_sensor_health")
        defaults = {
            "d555_serial": "261622302751",
            "d555_color_topic": "/realsense/D555_261622302751_Color",
            "d555_depth_topic": "/realsense/D555_261622302751_Depth",
            # Each stream is watched through its CameraInfo (same header as the
            # image, at ~0.3 kB instead of ~2 MB a frame): the D555 unicasts a
            # full image copy to every subscriber, so the observer must not be one.
            "d555_camera_info_suffix": "/camera_info",
            "avia_topic": "/avia/points", "e1r_topic": "/e1r/points",
            "receipt_timeout_sec": 2.0, "startup_grace_sec": 10.0,
            "min_rate_hz": 15.0, "rate_window_sec": 5.0, "clock_tolerance_sec": 2.0,
            "mavlink_endpoint": "tcp:127.0.0.1:5760", "mavlink_source_system": 254,
            "hflow_flow_topic": "/hflow/sensor_optical_flow", "hflow_range_topic": "/hflow/range",
            "hflow_min_quality": 20,
            "gnss_broker_port": 28785, "ptp_status_path": "/run/uav/time/ptp-status.json",
            "gnss_require_rtk_fixed": True,
        }
        self.declare_parameters("", list(defaults.items()))
        self.params = params = {key: self.get_parameter(key).value for key in defaults}
        self.started = time.monotonic()
        self.hardware_id = str(params["d555_serial"])
        self.monitors, self.topics, self.subs = {}, {}, []
        self.clouds = {kind: Sample() for kind in ("avia", "e1r")}
        self.drivers = {}
        self.jetson = {}   # isaac_ros_jetson_stats rows: name -> (received, level, message, values, hardware_id)
        self.dds_samples, self.dds_topics = {}, {}
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        for name, type_name, _ in TOPICS:
            msg_type = getattr(px4_messages, type_name)
            topic = topic_name(name, msg_type)
            self.dds_topics[name] = topic
            self.dds_samples[name] = Sample()
            self.subs.append(self.create_subscription(msg_type, topic,
                lambda msg, key=name: self._dds(key, msg), qos))
        for stream in ("color", "depth"):
            if not params[f"d555_{stream}_topic"]:
                continue        # an empty topic: not monitored (depth since 2026-10-02)
            self.monitors[stream] = StreamMonitor(self.started,
                timeout_sec=params["receipt_timeout_sec"], startup_grace_sec=params["startup_grace_sec"],
                min_rate_hz=params["min_rate_hz"], rate_window_sec=params["rate_window_sec"],
                clock_tolerance_sec=params["clock_tolerance_sec"])
            topic = str(params[f"d555_{stream}_topic"]) + str(params["d555_camera_info_suffix"])
            self.topics[stream] = topic
            self.subs.append(self.create_subscription(CameraInfo, topic,
                                                       lambda msg, key=stream: self._camera_info(key, msg), qos))
        for kind in self.clouds:
            self.subs.append(self.create_subscription(PointCloud2, params[f"{kind}_topic"],
                                                       lambda msg, key=kind: self._cloud(key, msg), qos))
        # H-Flow straight from the CAN listener (uav-hflow), not through PX4.
        self.hflow = {"flow": Sample(), "range": Sample()}
        self.subs.append(self.create_subscription(px4_messages.SensorOpticalFlow, str(params["hflow_flow_topic"]), self._hflow_flow, qos))
        self.subs.append(self.create_subscription(Range, str(params["hflow_range_topic"]), self._hflow_range, qos))
        self.subs.append(self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostics, 20))
        self.publisher = self.create_publisher(DiagnosticArray, "/uav/health", 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.observers = Observers(params)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0, self._publish, clock=self.steady_clock)
        self.get_logger().info("Sensor summaries on /uav/health; ERROR reserved for disconnected data links")

    def destroy_node(self):
        self.observers.close()
        return super().destroy_node()

    def _dds(self, name, msg):
        values = {"source_timestamp_us": msg.timestamp}
        if name == "timesync_status":
            values.update(source_protocol=msg.source_protocol,
                          round_trip_time_us=msg.round_trip_time,
                          observed_offset_us=msg.observed_offset,
                          estimated_offset_us=msg.estimated_offset)
        self.dds_samples[name].observe(time.monotonic(), values, msg.timestamp)

    def _camera_info(self, stream, msg):
        self.monitors[stream].observe(ImageMetadata(
            stamp_ns=msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
            width=msg.width, height=msg.height, frame_id=msg.header.frame_id), time.monotonic())

    def _cloud(self, kind, msg):
        self.clouds[kind].observe(time.monotonic(), {
            "topic": self.params[f"{kind}_topic"], "points": msg.width * msg.height,
            "frame_id": msg.header.frame_id,
            "layout_valid": (msg.point_step > 0 and msg.row_step >= msg.width * msg.point_step
                             and len(msg.data) == msg.row_step * msg.height),
            "content_accuracy_verified": False,
        }, msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)

    def _hflow_flow(self, msg):
        finite = all(math.isfinite(v) for v in msg.pixel_flow) and all(math.isfinite(v) for v in msg.delta_angle[:2])
        self.hflow["flow"].observe(time.monotonic(), {
            "quality": int(msg.quality), "integration_timespan_us": int(msg.integration_timespan_us),
            "pixel_flow_finite": finite, "distance_available": bool(msg.distance_available),
            "distance_m": float(msg.distance_m), "source_timestamp_us": int(msg.timestamp)}, int(msg.timestamp))

    def _hflow_range(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.hflow["range"].observe(time.monotonic(), {
            "range_m": float(msg.range), "min_range": float(msg.min_range), "max_range": float(msg.max_range),
            "field_of_view_rad": float(msg.field_of_view), "frame_id": msg.header.frame_id}, stamp)

    def _diagnostics(self, msg):
        for status in msg.status:
            level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
            if status.name.startswith("jetson_stats/"):
                self.jetson[status.name] = (time.monotonic(), level, status.message,
                                            {v.key: v.value for v in status.values}, status.hardware_id)
                continue
            if status.name not in ("avia/driver", "avia/clock", "e1r/driver", "hflow/driver", "d555/clock"):
                continue  # including our own summaries: no feedback loop
            self.drivers.setdefault(status.name, Sample()).observe(time.monotonic(), {
                **{v.key: v.value for v in status.values}, "level": level,
                "message": status.message, "hardware_id": status.hardware_id})

    @staticmethod
    def _status(name, hardware_id, assessment):
        return DiagnosticStatus(level=bytes([assessment.level]), name=name, hardware_id="",
            message=assessment.message,
            values=[KeyValue(key="Identity/source_id", value=hardware_id)] +
                   [KeyValue(key=str(k), value=str(v)) for k, v in assessment.values.items()])

    def _d555(self, now, wall):
        sections, live = {}, []
        model = self.drivers.get("d555/clock", Sample())
        for name, monitor in self.monitors.items():
            stream, clock = monitor.assess(now, wall)
            stream.values["topic"] = self.topics[name]
            sections[name.title()] = stream
            sections[name.title() + " timing"] = d555_timing(clock, model, now)
            live.append(monitor.last_received is not None and now - monitor.last_received <= monitor.timeout_sec)
        seen = any(m.total for m in self.monitors.values())
        connection = True if any(live) else False if seen else None
        names = [name.title() for name in self.monitors]
        timing = ("UTC via the D555 clock model" if all(sections[k + " timing"].level == OK for k in names)
                  else "timing unverified")
        streams = " + ".join({"color": "RGB", "depth": "depth"}[name] for name in self.monitors)
        message = f"{streams} live; {timing}" if all(live) else "Connected; one stream missing" if any(live) else (
            "D555 data connection lost" if seen else "Unknown; no D555 frames observed")
        if any(live) and any(sections[k].level != OK for k in names):
            message = f"Frames arriving; stream degraded; {timing}"
        return grouped(connection, message, sections)

    def _publish(self):
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        now = time.monotonic()
        samples, transport, gnss_transport, timing = self.observers.snapshot()
        results = [("D555 Camera", self.hardware_id, self._d555(now, time.time()))]
        for kind, label in (("avia", "Avia"), ("e1r", "E1R")):
            driver = self.drivers.get(f"{kind}/driver", Sample())
            assessment = lidar_health(kind, self.clouds[kind], driver, self.drivers.get(f"{kind}/clock", Sample()), now)
            if kind == "e1r":
                ptp = timing.get("checks", {}).get("PTP", check(False, "No PHC observation"))
                if now - timing.get("received", -100) > 6:
                    ptp = check(False, "PHC observation stale")
                assessment.values.update({f"Host PTP/{k}": v for k, v in {"message": ptp.message, **ptp.values}.items()})
            results.append((f"{label} LiDAR", driver.values.get("hardware_id", label), assessment))
        results.extend([
            ("H-FLOW Landing Sensor", str(self.drivers.get("hflow/driver", Sample()).values.get("hardware_id", "H-Flow DroneCAN")),
             hflow_health(self.hflow["flow"], self.hflow["range"], self.drivers.get("hflow/driver", Sample()), now, self.params["hflow_min_quality"])),
            ("GNSS Receiver", "Septentrio", gnss_health(samples, timing, gnss_transport, now, bool(self.params["gnss_require_rtk_fixed"]))),
            ("Hadron Thermal Camera", "Hadron 640R+", grouped(None, "Not integrated; runtime health unknown", {
                "Integration": check(False, "No thermal driver/health source configured")})),
            ("PX4 / MAVLink", "MAVLink 1/1", px4_health(samples, transport, now, self.started, self.params["startup_grace_sec"])),
            ("Jetson Companion", jetson_identity(self.jetson), jetson_health(self.jetson, now)),
        ])
        results.append(("PX4 / DDS", "PX4 1.17", dds_health(
            self.dds_samples, self.dds_topics, now, self.started, self.params["startup_grace_sec"],
            middleware=get_rmw_implementation_identifier())))
        output.status = [self._status(*row) for row in results]
        self.publisher.publish(output)
        self.diagnostics_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SensorHealth()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
