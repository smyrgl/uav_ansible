"""Holybro H-Flow on DroneCAN -> ROS 2, from a listen-only SocketCAN interface.

Topics (prefix /hflow): sensor_optical_flow (px4_msgs/SensorOpticalFlow),
distance_sensor (px4_msgs/DistanceSensor), range (sensor_msgs/Range) and
driver diagnostics on /diagnostics. Stamps are host receipt time (ROS time):
the H-Flow's DroneCAN timestamps are zero (no time sync). The flow integrals
are in the sensor's FRD axes (hflow_nominal_frd_frame), exactly as PX4 would
publish them; use TF or px4_ros_com frame_transforms to reach FLU.
"""
import collections
import math
import threading
import time

import dronecan
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import DistanceSensor, SensorOpticalFlow
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from .mapping import (READING_VALID, RangeSample, flow_fields, px4_device_id,
                      range_fields, stream_rate_hz)


class HFlowObserver(Node):
    def __init__(self):
        super().__init__("uav_hflow")
        defaults = {
            "can_interface": "can0", "can_bitrate": 1000000, "node_id": 124,
            "topic_prefix": "/hflow", "flow_frame_id": "hflow_nominal_frd_frame",
            "range_frame_id": "hflow_nominal_range_frame",
            "range_min_m": 0.08, "range_max_m": 30.0,
            "range_h_fov_rad": math.radians(12.4), "range_v_fov_rad": math.radians(6.2),
            "max_flow_rate_rad_s": 7.4, "distance_max_age_sec": 0.5, "stale_sec": 2.0,
        }
        self.declare_parameters("", list(defaults.items()))
        self.p = {k: self.get_parameter(k).value for k in defaults}
        prefix = str(self.p["topic_prefix"]).rstrip("/")
        self.pub_flow = self.create_publisher(SensorOpticalFlow, prefix + "/sensor_optical_flow", qos_profile_sensor_data)
        self.pub_dist = self.create_publisher(DistanceSensor, prefix + "/distance_sensor", qos_profile_sensor_data)
        self.pub_range = self.create_publisher(Range, prefix + "/range", qos_profile_sensor_data)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.lock = threading.Lock()
        self.receipts = {"flow": collections.deque(maxlen=2048), "range": collections.deque(maxlen=2048)}
        self.last = {"flow": None, "range": None, "status": None, "quality": None, "range_m": None, "reading_type": None}
        self.latest_distance = (None, None)   # (distance_m, monotonic receipt)
        self.frames = {"flow": 0, "range": 0, "status": 0, "other_node": 0}
        self.started = time.monotonic()
        self.device_id = px4_device_id(int(self.p["node_id"]))
        self.can = dronecan.make_node(str(self.p["can_interface"]), bitrate=int(self.p["can_bitrate"]))  # anonymous: never transmits
        self.can.add_handler(dronecan.com.hex.equipment.flow.Measurement, self._on_flow)
        self.can.add_handler(dronecan.uavcan.equipment.range_sensor.Measurement, self._on_range)
        self.can.add_handler(dronecan.uavcan.protocol.NodeStatus, self._on_status)
        self.running = True
        self.thread = threading.Thread(target=self._spin_can, name="dronecan", daemon=True)
        self.thread.start()
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info("H-Flow observer on %s (listen-only), node %d -> %s/*" % (self.p["can_interface"], self.p["node_id"], prefix))

    # --- DroneCAN thread -------------------------------------------------
    def _spin_can(self):
        while self.running:
            try:
                self.can.spin(0.1)
            except Exception as exc:  # keep observing; report through diagnostics
                self.last["error"] = str(exc)
                time.sleep(0.5)

    def _accept(self, event):
        wanted = int(self.p["node_id"])
        if wanted and event.transfer.source_node_id != wanted:
            self.frames["other_node"] += 1
            return False
        return True

    def _on_status(self, event):
        if not self._accept(event):
            return
        m = event.message
        with self.lock:
            self.frames["status"] += 1
            self.last["status"] = {"uptime_sec": m.uptime_sec, "health": m.health, "mode": m.mode,
                                   "vendor_specific_status_code": m.vendor_specific_status_code}

    def _on_range(self, event):
        if not self._accept(event):
            return
        m = event.message
        now = time.monotonic()
        sample = RangeSample(range_m=float(m.range), reading_type=int(m.reading_type),
                             field_of_view_rad=float(m.field_of_view), sensor_type=int(m.sensor_type),
                             sensor_id=int(m.sensor_id))
        range_value, px4_distance, quality = range_fields(sample, float(self.p["range_min_m"]), float(self.p["range_max_m"]))
        stamp = self.get_clock().now()
        rng = Range()
        rng.header.stamp = stamp.to_msg()
        rng.header.frame_id = str(self.p["range_frame_id"])
        rng.radiation_type = Range.INFRARED          # 850 nm laser time-of-flight
        rng.field_of_view = sample.field_of_view_rad
        rng.min_range = float(self.p["range_min_m"])
        rng.max_range = float(self.p["range_max_m"])
        rng.range = range_value
        if hasattr(rng, "variance"):
            rng.variance = 0.0
        dist = DistanceSensor()
        dist.timestamp = stamp.nanoseconds // 1000
        dist.device_id = self.device_id
        dist.min_distance = float(self.p["range_min_m"])
        dist.max_distance = float(self.p["range_max_m"])
        dist.current_distance = px4_distance
        dist.variance = 0.0
        dist.signal_quality = quality
        dist.type = DistanceSensor.MAV_DISTANCE_SENSOR_LASER
        dist.h_fov = float(self.p["range_h_fov_rad"])
        dist.v_fov = float(self.p["range_v_fov_rad"])
        dist.orientation = DistanceSensor.ROTATION_DOWNWARD_FACING
        dist.mode = DistanceSensor.MODE_ENABLED
        self.pub_range.publish(rng)
        self.pub_dist.publish(dist)
        with self.lock:
            self.frames["range"] += 1
            self.receipts["range"].append(now)
            self.last["range"] = now
            self.last["range_m"] = range_value
            self.last["reading_type"] = sample.reading_type
            if sample.reading_type == READING_VALID:
                self.latest_distance = (sample.range_m, now)

    def _on_flow(self, event):
        if not self._accept(event):
            return
        m = event.message
        now = time.monotonic()
        with self.lock:
            distance_m, received = self.latest_distance
        fields = flow_fields(m.integration_interval, m.rate_gyro_integral, m.flow_integral, m.quality,
                             distance_m, None if received is None else now - received,
                             float(self.p["distance_max_age_sec"]))
        stamp = self.get_clock().now()
        flow = SensorOpticalFlow()
        flow.timestamp = stamp.nanoseconds // 1000
        flow.timestamp_sample = flow.timestamp
        flow.device_id = self.device_id
        flow.pixel_flow = fields["pixel_flow"]
        flow.delta_angle = fields["delta_angle"]
        flow.delta_angle_available = fields["delta_angle_available"]
        flow.distance_m = fields["distance_m"]
        flow.distance_available = fields["distance_available"]
        flow.integration_timespan_us = fields["integration_timespan_us"]
        flow.quality = fields["quality"]
        flow.error_count = 0
        flow.max_flow_rate = float(self.p["max_flow_rate_rad_s"])
        flow.min_ground_distance = float(self.p["range_min_m"])
        flow.max_ground_distance = float(self.p["range_max_m"])
        flow.mode = SensorOpticalFlow.MODE_UNKNOWN
        self.pub_flow.publish(flow)
        with self.lock:
            self.frames["flow"] += 1
            self.receipts["flow"].append(now)
            self.last["flow"] = now
            self.last["quality"] = fields["quality"]

    # --- diagnostics (ROS timer) ------------------------------------------
    def _can_state(self):
        base = "/sys/class/net/%s/" % self.p["can_interface"]
        def read(name):
            try:
                with open(base + name) as f:
                    return f.read().strip()
            except OSError:
                return "unknown"
        return {"operstate": read("operstate"), "rx_packets": read("statistics/rx_packets"),
                "rx_dropped": read("statistics/rx_dropped"), "rx_errors": read("statistics/rx_errors")}

    def _diagnostics(self):
        now = time.monotonic()
        stale = float(self.p["stale_sec"])
        with self.lock:
            flow_fresh = self.last["flow"] is not None and now - self.last["flow"] <= stale
            range_fresh = self.last["range"] is not None and now - self.last["range"] <= stale
            values = {
                "can_interface": self.p["can_interface"], "listen_only": True, "node_id": self.p["node_id"],
                "flow_hz": round(stream_rate_hz(self.receipts["flow"], now), 1),
                "range_hz": round(stream_rate_hz(self.receipts["range"], now), 1),
                "flow_messages": self.frames["flow"], "range_messages": self.frames["range"],
                "status_messages": self.frames["status"], "other_node_messages": self.frames["other_node"],
                "last_quality": self.last["quality"], "last_range_m": self.last["range_m"],
                "last_reading_type": self.last["reading_type"],
                "flow_frame_id": self.p["flow_frame_id"], "range_frame_id": self.p["range_frame_id"],
                "timestamps": "host receipt (ROS time); DroneCAN timestamps are zero",
                "last_error": self.last.get("error"),
            }
            status = self.last["status"]
        if status:
            values.update({"node_" + k: v for k, v in status.items()})
        values.update({"can_" + k: v for k, v in self._can_state().items()})
        grace = now - self.started < 10.0
        if values["can_operstate"] not in ("up", "unknown"):
            level, message = DiagnosticStatus.ERROR, "CAN interface is not up"
        elif flow_fresh and range_fresh:
            level, message = DiagnosticStatus.OK, "Flow and range live"
        elif flow_fresh or range_fresh:
            level, message = DiagnosticStatus.WARN, "One of flow/range is stale"
        else:
            level, message = (DiagnosticStatus.WARN, "Waiting for H-Flow frames") if grace else (DiagnosticStatus.ERROR, "No H-Flow frames")
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [DiagnosticStatus(level=level, name="hflow/driver", message=message,
                                       hardware_id="H-Flow DroneCAN node %d" % int(self.p["node_id"]),
                                       values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])]
        self.pub_diag.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            self.can.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HFlowObserver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
