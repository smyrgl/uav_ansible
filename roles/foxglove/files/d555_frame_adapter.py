#!/usr/bin/env python3
"""D555 adapter: canonical frame names and UTC stamps for everything under /d555.

The firmware's CDR frame_id strings can contain trailing NUL padding. ROS typed
deserialization may already discard that padding, whereas a generic CDR client
can retain it. Republishing typed messages produces canonical ROS strings.

The D555 stamps in its own hardware clock. This node estimates that clock's
relation to UTC from the camera's IMU stream (d555_clock.py) and republishes
with UTC stamps: the mapped capture time while the model is valid, host
receive time until then. Every /d555 topic is therefore in UTC; /d555/clock
carries the model (and how good it is) for other processes and for the
flight bags. Calibration and image payloads are never changed.
"""

import os
import argparse
import json
import re
import time


def normalize_frame_id(frame_id, expected):
    """Accept the exact expected name, optionally followed by NUL padding."""
    if not isinstance(frame_id, str):
        raise ValueError("frame_id must be a string")
    normalized = frame_id.rstrip("\x00")
    if normalized != expected:
        raise ValueError(f"Unexpected D555 frame_id {frame_id!r}; expected {expected!r}")
    return normalized


def normalize_message_frame(message, expected):
    """Change only header.frame_id, validating before any mutation."""
    normalized = normalize_frame_id(message.header.frame_id, expected)
    message.header.frame_id = normalized
    return message


def set_stamp(message, utc_ns):
    """Replace header.stamp with a UTC time in nanoseconds."""
    message.header.stamp.sec = int(utc_ns // 1_000_000_000)
    message.header.stamp.nanosec = int(utc_ns % 1_000_000_000)
    return message


def header_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def receive_ns(info):
    """The DDS receive timestamp (host UTC) if the middleware gave one."""
    received = (info or {}).get("received_timestamp") or 0
    return int(received) if received > 0 else time.time_ns()


def utc_stamp(model, message, info):
    """Mapped capture time while the clock model is valid, receive time otherwise."""
    mapped = model.to_utc_ns(header_ns(message))
    return (mapped, "capture") if mapped is not None else (receive_ns(info), "receipt")


def create_adapter_node(serial, depth_hz=2.0, clock_window_s=60.0):
    from collections import Counter
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from std_msgs.msg import String
    from d555_clock import ClockModel

    class D555FrameAdapter(Node):
        def __init__(self):
            super().__init__("d555_frame_adapter")
            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            imu_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            latched_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.serial = serial
            self.model = ClockModel(window_s=clock_window_s)
            self.stamped = Counter()
            self._last_valid = None
            self._last_resets = 0
            self._relay_subscriptions = []
            self._relay_publishers = []
            native = f"/realsense/D555_{serial}"

            # The clock source: the IMU, 100 Hz in small packets. Its DDS source
            # timestamp is the camera's clock when it was sent; the receive
            # timestamp is host UTC. Republished with UTC stamps on /d555/imu.
            self._imu_pub = self.create_publisher(Imu, "/d555/imu", imu_qos)
            self._relay_subscriptions.append(
                self.create_subscription(Imu, native + "_Motion", self._imu, imu_qos))

            for stream in ("Color", "Depth"):
                source = f"{native}_{stream}/camera_info"
                expected = f"camera_{stream.lower()}_optical_frame"
                publisher = self.create_publisher(CameraInfo, f"/d555/{stream.lower()}/camera_info", latched_qos)
                self._relay_publishers.append(publisher)

                def relay(message, info, *, publisher=publisher, expected=expected, source=source):
                    if not self._normalized(message, expected, source):
                        return
                    stamp, kind = utc_stamp(self.model, message, info)
                    set_stamp(message, stamp)
                    self.stamped[f"camera_info_{kind}"] += 1
                    publisher.publish(message)

                self._relay_subscriptions.append(
                    self.create_subscription(CameraInfo, source, relay, sensor_qos))

            # Dashboard depth at depth_hz. Raw (serialized) subscription: only the
            # frames kept are deserialized. One raw-depth reader, as before (the
            # camera sends a full copy of each stream per reader).
            self._depth_period_ns = int(1e9 / depth_hz) if depth_hz > 0 else 0
            self._depth_last_ns = 0
            self._depth_pub = self.create_publisher(Image, "/d555/depth/throttled", sensor_qos)
            if self._depth_period_ns:
                self._depth_image_type = Image
                self._deserialize = deserialize_message
                self._relay_subscriptions.append(
                    self.create_subscription(Image, native + "_Depth", self._depth, sensor_qos, raw=True))

            self._clock_pub = self.create_publisher(String, "/d555/clock", latched_qos)
            self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._diag_types = (DiagnosticArray, DiagnosticStatus, KeyValue)
            self._string_type = String
            self.create_timer(1.0, self._solve)
            self.get_logger().info(
                f"D555 adapter ready: clock model from {native}_Motion, UTC stamps on /d555, "
                f"depth relay {depth_hz:g} Hz")

        def _normalized(self, message, expected, source):
            try:
                normalize_message_frame(message, expected)
                return True
            except ValueError as error:
                self.get_logger().warning(f"Dropping {source}: {error}", throttle_duration_sec=5.0)
                return False

        def _imu(self, message, info):
            sent = int((info or {}).get("source_timestamp") or 0)
            self.model.add(sent if sent > 0 else header_ns(message), receive_ns(info))
            if not self._normalized(message, "camera_imu_optical_frame", "D555 IMU"):
                return
            stamp, kind = utc_stamp(self.model, message, info)
            set_stamp(message, stamp)
            self.stamped[f"imu_{kind}"] += 1
            self._imu_pub.publish(message)

        def _depth(self, serialized, info):
            received = receive_ns(info)
            if received - self._depth_last_ns < self._depth_period_ns * 0.9:
                return
            self._depth_last_ns = received
            message = self._deserialize(serialized, self._depth_image_type)
            if not self._normalized(message, "camera_depth_optical_frame", "D555 depth"):
                return
            stamp, kind = utc_stamp(self.model, message, info)
            set_stamp(message, stamp)
            self.stamped[f"depth_{kind}"] += 1
            self._depth_pub.publish(message)

        def _solve(self):
            now = time.time_ns()
            model = self.model.solve(now)
            self._clock_pub.publish(self._string_type(data=json.dumps(model)))
            valid = model["valid"]
            if valid != self._last_valid:
                if valid:
                    self.get_logger().info(
                        f"D555 clock mapped to UTC: skew {model['skew_ppm']:.2f} ppm, "
                        f"bin-minimum residual {model['residual_rms_us']:.0f} us")
                else:
                    self.get_logger().warning(f"D555 clock model not valid: {model['reason']}")
                self._last_valid = valid
            if model["resets"] != self._last_resets:
                self.get_logger().warning("D555 device clock jumped back: camera restarted; new clock model")
                self._last_resets = model["resets"]
            DiagnosticArray, DiagnosticStatus, KeyValue = self._diag_types
            status = DiagnosticStatus(name="d555/clock", hardware_id=self.serial)
            status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
            status.message = (f"D555 clock mapped to UTC from its IMU (skew {model['skew_ppm']:.2f} ppm)"
                              if valid else f"D555 stamps on host receive time: {model['reason']}")
            values = {k: v for k, v in model.items() if k not in ("source", "bias_note")}
            values["timing_validated"] = "true" if valid else "false"
            values.update({f"stamped_{k}": n for k, n in sorted(self.stamped.items())})
            status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
            array = DiagnosticArray(status=[status])
            array.header.stamp.sec, array.header.stamp.nanosec = now // 1_000_000_000, now % 1_000_000_000
            self._diag_pub.publish(array)

    return D555FrameAdapter()


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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="261622302751", help="D555 native topic serial number")
    parser.add_argument("--depth-hz", type=float, default=2.0, help="dashboard depth relay rate (0 disables)")
    parser.add_argument("--clock-window-s", type=float, default=60.0, help="clock model sliding window")
    options, ros_args = parser.parse_known_args()
    if not re.fullmatch(r"[0-9]+", options.serial):
        parser.error("serial must contain digits only")
    if not 0 <= options.depth_hz <= 30 or not 15 <= options.clock_window_s <= 600:
        parser.error("depth-hz must be 0-30 and clock-window-s 15-600")

    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_adapter_node(options.serial, options.depth_hz, options.clock_window_s)
        _spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
