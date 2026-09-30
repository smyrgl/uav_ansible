#!/usr/bin/env python3
"""Normalize D555 native frame-name padding for visualization only.

The firmware's CDR frame_id strings can contain trailing NUL padding. ROS typed
deserialization may already discard that padding, whereas a generic CDR client
can retain it. Republishing typed messages produces canonical ROS strings. This
adapter never changes device capture stamps, calibration, or any image payloads.
"""

import argparse
import re


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


def create_adapter_node(serial):
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo

    class D555FrameAdapter(Node):
        def __init__(self):
            super().__init__("d555_frame_adapter")
            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            calibration_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._relay_subscriptions = []
            self._relay_publishers = []
            for stream in ("Color", "Depth"):
                native = f"/realsense/D555_{serial}_{stream}"
                destination = f"/d555/{stream.lower()}"
                expected = f"camera_{stream.lower()}_optical_frame"
                for message_type, source, output, output_qos in (
                    (CameraInfo, native + "/camera_info", destination + "/camera_info", calibration_qos),
                ):
                    publisher = self.create_publisher(message_type, output, output_qos)
                    self._relay_publishers.append(publisher)

                    def relay(message, publisher=publisher, expected=expected, source=source):
                        try:
                            normalize_message_frame(message, expected)
                        except ValueError as error:
                            self.get_logger().warning(
                                f"Dropping {source}: {error}", throttle_duration_sec=5.0
                            )
                            return
                        publisher.publish(message)

                    self._relay_subscriptions.append(
                        self.create_subscription(message_type, source, relay, sensor_qos)
                    )
            self.get_logger().info(
                "D555 CameraInfo frame adapter ready; device capture timestamps remain unchanged"
            )

    return D555FrameAdapter()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="261622302751", help="D555 native topic serial number")
    options, ros_args = parser.parse_known_args()
    if not re.fullmatch(r"[0-9]+", options.serial):
        parser.error("serial must contain digits only")

    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_adapter_node(options.serial)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

