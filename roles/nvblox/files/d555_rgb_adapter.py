#!/usr/bin/env python3
"""Rectified rgb8 colour for nvblox from the D555's native yuv422_yuy2 stream.

Why this node exists:

* nvblox 4.6 integrates colour only from ``rgb8`` or ``bgra8`` images. The D555's
  native DDS colour stream is ``yuv422_yuy2``.
* nvblox models every camera as a pure pinhole: it reads ``CameraInfo.k`` and
  ignores ``d``. The D555 colour intrinsics carry about 5 % radial distortion, so
  the image is undistorted onto the camera's own K and republished together with a
  zero-distortion CameraInfo.
* nvblox pairs image and CameraInfo with an ExactTime synchroniser. Both outputs
  therefore carry the *image's* stamp, independent of the camera's own CameraInfo
  cadence.

Intrinsics come from the frame-validated ``/d555/color/camera_info`` relay, never
from the native topic: the camera interleaves depth and colour intrinsics on both
of its native CameraInfo topics (roles/foxglove/README.md).

Device capture stamps are copied unchanged. Nothing here claims synchronisation
with any other sensor; the stamps are the D555's own clock.
"""

import argparse
import array
import collections
import math
import re
import time

import cv2
import numpy as np

EXPECTED_FRAME = "camera_color_optical_frame"
SUPPORTED_ENCODINGS = ("yuv422_yuy2", "rgb8")


def canonical_frame(frame_id, expected=EXPECTED_FRAME):
    """Accept the expected frame name, optionally NUL padded; reject anything else."""
    if not isinstance(frame_id, str):
        raise ValueError("frame_id must be a string")
    if frame_id.rstrip("\x00") != expected:
        raise ValueError(f"unexpected frame_id {frame_id!r}; expected {expected!r}")
    return expected


def pinhole_from_k(k):
    """Validate a row-major 3x3 pinhole matrix and return (fx, fy, cx, cy)."""
    values = [float(v) for v in k]
    if len(values) != 9 or not all(math.isfinite(v) for v in values):
        raise ValueError("K must hold nine finite values")
    fx, skew, cx, zero, fy, cy = values[:6]
    if fx <= 0.0 or fy <= 0.0 or skew != 0.0 or zero != 0.0 or values[6:] != [0.0, 0.0, 1.0]:
        raise ValueError("K is not a pinhole intrinsic matrix")
    return fx, fy, cx, cy


def rectified_camera_info_fields(k, width, height):
    """CameraInfo fields for an image undistorted onto K (zero distortion, P = [K|0])."""
    fx, fy, cx, cy = pinhole_from_k(k)
    width, height = int(width), int(height)
    if width <= 0 or height <= 0:
        raise ValueError("image size must be positive")
    return {
        "height": height,
        "width": width,
        "distortion_model": "plumb_bob",
        "d": [0.0, 0.0, 0.0, 0.0, 0.0],
        "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }


def yuy2_to_rgb(data, width, height, step):
    """Convert a packed YUY2 (Y0 U0 Y1 V0) payload to an (h, w, 3) uint8 RGB array."""
    width, height, step = int(width), int(height), int(step)
    if width <= 0 or height <= 0 or step != 2 * width:
        raise ValueError(f"yuv422_yuy2 step must be 2*width; got step {step} for width {width}")
    if len(data) != step * height:
        raise ValueError(f"payload holds {len(data)} bytes, expected {step * height}")
    frame = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 2)
    return cv2.cvtColor(frame, cv2.COLOR_YUV2RGB_YUY2)


def rgb8_view(data, width, height, step):
    """View an rgb8 payload as an (h, w, 3) uint8 array (pass-through encoding)."""
    width, height, step = int(width), int(height), int(step)
    if width <= 0 or height <= 0 or step != 3 * width or len(data) != step * height:
        raise ValueError("rgb8 payload does not match width, height and step")
    return np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)


class Rectifier:
    """Undistort onto the camera's own K, so the published K is unchanged."""

    def __init__(self, k, d, width, height):
        pinhole_from_k(k)
        dist = np.asarray([float(v) for v in d], dtype=np.float64)
        if dist.size not in (0, 4, 5, 8, 12, 14) or not np.all(np.isfinite(dist)):
            raise ValueError(f"unsupported distortion vector of length {dist.size}")
        self.width, self.height = int(width), int(height)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("image size must be positive")
        self.identity = dist.size == 0 or not np.any(dist)
        if not self.identity:
            k_matrix = np.asarray([float(v) for v in k], dtype=np.float64).reshape(3, 3)
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                k_matrix, dist, np.eye(3), k_matrix, (self.width, self.height), cv2.CV_16SC2
            )

    def apply(self, rgb):
        if rgb.shape[0] != self.height or rgb.shape[1] != self.width:
            raise ValueError(
                f"image is {rgb.shape[1]}x{rgb.shape[0]}, calibration is {self.width}x{self.height}"
            )
        if self.identity:
            return rgb
        return cv2.remap(rgb, self.map1, self.map2, cv2.INTER_LINEAR)


def create_adapter_node(options):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    class D555RgbAdapter(Node):
        def __init__(self):
            super().__init__("d555_rgb_adapter")
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
            self._serial = options.serial
            self._period_ns = int(1e9 / options.max_rate) if options.max_rate > 0 else 0
            self._rectifier = None
            self._info_fields = None
            self._info_key = None
            self._encoding = ""
            self._last_out_stamp_ns = None
            self._frames_in = 0
            self._frames_out = 0
            self._prev_in = 0
            self._prev_out = 0
            self._last_in = None
            self._last_out = None
            self._started = time.monotonic()
            self._prev_diag = self._started
            self._drops = collections.Counter()
            self._last_error = ""
            namespace = options.output_namespace.rstrip("/")
            self._image_pub = self.create_publisher(Image, f"{namespace}/image", sensor_qos)
            self._info_pub = self.create_publisher(CameraInfo, f"{namespace}/camera_info", sensor_qos)
            self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self.create_subscription(CameraInfo, options.camera_info_topic, self._on_camera_info, calibration_qos)
            self.create_subscription(Image, options.image_topic, self._on_image, sensor_qos)
            self.create_timer(1.0, self._publish_diagnostics)
            self.get_logger().info(
                f"{options.image_topic} ({'/'.join(SUPPORTED_ENCODINGS)}) -> {namespace}/image rgb8, "
                f"undistorted onto K from {options.camera_info_topic}, at most {options.max_rate} Hz; "
                "device capture stamps are preserved unchanged"
            )

        def _drop(self, kind, text):
            self._drops[kind] += 1
            self._last_error = f"{kind}: {text}"
            self.get_logger().warning(f"Dropping ({kind}): {text}", throttle_duration_sec=5.0)

        def _on_camera_info(self, message):
            try:
                canonical_frame(message.header.frame_id)
                key = (tuple(message.k), tuple(message.d), message.width, message.height)
                if key != self._info_key:
                    self._rectifier = Rectifier(message.k, message.d, message.width, message.height)
                    self._info_fields = rectified_camera_info_fields(message.k, message.width, message.height)
                    self._info_key = key
                    fx, fy, cx, cy = pinhole_from_k(message.k)
                    self.get_logger().info(
                        f"Colour intrinsics {message.width}x{message.height} fx={fx:.2f} fy={fy:.2f} "
                        f"cx={cx:.2f} cy={cy:.2f} d={[round(float(v), 5) for v in message.d]}; "
                        f"rectification {'active' if not self._rectifier.identity else 'not needed'}"
                    )
            except ValueError as error:
                self._drop("camera_info", str(error))

        def _on_image(self, message):
            self._frames_in += 1
            self._last_in = time.monotonic()
            self._encoding = message.encoding
            if self._rectifier is None:
                self._drop("no_intrinsics", "waiting for validated colour CameraInfo")
                return
            try:
                canonical_frame(message.header.frame_id)
                if message.encoding == "yuv422_yuy2":
                    rgb = yuy2_to_rgb(message.data, message.width, message.height, message.step)
                elif message.encoding == "rgb8":
                    rgb = rgb8_view(message.data, message.width, message.height, message.step)
                else:
                    raise ValueError(f"unsupported encoding {message.encoding!r}")
                rgb = self._rectifier.apply(rgb)
            except ValueError as error:
                self._drop("image", str(error))
                return
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if self._period_ns and self._last_out_stamp_ns is not None:
                elapsed = stamp_ns - self._last_out_stamp_ns
                if 0 <= elapsed < 0.9 * self._period_ns:
                    return  # rate limit on device stamps; a stamp regression passes through
            out = Image()
            out.header.stamp = message.header.stamp
            out.header.frame_id = EXPECTED_FRAME
            out.height, out.width = int(rgb.shape[0]), int(rgb.shape[1])
            out.encoding = "rgb8"
            out.is_bigendian = 0
            out.step = 3 * out.width
            out.data = array.array("B", np.ascontiguousarray(rgb).tobytes())
            info = CameraInfo()
            info.header = out.header
            for name, value in self._info_fields.items():
                setattr(info, name, value)
            self._image_pub.publish(out)
            self._info_pub.publish(info)
            self._frames_out += 1
            self._last_out_stamp_ns = stamp_ns
            self._last_out = time.monotonic()

        def _publish_diagnostics(self):
            now = time.monotonic()
            dt = max(now - self._prev_diag, 1e-3)
            in_hz = (self._frames_in - self._prev_in) / dt
            out_hz = (self._frames_out - self._prev_out) / dt
            self._prev_in, self._prev_out, self._prev_diag = self._frames_in, self._frames_out, now
            in_age = now - self._last_in if self._last_in is not None else -1.0
            out_age = now - self._last_out if self._last_out is not None else -1.0
            grace = now - self._started < 10.0
            expected_period = max(2.0, 2.0 * self._period_ns / 1e9 if self._period_ns else 2.0)
            status = DiagnosticStatus(name="d555/rgb_adapter", hardware_id=f"D555 {self._serial}")
            if in_age < 0.0 or in_age > 2.0:
                status.level = DiagnosticStatus.WARN if grace else DiagnosticStatus.ERROR
                status.message = "No native colour frames from the D555"
            elif self._rectifier is None:
                status.level = DiagnosticStatus.WARN
                status.message = "Waiting for validated colour CameraInfo"
            elif out_age < 0.0 or out_age > expected_period:
                status.level = DiagnosticStatus.ERROR
                status.message = "Frames arrive but none convert; see last_error"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Publishing rectified rgb8 for nvblox"
            values = {
                "input_hz": f"{in_hz:.2f}",
                "output_hz": f"{out_hz:.2f}",
                "max_output_hz": f"{1e9 / self._period_ns:.2f}" if self._period_ns else "unlimited",
                "input_encoding": self._encoding,
                "rectified": str(bool(self._rectifier is not None and not self._rectifier.identity)).lower(),
                "frames_in_total": str(self._frames_in),
                "frames_out_total": str(self._frames_out),
                "drops_total": str(sum(self._drops.values())),
                "last_error": self._last_error,
                "header_stamp_policy": "device_capture_time_unchanged",
            }
            if self._info_fields is not None:
                k = self._info_fields["k"]
                values.update(fx=f"{k[0]:.3f}", fy=f"{k[4]:.3f}", cx=f"{k[2]:.3f}", cy=f"{k[5]:.3f}")
                values["source_distortion"] = str([round(float(v), 5) for v in self._info_key[1]])
            status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
            report = DiagnosticArray()
            report.header.stamp = self.get_clock().now().to_msg()
            report.status = [status]
            self._diag_pub.publish(report)

    return D555RgbAdapter()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--serial", default="261622302751", help="D555 serial in the native topic names")
    parser.add_argument("--image-topic", default=None, help="native colour Image topic (default from --serial)")
    parser.add_argument("--camera-info-topic", default="/d555/color/camera_info",
                        help="frame-validated colour CameraInfo relay")
    parser.add_argument("--output-namespace", default="/d555/color/rect")
    parser.add_argument("--max-rate", type=float, default=5.0, help="max output rate in Hz (0 = every frame)")
    options, ros_args = parser.parse_known_args(argv)
    if not re.fullmatch(r"[0-9]+", options.serial):
        parser.error("serial must contain digits only")
    if options.image_topic is None:
        options.image_topic = f"/realsense/D555_{options.serial}_Color"
    if options.max_rate < 0:
        parser.error("max-rate must be >= 0")
    return options, ros_args


def main():
    options, ros_args = parse_args()

    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_adapter_node(options)
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
