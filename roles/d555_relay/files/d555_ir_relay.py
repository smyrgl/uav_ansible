#!/usr/bin/env python3
"""D555 infrared relay: the bag's lossless copy of the stereo IR pair.

The D555 is its own DDS publisher and sends every subscriber a private unicast
copy of each stream over the aircraft's single 1 GbE port: ~106 Mbit/s per IR
stream at 896x504 and 29.3 Hz, ~491 Mbit/s for the 1280x800 colour stream the
encoder reads, ~747 Mbit/s on the port with the Avia and PX4 (measured
2026-10-05). A second reader of the pair, the flight recorder, would push the
port past its ceiling, which is why raw camera topics were never recorded. This
node is the one camera-side reader of Infrared_1/2 and republishes them under
/d555/infra{1,2}, where any number of local consumers (the recorder, an offline
VSLAM, calibration, neural reconstruction) read them at zero link cost.

Per frame the serialized CDR buffer passes through untouched except for its
header: the stamp is mapped from the camera clock to UTC with the adapter's
model (/d555/clock, the mapping uav_camera and the vslam bridge apply too) and
the frame id is canonicalised. The camera pads frame ids with NULs and now and
then labels a frame with the other stream's name; those frames are dropped and
counted. The IR camera_info is relayed the same way (typed, small, 30 Hz), so
the image and camera_info of one frame carry one stamp.

Lazy by default: the camera is subscribed only while something reads an output
topic (count_subscribers, 1 Hz), so between bags the pair costs the link nothing.
Releasing is done by exiting: the D555 keeps unicasting to a reader that was
destroyed and stops only when the reader's DDS participant disappears (verified
2026-10-05: 747 Mbit/s stayed on the port after destroy_subscription, 527 after
a process restart), so after `release_after_s` seconds without a local reader
the node exits 0 and systemd (`Restart=always`) starts a fresh participant.
"""

import argparse
import json
import re
import struct
import time
from collections import Counter

CDR_LE = b"\x00\x01"           # encapsulation: CDR, little-endian
HEADER_END = 16                 # 4 encapsulation + sec(4) + nanosec(4) + frame_id length(4)
NS = 1_000_000_000


def parse_header(buf):
    """(device stamp ns, frame label, tail offset) of a message that starts with std_msgs/Header.

    The tail is the first byte after the frame_id string and its padding. For
    sensor_msgs/Image that is `height`, a uint32, so it sits on a 4-byte boundary.
    The label is the frame id without the NUL padding the camera appends.
    """
    if len(buf) < HEADER_END or bytes(buf[:2]) != CDR_LE:
        raise ValueError("not a little-endian CDR message")
    sec, nanosec, length = struct.unpack_from("<iIi", buf, 4)
    if length < 1 or HEADER_END + length > len(buf):
        raise ValueError(f"implausible frame_id length {length}")
    raw = bytes(buf[HEADER_END:HEADER_END + length - 1])        # without the terminating NUL
    label = raw.split(b"\x00", 1)[0].decode("ascii", "replace")
    tail = HEADER_END + ((length + 3) & ~3)
    return sec * NS + nanosec, label, tail


def restamp(buf, stamp_ns, frame_id):
    """The same message with header.stamp = stamp_ns and header.frame_id = frame_id.

    Only the header is rebuilt; the payload from `height` on is reused as is.
    Nothing after the header of a sensor_msgs/Image needs more than 4-byte
    alignment and the rebuilt header is a multiple of 4 bytes, so the tail stays
    valid wherever it lands.
    """
    _, _, tail = parse_header(buf)
    if stamp_ns < 0:
        raise ValueError("negative stamp")
    name = frame_id.encode("ascii") + b"\x00"
    pad = (-len(name)) % 4
    return b"".join((bytes(buf[:4]), struct.pack("<iIi", stamp_ns // NS, stamp_ns % NS, len(name)),
                     name, b"\x00" * pad, bytes(buf[tail:])))


def unwrap_near(device_ns, reference_ns, wrap_ns):
    """The representation of a possibly wrapped device time nearest the reference."""
    if not wrap_ns:
        return device_ns
    return device_ns + round((reference_ns - device_ns) / wrap_ns) * wrap_ns


class ClockMapping:
    """Device clock -> UTC from /d555/clock (the adapter's model; uav_camera and the vslam bridge use the same)."""

    def __init__(self, max_age_s=5.0):
        self.max_age_ns = int(max_age_s * NS)
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


def parse_streams(text):
    """'infra1:Infrared_1:camera_infra1_optical_frame,...' -> [(name, camera topic suffix, frame id)]."""
    streams = []
    for item in text.split(","):
        parts = item.strip().split(":")
        if len(parts) != 3 or not all(parts):
            raise ValueError(f"a stream is name:source:frame, got {item!r}")
        streams.append(tuple(parts))
    if len({s[0] for s in streams}) != len(streams):
        raise ValueError("stream names must be unique")
    return streams


def receive_ns(info):
    """The DDS receive timestamp (host UTC) if the middleware gave one."""
    received = (info or {}).get("received_timestamp") or 0
    return int(received) if received > 0 else time.time_ns()


def create_relay_node(options):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    latched_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class Stream:
        def __init__(self, name, source, frame):
            self.name, self.source, self.frame = name, source, frame
            self.image_in = self.info_in = None
            self.image_out = self.info_out = None
            self.readers = 0
            self.relayed = 0
            self.dropped = Counter()        # cdr, frame, info_frame
            self.stamped = Counter()        # capture, receipt
            self.last_relay_ns = 0

    class D555IRRelay(Node):
        def __init__(self):
            super().__init__("d555_ir_relay")
            self.lazy = options.lazy
            self.release_after_s = options.release_after_s
            self._was_reading = False
            self._idle_s = 0
            self.clock = ClockMapping(options.clock_max_age_s)
            self.create_subscription(String, options.clock_topic, lambda msg: self.clock.update(msg.data), latched_qos)
            native = f"/realsense/D555_{options.serial}"
            self.streams = []
            for name, source, frame in options.streams:
                stream = Stream(name, f"{native}_{source}", frame)
                stream.image_out = self.create_publisher(Image, f"{options.prefix}/{name}/image", sensor_qos)
                stream.info_out = self.create_publisher(CameraInfo, f"{options.prefix}/{name}/camera_info", sensor_qos)
                self.streams.append(stream)
            self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._types = (DiagnosticArray, DiagnosticStatus, KeyValue, Image, CameraInfo)
            self.create_timer(1.0, self._tick)
            self.get_logger().info(
                f"D555 IR relay: {', '.join(s.source for s in self.streams)} -> {options.prefix}/<name>/image,"
                f"camera_info ({'lazy: camera read only with a local reader' if self.lazy else 'always reading'};"
                f" UTC via {options.clock_topic})")

        def _wanted(self, stream):
            stream.readers = (self.count_subscribers(stream.image_out.topic_name)
                              + self.count_subscribers(stream.info_out.topic_name))
            return (not self.lazy) or stream.readers > 0

        def _tick(self):
            for stream in self.streams:
                wanted = self._wanted(stream)
                if wanted and stream.image_in is None:
                    self._subscribe(stream)
                elif not wanted and stream.image_in is not None:
                    self._unsubscribe(stream)
            self._diagnostics()
            self._release_if_idle()

        def _release_if_idle(self):
            """Exit once no stream has had a local reader for release_after_s.

            destroy_subscription does not make the D555 stop sending: the camera
            keeps unicasting the stream to the vanished reader until the reader's
            participant goes away (measured 2026-10-05). A fresh process is the
            only release; systemd restarts the relay, which comes back idle.
            """
            if not self.lazy or not self._was_reading:
                return
            if any(stream.image_in is not None for stream in self.streams):
                self._idle_s = 0
                return
            self._idle_s += 1
            if self._idle_s >= self.release_after_s:
                self.get_logger().info(
                    f"no local reader for {self._idle_s} s: exiting so the camera unmatches this participant"
                    " (it keeps sending to a destroyed reader); systemd restarts the relay idle")
                raise SystemExit(0)

        def _subscribe(self, stream):
            image_type, info_type = self._types[3], self._types[4]
            stream.image_in = self.create_subscription(
                image_type, stream.source,
                lambda buf, info, *, stream=stream: self._image(stream, buf, info), sensor_qos, raw=True)
            stream.info_in = self.create_subscription(
                info_type, stream.source + "/camera_info",
                lambda msg, info, *, stream=stream: self._info(stream, msg, info), sensor_qos)
            self._was_reading = True
            self.get_logger().info(f"{stream.name}: reading {stream.source} for {stream.readers} local reader(s)")

        def _unsubscribe(self, stream):
            self.destroy_subscription(stream.image_in)
            self.destroy_subscription(stream.info_in)
            stream.image_in = stream.info_in = None
            self.get_logger().info(f"{stream.name}: no local reader left, {stream.source} released")

        def _utc(self, device_ns, info):
            mapped = self.clock.to_utc_ns(device_ns)
            return (mapped, "capture") if mapped is not None else (receive_ns(info), "receipt")

        def _image(self, stream, buf, info):
            try:
                device_ns, label, _ = parse_header(buf)
            except ValueError as error:
                stream.dropped["cdr"] += 1
                self.get_logger().warning(f"{stream.name}: dropping a frame: {error}", throttle_duration_sec=5.0)
                return
            if label != stream.frame:
                stream.dropped["frame"] += 1
                self.get_logger().warning(f"{stream.name}: dropping a frame labelled {label!r}", throttle_duration_sec=5.0)
                return
            utc, kind = self._utc(device_ns, info)
            stream.image_out.publish(restamp(buf, utc, stream.frame))
            stream.stamped[kind] += 1
            stream.relayed += 1
            stream.last_relay_ns = time.monotonic_ns()

        def _info(self, stream, message, info):
            label = message.header.frame_id.split("\x00", 1)[0]
            if label != stream.frame:
                stream.dropped["info_frame"] += 1
                return
            utc, _ = self._utc(message.header.stamp.sec * NS + message.header.stamp.nanosec, info)
            message.header.frame_id = label
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(utc, NS)
            stream.info_out.publish(message)

        def _diagnostics(self):
            DiagnosticArray, DiagnosticStatus, KeyValue = self._types[:3]
            now = time.monotonic_ns()
            status = DiagnosticStatus(name="d555/ir_relay", hardware_id=options.serial)
            level, notes = DiagnosticStatus.OK, []
            values = {"lazy": str(self.lazy).lower(), "clock_model": self.clock.reason}
            reading = []
            for s in self.streams:
                active = s.image_in is not None
                if active:
                    reading.append(s.name)
                values.update({
                    f"{s.name}/local_readers": s.readers, f"{s.name}/reading_camera": str(active).lower(),
                    f"{s.name}/relayed": s.relayed, f"{s.name}/stamped_capture": s.stamped["capture"],
                    f"{s.name}/stamped_receipt": s.stamped["receipt"], f"{s.name}/dropped_frame_label": s.dropped["frame"],
                    f"{s.name}/dropped_info_label": s.dropped["info_frame"], f"{s.name}/dropped_cdr": s.dropped["cdr"]})
                if active and s.relayed and now - s.last_relay_ns > 2 * NS:
                    level = DiagnosticStatus.WARN
                    notes.append(f"{s.name}: no frames for {(now - s.last_relay_ns) / 1e9:.0f} s")
            if reading and self.clock.reason != "ok":
                level = DiagnosticStatus.WARN
                notes.append(f"frames stamped at receipt: {self.clock.reason}")
            status.level = level
            status.message = ("; ".join(notes) if notes else
                              (f"relaying {', '.join(reading)} with UTC stamps" if reading
                               else "idle: no local reader, camera not read"))
            status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
            array = DiagnosticArray(status=[status])
            array.header.stamp = self.get_clock().now().to_msg()
            self._diag_pub.publish(array)

    return D555IRRelay()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--serial", default="261622302751", help="D555 native topic serial number")
    parser.add_argument("--prefix", default="/d555", help="output topics: <prefix>/<name>/image and /camera_info")
    parser.add_argument("--streams", default="infra1:Infrared_1:camera_infra1_optical_frame,"
                                             "infra2:Infrared_2:camera_infra2_optical_frame",
                        help="comma-separated name:camera topic suffix:frame id")
    parser.add_argument("--clock-topic", default="/d555/clock", help="the adapter's device-clock -> UTC model")
    parser.add_argument("--clock-max-age-s", type=float, default=5.0, help="older model: stamp frames at receipt")
    parser.add_argument("--always", action="store_true", help="read the camera even with no local reader (default: lazy)")
    parser.add_argument("--release-after-s", type=int, default=5,
                        help="lazy mode: exit (for a systemd restart) after this many seconds without a local reader")
    options, ros_args = parser.parse_known_args()
    if not re.fullmatch(r"[0-9]+", options.serial):
        parser.error("serial must contain digits only")
    try:
        options.streams = parse_streams(options.streams)
    except ValueError as error:
        parser.error(str(error))
    options.lazy = not options.always
    if options.release_after_s < 1:
        parser.error("release-after-s must be at least 1")

    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_relay_node(options)
        # The default executor on purpose: raw subscriptions under rclpy 7.1.12's
        # EventsExecutor crashed the D555 adapter (SystemError), see the foxglove role.
        rclpy.spin(node)          # SystemExit(0) from the release timer propagates: see _release_if_idle
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
