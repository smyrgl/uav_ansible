"""Pure-Python tests of the relay's CDR header rewrite and clock mapping, plus a
round trip through rclpy's serializer when ROS is available."""

import struct
import time
import unittest

from d555_ir_relay import NS, ClockMapping, parse_header, parse_streams, restamp

FRAME = "camera_infra1_optical_frame"


def cdr_string(text, nul_pad=0):
    data = text.encode() + b"\x00" * (1 + nul_pad)
    return struct.pack("<I", len(data)) + data + b"\x00" * ((-len(data)) % 4)


def build_image(sec, nsec, frame, height=2, width=3, encoding="mono8", payload=b"abcdef", nul_pad=0):
    """A sensor_msgs/Image as CDR (little-endian), built independently of the module under test."""
    body = struct.pack("<iI", sec, nsec) + cdr_string(frame, nul_pad)
    body += struct.pack("<II", height, width) + cdr_string(encoding)
    body += struct.pack("<B", 0) + b"\x00" * 3 + struct.pack("<I", width)     # is_bigendian, padding, step
    body += struct.pack("<I", len(payload)) + payload
    return b"\x00\x01\x00\x00" + body


def read_image(buf):
    off = 4
    sec, nsec, n = struct.unpack_from("<iIi", buf, off)
    off += 12
    frame = bytes(buf[off:off + n - 1])
    off += (n + 3) & ~3
    height, width = struct.unpack_from("<II", buf, off)
    off += 8
    n, = struct.unpack_from("<I", buf, off)
    off += 4
    encoding = buf[off:off + n - 1].decode()
    off += (n + 3) & ~3
    bigendian = buf[off]
    off += 4
    step, = struct.unpack_from("<I", buf, off)
    off += 4
    n, = struct.unpack_from("<I", buf, off)
    off += 4
    return dict(sec=sec, nsec=nsec, frame=frame, height=height, width=width, encoding=encoding,
                bigendian=bigendian, step=step, data=bytes(buf[off:off + n]), end=off + n)


class HeaderTests(unittest.TestCase):
    def test_parse_strips_the_cameras_nul_padding(self):
        buf = build_image(1700, 250, FRAME, nul_pad=5)
        stamp, label, tail = parse_header(buf)
        self.assertEqual(stamp, 1700 * NS + 250)
        self.assertEqual(label, FRAME)
        self.assertEqual(tail % 4, 0)
        self.assertEqual(struct.unpack_from("<II", buf, tail), (2, 3))

    def test_restamp_keeps_the_payload_for_shorter_and_longer_names(self):
        original = build_image(1700, 250, FRAME, height=4, width=5, payload=bytes(range(20)), nul_pad=5)
        before = read_image(original)
        for name in (FRAME, "x", FRAME + "_longer_than_before_by_a_lot"):
            out = restamp(original, 1_800_000_000 * NS + 123_456_789, name)
            after = read_image(out)
            self.assertEqual((after["sec"], after["nsec"]), (1_800_000_000, 123_456_789))
            self.assertEqual(after["frame"], name.encode())
            for key in ("height", "width", "encoding", "bigendian", "step", "data"):
                self.assertEqual(after[key], before[key], key)
            self.assertEqual(after["end"], len(out))
            self.assertEqual(len(out) % 4, 0)

    def test_rejects_big_endian_and_truncated_buffers(self):
        buf = build_image(1, 2, FRAME)
        with self.assertRaises(ValueError):
            parse_header(b"\x00\x00" + buf[2:])
        with self.assertRaises(ValueError):
            parse_header(buf[:20])
        with self.assertRaises(ValueError):
            restamp(buf, -1, FRAME)

    def test_mislabelled_frame_is_visible_to_the_caller(self):
        buf = build_image(1, 2, "camera_infra2_optical_frame")
        self.assertEqual(parse_header(buf)[1], "camera_infra2_optical_frame")


class ClockTests(unittest.TestCase):
    def model(self, now, **override):
        model = {"valid": True, "device_ref_ns": 1000 * NS, "offset_ref_ns": 1_700_000_000 * NS,
                 "skew_ppm": 10.0, "wrap_ns": 0, "computed_utc_ns": now}
        model.update(override)
        return model

    def test_maps_with_offset_and_skew(self):
        import json
        now = time.time_ns()
        clock = ClockMapping(5.0)
        clock.update(json.dumps(self.model(now)))
        self.assertEqual(clock.reason, "ok")
        self.assertEqual(clock.to_utc_ns(1001 * NS, now), 1001 * NS + 1_700_000_000 * NS + 10_000)

    def test_stale_or_invalid_models_map_nothing(self):
        import json
        now = time.time_ns()
        clock = ClockMapping(5.0)
        clock.update(json.dumps(self.model(now - 6 * NS)))
        self.assertIsNone(clock.to_utc_ns(1001 * NS, now))
        self.assertEqual(clock.reason, "clock model stale")
        clock.update(json.dumps(self.model(now, valid=False, reason="too few bins")))
        self.assertIsNone(clock.to_utc_ns(1001 * NS, now))
        self.assertEqual(clock.reason, "too few bins")
        clock.update("not json")
        self.assertIsNone(clock.to_utc_ns(1001 * NS, now))

    def test_unwraps_a_wrapped_device_clock(self):
        import json
        now = time.time_ns()
        clock = ClockMapping(5.0)
        clock.update(json.dumps(self.model(now, device_ref_ns=4_000_000_000_000, wrap_ns=2 ** 32 * 1000, skew_ppm=0.0)))
        wrapped = (4_000_000_000_000 + 5 * NS) % (2 ** 32 * 1000)
        self.assertEqual(clock.to_utc_ns(wrapped, now), 4_000_000_000_000 + 5 * NS + 1_700_000_000 * NS)


class StreamsTests(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(parse_streams("infra1:Infrared_1:camera_infra1_optical_frame, infra2:Infrared_2:f2"),
                         [("infra1", "Infrared_1", "camera_infra1_optical_frame"), ("infra2", "Infrared_2", "f2")])
        for bad in ("infra1:Infrared_1", "a:b:c,a:d:e", "::"):
            with self.assertRaises(ValueError):
                parse_streams(bad)


class RclpyRoundTrip(unittest.TestCase):
    def test_rewritten_header_deserializes_to_the_same_image(self):
        try:
            from rclpy.serialization import deserialize_message, serialize_message
            from sensor_msgs.msg import Image
        except ImportError:
            self.skipTest("rclpy not available")
        image = Image()
        image.header.stamp.sec, image.header.stamp.nanosec = 1234, 56789
        image.header.frame_id = FRAME + "\x00\x00\x00"
        image.height, image.width, image.encoding, image.step = 3, 7, "mono8", 7
        image.data = bytes(range(21))
        buf = serialize_message(image)
        stamp, label, _ = parse_header(buf)
        self.assertEqual((stamp, label), (1234 * NS + 56789, FRAME))
        out = deserialize_message(restamp(buf, 1_800_000_000 * NS + 5, FRAME), Image)
        self.assertEqual((out.header.stamp.sec, out.header.stamp.nanosec), (1_800_000_000, 5))
        self.assertEqual(out.header.frame_id, FRAME)
        self.assertEqual((out.height, out.width, out.encoding, out.step), (3, 7, "mono8", 7))
        self.assertEqual(bytes(out.data), bytes(range(21)))


if __name__ == "__main__":
    unittest.main()
