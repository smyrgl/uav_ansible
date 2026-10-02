"""Run with Python unittest; ROS round trips run when Jazzy is sourced."""

import copy
import struct
import os
from pathlib import Path
from types import SimpleNamespace
import unittest

from d555_frame_adapter import normalize_frame_id, normalize_message_frame, set_stamp, utc_stamp

try:
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import CameraInfo
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class FrameNameTests(unittest.TestCase):
    def test_exact_and_trailing_nul_only(self):
        for stream in ("color", "depth"):
            expected = f"camera_{stream}_optical_frame"
            for padding in ("", "\0", "\0" * 100):
                self.assertEqual(normalize_frame_id(expected + padding, expected), expected)

    def test_unexpected_name_is_rejected_without_mutating_message(self):
        expected = "camera_color_optical_frame"
        for bad in ("", expected + " ", "/" + expected, expected + "\0unexpected",
                    "camera_depth_optical_frame", "camera\0_color_optical_frame", None):
            message = SimpleNamespace(header=SimpleNamespace(frame_id=bad))
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                normalize_message_frame(message, expected)
            self.assertEqual(message.header.frame_id, bad)


class UtcStampTests(unittest.TestCase):
    def test_mapped_capture_time_when_valid_receipt_time_otherwise(self):
        class Model:
            def __init__(self, offset): self.offset = offset
            def to_utc_ns(self, device_ns): return None if self.offset is None else device_ns + self.offset
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=598, nanosec=434542000)))
        info = {"received_timestamp": 1_790_913_854_321_647_000}
        stamp, kind = utc_stamp(Model(1_790_913_255_828_000_000), message, info)
        self.assertEqual((stamp, kind), (598_434_542_000 + 1_790_913_255_828_000_000, "capture"))
        self.assertEqual(utc_stamp(Model(None), message, info), (1_790_913_854_321_647_000, "receipt"))
        set_stamp(message, 1_790_913_854_262_542_000)
        self.assertEqual((message.header.stamp.sec, message.header.stamp.nanosec), (1_790_913_854, 262_542_000))


def pad_wire_frame_id(serialized, length=100):
    """Reproduce a padded native CDR Header string without changing its payload.

    Header starts with encapsulation (4), time (8), then the string length (4).
    CameraInfo contains doubles; added padding must preserve eight-byte alignment.
    """
    wire = bytes(serialized)
    if wire[:2] != b"\x00\x01":
        raise AssertionError("Expected Jazzy little-endian CDR encoding")
    old_length = struct.unpack_from("<I", wire, 12)[0]
    frame = wire[16:16 + old_length]
    if length < len(frame) or (length - ((old_length + 3) & ~3)) % 8:
        raise AssertionError("Test padding must fit the frame and maintain alignment")
    payload_offset = (16 + old_length + 3) & ~3
    return wire[:12] + struct.pack("<I", length) + frame.ljust(length, b"\0") + wire[payload_offset:]


def camera_info_wire_fields(wire):
    """Read exact field bytes while ignoring unspecified CDR alignment padding."""
    if wire[:2] != b"\x00\x01":
        raise AssertionError("Expected little-endian CDR")
    position = 4

    def take(size, alignment):
        nonlocal position
        position = 4 + ((position - 4 + alignment - 1) // alignment) * alignment
        result = wire[position:position + size]
        if len(result) != size:
            raise AssertionError("Truncated CameraInfo CDR")
        position += size
        return result

    def string():
        size = struct.unpack("<I", take(4, 4))[0]
        return take(size, 1).rstrip(b"\0")

    fields = {"stamp": take(8, 4), "frame_id": string(), "dimensions": take(8, 4),
              "distortion_model": string()}
    d_size = take(4, 4)
    fields["d_size"] = d_size
    fields["d"] = take(8 * struct.unpack("<I", d_size)[0], 8)
    fields["k"] = take(8 * 9, 8)
    fields["r"] = take(8 * 9, 8)
    fields["p"] = take(8 * 12, 8)
    fields["binning"] = take(8, 4)
    fields["roi_integers"] = take(16, 4)
    fields["roi_rectify"] = take(1, 1)
    return fields


@unittest.skipUnless(ROS_AVAILABLE, "Source ROS Jazzy to run actual CDR round-trip tests")
class RosSerializationTests(unittest.TestCase):
    def check_wire_round_trip(self, original, expected_frame):
        canonical = bytes(serialize_message(original))
        padded_wire = pad_wire_frame_id(canonical)
        self.assertNotEqual(padded_wire, canonical)
        received = deserialize_message(padded_wire, type(original))
        before = copy.deepcopy(received)
        normalize_message_frame(received, expected_frame)
        self.assertEqual(received.header.stamp, original.header.stamp)
        before.header.frame_id = expected_frame
        self.assertEqual(received, before)
        normalized_wire = bytes(serialize_message(received))
        # CDR alignment padding is unspecified; all semantic field bytes must match.
        self.assertEqual(camera_info_wire_fields(normalized_wire), camera_info_wire_fields(canonical))
        self.assertEqual(deserialize_message(normalized_wire, type(original)), original)
        frame_length = struct.unpack_from("<I", normalized_wire, 12)[0]
        self.assertEqual(frame_length, len(expected_frame.encode()) + 1)

    def test_camera_calibration_and_device_capture_stamp_are_preserved(self):
        for stream in ("color", "depth"):
            message = CameraInfo()
            message.header.frame_id = f"camera_{stream}_optical_frame"
            message.header.stamp.sec = 5029
            message.header.stamp.nanosec = 850580000
            message.width = 896
            message.height = 504
            message.distortion_model = "plumb_bob"
            message.d = [0.01, -0.02, 0.003, 0.004, 0.0]
            message.k = [600.0, 0.0, 448.0, 0.0, 601.0, 252.0, 0.0, 0.0, 1.0]
            message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            message.p = [600.0, 0.0, 448.0, 0.0, 0.0, 601.0, 252.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            message.binning_x = 1
            message.binning_y = 1
            message.roi.x_offset = 2
            message.roi.y_offset = 3
            message.roi.width = 890
            message.roi.height = 498
            message.roi.do_rectify = True
            self.check_wire_round_trip(message, message.header.frame_id)

    def test_captured_native_camera_info_keeps_exact_payload_and_timestamp_bytes(self):
        fixture = os.environ.get("NATIVE_D555_CDR_FIXTURE")
        if not fixture:
            self.skipTest("Set NATIVE_D555_CDR_FIXTURE for captured-camera CDR verification")
        wire = Path(fixture).read_bytes()
        received = deserialize_message(wire, CameraInfo)
        expected_frame = received.header.frame_id.rstrip("\0")
        self.assertIn(expected_frame, ("camera_color_optical_frame", "camera_depth_optical_frame"))
        before = copy.deepcopy(received)
        normalize_message_frame(received, expected_frame)
        normalized = bytes(serialize_message(received))
        old_length = struct.unpack_from("<I", wire, 12)[0]
        new_length = struct.unpack_from("<I", normalized, 12)[0]
        self.assertEqual(new_length, len(expected_frame.encode()) + 1)
        self.assertGreater(old_length, new_length)
        self.assertEqual(wire[:12], normalized[:12])
        self.assertEqual(camera_info_wire_fields(wire), camera_info_wire_fields(normalized))
        before.header.frame_id = expected_frame
        self.assertEqual(deserialize_message(normalized, CameraInfo), before)


if __name__ == "__main__":
    unittest.main()

