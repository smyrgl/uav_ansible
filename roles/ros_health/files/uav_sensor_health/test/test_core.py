import unittest
from dataclasses import replace

from uav_sensor_health.core import (
    EPOCH_FLOOR_SEC, ERROR, OK, STALE, WARN, ImageMetadata, StreamMonitor, classify_clock,
)


WALL = 1_790_319_674.0
IMAGE = ImageMetadata(stamp_ns=100_000_000_000, width=640, height=360,
                      step=1280, encoding="yuv422_yuy2", frame_id="d555_color",
                      data_bytes=460800)


class TestStreamHealth(unittest.TestCase):
    def test_startup_grace_expires_without_a_message(self):
        monitor = StreamMonitor(0.0)
        self.assertEqual(monitor.assess(9.9, WALL)[0].level, WARN)
        self.assertEqual(monitor.assess(10.0, WALL)[0].level, STALE)

    def test_data_recovers_from_never_seen_and_stale(self):
        monitor = StreamMonitor(0.0)
        self.assertEqual(monitor.assess(20, WALL)[0].level, STALE)
        monitor.observe(IMAGE, 20)
        self.assertEqual(monitor.assess(20.5, WALL)[0].level, OK)
        self.assertEqual(monitor.assess(23, WALL)[0].level, STALE)
        monitor.observe(replace(IMAGE, stamp_ns=101_000_000_000), 24)
        self.assertEqual(monitor.assess(24, WALL)[0].level, WARN)  # recent rate loss
        for i in range(1, 151):
            monitor.observe(replace(IMAGE, stamp_ns=101_000_000_000 + i), 24 + i / 30)
        self.assertEqual(monitor.assess(29, WALL)[0].level, OK)

    def test_device_clock_does_not_invalidate_receipt_freshness(self):
        monitor = StreamMonitor(0)
        monitor.observe(IMAGE, 1)
        stream, clock = monitor.assess(1.1, WALL)
        self.assertEqual(stream.level, OK)
        self.assertEqual(clock.level, WARN)
        self.assertEqual(clock.values["classification"], "device_clock")
        self.assertFalse(clock.values["synchronization_verified"])

    def test_host_clock_steps_do_not_change_monotonic_freshness(self):
        monitor = StreamMonitor(0)
        monitor.observe(IMAGE, 1)
        for wall in (WALL - 1e8, WALL, WALL + 1e8):
            self.assertEqual(monitor.assess(1.5, wall)[0].level, OK)
        self.assertEqual(monitor.assess(3.01, WALL - 1e8)[0].level, STALE)

    def test_stale_timestamp_is_not_reported_as_current_clock_evidence(self):
        monitor = StreamMonitor(0)
        monitor.observe(IMAGE, 1)
        stream, clock = monitor.assess(4, WALL)
        self.assertEqual(stream.level, STALE)
        self.assertEqual(clock.level, STALE)

    def test_repeated_stamps_with_fresh_images(self):
        monitor = StreamMonitor(0)
        for i in range(100):
            monitor.observe(IMAGE, i / 30)
        stream, clock = monitor.assess(3.3, WALL)
        self.assertEqual(stream.level, OK)
        self.assertIn("not advancing", clock.message)
        monitor.observe(replace(IMAGE, stamp_ns=IMAGE.stamp_ns + 1), 3.4)
        self.assertNotIn("not advancing", monitor.assess(3.4, WALL)[1].message)

    def test_timestamp_regression_is_visible_and_can_recover(self):
        monitor = StreamMonitor(0)
        monitor.observe(IMAGE, 0)
        monitor.observe(replace(IMAGE, stamp_ns=99_000_000_000), 0.1)
        clock = monitor.assess(0.1, WALL)[1]
        self.assertIn("backwards", clock.message)
        self.assertEqual(clock.values["timestamp_regressions"], 1)
        monitor.observe(replace(IMAGE, stamp_ns=100_000_000_000), 2.2)
        self.assertNotIn("backwards", monitor.assess(2.2, WALL)[1].message)

    def test_expected_image_rate_and_low_rate_are_distinct(self):
        for rate, expected_level in ((30, OK), (5, WARN)):
            monitor = StreamMonitor(0)
            for i in range(rate * 3):
                monitor.observe(replace(IMAGE, stamp_ns=IMAGE.stamp_ns + i), i / rate)
            stream = monitor.assess((rate * 3 - 1) / rate, WALL)[0]
            self.assertEqual(stream.level, expected_level)
            self.assertAlmostEqual(stream.values["rate_hz"], rate)

    def test_short_silence_reduces_rate_before_stream_timeout(self):
        monitor = StreamMonitor(0)
        for i in range(31):
            monitor.observe(replace(IMAGE, stamp_ns=IMAGE.stamp_ns + i), i / 30)
        stream = monitor.assess(2.5, WALL)[0]
        self.assertEqual(stream.level, WARN)
        self.assertEqual(stream.values["rate_hz"], 12.0)

    def test_old_rate_samples_expire(self):
        monitor = StreamMonitor(0)
        for i in range(100):
            monitor.observe(IMAGE, i / 30)
        monitor.assess(10, WALL)
        self.assertEqual(len(monitor.receipts), 0)
        monitor.observe(IMAGE, 11)
        self.assertEqual(monitor.assess(11, WALL)[0].values["rate_hz"], 0)

    def test_image_payload_length_and_shape_errors(self):
        for bad in (replace(IMAGE, data_bytes=100), replace(IMAGE, width=0),
                    replace(IMAGE, height=-1), replace(IMAGE, step=1),
                    replace(IMAGE, encoding="")):
            monitor = StreamMonitor(0)
            monitor.observe(bad, 1)
            self.assertEqual(monitor.assess(1, WALL)[0].level, WARN)

    def test_camera_info_header_only_metadata(self):
        header = ImageMetadata(stamp_ns=100_000_000_000, width=1280, height=800,
                               frame_id="camera_color_optical_frame")
        self.assertTrue(header.header_only)
        self.assertIsNone(header.layout_error())
        monitor = StreamMonitor(0)
        monitor.observe(header, 1)
        stream, _ = monitor.assess(1, WALL)
        self.assertEqual(stream.level, OK)
        self.assertEqual(stream.values["source"], "camera_info header")
        self.assertNotIn("image_bytes", stream.values)
        for bad in (replace(header, width=0), replace(header, height=-1)):
            monitor = StreamMonitor(0)
            monitor.observe(bad, 1)
            self.assertEqual(monitor.assess(1, WALL)[0].level, WARN)

    def test_empty_frame_id_is_warning(self):
        monitor = StreamMonitor(0)
        monitor.observe(replace(IMAGE, frame_id=""), 1)
        self.assertEqual(monitor.assess(1, WALL)[0].level, WARN)

    def test_metadata_cannot_retain_payload(self):
        self.assertNotIn("data", ImageMetadata.__dataclass_fields__)
        monitor = StreamMonitor(0)
        for i in range(10000):
            monitor.observe(IMAGE, i / 10000)
        self.assertLessEqual(len(monitor.receipts), 4096)

    def test_invalid_thresholds_fail_at_startup(self):
        for key, value in (("timeout_sec", 0), ("clock_tolerance_sec", -1),
                           ("rate_window_sec", 0), ("startup_grace_sec", -1),
                           ("min_rate_hz", -1), ("min_rate_hz", float("nan"))):
            with self.assertRaises(ValueError):
                StreamMonitor(0, **{key: value})


class TestClockClassification(unittest.TestCase):
    def test_epoch_proximity_never_proves_synchronization(self):
        monitor = StreamMonitor(0)
        monitor.observe(replace(IMAGE, stamp_ns=int(WALL * 1e9)), 1)
        clock = monitor.assess(1, WALL)[1]
        self.assertEqual(clock.level, WARN)
        self.assertEqual(clock.values["classification"], "epoch_compatible")
        self.assertFalse(clock.values["synchronization_verified"])

    def test_tai_scale_difference_is_detected_as_future(self):
        state, _ = classify_clock(int((WALL + 37) * 1e9), WALL, 2)
        self.assertEqual(state, "future")

    def test_missing_device_past_and_future_clock_states(self):
        for stamp, expected in ((0, "unset"), (-1, "unset"),
                                 (int((EPOCH_FLOOR_SEC - 1) * 1e9), "device_clock"),
                                 (int((WALL - 3) * 1e9), "past"),
                                 (int((WALL + 3) * 1e9), "future")):
            self.assertEqual(classify_clock(stamp, WALL, 2)[0], expected)


if __name__ == "__main__":
    unittest.main()
