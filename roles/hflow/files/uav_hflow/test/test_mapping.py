import math
import unittest

from uav_hflow.mapping import (READING_TOO_CLOSE, READING_TOO_FAR, READING_UNDEFINED, READING_VALID,
                               RangeSample, flow_fields, px4_device_id, range_fields, stream_rate_hz)


def sample(reading, r=0.226):
    return RangeSample(range_m=r, reading_type=reading, field_of_view_rad=0.1047, sensor_type=2, sensor_id=0)


class TestRange(unittest.TestCase):
    def test_valid_reading_passes_through(self):
        self.assertEqual(range_fields(sample(READING_VALID), 0.08, 30.0), (0.226, 0.226, -1))

    def test_too_close_and_too_far_follow_ros_and_px4_conventions(self):
        rng, dist, q = range_fields(sample(READING_TOO_CLOSE, 0.08), 0.08, 30.0)
        self.assertEqual((rng, dist, q), (-math.inf, 0.08, 0))
        rng, dist, q = range_fields(sample(READING_TOO_FAR, 30.0), 0.08, 30.0)
        self.assertEqual((rng, dist, q), (math.inf, 30.0, 0))

    def test_undefined_reading_is_nan(self):
        rng, dist, q = range_fields(sample(READING_UNDEFINED), 0.08, 30.0)
        self.assertTrue(math.isnan(rng) and math.isnan(dist) and q == 0)


class TestFlow(unittest.TestCase):
    def test_fields_keep_sensor_axes_and_units(self):
        f = flow_fields(0.01587, [-8e-05, -3e-05], [0.0, 0.00213], 69, 0.226, 0.1, 0.5)
        self.assertEqual(f["pixel_flow"], [0.0, 0.00213])
        self.assertEqual(f["delta_angle"][:2], [-8e-05, -3e-05])
        self.assertTrue(math.isnan(f["delta_angle"][2]))
        self.assertEqual(f["integration_timespan_us"], 15870)
        self.assertEqual(f["quality"], 69)
        self.assertTrue(f["distance_available"] and f["distance_m"] == 0.226)

    def test_stale_or_missing_distance_is_not_attached(self):
        for distance, age in ((0.226, 0.9), (None, None), (math.nan, 0.0)):
            f = flow_fields(0.016, [0, 0], [0, 0], 300, distance, age, 0.5)
            self.assertFalse(f["distance_available"])
            self.assertTrue(math.isnan(f["distance_m"]))
            self.assertEqual(f["quality"], 255)


class TestMisc(unittest.TestCase):
    def test_device_id_layout(self):
        self.assertEqual(px4_device_id(124), 3 | (124 << 8))

    def test_rate(self):
        self.assertEqual(stream_rate_hz([0.0, 0.5, 1.0], 1.0), 2.0)
        self.assertEqual(stream_rate_hz([0.0], 1.0), 0.0)


if __name__ == "__main__":
    unittest.main()
