"""Pure parts of the vslam bridge: clock mapping, frame labels, twist covariance."""
import json
import math
import unittest

from vslam_bridge import CentralVelocity, ClockMapping, anchor, frame_label, quat_multiply, rotate_inverse, twist_covariance

MODEL = {"valid": True, "computed_utc_ns": 1_000_000_000_000, "device_ref_ns": 7_000_000_000,
         "offset_ref_ns": 1_790_000_000_000_000_000, "skew_ppm": -8.0, "wrap_ns": 4_294_967_296_000}


class Clock(unittest.TestCase):
    def test_maps_with_offset_and_skew(self):
        c = ClockMapping()
        c.update(json.dumps(MODEL))
        now = MODEL["computed_utc_ns"] + 1_000_000_000
        self.assertEqual(c.to_utc_ns(7_000_000_000, now), MODEL["offset_ref_ns"] + 7_000_000_000)
        later = c.to_utc_ns(8_000_000_000, now)                     # 1 s on, -8 ppm: 8 us less
        self.assertEqual(later, MODEL["offset_ref_ns"] + 8_000_000_000 - 8_000)

    def test_invalid_stale_and_missing_map_nothing(self):
        c = ClockMapping()
        self.assertIsNone(c.to_utc_ns(1))
        c.update(json.dumps(dict(MODEL, valid=False, reason="warming up")))
        self.assertIsNone(c.to_utc_ns(7_000_000_000, MODEL["computed_utc_ns"]))
        self.assertEqual(c.reason, "warming up")
        c.update(json.dumps(MODEL))
        self.assertIsNone(c.to_utc_ns(7_000_000_000, MODEL["computed_utc_ns"] + 6_000_000_000))
        self.assertEqual(c.reason, "clock model stale")

    def test_device_wrap_is_unwrapped_near_the_reference(self):
        c = ClockMapping()
        c.update(json.dumps(MODEL))
        wrapped = 7_000_000_000 + 5_000_000 - MODEL["wrap_ns"]           # 5 ms after ref, seen wrapped
        self.assertEqual(c.to_utc_ns(wrapped % MODEL["wrap_ns"], MODEL["computed_utc_ns"]),
                         c.to_utc_ns(7_005_000_000, MODEL["computed_utc_ns"]))


class Helpers(unittest.TestCase):
    def test_frame_label_strips_nul_padding(self):
        self.assertEqual(frame_label("camera_infra1_optical_frame" + "\x00" * 37), "camera_infra1_optical_frame")

    def test_twist_covariance_is_diagonal(self):
        out = twist_covariance(0.05)
        self.assertAlmostEqual(out[0], 0.0025)
        self.assertAlmostEqual(out[14], 0.0025)
        self.assertEqual((out[1], out[21]), (0.0, 1e6))                  # angular: not provided


YAW90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
IDENTITY = (0.0, 0.0, 0.0, 1.0)
FRAME = 33_333_333


class Velocity(unittest.TestCase):
    def test_rotate_inverse(self):
        v = rotate_inverse(YAW90, (0.0, 1.0, 0.0))     # body x points along world +y
        for got, want in zip(v, (1.0, 0.0, 0.0)):
            self.assertAlmostEqual(got, want)

    def test_central_difference_at_the_middle_pose_in_its_body_frame(self):
        c = CentralVelocity()
        out = [c.add(i * FRAME, (0.0, 2.0 * i * FRAME * 1e-9, 0.0), YAW90, i) for i in range(3)]
        self.assertEqual(out[:2], [None, None])
        t, (vx, vy, vz), payload = out[2]
        self.assertEqual((t, payload), (FRAME, 1))         # stamped and posed at the middle sample
        self.assertAlmostEqual(vx, 2.0)
        self.assertAlmostEqual(vy, 0.0)

    def test_anchor_makes_the_source_pose_coincide_with_odom(self):
        odom_base = ((3.0, -1.0, 0.5), (0.0, 0.0, math.sin(0.3), math.cos(0.3)))
        source_base = ((0.2, 0.4, -0.1), (0.0, math.sin(-0.2), 0.0, math.cos(-0.2)))
        p, q = anchor(odom_base, source_base)
        rotated = rotate_inverse((-q[0], -q[1], -q[2], q[3]), source_base[0])   # R(q) v
        for got, want in zip(tuple(a + b for a, b in zip(p, rotated)) + quat_multiply(q, source_base[1]),
                             odom_base[0] + odom_base[1]):
            self.assertAlmostEqual(got, want)

    def test_gap_and_implausible_jump_start_a_new_window(self):
        c = CentralVelocity()
        for i in range(3):
            c.add(i * FRAME, (0.0, 0.0, 0.0), IDENTITY)
        self.assertIsNone(c.add(10 * FRAME, (0.0, 0.0, 0.0), IDENTITY))   # 0.23 s gap
        self.assertEqual(c.resets, 1)
        c.add(11 * FRAME, (0.0, 0.0, 0.0), IDENTITY)
        self.assertIsNone(c.add(12 * FRAME, (5.0, 0.0, 0.0), IDENTITY))   # 75 m/s: a relocalised origin
        self.assertEqual(c.resets, 2)


if __name__ == "__main__":
    unittest.main()
