"""Run with: python3 -m unittest test_d555_clock -v (no ROS needed)."""
import random
import unittest

from d555_clock import NS_PER_S, WRAP_32BIT_US_NS, ClockModel, map_device_ns, unwrap_near

OFFSET = 1_790_913_255_828_000_000   # UTC - device at device time 0
FLOOR = 150_000                      # minimum one-way latency, ns


def feed(model, start_s, seconds, skew_ppm=12.0, rate_hz=100, congested=(), seed=1, offset=OFFSET):
    rnd = random.Random(seed)
    t = start_s * NS_PER_S
    step = NS_PER_S // rate_hz
    for _ in range(int(seconds * rate_hz)):
        latency = FLOOR + int(rnd.expovariate(1 / 300_000))
        if int(t // NS_PER_S) in congested:
            latency += 5_000_000          # a whole second with no fast sample
        utc = t + offset + int(skew_ppm * 1e-6 * t)
        model.add(t % WRAP_32BIT_US_NS, utc + latency)
        t += step
    return t


def true_utc(device_ns, skew_ppm=12.0, offset=OFFSET):
    return device_ns + offset + int(skew_ppm * 1e-6 * device_ns)


class ClockModelTests(unittest.TestCase):
    def test_recovers_offset_and_skew_to_within_the_latency_floor(self):
        m = ClockModel()
        end = feed(m, 1000, 30)
        fit = m.solve(true_utc(end) + FLOOR)
        self.assertTrue(fit["valid"], fit["reason"])
        self.assertAlmostEqual(fit["skew_ppm"], 12.0, delta=2.0)
        for device in (end - NS_PER_S, end - 15 * NS_PER_S):
            error = m.to_utc_ns(device) - true_utc(device)
            # late by about the floor, never early by more than a few tens of microseconds
            self.assertGreater(error, FLOOR - 60_000)
            self.assertLess(error, FLOOR + 120_000)

    def test_needs_enough_bins_before_it_is_valid(self):
        m = ClockModel(min_bins=10)
        end = feed(m, 50, 5)
        fit = m.solve(true_utc(end))
        self.assertFalse(fit["valid"])
        self.assertIn("collecting", fit["reason"])
        self.assertIsNone(m.to_utc_ns(end))

    def test_congested_seconds_are_rejected(self):
        m = ClockModel()
        end = feed(m, 2000, 40, congested={2010, 2011, 2025})
        fit = m.solve(true_utc(end))
        self.assertTrue(fit["valid"], fit["reason"])
        self.assertLess(fit["bins_used"], fit["bins"])
        error = m.to_utc_ns(end) - true_utc(end)
        self.assertLess(abs(error - FLOOR), 150_000)

    def test_32_bit_microsecond_wrap_is_unwrapped(self):
        m = ClockModel()
        start = WRAP_32BIT_US_NS / NS_PER_S - 20      # 20 s before the counter rolls over
        end = feed(m, start, 40)
        fit = m.solve(true_utc(end))
        self.assertTrue(fit["valid"], fit["reason"])
        self.assertEqual(m.wraps, 1)
        self.assertEqual(m.resets, 0)
        # a header stamp from just before the wrap, given in raw (wrapped) form
        before = end - 30 * NS_PER_S
        self.assertLess(abs(m.to_utc_ns(before % WRAP_32BIT_US_NS) - true_utc(before) - FLOOR), 200_000)
        self.assertLess(abs(m.to_utc_ns(end % WRAP_32BIT_US_NS) - true_utc(end) - FLOOR), 200_000)

    def test_camera_restart_starts_a_new_model(self):
        m = ClockModel()
        feed(m, 3000, 20)
        later = OFFSET + 3100 * NS_PER_S   # the camera rebooted: device time restarts near zero
        end = feed(m, 5, 4, offset=later)
        self.assertEqual(m.resets, 1)
        self.assertEqual(m.wraps, 0)
        self.assertFalse(m.solve(true_utc(end, offset=later))["valid"])

    def test_stale_samples_invalidate(self):
        m = ClockModel(stale_s=3.0)
        end = feed(m, 100, 20)
        self.assertTrue(m.solve(true_utc(end))["valid"])
        fit = m.solve(true_utc(end) + 5 * NS_PER_S)
        self.assertFalse(fit["valid"])
        self.assertEqual(fit["reason"], "IMU samples stale")

    def test_published_dictionary_maps_like_the_model(self):
        m = ClockModel()
        end = feed(m, 400, 20)
        published = m.solve(true_utc(end))
        for device in (end, end - 7 * NS_PER_S, end + NS_PER_S // 2):
            self.assertEqual(map_device_ns(published, device), m.to_utc_ns(device))

    def test_unwrap_picks_the_nearest_representation(self):
        w = WRAP_32BIT_US_NS
        self.assertEqual(unwrap_near(5, w + 3, w), w + 5)
        self.assertEqual(unwrap_near(w - 5, w + 3, w), w - 5)
        self.assertEqual(unwrap_near(123, 456, 0), 123)


if __name__ == "__main__":
    unittest.main()
