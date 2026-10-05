"""Synthetic checks of the estimator-truth metrics (numpy only; no bag needed)."""
import math
import unittest

import numpy as np

import estimator_truth as et


def yaw_quat(yaw):
    return np.stack([np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2), np.cos(yaw / 2)], -1)


def circle_flight(n=1200, radius=20.0, hz=20.0):
    """A level circle at 2 m/s-ish: base_link positions, tangent yaw, UTC stamps."""
    t0 = 1_791_200_000 * et.NS
    t = t0 + (np.arange(n) * et.NS / hz).astype(np.int64)
    a = np.linspace(0, 2 * math.pi, n)
    p = np.stack([radius * np.cos(a), radius * np.sin(a), 10 + 0.5 * np.sin(3 * a)], -1)
    yaw = et.wrap(a + math.pi / 2)
    return t, p, yaw


class Geometry(unittest.TestCase):
    def test_enu_scale_and_axes(self):
        lat0, lon0 = math.radians(40.0), math.radians(-105.0)
        enu = et.geodetic_to_enu([lat0 + 1e-5, lat0], [lon0, lon0 + 1e-5], [100.0, 101.0], lat0, lon0, 100.0)
        self.assertAlmostEqual(enu[0, 1], 63.62, delta=0.05)         # 1e-5 rad north: M(40 deg) = 6361.8 km
        self.assertAlmostEqual(enu[0, 0], 0.0, delta=1e-3)
        self.assertAlmostEqual(enu[1, 0], 48.93, delta=0.05)         # 1e-5 rad east: N cos(40 deg) = 4893 km
        self.assertAlmostEqual(enu[1, 2], 1.0, delta=1e-3)

    def test_heading_to_yaw(self):
        self.assertAlmostEqual(float(et.heading_to_yaw(0.0)), math.pi / 2)        # north
        self.assertAlmostEqual(float(et.heading_to_yaw(90.0)), 0.0)               # east
        self.assertAlmostEqual(abs(float(et.heading_to_yaw(270.0))), math.pi)     # west (either sign of pi)
        self.assertAlmostEqual(float(et.heading_to_yaw(10.0, offset_deg=80.0)), 0.0)

    def test_quaternion_yaw_and_matrix(self):
        yaw = np.array([0.0, math.pi / 2, -2.0])
        q = yaw_quat(yaw)
        np.testing.assert_allclose(et.quat_to_yaw(q), yaw, atol=1e-9)
        lever = np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(np.einsum("nij,j->ni", et.quat_to_matrix(q), lever)[1], [0.0, 1.0, 0.0], atol=1e-9)


class Alignment(unittest.TestCase):
    def test_yaw_alignment_recovers_frame_offset(self):
        t, p, yaw = circle_flight()
        r = et.rotz(0.7); shift = np.array([100.0, -50.0, 3.0])
        src = p @ r.T + shift            # the same track expressed in a frame rotated by -0.7 and shifted
        r_fit, t_fit, yaw_fit = et.align_yaw(src, p)
        self.assertAlmostEqual(yaw_fit, -0.7, places=6)
        np.testing.assert_allclose(src @ r_fit.T + t_fit, p, atol=1e-6)

    def test_umeyama_matches_on_full_motion(self):
        t, p, yaw = circle_flight()
        r = et.rotz(1.0); src = p @ r.T + 5.0
        r6, t6 = et.umeyama(src, p)
        np.testing.assert_allclose(src @ r6.T + t6, p, atol=1e-6)


class Scoring(unittest.TestCase):
    def reference(self, t, p, yaw, lever):
        rot = et.quat_to_matrix(yaw_quat(yaw))
        antenna = p + np.einsum("nij,j->ni", rot, np.asarray(lever))
        return t, antenna, yaw

    def test_perfect_estimator_in_a_rotated_frame_scores_zero(self):
        t, p, yaw = circle_flight()
        lever = np.array(et.DEFAULT_LEVER_ARM)
        ref_t, ref_xyz, ref_yaw = self.reference(t, p, yaw, lever)
        r = et.rotz(0.4); est_p = p @ r.T + np.array([10.0, 20.0, -1.0]); est_yaw = et.wrap(yaw + 0.4)
        s = et.score_estimator(t, est_p, yaw_quat(est_yaw), lever, ref_t, ref_xyz, ref_yaw)
        self.assertLess(s["ape"]["max"], 1e-6)
        self.assertLess(s["rpe_1s"]["rms"], 1e-6)
        self.assertAlmostEqual(s["alignment"]["yaw_deg"], math.degrees(-0.4), delta=0.001)   # reported to 3 decimals
        self.assertAlmostEqual(s["yaw"]["offset_deg_median"], 0.0, places=4)
        self.assertLess(s["rpe_1s"]["yaw_deg"]["max"], 1e-6)
        self.assertIn("se3_tilt_deg", s["alignment"])

    def test_noise_and_drift_show_up_where_expected(self):
        rng = np.random.default_rng(1)
        t, p, yaw = circle_flight()
        lever = np.array(et.DEFAULT_LEVER_ARM)
        ref_t, ref_xyz, ref_yaw = self.reference(t, p, yaw, lever)
        drift = np.outer((t - t[0]) / et.NS, [0.01, 0.0, 0.0])          # 1 cm/s along x: 0.6 m over the minute
        est_p = p + drift + rng.normal(0, 0.02, p.shape)
        est_yaw = et.wrap(yaw + math.radians(3.0) + rng.normal(0, math.radians(0.2), len(yaw)))   # 3 deg heading bias
        s = et.score_estimator(t, est_p, yaw_quat(est_yaw), lever, ref_t, ref_xyz, ref_yaw)
        self.assertGreater(s["ape"]["rms"], 0.1)                      # the drift dominates
        self.assertLess(s["rpe_1s"]["rms"], 0.06)                     # 1 cm per second plus noise
        self.assertAlmostEqual(s["yaw"]["offset_deg_median"], 3.0, delta=0.3)
        self.assertLess(s["yaw"]["spread_deg_mad"], 0.5)

    def test_stationary_reference_is_reported_not_scored(self):
        t = (1_791_200_000 * et.NS + np.arange(200) * et.NS // 10).astype(np.int64)
        p = np.zeros((200, 3)); yaw = np.zeros(200)
        s = et.score_estimator(t, p, yaw_quat(yaw), et.DEFAULT_LEVER_ARM, t, p + 0.01, yaw)
        self.assertIn("alignment not defined", s["skipped"])

    def test_reference_gaps_mask_samples(self):
        t, p, yaw = circle_flight(n=600)
        ref_t, ref_xyz, ref_yaw = self.reference(t, p, yaw, et.DEFAULT_LEVER_ARM)
        keep = np.ones(len(t), bool); keep[200:300] = False                     # a 5 s hole in the reference
        s = et.score_estimator(t, p, yaw_quat(yaw), et.DEFAULT_LEVER_ARM, ref_t[keep], ref_xyz[keep], ref_yaw[keep])
        self.assertLess(s["matched"], len(t)); self.assertGreater(s["matched"], 400)

    def test_interpolation_wraps_yaw(self):
        ref_t = np.array([0, et.NS], dtype=np.int64)
        ref_yaw = np.array([math.radians(175.0), math.radians(-175.0)])
        _xyz, yaw, ok = et.interpolate_reference(ref_t, np.zeros((2, 3)), ref_yaw, np.array([et.NS // 2]), 2 * et.NS)
        self.assertAlmostEqual(abs(float(yaw[0])), math.pi, places=6)
        self.assertTrue(bool(ok[0]))


if __name__ == "__main__":
    unittest.main()
