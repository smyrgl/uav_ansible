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


# gnss_main_link in base_link (x950_description): what the scorer reads from a bag's /tf_static.
MAIN_ANTENNA = (-0.334278, 0.336696, 0.12298)

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
        lever = np.array(MAIN_ANTENNA)
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
        lever = np.array(MAIN_ANTENNA)
        ref_t, ref_xyz, ref_yaw = self.reference(t, p, yaw, lever)
        drift = np.outer((t - t[0]) / et.NS, [0.01, 0.0, 0.0])          # 1 cm/s along x: 0.6 m over the minute
        est_p = p + drift + rng.normal(0, 0.02, p.shape)
        est_yaw = et.wrap(yaw + math.radians(3.0) + rng.normal(0, math.radians(0.2), len(yaw)))   # 3 deg heading bias
        s = et.score_estimator(t, est_p, yaw_quat(est_yaw), lever, ref_t, ref_xyz, ref_yaw)
        self.assertGreater(s["ape"]["rms"], 0.1)                      # the drift dominates
        self.assertLess(s["rpe_1s"]["rms"], 0.06)                     # 1 cm per second plus noise
        self.assertAlmostEqual(s["yaw"]["offset_deg_median"], 3.0, delta=0.3)
        self.assertLess(s["yaw"]["spread_deg_mad"], 0.5)

    def test_tilted_estimator_frame_is_tilt_not_drift(self):
        """FAST-LIO's camera_init is its first IMU pose: a 5 deg tilt gives a large 4-DoF APE while the
        SE(3) fit removes it, and the tilt itself is reported."""
        t, p, yaw = circle_flight()
        lever = np.array(MAIN_ANTENNA)
        ref_t, ref_xyz, ref_yaw = self.reference(t, p, yaw, lever)
        a = math.radians(5.0)
        tilt = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])
        est_p = p @ tilt.T                   # positions expressed in a frame rolled 5 deg about x
        s = et.score_estimator(t, est_p, yaw_quat(yaw), lever, ref_t, ref_xyz, ref_yaw)
        self.assertGreater(s["ape"]["rms"], 0.5)                       # 20 m radius x sin(5 deg) ~ 1.7 m peak
        self.assertAlmostEqual(s["alignment"]["se3_tilt_deg"], 5.0, delta=0.15)   # the lever arm still turns with the body
        self.assertLess(s["ape_se3"]["rms"], 0.2)                     # the antenna lever arm still rotates with the body

    def test_only_fixed_ambiguity_headings_enter_the_yaw_reference(self):
        from types import SimpleNamespace as NS
        t0 = 1_791_200_000 * et.NS
        def msg(i, **kw):
            ns = t0 + i * et.NS // 50
            return NS(header=NS(stamp=NS(sec=ns // et.NS, nanosec=ns % et.NS)), **kw)
        pvt = [msg(i, mode=4, latitude=math.radians(40.0) + i * 1e-7, longitude=math.radians(-105.0), height=1600.0,
                   h_accuracy=1) for i in range(100)]
        att = [msg(i, error=0, mode=(2 if i < 50 else 1), heading=90.0) for i in range(100)]   # fixed, then float
        bag = {"/pvt": pvt, "/att": att}
        _t, _xyz, heading, info = et.reference_from_bag(bag, "/pvt", "/att", 0.0, False)
        self.assertAlmostEqual(float(heading[1][0]), math.radians(90.0))        # ENU yaw as published: 90 stays 90
        _t, _xyz, compass, _i = et.reference_from_bag(bag, "/pvt", "/att", 0.0, False, heading_convention="compass")
        self.assertAlmostEqual(float(compass[1][0]), 0.0)                        # compass 90 = east = ENU yaw 0
        self.assertAlmostEqual(info["heading_fixed_fraction"], 0.5)
        self.assertEqual(len(heading[0]), 50)                                   # only the fixed half, on its own stamps
        self.assertAlmostEqual(info["heading_coverage"], 0.53, delta=0.02)     # the 60 ms nearest-sample window carries 3 samples past the last fixed one
        _t, _xyz, heading_all, info_all = et.reference_from_bag(bag, "/pvt", "/att", 0.0, False, allow_float_heading=True)
        self.assertEqual(len(heading_all[0]), 100)
        self.assertAlmostEqual(info_all["heading_coverage"], 1.0, delta=0.02)

    def test_low_rate_heading_is_interpolated_on_its_own_stamps(self):
        """A 10 Hz heading during a 60 deg/s yaw: snapping it to the 20 Hz pose grid would be 25 ms off
        (1.5 deg per sample, 3 deg over a 1 s pair); interpolation on its own stamps is exact."""
        t, p, yaw = circle_flight(n=1200, hz=20.0)
        lever = np.array(MAIN_ANTENNA)
        ref_t, ref_xyz, _ = self.reference(t, p, yaw, lever)
        turn = np.linspace(0, math.radians(60.0) * (t[-1] - t[0]) / et.NS, len(t))   # 60 deg/s on top of the tangent
        yaw_fast = et.wrap(yaw + turn)
        head_t = t[::2] + 25_000_000                                            # 10 Hz, 25 ms off the pose stamps
        head_yaw = et.wrap(np.interp(head_t, t, np.unwrap(yaw_fast)))
        s = et.score_estimator(t, p, yaw_quat(yaw_fast), lever, ref_t, ref_xyz, (head_t, head_yaw))
        self.assertLess(s["rpe_1s"]["yaw_deg"]["max"], 0.1)
        self.assertLess(s["yaw"]["spread_deg_mad"], 0.1)

    def test_heading_stamp_lag_is_fitted_and_removed(self):
        """Headings stamped 80 ms late during a 60 deg/s yaw: the raw 1 s yaw RPE is several degrees,
        the fit finds the lag and the compensated metric is clean."""
        t, p, yaw = circle_flight(n=1200, hz=20.0)
        lever = np.array(MAIN_ANTENNA)
        ref_t, ref_xyz, _ = self.reference(t, p, yaw, lever)
        rate = math.radians(60.0) * np.sin(np.linspace(0, 6 * math.pi, len(t)))      # yaw rate swinging +-60 deg/s
        yaw_fast = et.wrap(yaw + np.cumsum(rate) * (t[1] - t[0]) / et.NS)
        head_t = t + 80_000_000                                                      # stamped 80 ms late
        s = et.score_estimator(t, p, yaw_quat(yaw_fast), lever, ref_t, ref_xyz, (head_t, yaw_fast))
        self.assertGreater(s["rpe_1s"]["yaw_deg"]["rms"], 1.0)
        self.assertAlmostEqual(s["yaw"]["heading_lag_fit_s"], 0.08, delta=0.011)
        self.assertLess(s["rpe_1s"]["yaw_deg_at_fitted_lag"]["rms"], 0.3)

    def test_stationary_reference_is_reported_not_scored(self):
        t = (1_791_200_000 * et.NS + np.arange(200) * et.NS // 10).astype(np.int64)
        p = np.zeros((200, 3)); yaw = np.zeros(200)
        s = et.score_estimator(t, p, yaw_quat(yaw), MAIN_ANTENNA, t, p + 0.01, yaw)
        self.assertIn("alignment not defined", s["skipped"])

    def test_reference_gaps_mask_samples(self):
        t, p, yaw = circle_flight(n=600)
        ref_t, ref_xyz, ref_yaw = self.reference(t, p, yaw, MAIN_ANTENNA)
        keep = np.ones(len(t), bool); keep[200:300] = False                     # a 5 s hole in the reference
        s = et.score_estimator(t, p, yaw_quat(yaw), MAIN_ANTENNA, ref_t[keep], ref_xyz[keep], ref_yaw[keep])
        self.assertLess(s["matched"], len(t)); self.assertGreater(s["matched"], 400)

    def test_interpolation_wraps_yaw(self):
        ref_t = np.array([0, et.NS], dtype=np.int64)
        ref_yaw = np.array([math.radians(175.0), math.radians(-175.0)])
        _xyz, yaw, ok = et.interpolate_reference(ref_t, np.zeros((2, 3)), ref_yaw, np.array([et.NS // 2]), 2 * et.NS)
        self.assertAlmostEqual(abs(float(yaw[0])), math.pi, places=6)
        self.assertTrue(bool(ok[0]))


class LeverArmFromTf(unittest.TestCase):
    def test_chains_the_recorded_static_transforms(self):
        from types import SimpleNamespace as N
        def tf(parent, child, xyz, q=(0.0, 0.0, 0.0, 1.0)):
            return N(header=N(frame_id=parent), child_frame_id=child,
                     transform=N(translation=N(x=xyz[0], y=xyz[1], z=xyz[2]), rotation=N(x=q[0], y=q[1], z=q[2], w=q[3])))
        yaw = (0.0, 0.0, 0.9976, 0.0698)              # the main mount's recorded rotation (2026-10-05 bag)
        message = N(transforms=[tf("base_link", "gnss_main_mount_link", (-0.334278, 0.336696, 0.0925), yaw),
                                tf("gnss_main_mount_link", "gnss_main_link", (0.0, 0.0, 0.03048))])
        lever = et.lever_arm_from_tf_static([message], "base_link", "gnss_main_link")
        np.testing.assert_allclose(lever, MAIN_ANTENNA, atol=1e-5)
        self.assertIsNone(et.lever_arm_from_tf_static([message], "base_link", "gnss_aux_link"))


if __name__ == "__main__":
    unittest.main()
