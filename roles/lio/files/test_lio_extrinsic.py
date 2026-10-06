"""Tests for lio_extrinsic: the static tree, FAST-LIO's convention, the legacy stand-in.

Run: python3 -m unittest test_lio_extrinsic (from this directory).
"""
import math
import unittest

import numpy as np

import lio_extrinsic as lx


def quaternion_about_y(angle):
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


class StaticTreeTest(unittest.TestCase):
    def setUp(self):
        # base_link -> avia_link (pitched 45 deg) -> lidar -> imu, as the A-S+ description has it
        self.tree = lx.StaticTree()
        self.tree.add("base_link", "avia_mount_link", (0.14766, 0, 0.09325), (0, 0, 0, 1))
        self.tree.add("avia_mount_link", "avia_link", (0.035707, 0, -0.039474), quaternion_about_y(math.pi / 4))
        self.tree.add("avia_link", "avia_nominal_lidar_frame", (0.0525, 0, 0.0324), (0, 0, 0, 1))
        self.tree.add("avia_nominal_lidar_frame", "avia_imu_frame", lx.LEGACY_IMU_IN_LIDAR, (0, 0, 0, 1))

    def test_lookup_chains_and_inverts(self):
        t_base_lidar = self.tree.lookup("base_link", "avia_nominal_lidar_frame")
        np.testing.assert_allclose(t_base_lidar[:3, 3], [0.243397, 0, 0.039565], atol=1e-5)   # the TF check of 2026-10-06
        t_lidar_base = self.tree.lookup("avia_nominal_lidar_frame", "base_link")
        np.testing.assert_allclose(t_lidar_base @ t_base_lidar, np.eye(4), atol=1e-12)
        self.assertIsNone(self.tree.lookup("base_link", "nowhere"))

    def test_fastlio_extrinsic_is_the_lidar_in_the_imu_frame(self):
        t, r, legacy = lx.extrinsic(self.tree, "avia_imu_frame", "avia_nominal_lidar_frame")
        self.assertFalse(legacy)
        np.testing.assert_allclose(t, [0.04165, 0.02326, -0.0284], atol=1e-12)    # FAST-LIO's avia.yaml
        np.testing.assert_allclose(r, np.eye(3).reshape(-1), atol=1e-12)

    def test_legacy_bags_get_the_factory_offset_only_when_allowed(self):
        old = lx.StaticTree()
        old.add("base_link", "avia_link", (0.2, 0, 0.1), (0, 0, 0, 1))
        old.add("avia_link", "avia_nominal_lidar_frame", (0.0525, 0, 0.0324), (0, 0, 0, 1))
        self.assertIsNone(lx.extrinsic(old, "avia_imu_frame", "avia_nominal_lidar_frame"))
        t, r, legacy = lx.extrinsic(old, "avia_imu_frame", "avia_nominal_lidar_frame", allow_legacy=True)
        self.assertTrue(legacy)
        np.testing.assert_allclose(t, [0.04165, 0.02326, -0.0284], atol=1e-12)

    def test_rendered_file_is_fastlio_parameters(self):
        text = lx.render([0.04165, 0.02326, -0.0284], list(np.eye(3).reshape(-1)), "test")
        self.assertIn("/**:\n  ros__parameters:\n    mapping:\n", text)
        self.assertIn("extrinsic_T: [0.04165, 0.02326, -0.0284]", text)
        self.assertIn("extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]", text)


if __name__ == "__main__":
    unittest.main()
