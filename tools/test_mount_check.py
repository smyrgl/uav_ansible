"""Tests for mount_check: plane fits, the mount-error convention, depth back-projection.

Run: python3 -m unittest test_mount_check (from tools/, numpy required).
"""
import math
import unittest
from types import SimpleNamespace as N

import numpy as np

import mount_check as mc


def about_y(angle):
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


def about_x(angle):
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def compose(qa, qb):
    ax, ay, az, aw = qa
    bx, by, bz, bw = qb
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def urdf_tree():
    """The A-S+ description's sensor frames, as the URDF says they are."""
    t = mc.lx.StaticTree()
    t.add("base_link", "avia_nominal_lidar_frame", (0.2434, 0.0, 0.0396), about_y(math.radians(45)))
    t.add("avia_nominal_lidar_frame", "avia_imu", (-0.04165, -0.02326, 0.0284), (0, 0, 0, 1))
    t.add("base_link", "e1r_nominal_lidar_frame", (-0.084, 0.0, -0.0793), about_y(math.radians(90)))
    t.add("base_link", "fmu_housing_link", (0.0, 0.0, 0.05), (0, 0, 0, 1))
    return t


def floor_in_sensor(true_rotation, true_translation, height=0.276, n=4000, seed=1):
    """Points of the floor z = -height (base_link) in front of/below the sensor, in its frame."""
    rng = np.random.default_rng(seed)
    floor = np.stack([rng.uniform(-2, 4, n), rng.uniform(-2, 2, n), np.full(n, -height)], axis=1)
    return (floor - true_translation) @ true_rotation           # R^T (p - t), row-wise


class Convention(unittest.TestCase):
    def test_a_mount_pitched_further_down_reads_positive(self):
        up = np.array([0.0, 0.0, 1.0])
        e = mc.quaternion_matrix(about_y(math.radians(2.0)))     # true = E * URDF, E about +y: more nose-down
        _angle, pitch, roll = mc.mount_error(e.T @ up, up)
        self.assertAlmostEqual(pitch, 2.0, places=6)
        self.assertAlmostEqual(roll, 0.0, places=6)

    def test_a_mount_rolled_right_reads_positive(self):
        up = np.array([0.0, 0.0, 1.0])
        e = mc.quaternion_matrix(about_x(math.radians(1.5)))
        _angle, pitch, roll = mc.mount_error(e.T @ up, up)
        self.assertAlmostEqual(roll, 1.5, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)


class Check(unittest.TestCase):
    def collector(self, avia_error_deg=0.0, imu_error_deg=0.0):
        c = mc.Collector()
        c.tree = urdf_tree()
        q_avia = compose(about_y(math.radians(avia_error_deg)), about_y(math.radians(45)))     # the true Avia
        r_avia = mc.quaternion_matrix(q_avia)
        c.clouds["/avia/points"] = ("avia_nominal_lidar_frame", [floor_in_sensor(r_avia, np.array([0.2434, 0, 0.0396]))])
        r_e1r = mc.quaternion_matrix(about_y(math.radians(90)))
        c.clouds["/e1r/points"] = ("e1r_nominal_lidar_frame", [floor_in_sensor(r_e1r, np.array([-0.084, 0, -0.0793]))])
        # The Avia IMU at rest: specific force = 'up' in its true frame (the Avia's rotation, plus its own error).
        r_imu = mc.quaternion_matrix(compose(about_y(math.radians(imu_error_deg)), q_avia))
        f = r_imu.T @ np.array([0.0, 0.0, 9.81])
        c.imus["/avia/imu"] = ("avia_imu", [tuple(f)] * 100)
        c.attitude = [("fmu_housing_link", (0.0, 0.0, 0.0, 1.0))] * 50    # level
        return c

    def test_right_mounts_agree(self):
        r = mc.evaluate(self.collector())
        for topic in ("/avia/points", "/e1r/points"):
            plane = r["planes"][topic]
            self.assertAlmostEqual(plane["height_m"], 0.276, places=3)
            self.assertLess(plane["tilt_deg"], 0.05)
        self.assertLess(r["imus"]["/avia/imu"]["tilt_deg"], 0.05)
        self.assertFalse([p for p in r["problems"] if "/avia/" in p or "/e1r/" in p or "URDF" in p])

    def test_a_two_degree_avia_error_shows_in_its_plane_and_its_accelerometer(self):
        r = mc.evaluate(self.collector(avia_error_deg=2.0))
        self.assertAlmostEqual(r["planes"]["/avia/points"]["pitch_deg"], 2.0, places=1)
        self.assertAlmostEqual(r["planes"]["/e1r/points"]["pitch_deg"], 0.0, places=2)
        self.assertAlmostEqual(r["imus"]["/avia/imu"]["pitch_deg"], 2.0, places=1)

    def test_a_frame_missing_from_the_urdf_is_reported(self):
        c = self.collector()
        c.clouds["/x/points"] = ("nowhere", [np.zeros((10, 3))])
        r = mc.evaluate(c)
        self.assertTrue(any("nowhere" in p for p in r["problems"]))


class Depth(unittest.TestCase):
    def test_back_projection(self):
        depth = np.full((8, 16), 2000, dtype="<u2")                # 2 m everywhere
        image = N(height=8, width=16, step=32, encoding="16UC1", data=depth.tobytes())
        info = N(k=[10.0, 0, 8.0, 0, 10.0, 4.0, 0, 0, 1])
        xyz = mc.depth_xyz(image, info, stride=1)
        self.assertEqual(xyz.shape, (128, 3))
        np.testing.assert_allclose(xyz[:, 2], 2.0)
        np.testing.assert_allclose(xyz[0], [(0 - 8) * 2 / 10, (0 - 4) * 2 / 10, 2.0])


if __name__ == "__main__":
    unittest.main()
