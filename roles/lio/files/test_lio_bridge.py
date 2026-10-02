"""Pure parts of the lio bridge: lever arm and central velocity."""
import math
import unittest

from lio_bridge import CentralVelocity, anchor, base_from_imu, quat_multiply, rotate, twist_covariance

LEVER = (0.22035, -0.02626, 0.1384)


def yaw(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


class Lever(unittest.TestCase):
    def test_level_pose_subtracts_the_lever_arm(self):
        base = base_from_imu((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), LEVER)
        for got, want in zip(base, (1.0 - LEVER[0], 2.0 - LEVER[1], 3.0 - LEVER[2])):
            self.assertAlmostEqual(got, want)

    def test_pure_yaw_about_base_link_gives_no_base_velocity(self):
        # The vehicle spins about base_link at 60 deg/s: the IMU moves on a circle,
        # base_link must not.
        c, frame = CentralVelocity(), 100_000_000
        out = None
        for i in range(3):
            q = yaw(math.radians(60) * i * 0.1)
            imu = rotate(q, LEVER)                          # base_link stays at the origin
            out = c.add(i * frame, base_from_imu(imu, q, LEVER), q)
        _, _, _, v, _ = out
        for component in v:
            self.assertAlmostEqual(component, 0.0, places=9)

    def test_forward_flight_reads_as_body_x(self):
        c, frame = CentralVelocity(), 100_000_000
        q = yaw(math.radians(90))                          # nose along world +y
        for i in range(3):
            out = c.add(i * frame, (0.0, 0.5 * i, 0.0), q)  # 5 m/s along world +y
        _, _, _, (vx, vy, vz), _ = out
        self.assertAlmostEqual(vx, 5.0)
        self.assertAlmostEqual(vy, 0.0)

    def test_anchor_makes_the_source_pose_coincide_with_odom(self):
        odom_base = ((3.0, -1.0, 0.5), yaw(math.radians(30)))
        source_base = ((0.2, 0.4, -0.1), yaw(math.radians(-50)))
        p, q = anchor(odom_base, source_base)
        # T_odom_source * T_source_base == T_odom_base
        composed_p = tuple(a + b for a, b in zip(p, rotate(q, source_base[0])))
        composed_q = quat_multiply(q, source_base[1])
        for got, want in zip(composed_p + composed_q, odom_base[0] + odom_base[1]):
            self.assertAlmostEqual(got, want)

    def test_twist_covariance(self):
        cov = twist_covariance(0.05)
        self.assertAlmostEqual(cov[0], 0.0025)
        self.assertEqual(cov[35], 1e6)


if __name__ == "__main__":
    unittest.main()
