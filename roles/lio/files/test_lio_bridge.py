"""Pure parts of the lio bridge: the IMU's mount, central velocity, the anchor."""
import math
import unittest

from lio_bridge import CentralVelocity, anchor, base_from_imu, quat_multiply, rotate, twist_covariance

LEVER = (0.22035, -0.02626, 0.1384)
LEVEL = (LEVER, (0.0, 0.0, 0.0, 1.0))                       # the Avia's old level mount
# The A-S+ cage: the Avia IMU in base_link, 45 deg nose-down (what /tf_static gives the bridge).
PITCHED = ((0.234037, -0.02326, 0.089096), (0.0, math.sin(math.pi / 8), 0.0, math.cos(math.pi / 8)))


def yaw(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def imu_pose(base_position, base_quaternion, mount):
    """The IMU's pose for a base_link pose: T_w_imu = T_w_base T_base_imu."""
    r, q_mount = mount
    return (tuple(a + b for a, b in zip(base_position, rotate(base_quaternion, r))),
            quat_multiply(base_quaternion, q_mount))


class Lever(unittest.TestCase):
    def test_level_pose_subtracts_the_lever_arm(self):
        base, q = base_from_imu((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), LEVEL)
        for got, want in zip(base + q, (1.0 - LEVER[0], 2.0 - LEVER[1], 3.0 - LEVER[2], 0.0, 0.0, 0.0, 1.0)):
            self.assertAlmostEqual(got, want)

    def test_pitched_mount_round_trips_any_base_pose(self):
        q_wb = quat_multiply(yaw(1.1), (math.sin(0.05), 0.0, 0.0, math.cos(0.05)))     # yawed, 5.7 deg of roll
        p, q = imu_pose((4.0, -2.0, 30.0), q_wb, PITCHED)
        base, q_base = base_from_imu(p, q, PITCHED)
        for got, want in zip(base + q_base, (4.0, -2.0, 30.0) + q_wb):
            self.assertAlmostEqual(got, want)

    def test_level_flight_stays_level_with_the_pitched_imu(self):
        """A level 10 m leg: the old axes-aligned lever arm would have read the IMU's 45 deg as a climb."""
        start, end = imu_pose((0.0, 0.0, 5.0), yaw(0.0), PITCHED), imu_pose((10.0, 0.0, 5.0), yaw(0.0), PITCHED)
        (a, qa), (b, qb) = base_from_imu(*start, PITCHED), base_from_imu(*end, PITCHED)
        self.assertAlmostEqual(b[0] - a[0], 10.0)
        self.assertAlmostEqual(b[2] - a[2], 0.0)
        self.assertAlmostEqual(qb[3], 1.0)                                              # base_link level

    def test_pure_yaw_about_base_link_gives_no_base_velocity(self):
        # The vehicle spins about base_link at 60 deg/s: the (pitched) IMU moves on a
        # circle, base_link must not.
        c, frame = CentralVelocity(), 100_000_000
        out = None
        for i in range(3):
            p, q = imu_pose((0.0, 0.0, 0.0), yaw(math.radians(60) * i * 0.1), PITCHED)   # base_link at the origin
            out = c.add(i * frame, *base_from_imu(p, q, PITCHED))
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
