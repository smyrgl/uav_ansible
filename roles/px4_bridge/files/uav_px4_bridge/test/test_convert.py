import math
import unittest

from uav_px4_bridge.convert import (VELOCITY_FRAME_BODY_FRD, VELOCITY_FRAME_NED, battery_fields, imu_fields,
                                    local_position_fields, odometry_fields, stamp_from_px4)
from uav_px4_bridge.frames import (frd_to_flu, ned_to_enu, px4_to_ros_orientation, px4_to_ros_rotation,
                                   quaternion_from_rotation, rotation_from_px4_quaternion, yaw_from_rotation)


def q_from_euler_zyx(yaw, pitch, roll):
    """PX4/px4_ros_com quaternion_from_euler convention (Z-Y-X), returns (w, x, y, z)."""
    cy, sy, cp, sp, cr, sr = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2), math.cos(roll / 2), math.sin(roll / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def q_mul(a, b):   # (w,x,y,z) Hamilton product
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


NED_ENU_Q = q_from_euler_zyx(math.pi / 2, 0.0, math.pi)          # px4_ros_com: quaternion_from_euler(pi, 0, pi/2)
AIRCRAFT_BASELINK_Q = q_from_euler_zyx(0.0, 0.0, math.pi)        # quaternion_from_euler(pi, 0, 0)


class TestFrames(unittest.TestCase):
    def test_vectors(self):
        self.assertEqual(ned_to_enu((1, 2, 3)), (2.0, 1.0, -3.0))
        self.assertEqual(frd_to_flu((1, 2, 3)), (1.0, -2.0, -3.0))

    def test_identity_attitude_faces_north_in_enu(self):
        yaw = yaw_from_rotation(px4_to_ros_rotation((1, 0, 0, 0)))
        self.assertAlmostEqual(yaw, math.pi / 2, places=9)

    def test_px4_east_heading_is_ros_yaw_zero(self):
        q = q_from_euler_zyx(math.pi / 2, 0.0, 0.0)          # PX4 yaw 90 deg = east
        self.assertAlmostEqual(yaw_from_rotation(px4_to_ros_rotation(q)), 0.0, places=9)

    def test_matches_px4_ros_com_quaternion_composition(self):
        for yaw, pitch, roll in ((0.3, -0.2, 0.1), (2.0, 0.5, -1.0), (-2.5, 0.0, 3.0)):
            q = q_from_euler_zyx(yaw, pitch, roll)
            ref = q_mul(q_mul(NED_ENU_Q, q), AIRCRAFT_BASELINK_Q)      # w,x,y,z
            ours = px4_to_ros_orientation(q)                            # x,y,z,w
            got = (ours[3], ours[0], ours[1], ours[2])
            sign = 1.0 if sum(a * b for a, b in zip(ref, got)) >= 0 else -1.0
            for a, b in zip(ref, got):
                self.assertAlmostEqual(a, sign * b, places=9)

    def test_quaternion_matrix_round_trip(self):
        q = q_from_euler_zyx(1.0, -0.4, 0.7)
        r = rotation_from_px4_quaternion(q)
        back = quaternion_from_rotation(r)
        self.assertAlmostEqual(abs(back[3] * q[0] + back[0] * q[1] + back[1] * q[2] + back[2] * q[3]), 1.0, places=9)


class TestConvert(unittest.TestCase):
    def test_stamp_epoch_detection(self):
        ns, synced = stamp_from_px4(1_790_745_190_486_074, 5)
        self.assertTrue(synced); self.assertEqual(ns, 1_790_745_190_486_074_000)
        ns, synced = stamp_from_px4(123_456_789, 42)
        self.assertFalse(synced); self.assertEqual(ns, 42)

    def test_imu_without_attitude_marks_orientation_unknown(self):
        f = imu_fields((0.1, 0.2, 0.3), (0.0, 0.0, -9.81))
        self.assertEqual(f["orientation_covariance"][0], -1.0)
        self.assertEqual(f["angular_velocity"], (0.1, -0.2, -0.3))
        self.assertEqual(f["linear_acceleration"], (0.0, 0.0, 9.81))

    def test_odometry_ned_velocity_ends_in_body_flu(self):
        # level, heading east (PX4 yaw 90): NED velocity (0, 2, 0) = east = straight ahead
        q = q_from_euler_zyx(math.pi / 2, 0.0, 0.0)
        f = odometry_fields((10, 20, -5), q, (0, 2, 0), VELOCITY_FRAME_NED, (0, 0, 0.1), (1, 2, 3), (0.1, 0.2, 0.3), (4, 5, 6))
        self.assertEqual(f["position"], (20.0, 10.0, 5.0))
        self.assertAlmostEqual(f["linear"][0], 2.0, places=9); self.assertAlmostEqual(f["linear"][1], 0.0, places=9)
        self.assertEqual(f["angular"], (0.0, 0.0, -0.1))
        self.assertEqual(f["pose_covariance"][0], 2.0)        # ENU x variance = NED east variance
        f2 = odometry_fields((0, 0, 0), q, (1, 0.5, 0), VELOCITY_FRAME_BODY_FRD, (0, 0, 0), (1, 1, 1), (1, 1, 1), (1, 1, 1))
        self.assertEqual(f2["linear"], (1.0, -0.5, 0.0))

    def test_local_position_invalid_axes_are_nan(self):
        f = local_position_fields(1, 2, 3, 4, 5, 6, True, False, False, True)
        self.assertEqual(f["position"][:2], (2.0, 1.0)); self.assertTrue(math.isnan(f["position"][2]))
        self.assertTrue(math.isnan(f["linear"][0])); self.assertEqual(f["linear"][2], -6.0)

    def test_battery_sign_and_unknowns(self):
        f = battery_fields(24.9, 3.2, 0.85, 12000, 1500, float("nan"), 6, [4.1] * 6 + [0.0] * 8, True, 0, 1200)
        self.assertEqual(f["current"], -3.2)            # discharging is negative in ROS
        self.assertAlmostEqual(f["charge"], 10.5); self.assertEqual(f["percentage"], 0.85)
        self.assertEqual(len(f["cell_voltage"]), 6); self.assertEqual(f["power_supply_status"], 2)
        g = battery_fields(0.0, -1.0, -1.0, 0, -1, float("nan"), 0, [0.0] * 14, False, 4, float("nan"))
        self.assertTrue(math.isnan(g["voltage"]) and math.isnan(g["current"]) and math.isnan(g["percentage"]))
        self.assertEqual(g["power_supply_health"], 5); self.assertFalse(g["present"])


if __name__ == "__main__":
    unittest.main()
