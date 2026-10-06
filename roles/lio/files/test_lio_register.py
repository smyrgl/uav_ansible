"""Registration of a second LiDAR with FAST-LIO's poses: interpolation, deskew, gating, decoding."""
import math
import unittest
from types import SimpleNamespace

import numpy as np

from lio_register import PoseBuffer, decode, quaternion_matrices, range_gate, register, sensor_in_imu, slerp

# The Avia IMU in base_link in the A-S+ cage (45 deg nose-down): what /tf_static gives the node.
IMU_ROTATION = quaternion_matrices(np.array([[0.0, math.sin(math.pi / 8), 0.0, math.cos(math.pi / 8)]]))[0]
IMU_TRANSLATION = np.array([0.234037, -0.02326, 0.089096])
E1R_ROTATION = quaternion_matrices(np.array([[0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)]]))[0]   # pitch +90 deg
E1R_TRANSLATION = np.array([-0.084, 0.0, -0.079312])         # base_link -> e1r_nominal_lidar_frame (URDF)


def yaw_quaternion(angle, tilt=None):
    """Rotation by angle about world z, applied after a fixed tilt (x, y, z, w)."""
    q = np.array([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
    if tilt is None:
        return q
    ax, ay, az, aw = q
    bx, by, bz, bw = tilt
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


class Rotations(unittest.TestCase):
    def test_quaternion_matrix_turns_x_into_y_for_a_quarter_yaw(self):
        r = quaternion_matrices(yaw_quaternion(math.pi / 2)[None])[0]
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)

    def test_e1r_mount_looks_down(self):
        np.testing.assert_allclose(E1R_ROTATION @ [1, 0, 0], [0, 0, -1], atol=1e-12)   # boresight
        np.testing.assert_allclose(E1R_ROTATION @ [0, 0, 1], [1, 0, 0], atol=1e-12)    # sensor up = forward

    def test_slerp_endpoints_midpoint_and_sign(self):
        q0, q1 = yaw_quaternion(0.0)[None], yaw_quaternion(math.pi / 2)[None]
        np.testing.assert_allclose(slerp(q0, q1, np.array([0.0])), q0, atol=1e-12)
        np.testing.assert_allclose(slerp(q0, q1, np.array([1.0])), q1, atol=1e-12)
        mid = slerp(q0, q1, np.array([0.5]))
        np.testing.assert_allclose(mid, yaw_quaternion(math.pi / 4)[None], atol=1e-12)
        np.testing.assert_allclose(slerp(q0, -q1, np.array([0.5])), mid, atol=1e-12)   # same rotation, other sign
        np.testing.assert_allclose(slerp(q0, q0, np.array([0.3])), q0, atol=1e-12)     # no rotation


class Buffer(unittest.TestCase):
    def test_interpolates_inside_and_refuses_gaps_and_extrapolation(self):
        b = PoseBuffer(max_gap_s=0.25)
        for t, x in ((0.0, 0.0), (0.1, 1.0), (0.2, 2.0), (0.6, 6.0), (0.7, 7.0)):
            self.assertTrue(b.add(int(t * 1e9), (x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
        times = np.array([-0.05, 0.0, 0.15, 0.2, 0.4, 0.65, 0.75]) * 1e9
        valid, positions, rotations = b.interpolate(times.astype(np.int64))
        # before the first pose (and on it: nothing older), inside, on a pose, across the gap, inside, after the last
        self.assertEqual(valid.tolist(), [False, False, True, True, False, True, False])
        np.testing.assert_allclose(positions[:, 0], [1.5, 2.0, 6.5], atol=1e-9)
        self.assertEqual(rotations.shape, (3, 3, 3))

    def test_time_going_backwards_starts_over(self):
        b = PoseBuffer()
        b.add(1_000_000_000, (0, 0, 0), (0, 0, 0, 1))
        b.add(1_100_000_000, (0, 0, 0), (0, 0, 0, 1))
        self.assertFalse(b.add(500_000_000, (0, 0, 0), (0, 0, 0, 1)))
        self.assertEqual(b.latest(), 500_000_000)
        self.assertEqual(len(b.t), 1)

    def test_keeps_only_recent_poses(self):
        b = PoseBuffer(keep_s=1.0)
        for i in range(30):
            b.add(i * 100_000_000, (0, 0, 0), (0, 0, 0, 1))
        self.assertEqual(int(b.t[0]), 1_900_000_000)


class Deskew(unittest.TestCase):
    def test_recovers_world_points_under_constant_motion(self):
        """10 Hz poses of the IMU moving at constant velocity and yaw rate (tilted 10 deg); E1R points fired at
        many times between them, through the E1R's and the (pitched) IMU's mounts, land back on the same world points."""
        v, w = np.array([15.0, -5.0, 2.0]), 1.5                                       # 16 m/s, 86 deg/s of yaw
        tilt = (math.sin(math.radians(5)), 0.0, 0.0, math.cos(math.radians(5)))       # 10 deg roll
        b = PoseBuffer()
        for k in range(11):
            t = k * 0.1
            b.add(int(round(t * 1e9)), tuple(v * t + [10.0, 5.0, 30.0]), tuple(yaw_quaternion(w * t, tilt)))
        rng = np.random.default_rng(1)
        world = rng.uniform([-20, -20, 0], [20, 20, 5], size=(4000, 3))
        times = rng.choice(np.arange(0.13, 0.85, 216e-6), size=len(world))            # 216 us firing slots
        times_ns = np.round(times * 1e9).astype(np.int64)
        sensor = np.empty_like(world)
        for i, (p_w, t) in enumerate(zip(world, times_ns / 1e9)):
            r_t = quaternion_matrices(yaw_quaternion(w * t, tilt)[None])[0]
            body = r_t.T @ (p_w - (v * t + [10.0, 5.0, 30.0]))
            in_base = IMU_ROTATION @ body + IMU_TRANSLATION                          # the IMU (body) frame -> base_link
            sensor[i] = E1R_ROTATION.T @ (in_base - E1R_TRANSLATION)
        mount = sensor_in_imu((IMU_ROTATION, IMU_TRANSLATION), (E1R_ROTATION, E1R_TRANSLATION))   # as the node does
        out, kept = register(sensor, times_ns, b, *mount)
        self.assertTrue(kept.all())
        np.testing.assert_allclose(out, world, atol=1e-6)

    def test_a_level_imu_reduces_to_the_old_lever_arm(self):
        lever = np.array([0.22035, -0.02626, 0.1384])
        r, t = sensor_in_imu((np.eye(3), lever), (E1R_ROTATION, E1R_TRANSLATION))
        np.testing.assert_allclose(r, E1R_ROTATION, atol=1e-12)
        np.testing.assert_allclose(t, E1R_TRANSLATION - lever, atol=1e-12)

    def test_ignoring_the_imu_pitch_would_misplace_the_e1r(self):
        """With the Avia pitched 45 deg, the old axes-aligned lever arm puts the E1R tens of cm off."""
        right = sensor_in_imu((IMU_ROTATION, IMU_TRANSLATION), (E1R_ROTATION, E1R_TRANSLATION))
        wrong = (E1R_ROTATION, E1R_TRANSLATION - IMU_TRANSLATION)
        self.assertGreater(np.linalg.norm(right[1] - wrong[1]), 0.2)
        self.assertAlmostEqual(math.degrees(math.acos((np.trace(right[0].T @ wrong[0]) - 1) / 2)), 45.0, places=6)

    def test_ignoring_the_motion_would_smear(self):
        """The same points with one pose for the whole frame miss by metres: the deskew matters."""
        b = PoseBuffer()
        b.add(0, (0.0, 0.0, 0.0), (0, 0, 0, 1))
        b.add(100_000_000, (2.0, 0.0, 0.0), (0, 0, 0, 1))                            # 20 m/s
        out, _ = register(np.zeros((2, 3)), np.array([10_000_000, 90_000_000]), b, np.eye(3), np.zeros(3))
        np.testing.assert_allclose(out[:, 0], [0.2, 1.8], atol=1e-9)


class Inputs(unittest.TestCase):
    def test_range_gate_drops_no_returns_nan_and_the_near_field(self):
        xyz = np.array([[0, 0, 0], [np.nan, 0, 0], [0.05, 0, 0], [0.2, 0, 0], [0, 50.0, 0]], float)
        self.assertEqual(range_gate(xyz, 0.1, 0.0).tolist(), [False, False, False, True, True])
        self.assertEqual(range_gate(xyz, 0.1, 40.0).tolist(), [False, False, False, True, False])

    def test_decode_reads_the_e1r_layout(self):
        dtype = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
                          "formats": ["<f4", "<f4", "<f4", "u1", "<u2", "<f8"],
                          "offsets": [0, 4, 8, 12, 14, 16], "itemsize": 24})
        a = np.zeros(2, dtype)
        a["x"], a["y"], a["z"], a["intensity"], a["timestamp"] = [0.5, 1.0], [2, 3], [-1, -2], [7, 255], [1790929500.000220, 1790929500.095954]
        field = lambda name, offset, kind: SimpleNamespace(name=name, offset=offset, datatype=kind)
        message = SimpleNamespace(fields=[field("x", 0, 7), field("y", 4, 7), field("z", 8, 7), field("intensity", 12, 2),
                                          field("ring", 14, 4), field("timestamp", 16, 8)],
                                  is_bigendian=False, point_step=24, width=2, height=1, data=a.tobytes())
        xyz, intensity, seconds = decode(message, "timestamp")
        np.testing.assert_allclose(xyz, [[0.5, 2, -1], [1.0, 3, -2]])
        self.assertEqual(intensity.tolist(), [7.0, 255.0])
        np.testing.assert_allclose(seconds, [1790929500.000220, 1790929500.095954], rtol=0, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
