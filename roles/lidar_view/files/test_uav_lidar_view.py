#!/usr/bin/env python3
"""Tests of the LiDAR view's pure parts (no GPU, no ROS): python3 -m pytest test_uav_lidar_view.py"""
import math
import os
import struct
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import uav_lidar_view as lv  # noqa: E402


def project(view, proj, point):
    """Screen (u, v) of a world point, v = -1 top, +1 bottom (as ChaseCamera frames it)."""
    clip = proj @ view @ np.append(point, 1.0)
    return clip[0] / clip[3], -clip[1] / clip[3]


class GeometryTest(unittest.TestCase):
    def test_rpy_matches_quaternion(self):
        roll, pitch, yaw = 0.1, -0.4, 2.0
        cr, sr, cp, sp, cy, sy = (f(a / 2) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
        q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
        np.testing.assert_allclose(lv.rpy_matrix(roll, pitch, yaw), lv.quat_matrix(*q), atol=1e-12)
        self.assertAlmostEqual(lv.yaw_of(q), yaw)

    def test_e1r_mount_looks_down_with_its_vertical_fov_fore_and_aft(self):
        mount = lv.transform((-0.084, 0, -0.0793), (0, math.pi / 2, 0))
        np.testing.assert_allclose(mount[:3, :3] @ (1, 0, 0), (0, 0, -1), atol=1e-12)   # optical axis
        np.testing.assert_allclose(mount[:3, :3] @ (0, 0, 1), (1, 0, 0), atol=1e-12)    # its 90 deg axis: forward
        lines = lv.place(lv.pyramid_lines(lv.E1R_FOV), np.eye(4), mount, 10.0)
        far = lines[np.argmin(lines[:, 2])]
        self.assertAlmostEqual(far[2], -0.0793 - 10.0)
        self.assertAlmostEqual(abs(far[0] + 0.084), 10.0 * math.tan(math.radians(45)), places=6)
        self.assertAlmostEqual(abs(far[1]), 10.0 * math.tan(math.radians(60)), places=6)

    def test_look_at_puts_the_target_in_the_centre(self):
        view = lv.look_at((-5, 2, 3), (1, 1, 0))
        u, v = project(view, lv.perspective(50, 16 / 9, 0.3, 1000), np.array([1.0, 1.0, 0.0]))
        self.assertAlmostEqual(u, 0.0)
        self.assertAlmostEqual(v, 0.0)

    def test_base_from_imu_rotates_the_lever_arm(self):
        q = (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))      # yaw 90 deg
        p, q_base = lv.base_from_imu((10, 0, 1), q, lv.transform((0.2, 0, 0.1)))
        np.testing.assert_allclose(p, (10, -0.2, 0.9), atol=1e-12)
        np.testing.assert_allclose(q_base, q, atol=1e-12)

    def test_base_from_imu_takes_the_pitched_mount_out(self):
        """The Avia IMU 45 deg nose-down in base_link: a level aircraft's IMU pose is pitched, its base is not."""
        mount = lv.transform((0.234, -0.023, 0.089), (0, math.pi / 4, 0))
        world_base = lv.transform((5.0, 1.0, 20.0), (0, 0, 0.3))
        imu = world_base @ mount
        p, q = lv.base_from_imu(imu[:3, 3], lv.matrix_quat(imu[:3, :3]), mount)
        np.testing.assert_allclose(p, (5.0, 1.0, 20.0), atol=1e-12)
        np.testing.assert_allclose(lv.quat_matrix(*q), world_base[:3, :3], atol=1e-12)

    def test_frames_come_from_the_description_text(self):
        urdf = """<robot name="t"><link name="base_link"/><link name="avia_link"/><link name="avia_imu"/>
          <joint name="a" type="fixed"><parent link="base_link"/><child link="avia_link"/><origin xyz="0.18 0 0.05" rpy="0 0.785398 0"/></joint>
          <joint name="b" type="fixed"><parent link="avia_link"/><child link="avia_imu"/><origin xyz="0 0 0.1" rpy="0 0 0"/></joint></robot>"""
        frames = lv.urdf_frames(urdf)
        np.testing.assert_allclose(frames["avia_imu"][:3, 3], (0.18 + 0.1 * math.sin(0.785398), 0, 0.05 + 0.1 * math.cos(0.785398)), atol=1e-6)

    def test_slerp_halfway(self):
        q = lv.slerp((0, 0, 0, 1), (0, 0, math.sin(0.5), math.cos(0.5)), 0.5)
        self.assertAlmostEqual(lv.yaw_of(q), 0.5)


class ChaseCameraTest(unittest.TestCase):
    def frame(self, agl, yaw=0.7):
        camera = lv.ChaseCamera(fovy=50.0)
        position = np.array([3.0, -2.0, 10.0 + agl])
        eye, target = camera.update(position, yaw, agl, 0.0)
        view, proj = lv.look_at(eye, target), lv.perspective(50.0, 16 / 9, 0.3, 3000)
        _, v_aircraft = project(view, proj, position)
        _, v_nadir = project(view, proj, position - (0, 0, agl))
        return camera, eye, v_aircraft, v_nadir

    def test_aircraft_just_above_centre_and_the_ground_below_it_in_frame(self):
        for agl in (0.3, 2.0, 5.0, 12.0, 30.0, 50.0):
            camera, eye, v_aircraft, v_nadir = self.frame(agl)
            self.assertAlmostEqual(v_aircraft, -0.1, places=6, msg=agl)
            self.assertLessEqual(v_nadir, 0.8 + 1e-6, msg=agl)
            self.assertGreater(v_nadir, v_aircraft, msg=agl)

    def test_boom_grows_with_height_so_the_nadir_sits_at_the_bottom(self):
        short, *_ = self.frame(0.3)
        self.assertAlmostEqual(short.length, 7.0)
        for agl in (12.0, 30.0, 50.0):
            camera, _, _, v_nadir = self.frame(agl)
            self.assertGreater(camera.length, 7.0)
            self.assertAlmostEqual(v_nadir, 0.8, places=6, msg=agl)

    def test_camera_behind_along_the_heading(self):
        _, eye, _, _ = self.frame(5.0, yaw=math.pi / 2)
        self.assertAlmostEqual(eye[0], 3.0)
        self.assertLess(eye[1], -2.0)
        self.assertGreater(eye[2], 15.0)

    def test_heading_lag_takes_the_short_way_round(self):
        camera = lv.ChaseCamera()
        camera.update((0, 0, 0), math.radians(170), 1.0, 0.0)
        camera.update((0, 0, 0), math.radians(-170), 1.0, 0.1)
        self.assertGreater(math.degrees(camera.yaw), 170.0)


class PoseBufferTest(unittest.TestCase):
    def test_interpolates_and_shows_one_interval_behind(self):
        poses = lv.PoseBuffer()
        for i in range(10):
            self.assertTrue(poses.add(100.0 + 0.1 * i, (i, 0, 0), (0, 0, 0, 1), arrival=5000.0 + 0.1 * i + 0.05))
        self.assertFalse(poses.add(100.5, (0, 0, 0), (0, 0, 0, 1)))
        position, _ = poses.at(100.25)
        np.testing.assert_allclose(position, (2.5, 0, 0))
        shown = poses.render_time(5000.95)                # just after the newest pose (100.9) arrived
        self.assertAlmostEqual(shown, 100.9 - 0.13 - 0.02)
        self.assertLess(poses.render_time(5001.04), 100.9)    # still interpolating just before the next one

    def test_time_going_back_restarts(self):
        poses = lv.PoseBuffer()
        poses.add(100.0, (0, 0, 0), (0, 0, 0, 1))
        poses.add(50.0, (1, 0, 0), (0, 0, 0, 1))
        self.assertEqual(len(poses.poses), 1)


class MapTest(unittest.TestCase):
    def cloud(self, n, seed=0):
        rng = np.random.default_rng(seed)
        return np.hstack([rng.uniform(-5, 5, (n, 3)), rng.uniform(0, 100, (n, 1))]).astype(np.float32)

    def test_dedup_makes_resends_idempotent(self):
        dedup = lv.VoxelDedup(0.05)
        a = self.cloud(5000)
        first = dedup.add(a)
        self.assertEqual(len(first), len(dedup))
        self.assertEqual(len(dedup.add(a)), 0)
        b = self.cloud(5000, seed=1)
        both = dedup.add(np.concatenate([a, b]))
        self.assertEqual(len(first) + len(both), len(np.unique(lv.voxel_keys(np.concatenate([a, b])[:, :3].astype(float), 0.05))))
        self.assertTrue(np.all(np.diff(dedup.keys) > 0))

    def test_dedup_drops_nan_and_stops_at_capacity(self):
        dedup = lv.VoxelDedup(0.05, cap=100)
        cloud = self.cloud(1000)
        cloud[:10, 0] = np.nan
        self.assertEqual(len(dedup.add(cloud)), 100)
        self.assertEqual(len(dedup.add(self.cloud(1000, seed=3))), 0)

    def room(self, ceiling=2.6):
        """A 6 x 6 m room: floor, four walls and a ceiling, points every 5 cm."""
        g = np.arange(-3, 3, 0.05)
        h = np.arange(0, ceiling, 0.05)
        floor = np.array([(x, y, 0.0) for x in g for y in g])
        top = floor + (0, 0, ceiling)
        walls = np.array([p for x in g for z in h for p in ((x, -3, z), (x, 3, z), (-3, x, z), (3, x, z))])
        return np.concatenate([floor, top, walls])

    def ground(self, half=10.0):
        g = np.arange(-half, half, 0.1)
        return [(x, y, 0.0) for x in g for y in g]

    def test_enclosed_indoors(self):
        occupancy = lv.Occupancy()
        occupancy.add(self.room())
        known, covered = occupancy.enclosure(0.2, 0.1, 0.35, 5.0)       # on the floor, 0.35 m up
        self.assertGreater(covered, 12)
        self.assertGreater(covered, 0.7 * known)              # every column but the walls'

    def test_live_scan_tells_a_room_from_the_open(self):
        """On the bench the map has no ceiling over the aircraft yet; the live scan suffices."""
        def avia_view(points, position):
            d = points - position
            az = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
            el = np.degrees(np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1])))
            return points[(np.abs(az) < 35.2) & (np.abs(el) < 38.6)]
        aircraft = np.array([0.0, 0.0, 0.35])
        room = self.room(ceiling=2.6) * (2.0, 2.0, 1.0)       # 12 x 12 m: walls 6 m ahead
        self.assertTrue(lv.scan_enclosed(avia_view(room, aircraft), aircraft))
        rng = np.random.default_rng(5)
        field = np.stack([rng.uniform(0, 120, 60000), rng.uniform(-80, 80, 60000), np.zeros(60000)], 1)
        tree = np.stack([8 + rng.normal(0, 1.5, 5000), rng.normal(0, 1.5, 5000), rng.uniform(1, 9, 5000)], 1)
        self.assertFalse(lv.scan_enclosed(avia_view(np.concatenate([field, tree]), aircraft), aircraft))
        self.assertFalse(lv.scan_enclosed(None, aircraft))

    def test_not_enclosed_in_the_open_by_a_tree_or_beside_a_facade(self):
        tree = lv.Occupancy()
        tree.add(np.array(self.ground() + [(4.0, 0.0, z) for z in np.arange(0, 12, 0.1)]))
        known, covered = tree.enclosure(0.0, 0.0, 0.35, 5.0)
        self.assertGreater(known, 12)
        self.assertEqual(covered, 0)                          # a trunk is not a ceiling, nor the ground
        facade = lv.Occupancy()
        wall = [(2.0, y, z) for y in np.arange(-10, 10, 0.1) for z in np.arange(0, 10, 0.1)]
        facade.add(np.array(self.ground() + wall))
        self.assertEqual(facade.enclosure(0.0, 0.0, 2.0, 5.0)[1], 0)

    def test_enclosed_under_a_canopy(self):
        rng = np.random.default_rng(4)
        r, t = 6.0 * np.sqrt(rng.random(40000)), rng.uniform(0, 2 * math.pi, 40000)
        canopy = np.stack([r * np.cos(t), r * np.sin(t), rng.uniform(5.0, 7.0, 40000)], 1)
        occupancy = lv.Occupancy()
        occupancy.add(np.concatenate([np.array(self.ground()), canopy, [(0.0, 0.0, z) for z in np.arange(0, 5, 0.1)]]))
        known, covered = occupancy.enclosure(1.0, 1.0, 1.5, 5.0)
        self.assertGreater(covered, 0.9 * known)

    def test_nadir_ground_from_points_below(self):
        rng = np.random.default_rng(2)
        ground = np.hstack([rng.uniform(-20, 20, (20000, 2)), rng.normal(1.5, 0.02, (20000, 1)), np.zeros((20000, 1))])
        self.assertAlmostEqual(lv.nadir_ground(ground, 1.0, 2.0, 21.5, 5.0), 1.5, places=2)
        self.assertIsNone(lv.nadir_ground(ground, 100.0, 2.0, 21.5, 5.0))
        self.assertIsNone(lv.nadir_ground(None, 0, 0, 0, 1))
        sparse = ground[::200]                                 # ~0.6 points/m^2: none within 0.5 m
        self.assertAlmostEqual(lv.nadir_ground(sparse, 1.0, 2.0, 21.5, 0.5), 1.5, places=1)

    def test_promoter_hides_strays_and_releases_cells_whole(self):
        promoter = lv.Promoter(need=3)
        rng = np.random.default_rng(6)
        floor = np.hstack([rng.uniform(0, 2, (2000, 2)), np.full((2000, 1), 0.1), np.zeros((2000, 1))]).astype(np.float32)
        strays = np.hstack([rng.uniform(-300, 300, (200, 3)), np.zeros((200, 1))]).astype(np.float32)
        self.assertEqual(len(promoter.add(np.concatenate([floor, strays]))), len(floor))   # 16 full cells, no stray
        self.assertEqual(len(promoter.add(floor[:5])), 5)     # a shown cell passes points straight through
        lone = np.array([[100.25, 100.25, 100.125, 0.0]], np.float32)    # the middle of a cell
        self.assertEqual(len(promoter.add(lone)), 0)
        self.assertEqual(len(promoter.add(lone + (0.01, 0, 0, 0))), 0)
        released = promoter.add(lone + (0.02, 0, 0, 0))       # the third: the cell shows, all three at once
        self.assertEqual(len(released), 3)
        promoter.clear()
        self.assertEqual(len(promoter.add(lone)), 0)
        self.assertEqual(len(lv.Promoter(need=1).add(strays)), len(strays))

    def test_colour_range_follows_the_local_ground(self):
        rng = np.random.default_rng(8)
        here = np.hstack([rng.uniform(-20, 20, (2000, 2)), rng.uniform(0, 3, (2000, 1))])
        far = np.hstack([rng.uniform(300, 400, (2000, 2)), rng.uniform(-150, 150, (2000, 1))])
        sample = np.concatenate([here, far]).astype(np.float32)
        lo, hi = lv.robust_range(lv.local_heights(sample, (0.0, 0.0)))
        self.assertGreater(lo, -0.5)
        self.assertLess(hi, 3.5)
        self.assertEqual(len(lv.local_heights(sample, (1000.0, 0.0))), len(sample))     # too few near: all

    def test_robust_range_has_a_minimum_width(self):
        self.assertEqual(lv.robust_range(np.zeros(100)), (-1.0, 1.0))
        self.assertEqual(lv.robust_range(np.zeros(0)), (-1.0, 5.0))

    def test_trail_spacing_and_thinning(self):
        trail = lv.Trail(step=0.1, cap=100)
        self.assertTrue(trail.add((0, 0, 0)))
        self.assertFalse(trail.add((0.05, 0, 0)))
        for i in range(1, 300):
            trail.add((0.2 * i, 0, 0))
        self.assertLessEqual(trail.count, 100)
        self.assertAlmostEqual(float(trail.view()[-1, 0]), 0.2 * 299, places=3)
        self.assertTrue(np.all(np.diff(trail.view()[:, 0]) > 0))


class ModelTest(unittest.TestCase):
    def test_urdf_meshes_land_in_base_link(self):
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(os.path.join(d, "meshes"))
            tri = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
            with open(os.path.join(d, "meshes", "a.stl"), "wb") as f:
                f.write(b"\0" * 80 + struct.pack("<I", 1) + np.zeros(3, np.float32).tobytes() + tri.tobytes() + b"\0\0")
            with open(os.path.join(d, "meshes", "b.stl"), "w") as f:
                f.write("solid b\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 0 0 1\nvertex 0 1 0\n"
                        "endloop\nendfacet\nendsolid b\n")
            urdf = os.path.join(d, "x.urdf")
            with open(urdf, "w") as f:
                f.write("""<robot name="x"><material name="m"><color rgba="0.1 0.2 0.3 1"/></material>
<link name="base_link"><visual><geometry><mesh filename="package://x/meshes/a.stl"/></geometry>
<material name="m"/></visual></link>
<link name="arm"><visual><origin xyz="0 0 1" rpy="0 0 0"/><geometry>
<mesh filename="package://x/meshes/b.stl" scale="2 2 2"/></geometry></visual></link>
<joint name="j" type="fixed"><origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><parent link="base_link"/>
<child link="arm"/></joint></robot>""")
            parts = lv.load_urdf_model(urdf, {"x": d})
        self.assertEqual(len(parts), 2)
        np.testing.assert_allclose(parts[0][0][0], tri)
        self.assertEqual(parts[0][1][:3], (0.1, 0.2, 0.3))
        # b's (0, 1, 0) * 2, moved up 1 in the arm, rotated 90 deg about z, then 1 forward: (-1, 0, 1)
        np.testing.assert_allclose(parts[1][0][0][2], (1 - 2, 0, 1), atol=1e-6)
        vertices = lv.mesh_vertices(parts)
        self.assertEqual(vertices.shape, (6, 9))
        # Visuals without a name go by their link's: the "arm" triangle's top centre.
        np.testing.assert_allclose(lv.rotor_centres(parts, "arm"), [(0, 0, 3)], atol=1e-6)
        self.assertEqual(len(lv.rotor_centres(parts, "motor")), 0)
        np.testing.assert_allclose(np.abs(vertices[0, 3:6]), (0, 0, 1), atol=1e-6)

    def test_overlay_lines_are_line_lists(self):
        for lines in (lv.cone_lines(lv.AVIA_FOV), lv.pyramid_lines(lv.E1R_FOV), lv.ring_lines(1.0)):
            self.assertEqual(len(lines) % 2, 0)
        cone = lv.cone_lines(lv.AVIA_FOV)
        self.assertAlmostEqual(float(np.max(cone[:, 2])), math.tan(math.radians(77.2 / 2)), places=6)


class HudTest(unittest.TestCase):
    def test_hud_is_an_rgba_frame(self):
        try:
            import PIL  # noqa: F401
        except ImportError:
            self.skipTest("Pillow not installed")
        image = lv.hud_image(320, 180, ["TITLE", "a line"], [("AVIA", (1, 1, 1))], "footer", "/nonexistent.ttf",
                             warning="NO POSE")
        self.assertEqual(image.shape, (180, 320, 4))
        self.assertGreater(int(image[..., 3].max()), 0)

    def test_hud_keeps_out_of_the_ground_station_bars(self):
        try:
            import PIL  # noqa: F401
        except ImportError:
            self.skipTest("Pillow not installed")
        top, right, bottom, left = 72, 102, 115, 102          # the defaults on 1280 x 720
        image = lv.hud_image(1280, 720, ["LIDAR MAP", "AGL 1.0 m", "MAP 1.00 M"], [("AVIA", (1, 1, 1)), ("E1R", (1, 0, 1))],
                             "2026-10-03 20:00:00 UTC    CUTAWAY", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
                             warning="NO FAST-LIO POSE", insets=(top, right, bottom, left))
        alpha = image[..., 3]
        self.assertGreater(int(alpha.max()), 0)
        for band in (alpha[:top], alpha[-bottom:], alpha[:, :left], alpha[:, -right:]):
            self.assertEqual(int(band.max()), 0)


if __name__ == "__main__":
    unittest.main()
