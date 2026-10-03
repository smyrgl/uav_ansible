"""Voxel map: keys, de-duplication, cap, the stray-return filter and the PCD writer."""
import unittest

import numpy as np

from lio_map import CellPromoter, VoxelMap, pcd_bytes, voxel_keys


class Map(unittest.TestCase):
    def test_keys_differ_per_voxel_and_handle_negatives(self):
        k = voxel_keys(np.array([[0.05, 0.05, 0.05], [0.15, 0.05, 0.05], [-0.05, 0.05, 0.05], [0.06, 0.07, 0.08]]), 0.1)
        self.assertEqual(len(set(k.tolist())), 3)
        self.assertEqual(k[0], k[3])

    def test_one_point_per_voxel_across_scans(self):
        m = VoxelMap(0.2, 100)
        scan = np.array([[0.0, 0.0, 0.0, 10], [0.05, 0.05, 0.0, 20], [1.0, 0.0, 0.0, 30]], np.float32)
        self.assertEqual(len(m.add(scan)), 2)
        self.assertEqual(len(m.add(scan + np.float32([0.01, 0, 0, 0]))), 0)   # same voxels
        new = m.add(np.array([[0.0, 1.0, 0.0, 5], [np.nan, 0, 0, 1]], np.float32))
        self.assertEqual(new.tolist(), [[0.0, 1.0, 0.0, 5.0]])                # only the new point, no NaN
        pts = m.points()
        self.assertEqual(pts.shape, (3, 4))
        self.assertEqual(pts[0, 3], 10)                                    # the first point seen

    def test_cap_marks_the_map_full(self):
        m = VoxelMap(1.0, 2)
        self.assertEqual(len(m.add(np.array([[i, 0, 0, 0] for i in range(5)], np.float32))), 2)
        self.assertTrue(m.full)
        self.assertEqual(len(m.add(np.array([[9, 9, 9, 0]], np.float32))), 0)

    def test_overview_from_new_fine_points_matches_the_whole_scans(self):
        """The overview fed only the fine map's new points holds the same coarse voxels as one fed every scan."""
        rng = np.random.default_rng(3)
        fine, coarse, direct = VoxelMap(0.05, 10**6), VoxelMap(0.2, 10**6), VoxelMap(0.2, 10**6)
        for _ in range(20):
            scan = np.column_stack([rng.uniform(-3, 3, (2000, 3)), np.ones(2000)]).astype(np.float32)
            coarse.add(fine.add(scan))
            direct.add(scan)
        self.assertEqual(coarse.keys, direct.keys)

    @staticmethod
    def wall_and_strays(rng, n_wall=3000, n_strays=60):
        """A 4 x 3 m wall 5 m ahead (the Avia indoors) plus stray returns at random ranges to 400 m."""
        wall = np.column_stack([np.full(n_wall, 5.0) + rng.normal(0, 0.01, n_wall), rng.uniform(-2, 2, n_wall),
                                rng.uniform(0, 3, n_wall), rng.uniform(10, 40, n_wall)])
        direction = rng.normal(size=(n_strays, 3))
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        strays = np.column_stack([direction * rng.uniform(12, 400, (n_strays, 1)), rng.uniform(5, 20, n_strays)])
        return wall.astype(np.float32), strays.astype(np.float32)

    def test_strays_never_reach_the_map_surfaces_do(self):
        rng = np.random.default_rng(7)
        promoter, fine = CellPromoter(3), VoxelMap(0.05, 10**6)
        reference = VoxelMap(0.05, 10**6)
        for _ in range(30):                                   # 3 s of Avia scans
            wall, strays = self.wall_and_strays(rng)
            fine.add(promoter.filter(np.concatenate([wall, strays])))
            reference.add(wall)
        pts = fine.points()
        self.assertEqual(int((np.linalg.norm(pts[:, :3], axis=1) > 10).sum()), 0)       # no stray
        self.assertEqual(fine.size, reference.size)          # and every wall voxel
        self.assertEqual(promoter.held_points, len(promoter.held))                      # one voxel per stray cell

    def test_a_cell_joins_on_its_third_distinct_voxel_with_all_three(self):
        promoter = CellPromoter(3)
        a, b, c = ([[0.12, 0.12, 0.12, 1]], [[0.22, 0.12, 0.12, 2]], [[0.32, 0.12, 0.12, 3]])
        self.assertEqual(len(promoter.filter(np.array(a, np.float32))), 0)
        self.assertEqual(len(promoter.filter(np.array(a, np.float32) + 0.001)), 0)     # same voxel: still one
        self.assertEqual(len(promoter.filter(np.array(b, np.float32))), 0)
        joined = promoter.filter(np.array(c, np.float32))
        self.assertEqual(sorted(joined[:, 3].tolist()), [1.0, 2.0, 3.0])                # the held two come along
        self.assertEqual((promoter.held_points, promoter.promoted_cells), (0, 1))
        self.assertEqual(len(promoter.filter(np.array([[0.42, 0.12, 0.12, 4]], np.float32))), 1)   # straight through

    def test_promoted_cells_survive_the_bulk_index_merge(self):
        promoter = CellPromoter(3)
        promoter.MERGE = 2
        cells = [np.array([[x + dx, 0.1, 0.1, 0] for dx in (0.01, 0.11, 0.21)], np.float32) for x in (0, 1, 2, 3, 4)]
        for cell in cells:
            self.assertEqual(len(promoter.filter(cell)), 3)    # complete in one scan
        self.assertEqual(promoter.promoted_cells, 5)
        self.assertGreater(len(promoter._main), 0)
        for x in (0, 1, 2, 3, 4):                              # one new voxel in each: straight through
            self.assertEqual(len(promoter.filter(np.array([[x + 0.41, 0.1, 0.1, 0]], np.float32))), 1)

    def test_held_cells_are_capped_oldest_first(self):
        promoter = CellPromoter(3, max_held_cells=10)
        for i in range(25):                                    # 25 strays, each its own cell
            promoter.filter(np.array([[i * 10.0, 0.1, 0.1, 0]], np.float32))
        self.assertLessEqual(len(promoter.held), 10)
        self.assertEqual(promoter.held_points + promoter.dropped_points, 25)
        self.assertIn(promoter.cell_keys(np.array([[240.0, 0.1, 0.1]]))[0], promoter.held)   # the newest stays

    def test_need_one_passes_everything_and_nan_never(self):
        scan = np.array([[1, 2, 3, 4], [np.nan, 0, 0, 1], [500, 0, 0, 1]], np.float32)
        self.assertEqual(len(CellPromoter(1).filter(scan)), 3)                         # the map drops the NaN
        self.assertEqual(len(CellPromoter(3).filter(scan)), 0)
        self.assertEqual(len(CellPromoter(3).filter(np.zeros((0, 4), np.float32))), 0)
        promoter = CellPromoter(3)
        promoter.filter(np.array([[1, 1, 1, 0]], np.float32))
        promoter.reset()
        self.assertEqual((promoter.held_points, promoter.promoted_cells, len(promoter.held)), (0, 0, 0))

    def test_overview_nesting_holds_through_the_filter(self):
        """Fed only the fine map's new points, the overview still matches one fed every filtered scan."""
        rng = np.random.default_rng(11)
        promoter = CellPromoter(3)
        fine, coarse, direct = VoxelMap(0.05, 10**6), VoxelMap(0.2, 10**6), VoxelMap(0.2, 10**6)
        for _ in range(15):
            wall, strays = self.wall_and_strays(rng)
            shown = promoter.filter(np.concatenate([wall, strays]))
            coarse.add(fine.add(shown))
            direct.add(shown)
        self.assertEqual(coarse.keys, direct.keys)

    def test_rollback_takes_back_what_arrived_since(self):
        m = VoxelMap(0.1, 100)
        for t, x in ((1.0, 0.0), (2.0, 1.0), (3.0, 2.0)):
            m.add(np.array([[x, 0, 0, t], [x, 0.5, 0, t]], np.float32), now=t)
        self.assertEqual(m.rollback(2.5), 2)                 # the two that arrived at 3.0
        self.assertEqual((m.size, len(m.keys)), (4, 4))
        self.assertEqual(sorted(m.points()[:, 3].tolist()), [1, 1, 2, 2])
        self.assertEqual(len(m.add(np.array([[2.0, 0, 0, 9]], np.float32), now=4.0)), 1)   # its voxel is free again
        self.assertEqual(m.rollback(100.0), 0)               # nothing after that
        self.assertEqual(m.rollback(0.0), 5)                 # everything
        self.assertEqual((m.size, len(m.keys), m.points().shape), (0, 0, (0, 4)))

    def test_rollback_reopens_a_full_map_and_forgets_old_marks(self):
        m = VoxelMap(0.1, 3, horizon_s=10.0)
        m.add(np.array([[0, 0, 0, 0], [1, 0, 0, 0]], np.float32), now=0.0)
        m.add(np.array([[2, 0, 0, 0], [3, 0, 0, 0]], np.float32), now=20.0)
        self.assertTrue(m.full)
        self.assertEqual(len(m.marks), 1)                    # the mark at 0 s is past the horizon
        self.assertEqual(m.rollback(15.0), 1)
        self.assertFalse(m.full)

    def test_seeded_cells_pass_straight_through(self):
        promoter = CellPromoter(3)
        promoter.seed(np.array([[0.1, 0.1, 0.1, 0]], np.float32))
        self.assertEqual(promoter.promoted_cells, 1)
        self.assertEqual(len(promoter.filter(np.array([[0.3, 0.3, 0.2, 0]], np.float32))), 1)

    def test_pcd_is_header_plus_float32(self):
        pts = np.array([[1, 2, 3, 4], [5, 6, 7, 8]], np.float32)
        blob = pcd_bytes(pts)
        header, data = blob.split(b"DATA binary\n", 1)
        self.assertIn(b"POINTS 2", header)
        self.assertEqual(np.frombuffer(data, "<f4").reshape(2, 4).tolist(), pts.tolist())


if __name__ == "__main__":
    unittest.main()
