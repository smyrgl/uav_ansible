"""Voxel map: keys, de-duplication, cap and the PCD writer."""
import unittest

import numpy as np

from lio_map import VoxelMap, pcd_bytes, voxel_keys


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

    def test_pcd_is_header_plus_float32(self):
        pts = np.array([[1, 2, 3, 4], [5, 6, 7, 8]], np.float32)
        blob = pcd_bytes(pts)
        header, data = blob.split(b"DATA binary\n", 1)
        self.assertIn(b"POINTS 2", header)
        self.assertEqual(np.frombuffer(data, "<f4").reshape(2, 4).tolist(), pts.tolist())


if __name__ == "__main__":
    unittest.main()
