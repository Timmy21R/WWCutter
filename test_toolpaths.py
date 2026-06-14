import importlib.util
import unittest

import numpy as np


spec = importlib.util.spec_from_file_location("wwcutter", "WWCutterV01.py")
wwcutter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wwcutter)


class ToolpathSynchronizationTests(unittest.TestCase):
    def setUp(self):
        self.controller = wwcutter.HotWireController.__new__(wwcutter.HotWireController)

    def test_tapered_profiles_share_progress_and_preserve_vertices(self):
        a = [(-1, -1), (0, 0), (10, 0), (10, 10), (0, 0)]
        b = [(-2, -2), (0, 0), (0, 5), (0, 20), (0, 0)]
        synced_a, synced_b = self.controller.synchronize_toolpaths(a, b)

        self.assertEqual(len(synced_a), len(synced_b))
        self.assertEqual(synced_a[0], a[0])
        self.assertEqual(synced_b[0], b[0])
        self.assertTrue(any(np.allclose(point, (10, 0)) for point in synced_a))
        self.assertTrue(any(np.allclose(point, (0, 5)) for point in synced_b))
        self.assertTrue(np.allclose(synced_a[1], synced_a[-1]))
        self.assertTrue(np.allclose(synced_b[1], synced_b[-1]))

    def test_long_lines_are_subdivided_on_both_sides(self):
        a = [(0, 0), (0, 0), (12, 0)]
        b = [(0, 0), (0, 0), (0, 3)]
        synced = self.controller.synchronize_toolpaths(a, b)

        for path in synced:
            segments = np.hypot(*np.diff(np.asarray(path[1:]), axis=0).T)
            self.assertLessEqual(max(segments), self.controller.MAX_SEGMENT_MM + 1e-9)

    def test_opposite_contour_directions_are_aligned(self):
        a = [(0, 0), (0, 0), (4, 0), (4, 2), (0, 2), (0, 0)]
        b = [(0, 0), (0, 0), (0, 1), (2, 1), (2, 0), (0, 0)]
        synced_a, synced_b = self.controller.synchronize_toolpaths(a, b)

        self.assertGreater(synced_a[2][0], synced_a[2][1])
        self.assertGreater(synced_b[2][0], synced_b[2][1])


if __name__ == "__main__":
    unittest.main()
