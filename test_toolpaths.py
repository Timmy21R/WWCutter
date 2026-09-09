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


class DxfToolpathTests(unittest.TestCase):
    def setUp(self):
        self.controller = wwcutter.HotWireController.__new__(wwcutter.HotWireController)

    @staticmethod
    def path_length(points):
        return sum(np.hypot(end[0] - start[0], end[1] - start[1])
                   for start, end in zip(points, points[1:]))

    def test_entry_line_connects_contours_without_synthetic_bridges(self):
        outer = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]
        hole = [(4, 4), (6, 4), (6, 6), (4, 6), (4, 4)]
        entry = [(0, 5), (4, 5)]

        joined = self.controller._join_dxf_paths(
            [outer, hole, entry], self.controller.DXF_JOIN_TOLERANCE_MM)
        toolpath = self.controller._trace_dxf_paths(joined)

        self.assertTrue(np.allclose(toolpath[0], toolpath[-1]))
        # 40 mm outer + 8 mm hole + the 4 mm entry travelled in and out.
        self.assertAlmostEqual(self.path_length(toolpath), 56.0, places=7)
        entry_uses = sum(
            (np.allclose(start, (0, 5)) and np.allclose(end, (4, 5))) or
            (np.allclose(start, (4, 5)) and np.allclose(end, (0, 5)))
            for start, end in zip(toolpath, toolpath[1:]))
        self.assertEqual(entry_uses, 2)

    def test_positioning_point_is_the_real_cut_start(self):
        points = [(0, 0), (10, 0), (10, 5), (0, 5), (0, 0)]

        processed = self.controller.process_toolpath(points, 0)

        self.assertEqual(processed[0], processed[1])
        self.assertAlmostEqual(self.path_length(processed), self.path_length(points), places=7)

    def test_dense_contour_does_not_depend_on_python_recursion_depth(self):
        angles = np.linspace(0.0, 2.0 * np.pi, 1501)
        contour = list(zip(np.cos(angles), np.sin(angles)))

        toolpath = self.controller._trace_dxf_paths([contour])

        self.assertEqual(len(toolpath), len(contour))
        self.assertTrue(np.allclose(toolpath[0], toolpath[-1]))


if __name__ == "__main__":
    unittest.main()
