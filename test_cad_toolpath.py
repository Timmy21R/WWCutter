import tempfile
import unittest
from pathlib import Path

import numpy as np

from cad_toolpath import (CADGeometryError, MachineGeometry,
                          auto_detect_section_pair, build_cad_toolpath,
                          load_step_model)

try:
    import cadquery as cq
except ImportError:
    cq = None


@unittest.skipIf(cq is None, "CadQuery is required for STEP tests")
class StepToolpathTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.step_path = str(Path(self.tempdir.name) / "swept-wing.step")

        # A deliberately tapered, swept, twisted and vertically offset loft.
        root_points = [
            (0, 0, 0), (100, 0, 0), (78, 18, 0), (12, 23, 0),
        ]
        angle = np.radians(17.0)
        rotation = np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]])
        tip_profile = np.array([(0, 0), (62, 0), (48, 11), (8, 14)]) @ rotation.T
        tip_points = [(x + 38, y + 26, 240) for x, y in tip_profile]

        root = cq.Wire.makePolygon(root_points, close=True)
        tip = cq.Wire.makePolygon(tip_points, close=True)
        self.solid = cq.Solid.makeLoft([root, tip], ruled=True)
        self.solid.exportStep(self.step_path)

    def tearDown(self):
        self.tempdir.cleanup()

    def test_auto_detects_caps_and_preserves_cad_geometry(self):
        model = load_step_model(self.step_path)
        root, tip = auto_detect_section_pair(model)

        self.assertGreaterEqual(root.area, tip.area)
        path = build_cad_toolpath(
            model, root.index, tip.index,
            MachineGeometry(700, 120, 700, 500, 10),
        )

        self.assertLess(path.surface_error, 1e-6)
        self.assertEqual(len(path.root_xy), len(path.tip_xy))
        self.assertEqual(len(path.root_xy), len(path.tower_left))

        # Re-intersect the generated tower wire at both foam faces. It must
        # recover the original CAD sections after their common placement shift.
        span = float(np.dot(tip.center - root.center, path.axis))
        root_fraction = 120.0 / 700.0
        tip_fraction = (120.0 + span) / 700.0
        reconstructed_root = path.tower_left + root_fraction * (
            path.tower_right - path.tower_left
        )
        reconstructed_tip = path.tower_left + tip_fraction * (
            path.tower_right - path.tower_left
        )
        np.testing.assert_allclose(reconstructed_root, path.root_xy, atol=1e-8)
        np.testing.assert_allclose(reconstructed_tip, path.tip_xy, atol=1e-8)

        # The previewed cut surface is reconstructed from the same tower wire,
        # at the actual depth of every matched root and tip point.
        for cut_section in (path.cut_root, path.cut_tip):
            fraction = cut_section[:, 2] / 700.0
            reconstructed = path.tower_left + fraction[:, None] * (
                path.tower_right - path.tower_left
            )
            np.testing.assert_allclose(
                reconstructed, cut_section[:, :2], atol=1e-8
            )

    def test_manual_face_order_can_be_swapped(self):
        model = load_step_model(self.step_path)
        root, tip = auto_detect_section_pair(model)
        machine = MachineGeometry(700, 120, 700, 500, 10)

        normal = build_cad_toolpath(model, root.index, tip.index, machine)
        swapped = build_cad_toolpath(model, tip.index, root.index, machine)

        self.assertLess(normal.surface_error, 1e-6)
        self.assertLess(swapped.surface_error, 1e-6)
        self.assertEqual(len(normal.tower_left), len(swapped.tower_left))

    def test_open_shell_boundary_loops_are_valid_manual_sections(self):
        open_path = str(Path(self.tempdir.name) / "open-wing.step")
        lateral_faces = [face for face in self.solid.Faces()
                         if face.geomType() != "PLANE"]
        cq.Shell.makeShell(lateral_faces).exportStep(open_path)

        model = load_step_model(open_path)
        self.assertEqual([section.kind for section in model.sections],
                         ["boundary", "boundary"])
        root, tip = auto_detect_section_pair(model)
        path = build_cad_toolpath(
            model, root.index, tip.index,
            MachineGeometry(700, 120, 700, 500, 10),
        )
        self.assertLess(path.surface_error, 1e-6)

    def test_rejects_a_projected_path_that_exceeds_travel(self):
        model = load_step_model(self.step_path)
        root, tip = auto_detect_section_pair(model)
        with self.assertRaisesRegex(CADGeometryError, "Projected tower paths need"):
            build_cad_toolpath(
                model, root.index, tip.index,
                MachineGeometry(700, 120, 40, 40, 5),
            )


@unittest.skipUnless(
    (Path(__file__).parent / "MasterSections.step").exists(),
    "Included MasterSections.step test piece is not present",
)
class IncludedStepRegressionTests(unittest.TestCase):
    def test_master_sections_can_ignore_or_cut_through_interior_contour(self):
        model = load_step_model(Path(__file__).parent / "MasterSections.step")
        root, tip = auto_detect_section_pair(model)
        ignored = build_cad_toolpath(
            model, root.index, tip.index,
            MachineGeometry(1000, 100, 1000, 1000, 10),
        )
        included = build_cad_toolpath(
            model, root.index, tip.index,
            MachineGeometry(1000, 100, 1000, 1000, 10),
            include_internal=True,
        )

        self.assertGreater(ignored.surface_error, 2.0)
        self.assertLess(ignored.surface_error, 3.0)
        self.assertEqual(ignored.ignored_interior_count, 1)
        self.assertEqual(ignored.interior_count, 0)
        self.assertFalse(ignored.limitations)

        self.assertEqual(included.ignored_interior_count, 0)
        self.assertEqual(included.interior_count, 1)
        self.assertFalse(included.limitations)
        self.assertGreater(len(included.root_xy), len(ignored.root_xy))

    @unittest.skipUnless(
        (Path(__file__).parent / "MasterSections12.step").exists(),
        "Included MasterSections12.step test piece is not present",
    )
    def test_master_sections_12_builds_and_identifies_blind_interiors(self):
        model = load_step_model(Path(__file__).parent / "MasterSections12.step")
        root, tip = auto_detect_section_pair(model)
        ignored = build_cad_toolpath(
            model, root.index, tip.index,
            MachineGeometry(1000, 100, 1000, 1000, 10),
        )
        included = build_cad_toolpath(
            model, root.index, tip.index,
            MachineGeometry(1000, 100, 1000, 1000, 10),
            include_internal=True,
        )

        self.assertGreater(ignored.surface_error, 5.0)
        self.assertLess(ignored.surface_error, 6.0)
        self.assertEqual(ignored.ignored_interior_count, 2)
        self.assertFalse(ignored.limitations)
        self.assertTrue(any("do not pass through both" in item
                            for item in included.limitations))


if __name__ == "__main__":
    unittest.main()
