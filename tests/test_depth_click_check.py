"""E2 tests for tools/depth_click_check.py (click patch median and calibration) without ROS.

The viewer itself needs ROS 2, cv_bridge, OpenCV and a GUI and is run locally (PR
evidence), not here. Depth images are plain nested lists in the 16UC1 millimetre unit.
"""
import argparse
import importlib.util
import math
from pathlib import Path
import unittest

from rescue_perception.target_utils import load_depth_calibration

TOOL = Path(__file__).resolve().parents[1] / 'tools' / 'depth_click_check.py'


def load_tool():
    spec = importlib.util.spec_from_file_location('depth_click_check', TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def image(width, height, value):
    return [[value] * width for _ in range(height)]


@unittest.skipUnless(TOOL.parent.is_dir(), 'installed-wheel checkout has no tools/')
class DepthClickCheckTest(unittest.TestCase):
    def setUp(self):
        self.tool = load_tool()

    def test_center_patch_median_in_millimetres(self):
        depth = image(9, 9, 900)
        for row in range(2, 7):
            depth[row][2:7] = [850, 852, 854, 856, 858]
        self.assertEqual(self.tool.patch_median_mm(depth, 4, 4), (854.0, 25, 25))

    def test_zero_and_nan_are_not_depth(self):
        depth = image(5, 5, 0)
        depth[2][2], depth[2][3], depth[1][1] = 860, float('nan'), 862
        self.assertEqual(self.tool.patch_median_mm(depth, 2, 2), (861.0, 2, 25))
        self.assertEqual(self.tool.patch_median_mm(image(5, 5, 0), 2, 2), (None, 0, 25))

    def test_patch_is_clipped_at_the_image_edge(self):
        self.assertEqual(self.tool.patch_median_mm(image(9, 9, 850), 0, 0), (850.0, 9, 9))

    def test_default_calibration_is_the_perception_robot6_file(self):
        expected = load_depth_calibration('package:robot6_depth.json', physical_robot_id='robot6')
        args = self.tool.parse_args(['--depth-topic', '/d', '--camera-info-topic', '/i'])
        scale, offset_m, label = self.tool.resolve_calibration(args, load_depth_calibration)
        self.assertEqual((scale, offset_m), (expected.scale, expected.offset_m))
        self.assertIn(expected.sha256, label)
        # 2026-10-12 user log: Raw 0.857 m -> Corrected 0.822 m with the 82 cm scale.
        self.assertEqual(f'{self.tool.corrected_m(0.857, scale, offset_m):.3f}', '0.822')

    def test_explicit_scale_and_offset_override_the_file(self):
        args = self.tool.parse_args(['--depth-topic', '/d', '--camera-info-topic', '/i',
                                     '--depth-scale', '1.0', '--depth-offset-m', '0.0'])
        scale, offset_m, label = self.tool.resolve_calibration(args, load_depth_calibration)
        self.assertEqual((scale, offset_m), (1.0, 0.0))
        self.assertIn('override', label)
        self.assertEqual(self.tool.corrected_m(0.857, scale, offset_m), 0.857)

    def test_bad_scale_is_rejected(self):
        for bad in ('0', '-1', 'nan', 'inf'):
            with self.subTest(bad=bad), self.assertRaises(SystemExit):
                self.tool.parse_args(['--depth-topic', '/d', '--camera-info-topic', '/i', '--depth-scale', bad])
        self.assertTrue(math.isfinite(self.tool.parse_args(
            ['--depth-topic', '/d', '--camera-info-topic', '/i', '--depth-offset-m', '-0.01']).depth_offset_m))

    def test_topics_are_required(self):
        with self.assertRaises(SystemExit):
            self.tool.parse_args(['--depth-topic', '/d'])


if __name__ == '__main__':
    unittest.main()
