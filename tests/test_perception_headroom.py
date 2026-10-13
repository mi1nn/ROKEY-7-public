"""TB-5 overhead clearance: repeat-judgement record for the field (OPS-115, 2026-10-13).

The stop itself is Nav2 collision_monitor with a depth point-cloud source (통합-로봇). This is the
field measuring tool: from one depth image, the points in a box ahead of TB-5 between the floor noise
and the clearance height (owner decision 46 cm) are counted the way collision_monitor counts them
(min_points), and the lowest point height is kept, so ten judgements at one spot show whether the
47 cm crossbar stays above 46 cm under depth noise. No depth, no fresh depth or no CameraInfo is
UNKNOWN, never PASS.
"""
import math
import unittest

from rescue_perception.headroom import HeadroomPolicy, depth_points, judge, summary

POLICY = HeadroomPolicy(clearance_m=0.46, floor_noise_m=0.05, box_x_min_m=0.3, box_x_max_m=1.2,
                        box_half_width_m=0.2, min_points=3)
# Optical -> base_link: camera 0.28 m up looking along +x (optical z -> base x, x -> -y, y -> -z).
OPTICAL_TO_BASE = ((0.0, 0.0, 0.28), (-0.5, 0.5, -0.5, 0.5))


class JudgeTests(unittest.TestCase):
    def test_crossbar_above_the_clearance_passes(self):
        points = [(0.8, 0.0, 0.47)] * 5 + [(0.8, 0.0, 0.02)] * 5   # crossbar 47 cm and floor noise
        result = judge(points, POLICY)
        self.assertEqual(('PASS', 0, 0.47), (result['decision'], result['points_below'], result['lowest_m']))

    def test_crossbar_below_the_clearance_stops(self):
        points = [(0.8, 0.0, 0.45)] * 3
        self.assertEqual(('STOP', 3), (judge(points, POLICY)['decision'], judge(points, POLICY)['points_below']))

    def test_fewer_points_than_min_points_do_not_stop(self):   # same rule as collision_monitor min_points
        self.assertEqual('PASS', judge([(0.8, 0.0, 0.45)] * 2, POLICY)['decision'])

    def test_points_outside_the_box_or_floor_are_ignored(self):
        points = [(0.1, 0.0, 0.40), (2.0, 0.0, 0.40), (0.8, 0.5, 0.40), (0.8, 0.0, 0.03)] * 3
        self.assertEqual(('PASS', None), (judge(points, POLICY)['decision'], judge(points, POLICY)['lowest_m']))

    def test_no_depth_is_unknown_not_pass(self):
        self.assertEqual('UNKNOWN', judge(None, POLICY)['decision'])

    def test_policy_values_are_checked(self):
        for bad in (dict(clearance_m=0.0), dict(floor_noise_m=0.5), dict(box_x_max_m=0.2), dict(min_points=0),
                    dict(box_half_width_m=math.nan)):
            values = dict(POLICY.__dict__) | bad
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                HeadroomPolicy(**values)


class DepthPointsTests(unittest.TestCase):
    def test_depth_pixels_go_to_base_link(self):
        try:
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest('NumPy not installed')
        # 3x3 image, f 1 px, centre (1, 1): the centre pixel at 1000 mm is 1 m straight ahead at camera height.
        points = depth_points([[0, 0, 0], [0, 1000, 0], [0, 0, 0]], '16UC1', fx=1.0, fy=1.0, cx=1.0, cy=1.0,
                              optical_to_base=OPTICAL_TO_BASE, stride=1)
        self.assertEqual(1, len(points))
        for got, want in zip(points[0], (1.0, 0.0, 0.28)):
            self.assertAlmostEqual(want, got)


class SummaryTests(unittest.TestCase):
    def test_repeat_summary(self):
        runs = [dict(decision='PASS', lowest_m=0.47), dict(decision='STOP', lowest_m=0.455),
                dict(decision='PASS', lowest_m=None), dict(decision='UNKNOWN', lowest_m=None)]
        s = summary(runs)
        self.assertEqual({'PASS': 2, 'STOP': 1, 'UNKNOWN': 1}, s['counts'])
        self.assertEqual((0.455, 0.47), (s['lowest_min_m'], s['lowest_max_m']))


if __name__ == '__main__':
    unittest.main()
