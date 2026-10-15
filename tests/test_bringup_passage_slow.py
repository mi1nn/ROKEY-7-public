"""Slow down in front of anything hanging above the lidar plane (OPS-189), no ROS.

Owner decision 2026-10-13 night: measure the passage live, show it passable once, then pass. At
0.26 m/s the robot can be through a tunnel before the height judgement (about 1.5 s) is out.
So Nav2 collision_monitor gets one more polygon: PassageSlow (action slowdown) in front of the
robot, fed by the same depth points in the band from the lidar plane
(passage.overhead.lidar_plane_m, 0.23 m measured 2026-10-13) up to passage.overhead.top_m: an
overhang the lidar cannot see slows the robot down. Lower than 0.46 m, PassageStop still stops
it. All values once, in the field run file's shared robot section.
"""
import json
import unittest

from test_bringup_field_launch import FIELD, FieldCase
from test_bringup_passage import BASE


class FieldValues(FieldCase):
    def test_the_repository_field_file_has_one_slow_policy(self):
        r = json.loads(FIELD.read_text())['robot']
        self.assertEqual((0.23, 1.0), (r['passage.overhead.lidar_plane_m'], r['passage.overhead.top_m']))
        self.assertEqual(([0.4, 1.2], 0.3), (r['passage.slow.front_m'], r['passage.slow.ratio']))

    def test_robot_launch_gets_the_slow_values(self):
        p = self.robot(field=self.field(), occupancy_sha256='')['passage']
        self.assertEqual((0.23, 1.0, [0.4, 1.2], 0.3),
                         (p['overhead_lidar_plane_m'], p['overhead_top_m'], p['slow_front_m'], p['slow_ratio']))

    def test_bad_values_are_refused(self):
        from rescue_bringup.field_launch import FieldLaunchError

        def put(key, value):
            return lambda raw: raw['robot'].__setitem__(key, value)
        for name, change in {'band upside down': put('passage.overhead.top_m', 0.2),
                             'band below the floor': put('passage.overhead.lidar_plane_m', -0.1),
                             'ratio 0': put('passage.slow.ratio', 0.0), 'ratio 1': put('passage.slow.ratio', 1.0),
                             'front reversed': put('passage.slow.front_m', [1.2, 0.4]),
                             'missing': lambda raw: raw['robot'].pop('passage.slow.ratio')}.items():
            with self.subTest(name), self.assertRaises(FieldLaunchError):
                self.field(change)


class SlowPolygon(unittest.TestCase):
    PASSAGE = dict(min_height_m=0.05, max_height_m=0.46, front_m=[0.4, 1.0], half_width_m=0.2, min_points=5,
                   polygon_name='PassageStop', overhead_lidar_plane_m=0.23, overhead_top_m=1.0,
                   slow_front_m=[0.4, 1.2], slow_ratio=0.3)

    def test_adds_a_slowdown_polygon_on_the_overhead_band(self):
        from rescue_bringup.field_launch import nav2_with_passage
        p = nav2_with_passage(json.loads(json.dumps(BASE)), self.PASSAGE)['collision_monitor']['ros__parameters']
        self.assertEqual(['FootprintApproach', 'PassageSlow'], p['polygons'])   # no depth stop (2026-10-16, A)
        self.assertEqual(['scan', 'passage_overhead'], p['observation_sources'])
        slow = p['PassageSlow']
        self.assertEqual(('polygon', 'slowdown', 0.3, 5, ['passage_overhead'], True),
                         (slow['type'], slow['action_type'], slow['slowdown_ratio'], slow['min_points'],
                          slow['sources_names'], slow['enabled']))
        self.assertEqual([[1.2, 0.2], [1.2, -0.2], [0.4, -0.2], [0.4, 0.2]], json.loads(slow['points']))
        band = p['passage_overhead']
        self.assertEqual(('pointcloud', 'passage_points', 0.23, 1.0),
                         (band['type'], band['topic'], band['min_height'], band['max_height']))


if __name__ == '__main__':
    unittest.main()
