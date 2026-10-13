"""TB-5 Nav2 parameters: collision_monitor stops TB-5 before a passage lower than 46 cm (OPS-115/128).

Decision 2026-10-13 (scope Q-25): pass only with 46 cm headroom or more. The field Nav2
obstacle layers use the lidar only, which does not see the overhead board. TB-5 therefore
runs the turtlebot4 nav2.yaml (config/robots/turtlebot4_nav2.baseline.yaml) unchanged except
collision_monitor: a front stop polygon PassageStop fed only by TB-5 depth points
(<ns>/passage_points from tb5_passage_points.launch.py) in the height band up to 0.46 m above
base_link. Depth points above 0.46 m (a board high enough) do not stop it. The existing
FootprintApproach polygon keeps reading the lidar only. If depth stops arriving,
collision_monitor's source_timeout stops TB-5 (no silent pass).
"""
from pathlib import Path
import unittest

from rescue_bringup.passage_launch import passage_points_values

REPO = Path(__file__).resolve().parents[1]
BASELINE = REPO / 'config' / 'robots' / 'turtlebot4_nav2.baseline.yaml'
TB5 = REPO / 'config' / 'robots' / 'tb5_nav2.yaml'


class Tb5Nav2(unittest.TestCase):
    def setUp(self):
        try:
            import yaml
        except ImportError:
            self.skipTest('PyYAML not installed')
        self.base, self.tb5 = yaml.safe_load(BASELINE.read_text()), yaml.safe_load(TB5.read_text())

    def test_only_collision_monitor_differs_from_the_turtlebot4_baseline(self):
        self.assertEqual({k: v for k, v in self.base.items() if k != 'collision_monitor'},
                         {k: v for k, v in self.tb5.items() if k != 'collision_monitor'})
        base, tb5 = (d['collision_monitor']['ros__parameters'] for d in (self.base, self.tb5))
        changed = {k for k in set(base) | set(tb5) if base.get(k) != tb5.get(k)}
        self.assertEqual({'polygons', 'observation_sources', 'FootprintApproach', 'PassageStop', 'passage_depth'},
                         changed)

    def test_passage_stop_polygon_reads_depth_points_up_to_46_cm(self):
        p = self.tb5['collision_monitor']['ros__parameters']
        self.assertEqual(['FootprintApproach', 'PassageStop'], p['polygons'])
        self.assertEqual(['scan', 'passage_depth'], p['observation_sources'])
        stop = p['PassageStop']
        self.assertEqual(('polygon', 'stop', True), (stop['type'], stop['action_type'], stop['enabled']))
        self.assertEqual(['passage_depth'], stop['sources_names'])
        xs = [x for x, _ in stop['points']]
        self.assertEqual((0.4, 1.0), (min(xs), max(xs)))         # in front of TB-5 only
        self.assertGreaterEqual(stop['min_points'], 2)           # one stray point does not stop it
        depth = p['passage_depth']
        self.assertEqual(('pointcloud', 'passage_points', True), (depth['type'], depth['topic'], depth['enabled']))
        self.assertEqual((0.05, 0.46), (depth['min_height'], depth['max_height']))
        self.assertEqual('collision_monitor_state', p['state_topic'])   # the executor reads it (#219)
        self.assertGreater(p['source_timeout'], 0)

    def test_existing_polygon_keeps_the_lidar_only(self):
        p = self.tb5['collision_monitor']['ros__parameters']
        self.assertEqual(['scan'], p['FootprintApproach']['sources_names'])
        base = self.base['collision_monitor']['ros__parameters']['FootprintApproach']
        self.assertEqual(base, {k: v for k, v in p['FootprintApproach'].items() if k != 'sources_names'})


class PassagePointsLaunch(unittest.TestCase):
    def test_depth_to_points_in_the_robot_namespace(self):
        v = passage_points_values(namespace='/robot5', depth_topic='/robot5/oakd/stereo/image_raw',
                                  camera_info_topic='/robot5/oakd/stereo/camera_info')
        self.assertEqual({'namespace': '/robot5', 'image_rect': '/robot5/oakd/stereo/image_raw',
                          'camera_info': '/robot5/oakd/stereo/camera_info', 'points': '/robot5/passage_points'}, v)

    def test_topics_are_required_absolute_and_in_this_robot(self):
        good = dict(namespace='/robot5', depth_topic='/robot5/oakd/stereo/image_raw',
                    camera_info_topic='/robot5/oakd/stereo/camera_info')
        for change in (dict(depth_topic=''), dict(camera_info_topic=''), dict(depth_topic='oakd/stereo/image_raw'),
                       dict(depth_topic='/robot6/oakd/stereo/image_raw'), dict(namespace='robot5'),
                       dict(depth_topic='/robot5/oakd/rgb/image_raw/compressed')):
            with self.subTest(change=change), self.assertRaises(ValueError):
                passage_points_values(**(good | change))


if __name__ == '__main__':
    unittest.main()
