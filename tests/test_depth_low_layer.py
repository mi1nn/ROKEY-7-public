"""Low objects the lidar plane misses are kept out of paths (재범 decision 2026-10-16 03:1x, option D, grade 4; no ROS).

After PassageStop was removed (#398) nothing stopped the robots in front of things under the lidar plane
(0.23 m): the TB-5 dock (0.1 m), a doll on the floor. 재범: 「1 피하게 하자」.

- overhead_passage also publishes <ns>/rescue/low_obstacle_points: its depth points with z replaced by the
  height above the floor seen in the box (floor_height, #411), so a floor at base_link z -0.2 (field, #254) and at
  0 (simulation) give the same numbers. Nothing is published while no floor is seen.
- Both costmaps get a depth-only ObstacleLayer, depth_low_layer, with two sources on that cloud: marking for heights
  passage.min_height_m (0.05) .. passage.overhead.lidar_plane_m (0.23), clearing by raytrace down to the floor
  points. The lidar scan is not a source of this layer, so a scan passing over the dock cannot clear it.
- Ranges are the scan source's of the same costmap; observation_persistence and expected_update_rate are Nav2's
  defaults (0.0): with no cloud the layer keeps what it marked (nothing clears it), and a silent camera does not
  make the costmap stale.

Before this change (ba21433) these tests were expectedFailure.
"""
import unittest

from test_bringup_no_backup import vendor_params

try:
    import numpy as np
except ImportError:   # installed-wheel check
    np = None

FLOOR = -0.2   # field floor in base_link (#254)


def grid(x0, x1, z, *, y0=-0.2, y1=0.2, step=0.02):
    return [(x0 + i * step, y0 + j * step, z) for i in range(int(round((x1 - x0) / step)) + 1)
            for j in range(int(round((y1 - y0) / step)) + 1)]


@unittest.skipIf(np is None, 'NumPy not installed')
class LowObstaclePoints(unittest.TestCase):
    def policy(self):
        from test_perception_overhead import POLICY
        return POLICY

    def test_heights_are_from_the_floor_seen(self):
        from rescue_perception.overhead import low_obstacle_points
        floor = grid(0.4, 1.4, FLOOR)
        dock = grid(0.9, 1.0, FLOOR + 0.10, y0=-0.15, y1=0.15)
        wall = [(1.3, y, FLOOR + h) for y in (-0.1, 0.0, 0.1) for h in (0.10, 0.30, 0.40)]
        out = low_obstacle_points(np.array(floor + dock + wall), self.policy())
        self.assertIsNotNone(out)
        h = out[:, 2]
        self.assertTrue((abs(h[np.isclose(out[:, 0], 0.4)]) < 1e-9).all())          # floor at 0
        self.assertTrue(np.isclose(h, 0.10).any())                                   # the dock at 0.1
        self.assertFalse((h > self.policy().lidar_plane_m + 1e-9).any())              # the lidar's part is not here
        self.assertTrue(np.isclose(out[np.isclose(out[:, 0], 1.3)][:, 2], 0.10).any())   # the wall's foot is

    def test_no_floor_seen_publishes_nothing(self):
        from rescue_perception.overhead import low_obstacle_points
        self.assertIsNone(low_obstacle_points(np.array(grid(0.9, 1.0, 0.3)), self.policy()))


class Costmaps(unittest.TestCase):
    PASSAGE = dict(min_height_m=0.05, overhead_lidar_plane_m=0.23)

    def params(self):
        from rescue_bringup.nav2_field import nav2_with_depth_low_layer
        return nav2_with_depth_low_layer(vendor_params(), self.PASSAGE)

    def test_both_costmaps_get_the_layer_before_inflation(self):
        params = self.params()
        for costmap in ('global_costmap', 'local_costmap'):
            with self.subTest(costmap=costmap):
                p = params[costmap][costmap]['ros__parameters']
                self.assertEqual('depth_low_layer', p['plugins'][-2])
                self.assertEqual('inflation_layer', p['plugins'][-1])

    def test_marking_and_clearing_sources_and_no_lidar(self):
        layer = self.params()['local_costmap']['local_costmap']['ros__parameters']['depth_low_layer']
        self.assertEqual('nav2_costmap_2d::ObstacleLayer', layer['plugin'])
        self.assertEqual('depth_low depth_floor', layer['observation_sources'])
        mark, clear = layer['depth_low'], layer['depth_floor']
        self.assertEqual(('rescue/low_obstacle_points', 'PointCloud2', True, False, 0.05, 0.23),
                         (mark['topic'], mark['data_type'], mark['marking'], mark['clearing'],
                          mark['min_obstacle_height'], mark['max_obstacle_height']))
        self.assertEqual((False, True, -0.05, 0.23),
                         (clear['marking'], clear['clearing'], clear['min_obstacle_height'],
                          clear['max_obstacle_height']))
        for source in (mark, clear):   # the scan source's ranges; Nav2 defaults for time
            self.assertEqual((2.5, 0.0, 3.0, 0.0, 0.0, 0.0),
                             (source['obstacle_max_range'], source['obstacle_min_range'],
                              source['raytrace_max_range'], source['raytrace_min_range'],
                              source['observation_persistence'], source['expected_update_rate']))
        self.assertNotIn('scan', layer['observation_sources'].split())


if __name__ == '__main__':
    unittest.main()
