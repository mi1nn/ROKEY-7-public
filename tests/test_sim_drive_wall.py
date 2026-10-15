"""scripts/sim/drive_wall.py: the stage-1 wall follower must not take its own body for a wall.

Since #431 the simulated lidar measures from 0.10 m and beams at about +-57..60 deg (robot frame) hit the TurtleBot4
shell posts at 0.10..0.14 m [측정 2026-10-16]. The follower read them as a wall ahead-right and turned on the spot for
the whole 300 s sim2 drive. Ranges under the real floor 0.164 m are no return, as the relay sends them to the field.
"""
from pathlib import Path
import math
import sys
import unittest

TOOL = Path(__file__).resolve().parents[1] / 'scripts' / 'sim' / 'drive_wall.py'
sys.path.insert(0, str(TOOL.parent))


class Scan:
    def __init__(self, ranges):
        self.ranges = ranges
        self.angle_min, self.angle_increment = -math.pi, 2 * math.pi / len(ranges)
        self.range_min, self.range_max = 0.10, 12.0


@unittest.skipUnless(TOOL.exists(), 'installed-wheel checkout has no scripts/')
class SelfHits(unittest.TestCase):
    def setUp(self):
        try:
            import drive_wall
        except ImportError as exc:   # rclpy / geometry_msgs not installed (CI without ROS)
            self.skipTest(f'ROS Python packages not installed: {exc}')
        self.dw = drive_wall

    def test_returns_under_the_real_floor_are_not_walls(self):
        ranges = [2.0] * 640
        for i in range(640):   # robot frame bearing = scan angle + 90 deg; -58 deg is front-right
            bearing = math.degrees(-math.pi + i * 2 * math.pi / 640) + self.dw.LIDAR_YAW_DEG
            bearing = (bearing + 180.0) % 360.0 - 180.0
            if abs(bearing + 58.0) < 2.0:
                ranges[i] = 0.12          # the shell post
        self.assertEqual(2.0, self.dw.sector_min(Scan(ranges), -70, -25))

    def test_a_real_near_wall_still_counts(self):
        self.assertAlmostEqual(0.2, self.dw.sector_min(Scan([0.2] * 640), -70, -25))


if __name__ == '__main__':
    unittest.main()
