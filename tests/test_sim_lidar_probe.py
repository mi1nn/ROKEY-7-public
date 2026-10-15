"""scripts/sim/lidar_probe.py: the parts that need no Gazebo (ray casting, leak counting, the probe world text).

Running the probe itself needs the Gazebo server; its measurements are recorded in config/sim/README.md.
"""
import math
from pathlib import Path
import sys
import unittest
import xml.dom.minidom

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / 'scripts' / 'sim' / 'lidar_probe.py'
sys.path.insert(0, str(TOOL.parent))
HAS_TOOL = TOOL.exists()   # the installed-wheel check copies tests/ and config/ only

ROOM = {'objects': [
    {'name': 's', 'kind': 'wall', 'x1': 0.0, 'y1': 0.0, 'x2': 4.0, 'y2': 0.0},
    {'name': 'e', 'kind': 'wall', 'x1': 4.0, 'y1': 0.0, 'x2': 4.0, 'y2': 3.0},
    {'name': 'n', 'kind': 'wall', 'x1': 4.0, 'y1': 3.0, 'x2': 0.0, 'y2': 3.0},
    {'name': 'w', 'kind': 'wall', 'x1': 0.0, 'y1': 3.0, 'x2': 0.0, 'y2': 0.0},
    {'name': 'post', 'kind': 'box', 'x': 3.0, 'y': 1.5, 'yaw': 0.0, 'size': [0.2, 0.4, 0.41]},
    {'name': 'lintel', 'kind': 'overhead', 'x': 2.0, 'y': 2.0, 'yaw': 0.0, 'size': [0.8, 0.01, 0.41],
     'clearance_m': 0.3},
    {'name': 'doll', 'kind': 'cylinder', 'x': 1.0, 'y': 2.0, 'radius': 0.07, 'length': 0.2},
]}
WORLD = '<?xml version="1.0"?>\n<sdf version="1.8">\n  <world name="room">\n  </world>\n</sdf>\n'


@unittest.skipUnless(HAS_TOOL, 'installed-wheel checkout has no scripts/')
class LidarProbe(unittest.TestCase):
    def setUp(self):
        import lidar_probe
        self.lp = lidar_probe
        self.segments = lidar_probe.wall_segments(ROOM)

    def test_only_walls_and_boxes_stop_a_ray_at_lidar_height(self):
        self.assertEqual(4 + 4, len(self.segments))   # the overhead lintel and the round doll are not in the plane
        self.assertAlmostEqual(1.0, self.lp.cast(self.segments, 1.0, 1.0, -math.pi / 2))     # south wall
        self.assertAlmostEqual(1.9, self.lp.cast(self.segments, 1.0, 1.5, 0.0))              # the post's west face
        self.assertAlmostEqual(math.sqrt(2), self.lp.cast(self.segments, 1.0, 1.0, -3 * math.pi / 4))   # corner

    def test_a_ray_beyond_its_wall_is_a_leak_and_a_short_one_is_not(self):
        x, y = 1.0, 1.0
        true = [self.lp.cast(self.segments, x, y, self.lp.ray_angle(k)) for k in range(self.lp.RAYS)]
        measured = list(true)
        measured[100] = true[100] + 0.5     # passed through the wall
        measured[200] = true[200] - 0.5     # hit something the layout does not have (a robot): not a leak
        measured[300] = math.inf            # no return where a wall is
        rays = self.lp.count_leaks(self.segments, x, y, measured)
        leaked = [r for r in rays if r[1]]
        self.assertEqual(2, len(leaked))
        self.assertEqual(self.lp.RAYS, len(rays))   # every ray has a wall in range inside this room
        hx, hy = leaked[0][2], leaked[0][3]         # where the ray should have stopped: on a wall
        self.assertTrue(abs(hx) < 1e-9 or abs(hy) < 1e-9 or abs(hx - 4) < 1e-9 or abs(hy - 3) < 1e-9)

    def test_ranges_are_read_from_gz_topic_text(self):
        text = 'header {\n}\n' + ''.join(f'ranges: {v}\n' for v in ['1.5', 'inf'] + ['2'] * (self.lp.RAYS - 2))
        ranges = self.lp.read_ranges(text)
        self.assertEqual(self.lp.RAYS, len(ranges))
        self.assertEqual(math.inf, ranges[1])
        self.assertIsNone(self.lp.read_ranges('ranges: 1\n'))   # a partial message is not a scan

    def test_the_probe_world_adds_one_sensors_system_and_the_lidars(self):
        text = self.lp.probe_world(WORLD, [(1.0, 1.0), (2.0, 1.0)], 0.1, shown=[(3.5, 2.5)])
        xml.dom.minidom.parseString(text)
        self.assertEqual(1, text.count('gz-sim-sensors-system'))
        self.assertEqual(3, text.count('type="gpu_lidar"'))
        self.assertEqual(1, text.count('<visualize>true</visualize>'))   # only the shown lidar draws its rays
        self.assertEqual(3, text.count('<min>0.1</min>'))   # the range floor given, on every lidar
        self.assertIn('<topic>probe1_scan</topic>', text)
        with self.assertRaises(ValueError):
            self.lp.probe_world('<sdf/>', [(1.0, 1.0)])

    def test_an_extra_model_loses_its_sensors_system_and_is_placed(self):
        model = ("<?xml version='1.0'?><sdf version='1.11'><model name='turtlebot4'><pose>1 2 3 0 0 0</pose>"
                 "<link name='base_link'><sensor name='rplidar' type='gpu_lidar'/></link>"
                 "<plugin name='gz::sim::systems::Sensors' filename='libgz-sim-sensors-system.so'>"
                 "<render_engine>ogre</render_engine></plugin>"
                 "<plugin name='x' filename='libgz_ros2_control-system.so'><a/></plugin>"
                 "<plugin name='c' filename='libgz-sim-contact-system.so'/></model></sdf>")
        out = self.lp.extra_model(model, 'extra0', 2.7, 0.72, 1.5708)
        self.assertNotIn('sensors-system', out)
        self.assertNotIn('ros2_control', out)
        self.assertIn('contact-system', out)
        self.assertIn("type='gpu_lidar'", out)                # its sensors stay in the scene
        self.assertIn('<pose>2.7 0.72 0 0 0 1.5708</pose>', out)
        self.assertNotIn('<pose>1 2 3 0 0 0</pose>', out)
        self.assertIn('<static>true</static>', out)


if __name__ == '__main__':
    unittest.main()
