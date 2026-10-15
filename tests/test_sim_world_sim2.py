"""config/sim/worlds/sim2.sdf: the TurtleBot4 simulator maze taken as a second test world (재범 2026-10-16).

The file is the upstream maze.sdf with only the world name and a source comment changed, so run_sim.sh can start it
by name. The start places in sim2.spawn.json must stand clear of the maze blocks, with the dock that the spawn launch
puts 0.157 m ahead of each robot.
"""
import json
import math
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

WORLDS = Path(__file__).resolve().parents[1] / 'config' / 'sim' / 'worlds'
HAS_WORLD = (WORLDS / 'sim2.sdf').exists()
ROBOT_CLEAR_M = 0.5    # robot radius about 0.17 m plus room to turn [설정 2026-10-16]
DOCK_CLEAR_M = 0.3     # dock about 0.15 m across [설정]
DOCK_AHEAD_M = 0.157   # turtlebot4_spawn / spawn_extra put the dock this far ahead of the robot


def boxes(sdf_text):
    """(x, y, yaw, size_x, size_y) of every box collision, in the world frame (static models, model pose + link pose)."""
    out = []
    root = ET.fromstring(sdf_text)
    for model in root.iter('model'):
        mx, my = (float(v) for v in (model.findtext('pose') or '0 0').split()[:2])
        for link in model.iter('link'):
            size = link.find('collision/geometry/box/size')
            if size is None:
                continue
            px, py, _, _, _, yaw = (float(v) for v in (link.findtext('pose') or '0 0 0 0 0 0').split())
            sx, sy, _ = (float(v) for v in size.text.split())
            out.append((mx + px, my + py, yaw, sx, sy))
    return out


def clearance(x, y, rects):
    best = math.inf
    for bx, by, yaw, sx, sy in rects:
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx, ly = c * (x - bx) - s * (y - by), s * (x - bx) + c * (y - by)
        best = min(best, math.hypot(max(abs(lx) - sx / 2, 0.0), max(abs(ly) - sy / 2, 0.0)))
    return best


@unittest.skipUnless(HAS_WORLD, 'no config/sim (installed-wheel checkout)')
class Sim2World(unittest.TestCase):
    def setUp(self):
        self.text = (WORLDS / 'sim2.sdf').read_text()
        self.rects = boxes(self.text)

    def test_the_world_is_named_after_its_file(self):
        self.assertIsNotNone(re.search(r"<world name=['\"]sim2['\"]>", self.text))

    def test_it_is_a_walled_maze(self):
        self.assertEqual(40, len(self.rects))   # 4 border walls + 36 blocks, as upstream maze.sdf
        xs = [r[0] for r in self.rects]
        ys = [r[1] for r in self.rects]
        self.assertAlmostEqual(20.01, max(xs) - min(xs), places=2)
        self.assertAlmostEqual(20.01, max(ys) - min(ys), places=2)

    def test_every_start_place_and_its_dock_are_clear_of_the_blocks(self):
        spawn = json.loads((WORLDS / 'sim2.spawn.json').read_text())
        self.assertEqual('sim2', spawn['world'])
        self.assertEqual({'robot6', 'robot5'}, set(spawn['robots']))
        for name, start in spawn['robots'].items():
            with self.subTest(name):
                dock_x = start['x'] + DOCK_AHEAD_M * math.cos(start['yaw'])
                dock_y = start['y'] + DOCK_AHEAD_M * math.sin(start['yaw'])
                self.assertGreaterEqual(clearance(start['x'], start['y'], self.rects), ROBOT_CLEAR_M)
                self.assertGreaterEqual(clearance(dock_x, dock_y, self.rects), DOCK_CLEAR_M)

    def test_the_clearance_check_sees_a_block(self):
        self.assertAlmostEqual(0.0, clearance(-10.005, 0.0, self.rects))   # on the west border wall


if __name__ == '__main__':
    unittest.main()
