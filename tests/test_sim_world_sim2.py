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
        # the maze blocks only (what the robots and docks must stand clear of); the roofs are overhead
        self.blocks = [r for r in boxes((WORLDS / 'maze.base.sdf').read_text())]

    def test_the_world_is_named_after_its_file(self):
        self.assertIsNotNone(re.search(r"<world name=['\"]sim2['\"]>", self.text))

    def test_it_is_a_walled_maze(self):
        self.assertEqual(40 + 2, len(self.rects))   # 4 border walls + 36 blocks (upstream maze.sdf) + 2 tunnel roofs
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
                self.assertGreaterEqual(clearance(start['x'], start['y'], self.blocks), ROBOT_CLEAR_M)
                self.assertGreaterEqual(clearance(dock_x, dock_y, self.blocks), DOCK_CLEAR_M)

    def test_the_two_robots_and_docks_stand_side_by_side_close(self):
        robots = json.loads((WORLDS / 'sim2.spawn.json').read_text())['robots']
        a, b = robots['robot6'], robots['robot5']
        self.assertEqual(a['yaw'], b['yaw'])                                  # facing the same way, docks in a row
        self.assertAlmostEqual(0.5, math.hypot(a['x'] - b['x'], a['y'] - b['y']), places=3)
        self.assertGreater(math.hypot(a['x'] - b['x'], a['y'] - b['y']), 0.34)  # robot diameter

    def test_each_tunnel_roof_spans_a_corridor_and_the_doll_is_clear(self):
        layout = json.loads((WORLDS / 'sim2.layout.json').read_text())
        objects = {o['name']: o for o in layout['objects']}
        self.assertEqual(0.47, objects['tunnel1_roof']['clearance_m'])
        self.assertEqual(0.30, objects['tunnel2_roof']['clearance_m'])
        for name in ('tunnel1_roof', 'tunnel2_roof'):
            with self.subTest(name):
                roof = objects[name]
                self.assertGreaterEqual(clearance(roof['x'], roof['y'], self.blocks), 0.45)   # floor under it is free
                # blocks on both long sides: the roof closes the corridor, there is no way around it
                for side in (-1, 1):
                    edge_y = roof['y'] + side * (roof['size'][1] / 2 - 0.02)
                    self.assertLess(clearance(roof['x'], edge_y, self.blocks), 0.05)
        doll = json.loads((WORLDS / 'sim2.spawn.json').read_text())['doll']
        self.assertGreaterEqual(clearance(doll[0], doll[1], self.blocks), 0.3)

    def test_the_clearance_check_sees_a_block(self):
        self.assertAlmostEqual(0.0, clearance(-10.005, 0.0, self.rects))   # on the west border wall


if __name__ == '__main__':
    unittest.main()
