"""config/sim/worlds/arena.layout.json carries every pillar of the floor plan arena-map.svg (sim1, 2026-10-16).

The plan's hatched blocks smaller than 0.4 m^2 are pillars the robot drives around (the box pillar in the main zone,
the pillar at the north stub wall). Each must be a layout box with the plan's footprint (1 cm). The 「박스 기둥」 was
lost when tunnel 2 was edited in #364 and went unnoticed through four simulation runs; this keeps it from happening again.
"""
import json
from pathlib import Path
import re
import unittest

WORLDS = Path(__file__).resolve().parents[1] / 'config' / 'sim' / 'worlds'
HAS_PLAN = (WORLDS / 'arena-map.svg').exists()
PLAN_H_CM = 557   # plan y grows south; world y = (557 - plan y) / 100 (arena.layout.json note)


def plan_pillars(svg):
    """(x0, y0, x1, y1) in metres, world frame, of the hatched blocks under 0.4 m^2."""
    out = []
    for m in re.finditer(r'<rect x="([\d.]+)" y="([\d.]+)" width="([\d.]+)" height="([\d.]+)" fill="url\(#blk\)"', svg):
        x, y, w, h = (float(v) for v in m.groups())
        if w * h / 1e4 < 0.4:
            out.append((x / 100, (PLAN_H_CM - y - h) / 100, (x + w) / 100, (PLAN_H_CM - y) / 100))
    return out


@unittest.skipUnless(HAS_PLAN, 'no config/sim (installed-wheel checkout)')
class ArenaPlan(unittest.TestCase):
    def test_every_plan_pillar_is_a_layout_box(self):
        pillars = plan_pillars((WORLDS / 'arena-map.svg').read_text())
        self.assertEqual(2, len(pillars))   # the box pillar and the pillar at the north stub wall
        boxes = [o for o in json.loads((WORLDS / 'arena.layout.json').read_text())['objects'] if o['kind'] == 'box']
        for x0, y0, x1, y1 in pillars:
            with self.subTest(pillar=(x0, y0, x1, y1)):
                match = [b for b in boxes
                         if abs(b['x'] - (x0 + x1) / 2) < 0.01 and abs(b['y'] - (y0 + y1) / 2) < 0.01
                         and abs(b['size'][0] - (x1 - x0)) < 0.01 and abs(b['size'][1] - (y1 - y0)) < 0.01]
                self.assertEqual(1, len(match), f'no layout box for the plan pillar {(x0, y0, x1, y1)}')


class ArenaTunnels(unittest.TestCase):
    """Each tunnel is an upright board (wall panels, 0.41 m) across the way on the plan's dashed line, its lower edge
    at the clearance, as 재범's photos show (2026-10-16 07:4x tunnel 1: two panels side by side across the corridor on
    small MDF boxes on top of the two side walls; 08:0x tunnel 2: panels lying straight on the whitebox wall top and
    the box pillar top). From 2026-10-16 #428 to this change both were plates over
    the whole plan area instead, because our overhead judge answered UNKNOWN NO_FLOOR_SEEN in front of the thin board
    [측정 2026-10-16]; the real tunnel is the thin board, so that answer is what the real robot meets."""
    LINES = {'tunnel1_lintel': r'<line x1="([\d.]+)" y1="([\d.]+)" x2="([\d.]+)" y2="([\d.]+)" stroke="#1f8a5b" stroke-width="3" stroke-dasharray',
             'tunnel2_lintel': r'<line x1="([\d.]+)" y1="([\d.]+)" x2="([\d.]+)" y2="([\d.]+)" stroke="#c23a2b" stroke-width="3" stroke-dasharray'}
    # tunnel 1: plan 「47 cm」, the board on blocks on the wall tops; tunnel 2: the board straight on the whitebox wall
    # top and the box pillar top in 재범's photo 2026-10-16 08:0x (wall 0.41 m) [추정]; it was 0.30 before (재범 2026-10-15
    # 「30 cm 띄워야」, set before the photo)
    CLEARANCE_M = {'tunnel1_lintel': 0.47, 'tunnel2_lintel': 0.41}
    PANEL_M, BOARD_M = 0.41, 0.018   # a wall panel's height and thickness (arena.layout.json wall)
    # Tunnel 1 opened 0.22 m to the west, 0.68 -> 0.90 m (재범 2026-10-16 「입구 좀 넓혀봐」): the coordinator sends a
    # frontier goal only to a cell 0.35 m clear (0.385 m with the half cell), 15 free 0.05 m cells across; the 13회차
    # SLAM map had 12 in the 0.68 m gap and the frontier past it was left out (FRONTIER_GOAL_TIGHT).
    WIDEN_WEST_M = {'tunnel1_lintel': 0.22, 'tunnel2_lintel': 0.0}

    def objects(self):
        return {o['name']: o for o in json.loads((WORLDS / 'arena.layout.json').read_text())['objects']}

    @unittest.skipUnless(HAS_PLAN, 'no config/sim (installed-wheel checkout)')
    def test_every_tunnel_is_an_upright_board_on_the_plan_line_at_its_clearance(self):
        svg = (WORLDS / 'arena-map.svg').read_text()
        objects = self.objects()
        for name, pattern in self.LINES.items():
            with self.subTest(name):
                px1, py1, px2, py2 = (float(v) for v in re.search(pattern, svg).groups())
                x0, x1 = min(px1, px2) / 100 - self.WIDEN_WEST_M[name], max(px1, px2) / 100
                y0, y1 = (PLAN_H_CM - max(py1, py2)) / 100, (PLAN_H_CM - min(py1, py2)) / 100
                board = objects.get(name)
                self.assertIsNotNone(board, f'no {name} in the layout')
                self.assertEqual('overhead', board['kind'])
                self.assertAlmostEqual((x0 + x1) / 2, board['x'], delta=0.01)
                self.assertAlmostEqual((y0 + y1) / 2, board['y'], delta=0.01)
                across, thin = (x1 - x0, 1) if y1 - y0 < 1e-9 else (y1 - y0, 0)   # thin across the line
                self.assertGreaterEqual(board['size'][1 - thin], across - 0.01)   # it reaches over its supports
                self.assertLessEqual(board['size'][1 - thin], across + 0.2)
                self.assertEqual(self.BOARD_M, board['size'][thin])
                self.assertEqual(self.PANEL_M, board['size'][2])
                self.assertEqual(self.CLEARANCE_M[name], board['clearance_m'])
        self.assertNotIn('tunnel1_roof', objects)
        self.assertNotIn('tunnel2_roof', objects)

    def test_tunnel1_board_rests_on_a_block_on_each_side_wall(self):
        objects = self.objects()
        board, west, east = objects['tunnel1_lintel'], objects['whitebox_east'], objects['outer_east']
        for block, wall in ((objects['tunnel1_block_w'], west), (objects['tunnel1_block_e'], east)):
            with self.subTest(block['name']):
                self.assertEqual('overhead', block['kind'])
                self.assertAlmostEqual(wall['x1'], block['x'], delta=0.005)          # on the wall top
                self.assertAlmostEqual(board['y'], block['y'], delta=0.005)
                self.assertEqual(0.41, block['clearance_m'])                          # wall height
                self.assertAlmostEqual(board['clearance_m'], block['clearance_m'] + block['size'][2], delta=1e-9)

    def test_tunnel1_gap_is_0_90_m_and_its_board_spans_it(self):
        objects = self.objects()
        west, east, board = objects['whitebox_east'], objects['outer_east'], objects['tunnel1_lintel']
        self.assertEqual(west['x1'], west['x2'])
        self.assertAlmostEqual(0.90, east['x1'] - west['x1'], delta=0.005)
        self.assertLess(board['x'] - board['size'][0] / 2, west['x1'])     # reaches over both side walls
        self.assertGreaterEqual(board['x'] + board['size'][0] / 2, east['x1'] - 0.005)
        for name in ('whitebox_south', 'whitebox_north'):   # the enclosure stays closed at its new east wall
            self.assertAlmostEqual(west['x1'], max(objects[name]['x1'], objects[name]['x2']), delta=0.005)


@unittest.skipUnless(HAS_PLAN, 'no config/sim (installed-wheel checkout)')
class ArenaWallFeet(unittest.TestCase):
    def test_walls_stand_on_feet_marked_estimated(self):
        """재범 photo 2026-10-16: L brackets at the panel joints reach about 0.1 m out on the floor, under the lidar."""
        feet = json.loads((WORLDS / 'arena.layout.json').read_text()).get('wall_feet')
        self.assertIsNotNone(feet, 'no wall_feet in the layout')
        self.assertTrue(feet['estimate'])
        self.assertLess(feet['size'][2], 0.19)   # under the lidar plane
        self.assertAlmostEqual(0.10, feet['size'][1], delta=0.02)


if __name__ == '__main__':
    unittest.main()
