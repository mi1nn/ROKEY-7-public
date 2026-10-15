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
    """Each plan tunnel area (tunnel 1 green, tunnel 2 hatched) is one overhead roof over the whole area (1 cm).
    A 1 cm wide lintel along the plan's dashed line left no floor under the structure in the depth box, so our own
    overhead judge answered UNKNOWN NO_FLOOR_SEEN in front of both tunnels [측정 2026-10-16]."""
    AREAS = {'tunnel1_roof': r'<rect x="([\d.]+)" y="([\d.]+)" width="([\d.]+)" height="([\d.]+)" fill="rgba\(31,138,91',
             'tunnel2_roof': r'<rect x="([\d.]+)" y="([\d.]+)" width="([\d.]+)" height="([\d.]+)" fill="url\(#hatch\)"'}
    CLEARANCE_M = {'tunnel1_roof': 0.47, 'tunnel2_roof': 0.30}   # plan 「47 cm」 / 재범 2026-10-15

    @unittest.skipUnless(HAS_PLAN, 'no config/sim (installed-wheel checkout)')
    def test_every_plan_tunnel_area_is_one_roof_at_its_clearance(self):
        svg = (WORLDS / 'arena-map.svg').read_text()
        objects = {o['name']: o for o in json.loads((WORLDS / 'arena.layout.json').read_text())['objects']}
        for name, pattern in self.AREAS.items():
            with self.subTest(name):
                x, y, w, h = (float(v) for v in re.search(pattern, svg).groups())
                roof = objects.get(name)
                self.assertIsNotNone(roof, f'no {name} in the layout')
                self.assertEqual('overhead', roof['kind'])
                self.assertAlmostEqual((x + w / 2) / 100, roof['x'], delta=0.01)
                self.assertAlmostEqual((PLAN_H_CM - y - h / 2) / 100, roof['y'], delta=0.01)
                self.assertAlmostEqual(w / 100, roof['size'][0], delta=0.01)
                self.assertAlmostEqual(h / 100, roof['size'][1], delta=0.01)
                self.assertEqual(self.CLEARANCE_M[name], roof['clearance_m'])


if __name__ == '__main__':
    unittest.main()
