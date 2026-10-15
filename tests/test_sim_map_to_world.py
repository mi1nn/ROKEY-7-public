"""scripts/sim/map_to_world.py: a SLAM map (pgm + yaml) and a layout list -> a Gazebo world (no ROS).

Simulation track stage 1 (재범 decision 2026-10-15 evening): the practice field rebuilt in Gazebo from the
10-14 TB-6 map. The map is only an input here; the robots in the simulation never get it.
"""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / 'scripts' / 'sim' / 'map_to_world.py'
sys.path.insert(0, str(TOOL.parent))
HAS_TOOL = TOOL.exists()   # the installed-wheel check copies tests/ and config/ only

# 6 x 4 cells, top row first (pgm order). 0 = occupied (black), 254 = free, 205 = unknown.
#   row 0: # # # # # #
#   row 1: # . . . . .
#   row 2: # . . ? . #     <- one lone occupied cell at the right end
#   row 3: # . . . . .
PIXELS = [
    [0, 0, 0, 0, 0, 0],
    [0, 254, 254, 254, 254, 254],
    [0, 254, 254, 205, 254, 0],
    [0, 254, 254, 254, 254, 254],
]


def write_map(directory, pixels=PIXELS, origin='[-1.0, -0.5, 0.0]', negate=0, binary=True):
    height, width = len(pixels), len(pixels[0])
    body = bytes(v for row in pixels for v in row)
    pgm = directory / 'm.pgm'
    if binary:
        pgm.write_bytes(f'P5\n# test\n{width} {height}\n255\n'.encode() + body)
    else:
        pgm.write_text(f'P2\n{width} {height}\n255\n' + '\n'.join(' '.join(map(str, r)) for r in pixels) + '\n')
    yaml = directory / 'm.yaml'
    yaml.write_text(f'image: m.pgm\nmode: trinary\nresolution: 0.5\norigin: {origin}\nnegate: {negate}\n'
                    'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return yaml


def layout(**changes):
    raw = {
        'wall': {'height_m': 0.6, 'thickness_m': 0.018, 'color': [0.55, 0.38, 0.22], 'estimate': True},
        'floor': {'color': [0.5, 0.5, 0.5]},
        'min_cells': 2,
        'robots': {'robot6': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                   'robot5': {'x': 0.0, 'y': -1.86, 'yaw': 0.0, 'source': 'tape 2026-10-15'}},
        'objects': [
            {'name': 'white_box', 'kind': 'box', 'x': 1.0, 'y': 0.5, 'yaw': 0.3, 'size': [0.4, 0.3, 0.3],
             'color': [1, 1, 1], 'estimate': True},
            {'name': 'doll_yellow', 'kind': 'cylinder', 'x': 1.5, 'y': 0.2, 'radius': 0.1, 'length': 0.3,
             'color': [1, 0.85, 0.1], 'estimate': True},
            {'name': 'tunnel', 'kind': 'overhead', 'x': 0.5, 'y': -0.2, 'yaw': 0.0, 'size': [0.6, 0.5, 0.02],
             'clearance_m': 0.46, 'color': [0.55, 0.38, 0.22], 'estimate': True},
        ],
    }
    raw.update(changes)
    return raw


@unittest.skipUnless(HAS_TOOL, 'installed-wheel checkout has no scripts/')
class Tmp(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.dir = Path(tmp.name)


class ReadMap(Tmp):
    def test_occupied_cells_bottom_up(self):
        import map_to_world as mw
        for binary in (True, False):
            with self.subTest(binary=binary):
                grid = mw.read_map(write_map(self.dir, binary=binary))
                self.assertEqual((6, 4, 0.5, (-1.0, -0.5, 0.0)), (grid.width, grid.height, grid.resolution,
                                                                   grid.origin))
                # pgm row 0 is the top: y index 3
                self.assertEqual({(c, 3) for c in range(6)} | {(0, 2), (0, 1), (0, 0), (5, 1)}, grid.occupied)

    def test_negate_and_unknown(self):
        import map_to_world as mw
        grid = mw.read_map(write_map(self.dir, pixels=[[255 - v for v in row] for row in PIXELS], negate=1))
        self.assertIn((0, 0), grid.occupied)
        self.assertNotIn((3, 1), grid.occupied)          # 205 unknown is not a wall

    def test_unexpected_files_are_refused(self):
        import map_to_world as mw
        yaml = write_map(self.dir)
        for name, text in (('raw mode', yaml.read_text().replace('trinary', 'raw')),
                           ('no resolution', yaml.read_text().replace('resolution: 0.5\n', ''))):
            with self.subTest(name), self.assertRaises(mw.WorldError):
                yaml.write_text(text)
                mw.read_map(yaml)
        write_map(self.dir)
        (self.dir / 'm.pgm').write_bytes(b'P6\n1 1\n255\n\x00\x00\x00')
        with self.assertRaises(mw.WorldError):
            mw.read_map(self.dir / 'm.yaml')


class Rectangles(Tmp):
    def test_cells_merge_into_few_boxes_and_lone_cells_drop(self):
        import map_to_world as mw
        grid = mw.read_map(write_map(self.dir))
        cells = mw.drop_small(grid.occupied, min_cells=2)
        self.assertNotIn((5, 1), cells)                 # the lone cell is noise
        rects = mw.rectangles(cells)
        self.assertEqual(sorted([(0, 0, 1, 3), (0, 3, 6, 1)]), sorted(rects))   # (col, row, w, h)
        covered = {(c + i, r + j) for c, r, w, h in rects for i in range(w) for j in range(h)}
        self.assertEqual(cells, covered)

    def test_box_poses_follow_the_map_origin(self):
        import map_to_world as mw
        grid = mw.read_map(write_map(self.dir))
        x, y, yaw, sx, sy = mw.cell_box((0, 3, 6, 1), grid)
        self.assertEqual((0.5, 1.25, 0.0, 3.0, 0.5), (round(x, 6), round(y, 6), yaw, sx, sy))
        turned = mw.read_map(write_map(self.dir, origin=f'[1.0, 2.0, {math.pi / 2}]'))
        x, y, yaw, sx, sy = mw.cell_box((0, 0, 1, 1), turned)   # cell centre (0.25, 0.25) in the map image
        self.assertEqual((0.75, 2.25), (round(x, 6), round(y, 6)))
        self.assertAlmostEqual(math.pi / 2, yaw)


class World(Tmp):
    def build(self, raw=None, name='practice'):
        import map_to_world as mw
        path = self.dir / 'layout.json'
        path.write_text(json.dumps(raw or layout()))
        return mw.build(mw.read_map(write_map(self.dir)), mw.load_layout(path), name=name)

    def test_world_has_walls_floor_objects_and_the_name(self):
        world, spawn, estimates = self.build()
        root = ET.fromstring(world)
        self.assertEqual('practice', root.find('world').get('name'))   # turtlebot4 world:=practice
        names = [m.get('name') for m in root.iter('model')]
        self.assertIn('ground_plane', names)
        self.assertEqual(2, len([n for n in names if n.startswith('wall_')]))
        for name in ('white_box', 'doll_yellow', 'tunnel'):
            self.assertIn(name, names)
        walls = [m for m in root.iter('model') if m.get('name').startswith('wall_')]
        heights = {float(m.find('.//collision/geometry/box/size').text.split()[2]) for m in walls}
        self.assertEqual({0.6}, heights)
        tunnel = next(m for m in root.iter('model') if m.get('name') == 'tunnel')
        z = float(tunnel.find('pose').text.split()[2])
        self.assertAlmostEqual(0.46 + 0.01, z)              # bottom face at the clearance

    def test_a_measured_wall_segment_uses_the_wall_height_and_thickness(self):
        """Walls the map did not see (measured outline) are segments; the MDF height and thickness apply."""
        raw = layout(objects=[{'name': 'outline_south', 'kind': 'wall', 'x1': 0.0, 'y1': 0.0, 'x2': 3.0,
                               'y2': 4.0, 'estimate': True}])
        world, _, estimates = self.build(raw)
        model = next(m for m in ET.fromstring(world).iter('model') if m.get('name') == 'outline_south')
        x, y, z, _, _, yaw = (float(v) for v in model.find('pose').text.split())
        size = [float(v) for v in model.find('.//collision/geometry/box/size').text.split()]
        self.assertEqual((1.5, 2.0, 0.3), (x, y, z))
        self.assertAlmostEqual(math.atan2(4, 3), yaw, places=5)
        self.assertEqual([5.0, 0.018, 0.6], size)
        self.assertEqual(['wall', 'outline_south'], estimates)

    def test_spawn_and_estimates(self):
        _, spawn, estimates = self.build()
        self.assertEqual({'robot6': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                          'robot5': {'x': 0.0, 'y': -1.86, 'yaw': 0.0}}, spawn['robots'])
        self.assertEqual(['wall', 'white_box', 'doll_yellow', 'tunnel'], estimates)

    def test_no_value_is_made_up(self):
        import map_to_world as mw
        for name, raw in (('no wall height', layout(wall={'thickness_m': 0.018, 'color': [1, 1, 1]})),
                          ('no robots', {k: v for k, v in layout().items() if k != 'robots'}),
                          ('object without size', layout(objects=[{'name': 'b', 'kind': 'box', 'x': 0, 'y': 0,
                                                                    'yaw': 0, 'color': [1, 1, 1]}])),
                          ('unknown kind', layout(objects=[{'name': 'b', 'kind': 'cone', 'x': 0, 'y': 0}])),
                          ('two objects one name', layout(objects=[layout()['objects'][0]] * 2)),
                          ('unknown key', layout(colour=[1, 1, 1]))):
            with self.subTest(name), self.assertRaises(mw.WorldError):
                self.build(raw)

    def test_command_line_writes_world_and_spawn(self):
        (self.dir / 'layout.json').write_text(json.dumps(layout()))
        out = self.dir / 'out'
        result = subprocess.run([sys.executable, '-I', str(TOOL), '--map', str(write_map(self.dir)),
                                 '--layout', str(self.dir / 'layout.json'), '--name', 'practice',
                                 '--out-dir', str(out)], capture_output=True, text=True)
        self.assertEqual(0, result.returncode, result.stderr)
        ET.parse(out / 'practice.sdf')
        spawn = json.loads((out / 'practice.spawn.json').read_text())
        self.assertEqual('practice', spawn['world'])
        self.assertIn('[추정]', result.stdout)


@unittest.skipUnless(HAS_TOOL, 'installed-wheel checkout has no scripts/')
class RepositoryWorld(unittest.TestCase):
    """config/sim/worlds/<name>.sdf and .spawn.json are what the tool makes from the map and layout now."""

    def test_committed_worlds_are_up_to_date(self):
        import map_to_world as mw
        worlds = REPO / 'config' / 'sim' / 'worlds'
        layouts = sorted(worlds.glob('*.layout.json')) if worlds.exists() else []
        if not layouts:
            self.skipTest('no config/sim (installed-wheel checkout)')
        for path in layouts:
            name = path.name[:-len('.layout.json')]
            with self.subTest(world=name):
                maps = REPO / 'config' / 'sim' / 'maps'
                source = json.loads((worlds / f'{name}.sources.json').read_text())['map']
                world, spawn, _ = mw.build(mw.read_map(maps / source), mw.load_layout(path), name=name,
                                           source=source)
                self.assertEqual(world, (worlds / f'{name}.sdf').read_text())
                self.assertEqual(spawn, json.loads((worlds / f'{name}.spawn.json').read_text()))


if __name__ == '__main__':
    unittest.main()
