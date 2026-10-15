"""scripts/sim/compare_maps.py: how much a simulated SLAM map overlaps the real one (no ROS).

Simulation track stage 1 (재범 decision 2026-10-15 evening). The maps are made here from pixels, so nothing
depends on a simulator or a field file.
"""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / 'scripts' / 'sim' / 'compare_maps.py'
sys.path.insert(0, str(TOOL.parent))
HAS_TOOL = TOOL.exists()   # the installed-wheel check copies tests/ and config/ only

WIDTH, HEIGHT = 60, 40


def room():
    """A 60 x 40 cell room with an L-shaped inner wall and one short post: not symmetric, so one turn fits."""
    pixels = [[254] * WIDTH for _ in range(HEIGHT)]
    for c in range(WIDTH):
        pixels[0][c] = pixels[HEIGHT - 1][c] = 0
    for r in range(HEIGHT):
        pixels[r][0] = pixels[r][WIDTH - 1] = 0
    for c in range(10, 35):
        pixels[20][c] = 0
    for r in range(20, 33):
        pixels[r][34] = 0
    pixels[8][45] = pixels[9][45] = pixels[10][45] = 0
    return pixels


def write_map(directory, name, pixels, origin='[0.0, 0.0, 0.0]', resolution=0.05):
    height, width = len(pixels), len(pixels[0])
    (directory / f'{name}.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()
                                            + bytes(v for row in pixels for v in row))
    yaml = directory / f'{name}.yaml'
    yaml.write_text(f'image: {name}.pgm\nmode: trinary\nresolution: {resolution}\norigin: {origin}\nnegate: 0\n'
                    'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return yaml


def run(*args):
    return subprocess.run([sys.executable, '-I', str(TOOL), *map(str, args)], capture_output=True, text=True)


@unittest.skipUnless(HAS_TOOL, 'installed-wheel checkout has no scripts/')
class CompareMaps(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.dir = Path(tmp.name)

    def result(self, *args):
        done = run(*args, '--json')
        self.assertEqual(0, done.returncode, done.stderr)
        return json.loads(done.stdout)

    def test_the_same_map_overlaps_completely(self):
        a = write_map(self.dir, 'a', room())
        out = self.result('--sim', a, '--real', a)
        self.assertEqual(1.0, out['same_frame']['overlap'])
        self.assertEqual(1.0, out['aligned']['overlap'])
        # the walls are cells of 0.05 m: a fit within a cell or two is the best this can say about a turn
        self.assertAlmostEqual(0.0, (out['transform']['turn_deg'] + 180) % 360 - 180, delta=2.5)

    def test_a_turned_and_shifted_copy_is_found_again(self):
        real = write_map(self.dir, 'real', room())
        # the same walls seen from another start pose: turned 31 deg and moved, as a SLAM map is
        sim = write_map(self.dir, 'sim', room(), origin=f'[1.7, -0.9, {math.radians(31):.5f}]')
        out = self.result('--sim', sim, '--real', real)
        self.assertLess(out['same_frame']['overlap'], 0.5, 'the frames differ: no move, no overlap')
        self.assertGreater(out['aligned']['overlap'], 0.95)
        self.assertAlmostEqual(-31.0, (out['transform']['turn_deg'] + 180) % 360 - 180, delta=1.0)

    def test_no_align_keeps_the_given_frames(self):
        real = write_map(self.dir, 'real', room())
        sim = write_map(self.dir, 'sim', room(), origin='[1.7, -0.9, 0.5]')
        out = self.result('--sim', sim, '--real', real, '--no-align')
        self.assertEqual(out['same_frame'], out['aligned'])
        self.assertEqual(0.0, out['transform']['turn_deg'])

    def test_a_partly_mapped_field_has_full_precision_and_lower_recall(self):
        real = write_map(self.dir, 'real', room())
        half = room()
        for row in half:
            row[WIDTH // 2:] = [205] * (WIDTH - WIDTH // 2)   # the right half was never seen: unknown
        sim = write_map(self.dir, 'sim', half, origin='[0.4, 0.2, 0.0]')
        out = self.result('--sim', sim, '--real', real)['aligned']
        self.assertGreater(out['precision'], 0.95)
        self.assertLess(out['recall'], 0.8)
        self.assertLess(out['overlap'], 0.95)

    def test_a_different_field_scores_low(self):
        other = [[254] * WIDTH for _ in range(HEIGHT)]
        for r in range(5, 35):
            other[r][5] = other[r][12] = other[r][30] = 0
        out = self.result('--sim', write_map(self.dir, 'sim', other), '--real', write_map(self.dir, 'real', room()))
        self.assertLess(out['aligned']['overlap'], 0.6)

    def test_the_picture_is_a_png(self):
        a = write_map(self.dir, 'a', room())
        png = self.dir / 'overlay.png'
        done = run('--sim', a, '--real', a, '--png', png)
        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual(b'\x89PNG\r\n\x1a\n', png.read_bytes()[:8])
        self.assertIn('겹침 100.0%', done.stdout)

    def test_bad_input_is_refused_not_guessed(self):
        good = write_map(self.dir, 'good', room())
        broken = self.dir / 'broken.yaml'
        broken.write_text('image: good.pgm\nmode: trinary\n')
        scaled = write_map(self.dir, 'scaled', [[254] * 5 for _ in range(5)])   # no wall at all
        for name, sim in (('missing keys', broken), ('no occupied cell', scaled), ('no file', self.dir / 'x.yaml')):
            with self.subTest(name):
                done = run('--sim', sim, '--real', good)
                self.assertEqual(2, done.returncode)
                self.assertIn('compare_maps:', done.stderr)


if __name__ == '__main__':
    unittest.main()
