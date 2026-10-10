"""Console map panel: PGM -> PNG is exact, map-frame poses land on the right pixel, nothing is guessed."""
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import struct
import tempfile
import unittest
import zlib

from rescue_console.app import current_view
from rescue_console.mapimage import (MAP_FRAME, PNG_SIGNATURE, encode_png, load_map, map_overlay,
                                     parse_map_yaml, read_pgm, world_to_pixel)
from rescue_console.render import render_fragment, render_page
from rescue_console.view import ConsoleError
from test_console_view import LedgerFixture, digest_dir
from test_console_web import ADMIN, AccountsFixture

HAS_FLASK = importlib.util.find_spec('flask') is not None
ROOT = Path(__file__).resolve().parents[1]
YAML = """image: map.pgm
mode: trinary
resolution: 0.050
origin: [-1.0, -0.5, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
WIDTH, HEIGHT = 4, 3
PIXELS = bytes(range(0, 240, 20))   # 12 distinct grey values, top row first


def pgm(width=WIDTH, height=HEIGHT, pixels=PIXELS, header=None):
    return (header or f'P5\n# made by test\n{width} {height}\n255\n'.encode()) + pixels


def decode_png(data):
    """Minimal PNG reader for the files encode_png writes (8-bit grey, filter 0)."""
    assert data[:8] == PNG_SIGNATURE
    pos, chunks = 8, []
    while pos < len(data):
        length, kind = struct.unpack('>I4s', data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + length]
        crc, = struct.unpack('>I', data[pos + 8 + length:pos + 12 + length])
        assert crc == zlib.crc32(kind + body), kind
        chunks.append((kind, body))
        pos += 12 + length
    assert [kind for kind, _ in chunks] == [b'IHDR', b'IDAT', b'IEND']
    width, height, depth, colour, _, _, _ = struct.unpack('>IIBBBBB', chunks[0][1])
    assert (depth, colour) == (8, 0)
    raw = zlib.decompress(chunks[1][1])
    rows = [raw[r * (width + 1):(r + 1) * (width + 1)] for r in range(height)]
    assert all(row[0] == 0 for row in rows)
    return width, height, b''.join(row[1:] for row in rows)


class MapFiles(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)

    def write(self, yaml_text=YAML, pgm_bytes=None):
        (self.dir / 'map.yaml').write_text(yaml_text, encoding='utf-8')
        (self.dir / 'map.pgm').write_bytes(pgm() if pgm_bytes is None else pgm_bytes)
        return self.dir / 'map.yaml'


class PgmPngTests(MapFiles):
    def test_pgm_header_with_comment_is_read_exactly(self):
        self.assertEqual(read_pgm(pgm()), (WIDTH, HEIGHT, PIXELS, 255))

    def test_png_round_trips_the_pixels(self):
        self.assertEqual(decode_png(encode_png(WIDTH, HEIGHT, PIXELS)), (WIDTH, HEIGHT, PIXELS))

    def test_loaded_map_png_is_the_pgm_as_saved(self):
        image = load_map(self.write())
        self.assertEqual(decode_png(image.png), (WIDTH, HEIGHT, PIXELS))
        self.assertEqual(image.meta['pgm_sha256'], hashlib.sha256(pgm()).hexdigest())
        self.assertEqual((image.meta['width_px'], image.meta['height_px']), (WIDTH, HEIGHT))
        self.assertEqual(image.meta['origin'], [-1.0, -0.5, 0.0])
        self.assertTrue(image.meta['url'].startswith('map.png?v='))

    def test_negate_flips_so_dark_stays_occupied(self):
        image = load_map(self.write(YAML.replace('negate: 0', 'negate: 1')))
        self.assertEqual(decode_png(image.png)[2], bytes(255 - v for v in PIXELS))

    def test_bad_pgm_is_refused(self):
        cases = {
            'ascii P2': pgm(header=b'P2\n4 3\n255\n'),
            'short raster': pgm(pixels=PIXELS[:-1]),
            'long raster': pgm(pixels=PIXELS + b'\x00'),
            'maxval 16 bit': pgm(header=b'P5\n4 3\n65535\n'),
            'zero size': pgm(header=b'P5\n0 3\n255\n', pixels=b''),
            'truncated header': b'P5\n4',
        }
        for label, data in cases.items():
            with self.subTest(label), self.assertRaises(ConsoleError) as caught:
                read_pgm(data)
            self.assertEqual(caught.exception.detail_code, 'INVALID_INPUT')

    def test_bad_yaml_is_refused_not_defaulted(self):
        cases = {
            'missing origin': YAML.replace('origin: [-1.0, -0.5, 0]\n', ''),
            'unknown key': YAML + 'free_thres: 0.2\n',
            'nested': YAML + 'extra:\n  a: 1\n',
            'rotated origin': YAML.replace('[-1.0, -0.5, 0]', '[-1.0, -0.5, 0.3]'),
            'two-value origin': YAML.replace('[-1.0, -0.5, 0]', '[-1.0, -0.5]'),
            'zero resolution': YAML.replace('0.050', '0'),
            'text resolution': YAML.replace('0.050', 'fine'),
            'negate 2': YAML.replace('negate: 0', 'negate: 2'),
        }
        for label, text in cases.items():
            with self.subTest(label), self.assertRaises(ConsoleError) as caught:
                load_map(self.write(text))
            self.assertEqual(caught.exception.detail_code, 'INVALID_INPUT')

    def test_missing_files_are_storage_errors_and_nothing_is_created(self):
        with self.assertRaises(ConsoleError) as caught:
            load_map(self.dir / 'absent.yaml')
        self.assertEqual(caught.exception.detail_code, 'STORAGE_ERROR')
        (self.dir / 'map.yaml').write_text(YAML, encoding='utf-8')
        with self.assertRaises(ConsoleError) as caught:
            load_map(self.dir / 'map.yaml')                      # yaml present, pgm absent
        self.assertEqual(caught.exception.detail_code, 'STORAGE_ERROR')
        self.assertEqual(sorted(p.name for p in self.dir.iterdir()), ['map.yaml'])

    def test_flat_yaml_subset(self):
        self.assertEqual(parse_map_yaml('image: "a b.pgm"  # quoted\nresolution: 1\norigin: [0, 0, 0]\n'),
                         {'image': 'a b.pgm', 'resolution': 1, 'origin': [0, 0, 0]})


class ProjectionTests(MapFiles):
    def setUp(self):
        super().setUp()
        self.meta = load_map(self.write()).meta     # 4 x 3 px, 0.05 m/px, origin (-1.0, -0.5)

    def test_origin_is_bottom_left_and_y_is_flipped(self):
        self.assertEqual(world_to_pixel(self.meta, -1.0, -0.5), (0, 3))                  # bottom-left corner
        self.assertEqual(world_to_pixel(self.meta, -1.0 + 4 * 0.05, -0.5 + 3 * 0.05), (4, 0))  # top-right
        self.assertEqual(world_to_pixel(self.meta, -1.0 + 0.025, -0.5 + 0.025), (0.5, 2.5))    # centre of bottom-left cell
        self.assertEqual(world_to_pixel(self.meta, -1.0 + 0.025, -0.5 + 3 * 0.05 - 0.025), (0.5, 0.5))

    def test_points_off_the_image_are_none_not_clamped(self):
        for x, y in ((-1.01, -0.4), (-0.79, -0.4), (-0.9, -0.51), (-0.9, -0.34)):
            with self.subTest(x=x, y=y):
                self.assertIsNone(world_to_pixel(self.meta, x, y))

    def test_overlay_plots_map_frame_only_and_lists_the_rest(self):
        robots = [
            {'robot_id': 'robot_a', 'role': 'SCOUT', 'pose': {'x': -0.9, 'y': -0.45, 'yaw': 1.0, 'frame_id': MAP_FRAME}},
            {'robot_id': 'robot_b', 'role': 'SUPPORT', 'pose': {'x': -0.9, 'y': -0.45, 'yaw': 0.0, 'frame_id': 'odom'}},
            {'robot_id': 'robot_c', 'role': None, 'pose': None},
        ]
        targets = [
            {'target_id': 't-in', 'stage': 'NOT_RESCUED', 'position': {'x': -0.85, 'y': -0.4, 'frame_id': MAP_FRAME}},
            {'target_id': 't-out', 'stage': 'RESCUED', 'position': {'x': 3.0, 'y': 1.0, 'frame_id': MAP_FRAME}},
        ]
        layer = map_overlay(self.meta, robots, targets)
        self.assertEqual([(r['id'], r['px'], r['py'], r['yaw']) for r in layer['robots']], [('robot_a', 2.0, 2.0, 1.0)])
        self.assertEqual([(t['id'], t['px'], t['py'], t['stage']) for t in layer['targets']],
                         [('t-in', 3.0, 1.0, 'NOT_RESCUED')])
        self.assertEqual([(w['kind'], w['id'], w['frame_id']) for w in layer['frame_mismatch']],
                         [('robot', 'robot_b', 'odom')])
        self.assertEqual([(w['kind'], w['id']) for w in layer['off_map']], [('target', 't-out')])
        self.assertTrue(layer['available'])


class MapScreenTests(LedgerFixture):
    """The fixture puts robots at (0.5, -1.0) and targets near (1, 2) / (3.75, -4.0), all in frame map."""

    def map_image(self, origin='[-1.0, -5.0, 0]', size=(120, 160)):
        folder = self.dir / 'map'
        folder.mkdir(exist_ok=True)
        (folder / 'map.yaml').write_text(YAML.replace('[-1.0, -0.5, 0]', origin), encoding='utf-8')
        width, height = size
        (folder / 'map.pgm').write_bytes(pgm(width, height, bytes(width * height), f'P5\n{width} {height}\n255\n'.encode()))
        return load_map(folder / 'map.yaml')

    def test_page_draws_robots_and_targets_on_the_map(self):
        view = current_view(self.db, None, clock=lambda: 1, map_image=self.map_image())
        # status fixture is not passed, so robots have no pose: only ledger targets are drawn
        self.assertEqual(view['map']['robots'], [])
        self.assertEqual(len(view['map']['targets']), len(view['targets']))
        html = render_fragment(view)
        self.assertIn('<image href="map.png?v=', html)
        self.assertEqual(html.count('<g class="victim '), len(view['targets']))
        page = render_page(view)
        self.assertEqual(re.findall(r"fetch\('([^']*)'", page), ['fragment'])   # the image is not a poll

    def test_robot_heading_and_off_map_warning(self):
        status_path = self.dir / 'status.json'
        status_path.write_text(json.dumps(self.status), encoding='utf-8')
        # 1 m x 1 m map around the robots only: every target is off the map
        image = self.map_image(origin='[0.0, -1.5, 0]', size=(20, 20))
        view = current_view(self.db, status_path, clock=lambda: 1, map_image=image)
        self.assertEqual([r['id'] for r in view['map']['robots']], ['robot_a', 'robot_b'])
        self.assertEqual({w['kind'] for w in view['map']['off_map']}, {'target'})
        self.assertEqual(len([w for w in view['warnings'] if w.startswith('지도 밖 좌표')]), len(view['targets']))
        html = render_fragment(view)
        self.assertEqual(html.count('class="heading"'), 2)
        self.assertIn('지도 밖', html)

    def test_without_map_the_placeholder_stays(self):
        view = current_view(self.db, None, clock=lambda: 1)
        self.assertFalse(view['map']['available'])
        html = render_fragment(view)
        self.assertIn('id="map" class="placeholder"', html)
        self.assertNotIn('<img', html)

    def test_untrusted_ids_are_escaped_in_the_overlay(self):
        layer = map_overlay(self.map_image().meta, [{'robot_id': '<r>', 'role': '"x', 'pose': {
            'x': 0.0, 'y': 0.0, 'yaw': None, 'frame_id': MAP_FRAME}}], [])
        html = render_fragment(dict(self.view(), map=layer))
        self.assertNotIn('<r>', html)
        self.assertIn('&lt;r&gt;', html)


class PreQuakeMapAssetTests(unittest.TestCase):
    def test_pre_quake_map_matches_its_manifest(self):
        manifest_path = ROOT / 'assets' / 'manifest' / 'pre_quake_map.json'
        if not manifest_path.exists():
            self.skipTest('source checkout only: assets are not installed')
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        folder = ROOT / manifest['asset_directory']
        for name, digest in manifest['files'].items():
            self.assertEqual(hashlib.sha256((folder / name).read_bytes()).hexdigest(), digest, name)
        image = load_map(folder / 'map.yaml')
        meta = image.meta
        self.assertEqual([meta['width_px'], meta['height_px']], manifest['size_px'])
        self.assertEqual(meta['resolution'], manifest['resolution_m'])
        self.assertEqual(meta['origin'], [float(v) for v in manifest['origin']])
        self.assertEqual(meta['pgm_sha256'], manifest['files']['map.pgm'])
        width, height, pixels = decode_png(image.png)
        self.assertEqual((width, height), tuple(manifest['size_px']))
        self.assertEqual(pixels, read_pgm((folder / 'map.pgm').read_bytes())[2])
        # the far corner of the stated area is the top-right image corner
        x, y = manifest['origin'][0] + manifest['area_m'][0], manifest['origin'][1] + manifest['area_m'][1]
        self.assertEqual(world_to_pixel(meta, x, y), (float(width), 0.0))


class MapRouteTests(AccountsFixture):
    def app(self, ledger=None, map_path=None):
        from rescue_console.app import create_app
        from test_console_web import SECRET
        return create_app(ledger or self.db, accounts_path=self.accounts, secret_key=SECRET, map_path=map_path)

    def map_path(self):
        (self.dir / 'map.yaml').write_text(YAML, encoding='utf-8')
        (self.dir / 'map.pgm').write_bytes(pgm())
        return self.dir / 'map.yaml'

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_map_png_needs_login_and_is_get_only(self):
        app = self.app(map_path=self.map_path())
        client = app.test_client()
        self.assertEqual(client.get('/map.png').status_code, 401)
        self.login(client, *ADMIN)
        before = digest_dir(self.dir)
        response = client.get('/map.png')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.mimetype, 'image/png')
        self.assertEqual(decode_png(response.get_data()), (WIDTH, HEIGHT, PIXELS))
        self.assertIn('max-age', response.headers['Cache-Control'])
        self.assertEqual(client.post('/map.png').status_code, 405)
        self.assertIn('<image href="map.png?v=', client.get('/').get_data(as_text=True))
        self.assertIsNotNone(client.get('/api/view').get_json()['map']['url'])
        self.assertEqual(digest_dir(self.dir), before)

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_no_map_is_404_and_bad_map_fails_at_startup(self):
        client = self.app().test_client()
        self.login(client, *ADMIN)
        self.assertEqual(client.get('/map.png').status_code, 404)
        (self.dir / 'bad.yaml').write_text('image: none.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n', encoding='utf-8')
        with self.assertRaises(ConsoleError) as caught:
            self.app(map_path=self.dir / 'bad.yaml')
        self.assertEqual(caught.exception.detail_code, 'STORAGE_ERROR')


if __name__ == '__main__':
    unittest.main()
