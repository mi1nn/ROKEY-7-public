"""SLAM OccupancyGrid -> map_server files the console reads (--live-map); written so a reader never sees half a map."""
from pathlib import Path
import tempfile
import unittest

from rescue_console.mapimage import load_map
from rescue_operations.map_snapshot import LIVE_YAML, grid_to_pgm, write_snapshot, yaw_from_quaternion


def grid(width=2, height=2, data=(-1, 0, 100, 50), resolution=0.05, origin=(-1.0, -0.5), yaw_q=(0.0, 0.0, 0.0, 1.0)):
    return dict(width=width, height=height, data=list(data), resolution=resolution, origin_x=origin[0],
                origin_y=origin[1], orientation=yaw_q)


class GridToPgm(unittest.TestCase):
    def test_trinary_values_and_rows_flipped(self):
        """OccupancyGrid row 0 is the bottom (origin) row; PGM row 0 is the top. map_saver trinary values."""
        data = grid_to_pgm(2, 2, [-1, 0, 100, 50])
        header, pixels = data[:-4], data[-4:]
        self.assertEqual(b'P5\n2 2\n255\n', header)
        # top row = grid row 1 (100 occupied -> 0, 50 between thresholds -> unknown 205)
        # bottom row = grid row 0 (-1 unknown -> 205, 0 free -> 254)
        self.assertEqual(bytes([0, 205, 205, 254]), pixels)

    def test_wrong_cell_count_is_refused(self):
        with self.assertRaises(ValueError):
            grid_to_pgm(2, 2, [0, 0, 0])

    def test_yaw(self):
        self.assertEqual(0.0, yaw_from_quaternion(0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(1.5707963, yaw_from_quaternion(0.0, 0.0, 0.7071068, 0.7071068), places=5)


class WriteSnapshot(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())

    def test_console_reads_what_is_written(self):
        sha = write_snapshot(self.dir, **grid())
        image = load_map(self.dir / LIVE_YAML)
        self.assertEqual((2, 2), (image.meta['width_px'], image.meta['height_px']))
        self.assertEqual([-1.0, -0.5, 0.0], image.meta['origin'])
        self.assertEqual(0.05, image.meta['resolution'])
        self.assertEqual(sha, image.meta['pgm_sha256'])

    def test_new_map_replaces_and_old_images_are_pruned(self):
        first = write_snapshot(self.dir, **grid())
        second = write_snapshot(self.dir, **grid(data=(0, 0, 0, 0)))
        third = write_snapshot(self.dir, **grid(data=(100, 100, 100, 100)))
        self.assertEqual(third, load_map(self.dir / LIVE_YAML).meta['pgm_sha256'])
        images = sorted(p.name for p in self.dir.glob('*.pgm'))
        # the current image and the one before it stay (a reader may hold the previous yaml)
        self.assertEqual(sorted([f'map_{second[:12]}.pgm', f'map_{third[:12]}.pgm']), images)
        self.assertNotIn(f'map_{first[:12]}.pgm', images)
        self.assertEqual([], list(self.dir.glob('*.tmp')))

    def test_same_map_again_keeps_one_image(self):
        write_snapshot(self.dir, **grid())
        write_snapshot(self.dir, **grid())
        self.assertEqual(1, len(list(self.dir.glob('*.pgm'))))

    def test_rotated_origin_is_refused(self):
        """The console draws only yaw 0 maps; refuse here instead of writing a map it cannot show."""
        with self.assertRaises(ValueError):
            write_snapshot(self.dir, **grid(yaw_q=(0.0, 0.0, 0.7071068, 0.7071068)))
        self.assertFalse((self.dir / LIVE_YAML).exists())


if __name__ == '__main__':
    unittest.main()
