"""Map archive for replay: db_writer keeps the run's SLAM and frozen maps (작전·웹개발 2026-10-15; E2).

Every kept map is a PGM + YAML under ``<map_dir>/<run_id>/`` and one ``map_snapshots`` row of the open run.
The live SLAM map (TB-6, SCOUT) is thinned to one every ARCHIVE_PERIOD_NS and an image already kept is not
kept again; its revision is the SLAM robot's current MapState revision, so nothing is kept before one is
heard. The two frozen maps are always kept, each once its /rescue/map/frozen_revision text names its
occupancy hash (never guessed). Between runs nothing is kept. The map_snapshot node and its live_map.yaml
for the console stay as they are.
"""
from pathlib import Path
import hashlib
import tempfile
import unittest

from rescue_contracts.occupancy import frozen_revision_text, occupancy_sha256
from rescue_operations import site_state
from rescue_operations.map_snapshot import grid_to_pgm
from test_operations_site_state import uid

ROBOTS = ('robot_a', 'robot_b')
SECOND = 1_000_000_000
T0 = 1_760_000_000 * SECOND


def grid(cells=(0, 100, -1, 0), width=2, height=2):
    return dict(frame='robot_a/map', width=width, height=height, resolution=0.05, origin=(-1.0, -2.0, 0.0),
                data=list(cells))


def current_map(revision=0, at=T0):
    return dict(robot_id='robot_a', map_session_id=uid(5), map_revision=revision, generated_at_ns=at,
                received_at_ns=at)


def revision_text(fields, revision):
    return frozen_revision_text(map_session_id=uid(5), map_revision=revision,
                                occupancy_sha256=occupancy_sha256(**fields))


class Case(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.path, self.out = root / 'site_state.sqlite3', root / 'maps'
        self.ledger = site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)
        self.addCleanup(self.ledger.close)

    def archive(self):
        from rescue_operations.map_archive import MapArchive
        return MapArchive(self.ledger, out_dir=self.out, robot_id='robot_a')

    def maps(self):
        return sorted(site_state.read_map_snapshots(self.path)['maps'], key=lambda m: m['at_ns'])


class LiveMapTests(Case):
    def test_a_live_map_is_kept_in_the_run_folder_with_its_row(self):
        self.ledger.record_current_map(current_map(revision=3))
        archive = self.archive()
        self.assertTrue(archive.on_live_map(grid(), received_at_ns=T0))
        (row,) = self.maps()
        run = self.ledger.run_id
        image = Path(row['path'])
        self.assertEqual(self.out / run, image.parent)
        self.assertEqual(('robot_a', 3, T0), (row['robot_id'], row['revision'], row['at_ns']))
        pgm = grid_to_pgm(2, 2, grid()['data'])
        self.assertEqual(pgm, image.read_bytes())
        self.assertEqual(hashlib.sha256(pgm).hexdigest(), row['sha256'])
        yaml = image.with_suffix('.yaml').read_text()
        self.assertIn(f'image: {image.name}', yaml)
        self.assertIn('origin: [-1.0, -2.0, 0.0]', yaml)

    def test_live_maps_are_thinned_and_an_image_kept_once(self):
        from rescue_operations.map_archive import ARCHIVE_PERIOD_NS
        self.ledger.record_current_map(current_map())
        archive = self.archive()
        self.assertTrue(archive.on_live_map(grid(), received_at_ns=T0))
        self.assertFalse(archive.on_live_map(grid((100, 100, -1, 0)), received_at_ns=T0 + ARCHIVE_PERIOD_NS - 1))
        self.assertFalse(archive.on_live_map(grid(), received_at_ns=T0 + ARCHIVE_PERIOD_NS))  # same image
        self.assertTrue(archive.on_live_map(grid((100, 0, 0, 0)), received_at_ns=T0 + ARCHIVE_PERIOD_NS))
        self.assertEqual(2, len(self.maps()))

    def test_no_map_state_yet_keeps_nothing_and_says_so_once(self):
        archive = self.archive()
        self.assertFalse(archive.on_live_map(grid(), received_at_ns=T0))
        self.assertFalse(archive.on_live_map(grid(), received_at_ns=T0 + 10 * SECOND))
        self.assertEqual([], self.maps())
        errors = site_state.read_error_log(self.path)['errors']
        self.assertEqual(1, len(errors))
        self.assertIn('MAP_ARCHIVE_SKIPPED', errors[0]['detail'])

    def test_between_runs_nothing_is_kept(self):
        self.ledger.record_current_map(current_map())
        self.ledger.close_run(at_ns=T0)
        self.assertFalse(self.archive().on_live_map(grid(), received_at_ns=T0 + 1))
        self.assertFalse(self.out.exists() and any(self.out.rglob('*.pgm')))

    def test_a_new_run_keeps_the_same_image_again_in_its_own_folder(self):
        self.ledger.record_current_map(current_map())
        archive = self.archive()
        archive.on_live_map(grid(), received_at_ns=T0)
        old = self.ledger.close_run(at_ns=T0 + 1)
        new = self.ledger.open_run(at_ns=T0 + 2)
        self.ledger.record_current_map(current_map(at=T0 + 3))
        self.assertTrue(archive.on_live_map(grid(), received_at_ns=T0 + 4))
        self.assertEqual(1, len(site_state.read_map_snapshots_range(self.path, run_id=old)['maps']))
        (row,) = site_state.read_map_snapshots_range(self.path, run_id=new)['maps']
        self.assertEqual(self.out / new, Path(row['path']).parent)

    def test_a_map_the_writer_cannot_draw_is_refused_into_the_error_log(self):
        self.ledger.record_current_map(current_map())
        self.assertFalse(self.archive().on_live_map(grid(width=3), received_at_ns=T0))  # 4 cells for 3x2
        self.assertEqual([], self.maps())
        self.assertIn('MAP_ARCHIVE_SKIPPED', site_state.read_error_log(self.path)['errors'][0]['detail'])


class FrozenMapTests(Case):
    def test_a_frozen_map_is_kept_once_its_revision_names_it_in_either_order(self):
        archive = self.archive()
        first, second = grid(), grid((100, 100, 0, 0))
        self.assertFalse(archive.on_frozen_grid(first, received_at_ns=T0))
        self.assertTrue(archive.on_frozen_revision(revision_text(first, 1), received_at_ns=T0 + 1))
        self.assertFalse(archive.on_frozen_revision(revision_text(second, 2), received_at_ns=T0 + 2))
        self.assertTrue(archive.on_frozen_grid(second, received_at_ns=T0 + 3))
        rows = self.maps()
        self.assertEqual([1, 2], [row['revision'] for row in rows])
        self.assertEqual(['frozen-r1', 'frozen-r2'], [Path(row['path']).name.rsplit('-', 1)[0] for row in rows])

    def test_frozen_maps_are_kept_even_right_after_a_live_one(self):
        self.ledger.record_current_map(current_map())
        archive = self.archive()
        archive.on_live_map(grid(), received_at_ns=T0)
        archive.on_frozen_grid(grid(), received_at_ns=T0 + 1)
        self.assertTrue(archive.on_frozen_revision(revision_text(grid(), 1), received_at_ns=T0 + 2))
        self.assertEqual(2, len(self.maps()))

    def test_a_revision_for_another_grid_keeps_nothing(self):
        archive = self.archive()
        archive.on_frozen_grid(grid(), received_at_ns=T0)
        self.assertFalse(archive.on_frozen_revision(revision_text(grid((1, 1, 1, 1)), 1), received_at_ns=T0 + 1))
        self.assertEqual([], self.maps())

    def test_a_bad_revision_text_is_refused_into_the_error_log(self):
        self.assertFalse(self.archive().on_frozen_revision('{"format": "other"}', received_at_ns=T0))
        self.assertIn('MAP_ARCHIVE_SKIPPED', site_state.read_error_log(self.path)['errors'][0]['detail'])


class NodeParameterTests(unittest.TestCase):
    def params(self, **changes):
        from test_operations_ros_node import params
        return params(**({'map_archive_topic': '/robot6/map', 'map_archive_dir': '/var/lib/rescue/maps',
                          'frozen_map_topic': '/rescue/map/frozen',
                          'frozen_revision_topic': '/rescue/map/frozen_revision'} | changes))

    def test_archive_is_on_with_topic_and_dir_and_off_with_neither(self):
        from rescue_operations.ros_node import node_config
        self.assertEqual(('/robot6/map', '/var/lib/rescue/maps'),
                         (node_config(self.params())['map_archive_topic'],
                          node_config(self.params())['map_archive_dir']))
        off = node_config(self.params(map_archive_topic='', map_archive_dir=''))
        self.assertEqual('', off['map_archive_dir'])

    def test_half_or_relative_settings_are_refused(self):
        from rescue_operations.ros_node import node_config
        from rescue_operations.schema import OperationsError
        for change in (dict(map_archive_topic=''), dict(map_archive_dir=''), dict(map_archive_dir='maps'),
                       dict(map_archive_topic='robot6/map')):
            with self.subTest(change=change), self.assertRaises(OperationsError):
                node_config(self.params(**change))

    def test_launch_gives_the_archive_the_console_map_folder_and_the_coordinator_s_frozen_topics(self):
        from rescue_coordination import ros_node as coordinator
        from test_bringup_operations_launch import build
        params = build(map_topic='/robot6/map', map_dir='/var/lib/rescue/maps')
        self.assertEqual(('/robot6/map', '/var/lib/rescue/maps', coordinator.FROZEN_MAP_TOPIC,
                          coordinator.FROZEN_REVISION_TOPIC),
                         (params['map_archive_topic'], params['map_archive_dir'], params['frozen_map_topic'],
                          params['frozen_revision_topic']))
        off = build(map_topic='', map_dir='')
        self.assertEqual(('', ''), (off['map_archive_topic'], off['map_archive_dir']))


if __name__ == '__main__':
    unittest.main()
