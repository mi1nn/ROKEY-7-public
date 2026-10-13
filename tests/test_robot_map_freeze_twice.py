"""The TB-6 SLAM map is frozen twice per run (OPS-154/159), no ROS.

Scenario 2026-10-13 night: 1st freeze (R0+1) when the doll is found: TB-5 goes on that map while
TB-6 keeps searching with SLAM, so no TB-6 pose is needed or published. 2nd freeze (R0+2) when
the search is done: TB-6 stops, SLAM -> AMCL from the frozen pose (fresh SLAM pose required, as
before). A third request is refused. Each freeze is hashed, saved and published with its
revision (rescue_contracts.occupancy.frozen_revision_text). A restarted node reloads every
freeze of this run from output_dir (no record_path) and republishes them in order.
"""
import json
from pathlib import Path
import tempfile
import unittest

from rescue_contracts.occupancy import parse_frozen_revision
from rescue_robot.map_freeze import FreezeError, MapFreeze, load_records, revision_text

from test_robot_map_freeze import NOW, SESSION, grid, pose, sha

SECOND = grid(data=[0] * 10 + [100, 100])   # the map grew while TB-6 kept searching


class FreezeCase(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.now = NOW
        self.f = self.started()

    def make(self):
        return MapFreeze(map_frame='map', map_session_id=SESSION, map_revision=4, pose_max_age_ns=2_000_000_000,
                         output_dir=Path(self.dir.name), wall_clock_ns=lambda: self.now, git_sha='abc123')

    def started(self):
        f = self.make()
        f.on_pose(pose())   # TB-6 start pose
        return f


class TwoFreezes(FreezeCase):
    def test_first_freeze_is_r1_without_a_tb6_pose(self):
        self.f.on_map(grid())
        self.now += 10_000_000_000             # TB-6 is moving on: no fresh pose, none needed
        first = self.f.freeze()
        self.assertEqual((5, False, None), (first['map_revision'], first['final'], first['pose']))
        self.assertEqual(sha(grid()), first['occupancy_sha256'])
        self.assertEqual(pose(), first['start_pose'])

    def test_second_freeze_is_r2_with_a_fresh_tb6_pose_then_no_more(self):
        self.f.on_map(grid())
        self.f.freeze()
        self.f.on_map(SECOND)
        self.now += 10_000_000_000
        with self.assertRaises(FreezeError):   # TB-6 changes to AMCL from this pose: it must be fresh
            self.f.freeze()
        self.assertEqual(1, len(self.f.records))
        self.f.on_pose(pose(stamp_ns=self.now - 100_000_000, position=(3.0, 1.0, 0.0)))
        second = self.f.freeze()
        self.assertEqual((6, True, sha(SECOND)), (second['map_revision'], second['final'], second['occupancy_sha256']))
        self.assertEqual(3.0, second['pose']['position'][0])
        with self.assertRaisesRegex(FreezeError, 'two'):
            self.f.freeze()
        self.assertEqual([5, 6], [r['map_revision'] for r in self.f.records])
        self.assertEqual(2, len(list(Path(self.dir.name).glob('frozen-*.json'))))

    def test_the_same_grid_twice_is_two_revisions(self):
        self.f.on_map(grid())
        self.f.freeze()
        self.f.on_pose(pose())
        second = self.f.freeze()
        self.assertEqual((sha(grid()), 6), (second['occupancy_sha256'], second['map_revision']))

    def test_revision_text_names_the_grid_hash_and_revision(self):
        self.f.on_map(grid())
        record = self.f.freeze()
        self.assertEqual(dict(map_session_id=SESSION, map_revision=5, occupancy_sha256=sha(grid())),
                         parse_frozen_revision(revision_text(record)))


class Restart(FreezeCase):
    def load(self):
        return load_records(Path(self.dir.name), map_frame='map', map_session_id=SESSION, map_revision=4)

    def test_no_freeze_yet(self):
        self.assertEqual([], self.load())

    def test_after_the_first_the_second_is_still_possible(self):
        self.f.on_map(grid())
        first = self.f.freeze()
        records = self.load()
        self.assertEqual([first['occupancy_sha256']], [r['occupancy_sha256'] for r in records])
        f = self.make()
        f.restore(records)
        f.on_map(SECOND)
        f.on_pose(pose(stamp_ns=self.now - 100_000_000))
        self.assertEqual(6, f.freeze()['map_revision'])

    def test_after_both_nothing_more(self):
        self.f.on_map(grid())
        self.f.freeze()
        self.f.on_map(SECOND)
        self.f.on_pose(pose())
        self.f.freeze()
        records = self.load()
        self.assertEqual([5, 6], [r['map_revision'] for r in records])
        self.assertEqual([False, True], [r['final'] for r in records])
        f = self.make()
        f.restore(records)
        with self.assertRaises(FreezeError):
            f.freeze()

    def test_records_that_do_not_make_one_run_are_refused(self):
        self.f.on_map(grid())
        path = Path(self.f.freeze()['path'])
        copy = path.with_name('frozen-copy-1.json')
        copy.write_text(path.read_text())                  # two R1 records
        with self.assertRaises(FreezeError):
            self.load()
        copy.unlink()
        with self.assertRaises(FreezeError):               # another run's R0
            load_records(Path(self.dir.name), map_frame='map', map_session_id=SESSION, map_revision=5)
        data = json.loads(path.read_text())
        data['map_revision'] = 6                           # an R2 without an R1
        path.write_text(json.dumps(data))
        with self.assertRaises(FreezeError):
            self.load()


if __name__ == '__main__':
    unittest.main()
