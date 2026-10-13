"""Freezing the TB-6 SLAM map (map_freeze on compute_a, OPS-113/117/154), no ROS.

On /rescue/map/freeze the latest TB-6 SLAM map is captured, hashed and written to disk; the node
then publishes the exact grid on /rescue/map/frozen with its revision. Twice per run
(test_robot_map_freeze_twice): the first (R0+1) needs no TB-6 pose; the second (R0+2) captures
the fresh TB-6 SLAM pose for /rescue/map/frozen_pose/robot_a (AMCL start for TB-6). A
restarted node reloads the saved records and republishes the identical grids (same hash, so
the robots do not see a different frozen map). Nothing is frozen without a valid map in the
map frame.
"""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from rescue_robot.live_sources import Grid
from rescue_robot.map_freeze import FreezeError, MapFreeze, load_record, load_records

SESSION = '22222222-2222-4222-8222-222222222222'
NOW = 50_000_000_000
YAW_Q = (0., 0., math.sin(0.15), math.cos(0.15))   # origin turned: the stored quaternion must survive


def grid(**changes):
    g = dict(frame='map', width=4, height=3, resolution=0.05, position=(-1.0, -0.5, 0.0), orientation=YAW_Q,
             data=[0] * 11 + [100])
    g.update(changes)
    return g


def pose(**changes):
    p = dict(frame='map', stamp_ns=NOW - 100_000_000, position=(0.2, 0.1, 0.0), orientation=(0., 0., 0., 1.),
             covariance=[0.0] * 36)
    p.update(changes)
    return p


def sha(g):
    """The hash the robots compute from the frozen OccupancyGrid (live_sources.Grid)."""
    (px, py, pz), (qx, qy, qz, qw) = g['position'], g['orientation']
    message = NS(header=NS(frame_id=g['frame']), info=NS(width=g['width'], height=g['height'],
                 resolution=g['resolution'], origin=NS(position=NS(x=px, y=py, z=pz),
                                                       orientation=NS(x=qx, y=qy, z=qz, w=qw))), data=g['data'])
    return Grid(message).sha256


class Freeze(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.now = NOW
        self.f = self.make()

    def make(self):
        return MapFreeze(map_frame='map', map_session_id=SESSION, map_revision=0, pose_max_age_ns=2_000_000_000,
                         output_dir=Path(self.dir.name), wall_clock_ns=lambda: self.now, git_sha='abc123')

    def test_freeze_captures_hashes_and_saves_the_latest_map(self):
        self.f.on_map(grid(data=[0] * 12))
        self.f.on_map(grid())                  # the latest map counts
        self.f.on_pose(pose())
        record = self.f.freeze()
        self.assertEqual(sha(grid()), record['occupancy_sha256'])
        self.assertEqual((SESSION, 1), (record['map_session_id'], record['map_revision']))
        self.assertEqual(grid(), record['grid'])
        self.assertIsNone(record['pose'])      # first freeze: TB-6 keeps SLAM, no AMCL start
        self.assertEqual('abc123', record['git_sha'])
        saved = Path(record['path'])
        self.assertEqual(Path(self.dir.name), saved.parent)
        self.assertEqual(record['occupancy_sha256'], json.loads(saved.read_text())['occupancy_sha256'])
        self.assertTrue(saved.with_suffix('.pgm').exists() and saved.with_suffix('.yaml').exists())
        self.assertEqual([record], self.f.records)

    def test_nothing_is_frozen_without_a_valid_map_in_the_map_frame(self):
        for g in (None, grid(frame='odom'), grid(data=[0] * 11), grid(orientation=(0., 0.1, 0., 1.))):
            with self.subTest(g=g):
                f = self.make()
                if g is not None:
                    f.on_map(g)
                f.on_pose(pose())
                with self.assertRaises(FreezeError):
                    f.freeze()
                self.assertEqual([], f.records)
                # only the TB-6 start pose (OPS-141) may be there; no freeze file
                self.assertEqual([], list(Path(self.dir.name).glob('frozen-*')))

    def test_no_second_freeze_without_a_fresh_slam_pose_on_the_map(self):
        for p in (pose(stamp_ns=NOW - 3_000_000_000), pose(frame='odom'), pose(stamp_ns=NOW + 3_000_000_000),
                  pose(orientation=(0., 0., 0.5, 0.5))):
            with self.subTest(p=p), tempfile.TemporaryDirectory() as d:
                f = MapFreeze(map_frame='map', map_session_id=SESSION, map_revision=0, pose_max_age_ns=2_000_000_000,
                              output_dir=Path(d), wall_clock_ns=lambda: NOW, git_sha='abc123')
                f.on_pose(pose())
                f.on_map(grid())
                f.freeze()
                f.on_pose(p)
                with self.assertRaises(FreezeError):
                    f.freeze()
                self.assertEqual([1], [r['map_revision'] for r in f.records])

    def test_a_restarted_node_republishes_the_identical_grids(self):
        self.f.on_map(grid()); self.f.on_pose(pose())
        self.f.freeze()
        record = self.f.freeze()
        first, again = load_records(Path(self.dir.name), map_frame='map', map_session_id=SESSION, map_revision=0)
        self.assertEqual(record['grid'], again['grid'])
        self.assertEqual(record['occupancy_sha256'], sha(again['grid']))
        self.assertEqual(record['pose'], again['pose'])
        f = self.make()
        f.restore([first, again])
        with self.assertRaises(FreezeError):
            f.freeze()

    def test_a_record_of_another_run_or_edited_by_hand_is_refused(self):
        self.f.on_map(grid()); self.f.on_pose(pose())
        path = Path(self.f.freeze()['path'])
        with self.assertRaises(FreezeError):
            load_record(path, map_frame='map', map_session_id='33333333-3333-4333-8333-333333333333', map_revision=0)
        data = json.loads(path.read_text())
        data['grid']['data'][0] = 100
        path.write_text(json.dumps(data))
        with self.assertRaises(FreezeError):
            load_record(path, map_frame='map', map_session_id=SESSION, map_revision=0)


if __name__ == '__main__':
    unittest.main()
