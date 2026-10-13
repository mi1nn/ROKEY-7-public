"""Coordinates measured in advance are relative to the TB-6 start pose (OPS-141), no ROS.

Decision 2026-10-13 evening: the SLAM map origin is the Create3 odom origin, not necessarily
the floor mark TB-6 starts from. So every coordinate measured in advance (patrol points, TB-6
docking approach, TB-5 AMCL start) is written relative to the TB-6 start pose; at run time
the first valid TB-6 SLAM pose after the start (map frame) turns it into map coordinates.
map_freeze (compute_a, up for the whole run) records that pose once per run, keeps it in
output_dir (a restart republishes it, never re-captures), and publishes it latched on
/rescue/map/start_pose/robot_a. No start pose = nothing published (the others wait).
"""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from rescue_contracts.occupancy import compose_planar, relative_planar
from rescue_robot.map_freeze import START_POSE_TOPIC, FreezeError, MapFreeze

SESSION = '22222222-2222-4222-8222-222222222222'
NOW = 50_000_000_000


def pose(x=0.5, y=-0.2, yaw=0.3, frame='map', stamp_ns=NOW - 100_000_000):
    return dict(frame=frame, stamp_ns=stamp_ns, position=(x, y, 0.0),
                orientation=(0., 0., math.sin(yaw / 2), math.cos(yaw / 2)), covariance=[0.0] * 36)


class PlanarTransform(unittest.TestCase):
    def test_relative_then_compose_is_the_same_pose(self):
        start, p = (0.5, -0.2, 0.3), (2.0, 1.0, -1.2)
        rel = relative_planar(start, p)
        back = compose_planar(start, rel)
        for a, b in zip(p, back):
            self.assertAlmostEqual(a, b)

    def test_known_values(self):
        # start turned 90 degrees at (1, 0): one metre ahead of it is (1, 1), facing +y
        x, y, yaw = compose_planar((1.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(1.0, x); self.assertAlmostEqual(1.0, y); self.assertAlmostEqual(math.pi / 2, yaw)
        self.assertEqual((0.0, 0.0, 0.0), compose_planar((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))

    def test_yaw_wraps_to_pi(self):
        _, _, yaw = compose_planar((0.0, 0.0, 3.0), (0.0, 0.0, 1.0))
        self.assertTrue(-math.pi <= yaw <= math.pi)

    def test_bad_values_are_refused(self):
        for bad in ((0.0, 0.0), (0.0, math.nan, 0.0), (True, 0.0, 0.0)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                compose_planar(bad, (0.0, 0.0, 0.0))


class StartPose(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)

    def make(self):
        return MapFreeze(map_frame='map', map_session_id=SESSION, map_revision=0, pose_max_age_ns=2_000_000_000,
                         output_dir=Path(self.dir.name), wall_clock_ns=lambda: NOW, git_sha='abc')

    def test_the_first_valid_slam_pose_is_the_start_once(self):
        f = self.make()
        self.assertIsNone(f.start_pose)
        self.assertIsNone(f.on_pose(pose(frame='odom')))         # not on the map: not a start
        captured = f.on_pose(pose())
        self.assertEqual(pose(), captured)
        self.assertIsNone(f.on_pose(pose(x=3.0)))                # later poses never move it
        self.assertEqual(pose(), f.start_pose)
        saved = json.loads((Path(self.dir.name) / 'start_pose.json').read_text())
        self.assertEqual((SESSION, 0.5), (saved['map_session_id'], saved['pose']['position'][0]))

    def test_a_restarted_node_keeps_the_saved_start(self):
        self.make().on_pose(pose())
        again = self.make()
        self.assertEqual(pose(), again.start_pose)
        self.assertIsNone(again.on_pose(pose(x=3.0)))
        self.assertEqual(pose(), again.start_pose)

    def test_a_start_of_another_run_is_refused(self):
        self.make().on_pose(pose())
        path = Path(self.dir.name) / 'start_pose.json'
        data = json.loads(path.read_text())
        data['map_session_id'] = '33333333-3333-4333-8333-333333333333'
        path.write_text(json.dumps(data))
        with self.assertRaises(FreezeError):
            self.make()

    def test_the_freeze_record_carries_the_start(self):
        f = self.make()
        f.on_pose(pose())
        f.on_map(dict(frame='map', width=2, height=1, resolution=0.05, position=(0.0, 0.0, 0.0),
                      orientation=(0., 0., 0., 1.), data=[0, 0]))
        f.on_pose(pose(x=1.0))
        f.freeze()
        record = f.freeze()                      # both freezes carry it; the second also the TB-6 pose
        self.assertEqual(pose(), record['start_pose'])
        self.assertEqual(1.0, record['pose']['position'][0])

    def test_topic(self):
        self.assertEqual('/rescue/map/start_pose/robot_a', START_POSE_TOPIC)


class Tb5StartFromTheTb6Start(unittest.TestCase):
    def test_tb5_start_is_read_relative_to_the_tb6_start(self):
        from test_bringup_field_launch import FieldCase  # noqa: F401  (fixtures)
        from test_bringup_map_lifecycle import LocalizationLaunch
        case = LocalizationLaunch('test_tb5_starts_amcl_from_the_measured_start_on_the_frozen_map')
        case.setUp()
        self.addCleanup(case.tmp.cleanup)
        v = case.values('robot_b')
        self.assertEqual('/rescue/map/start_pose/robot_a', v['start_pose_topic'])
        self.assertEqual([0.4, -0.3, 0.1], v['initial_pose_xy_yaw'])        # as written: relative to TB-6 start
        a = case.values('robot_a')
        self.assertIsNone(a['start_pose_topic'])                            # TB-6 starts from its frozen pose


class CaptureRelative(unittest.TestCase):
    def tool(self):
        import runpy
        import sys
        repo = Path(__file__).resolve().parents[1]
        script = repo / 'scripts' / 'field' / 'capture_initial_pose.py'
        if not script.exists():
            self.skipTest('installed-wheel checkout has no scripts/')
        return runpy.run_path(str(script))

    def message(self, x, y, yaw, frame='map'):
        return NS(header=NS(frame_id=frame), pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0),
                  orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))), covariance=[0.0] * 36))

    def test_prints_the_pose_relative_to_the_tb6_start(self):
        snippet = self.tool()['snippet']
        start = self.message(1.0, 0.0, math.pi / 2)
        text = snippet(self.message(1.0, 1.0, math.pi / 2), start=start, map_frame='map')
        value = json.loads('{' + text + '}')['initial_pose_xy_yaw']
        for a, b in zip([1.0, 0.0, 0.0], value):
            self.assertAlmostEqual(a, b, places=3)

    def test_start_must_be_on_the_map_too(self):
        tool = self.tool()
        with self.assertRaises(tool['CaptureError']):
            tool['snippet'](self.message(1.0, 1.0, 0.0), start=self.message(0, 0, 0, frame='odom'), map_frame='map')


if __name__ == '__main__':
    unittest.main()
