"""로봇 직접 관측(robot_watch): /robotN 기본 토픽을 구독만 해서 관제 웹이 읽는 파일로 쓴다(재범 허용 2026-10-13)."""
import json
import math
from pathlib import Path
import tempfile
import unittest

from rescue_operations.robot_watch import WATCH_SCHEMA, battery_percent, pose_from_transform, write_watch


class BatteryPercent(unittest.TestCase):
    def test_fraction_to_percent_and_unknown_is_none(self):
        self.assertEqual(81.0, battery_percent(0.81))
        for unknown in (float('nan'), -1.0, 1.5):   # BatteryState: NaN = 모름; 범위 밖도 지어내지 않음
            with self.subTest(unknown=unknown):
                self.assertIsNone(battery_percent(unknown))


class PoseFromTransform(unittest.TestCase):
    def test_xy_and_yaw(self):
        pose = pose_from_transform(1.5, -2.0, (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)), 'map')
        self.assertEqual((1.5, -2.0, 'map'), (pose['x'], pose['y'], pose['frame_id']))
        self.assertAlmostEqual(math.pi / 2, pose['yaw'], places=6)


class WriteWatch(unittest.TestCase):
    def test_written_whole_and_readable(self):
        path = Path(tempfile.mkdtemp()) / 'robot_watch.json'
        robots = {'robot_b': {'namespace': '/robot5', 'last_seen_ns': 5, 'battery_percent': 81.0, 'docked': True,
                              'pose': None, 'pose_seen_ns': None}}
        write_watch(path, written_at_ns=7, robots=robots)
        data = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual({'schema': WATCH_SCHEMA, 'written_at_ns': 7, 'robots': robots}, data)
        self.assertEqual([], list(path.parent.glob('*.tmp')))


if __name__ == '__main__':
    unittest.main()
