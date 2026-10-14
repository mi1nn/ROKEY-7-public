"""--robot-watch: 프로젝트 상태(/rescue/robots/...)가 없어도 로봇 기본 토픽의 직접 관측을 로봇 카드와 지도에 보인다.

직접 관측은 구독만 하는 운영 쪽 노드(robot_watch)가 쓴 파일이다(재범 허용 2026-10-13). 프로젝트 상태가 있으면
그 값이 먼저이고, 직접 관측은 덧붙는 줄이다. 낡은 관측의 위치는 그리지 않는다.
"""
import json
import unittest

from rescue_console.app import current_view
from rescue_console.mapimage import map_overlay
from rescue_console.render import render_fragment
from rescue_console.sources import load_robot_watch
from rescue_console.view import build_view
from rescue_console.sources import load_ledger
from test_console_view import LedgerFixture, SECOND, T0

NAMES = {'robot_a': 'TB-6', 'robot_b': 'TB-5'}
NOW = T0 + 30 * SECOND


def watch(*, seen=NOW - SECOND, written=NOW - SECOND, pose=True):
    return {'schema': 'robot_watch/1', 'written_at_ns': written, 'robots': {
        'robot_a': {'namespace': '/robot6', 'last_seen_ns': seen, 'battery_percent': 64.0, 'docked': False,
                    'pose': {'x': 0.5, 'y': -1.0, 'yaw': 0.0, 'frame_id': 'map'} if pose else None,
                    'pose_seen_ns': seen if pose else None},
        'robot_b': {'namespace': '/robot5', 'last_seen_ns': seen, 'battery_percent': 81.0, 'docked': True,
                    'pose': None, 'pose_seen_ns': None}}}


class RobotWatchTests(LedgerFixture):
    def view(self, data, *, with_status=False, problem=None, map_robot='robot_a'):
        status = dict(self.status)
        if not with_status:
            status['robot_status'] = {'schema_version': '1.1', 'robots': []}
        return build_view(load_ledger(self.db), status, built_at_ns=NOW, robot_names=NAMES,
                          robot_watch={'stale_s': 5.0, 'data': data, 'problem': problem, 'map_robot': map_robot})

    def test_only_the_map_robot_direct_pose_is_drawn(self):
        # 현장 2026-10-14 18:57: 각자 자기 SLAM 의 'map' 이라 TB-5 가 TB-6 지도 원점에 겹쳐 그려졌다
        data = watch()
        data['robots']['robot_a']['pose'] = {'x': -0.0, 'y': 0.0, 'yaw': 0.0, 'frame_id': 'map'}
        data['robots']['robot_b'].update(pose={'x': -0.13, 'y': 0.0, 'yaw': -3.03, 'frame_id': 'map'},
                                         pose_seen_ns=NOW - SECOND)
        view = self.view(data)
        rows = {r['robot_id']: r for r in view['robots']}
        self.assertEqual('/robot5 map', rows['robot_b']['pose']['frame_id'])   # 옮기지 않고 frame 만 자기 것으로
        self.assertEqual((-0.13, 0.0), (rows['robot_b']['pose']['x'], rows['robot_b']['pose']['y']))
        meta = {'origin': [-1.0, -1.0, 0.0], 'resolution': 0.05, 'width_px': 100, 'height_px': 100, 'source': 'live'}
        layer = map_overlay(meta, view['robots'], [])
        self.assertEqual(['robot_a'], [r['id'] for r in layer['robots']])
        self.assertEqual(['robot_b'], [w['id'] for w in layer['frame_mismatch']])
        self.assertIn('자기 지도 기준, 이 지도와 다름', render_fragment(view))

    def test_without_project_status_the_watch_fills_the_card(self):
        view = self.view(watch())
        rows = {r['robot_id']: r for r in view['robots']}
        self.assertEqual((81.0, True), (rows['robot_b']['watch']['battery_percent'], rows['robot_b']['watch']['docked']))
        self.assertEqual({'x': 0.5, 'y': -1.0, 'yaw': 0.0, 'frame_id': 'map'}, rows['robot_a']['pose'])
        self.assertIsNone(rows['robot_b']['pose'])               # TB-5: 지도 위치 없음, 지어내지 않음
        html = render_fragment(view)
        self.assertIn('직접 관측', html)
        self.assertIn('81%', html)
        self.assertIn('도킹', html)
        self.assertIn('프로젝트 상태 없음', html)

    def test_project_status_comes_first(self):
        view = self.view(watch(), with_status=True)
        robot_a = {r['robot_id']: r for r in view['robots']}['robot_a']
        self.assertNotEqual({'x': 0.5, 'y': -1.0, 'yaw': 0.0, 'frame_id': 'map'}, robot_a['pose'])
        self.assertIsNotNone(robot_a['watch'])

    def test_stale_watch_is_said_and_its_pose_not_drawn(self):
        view = self.view(watch(seen=NOW - 10 * SECOND))
        robot_a = {r['robot_id']: r for r in view['robots']}['robot_a']
        self.assertTrue(robot_a['watch']['stale'])
        self.assertIsNone(robot_a['pose'])
        self.assertIn('직접 관측 낡음', render_fragment(view))

    def test_a_dead_watcher_makes_every_robot_stale(self):
        view = self.view(watch(written=NOW - 10 * SECOND))
        self.assertTrue(all(r['watch']['stale'] for r in view['robots']))
        self.assertIn('로봇 직접 관측 파일이 10초 동안 갱신 안 됨', ' '.join(view['warnings']))

    def test_a_watched_robot_outside_the_deployment_is_said_not_dropped(self):
        data = watch()
        data['robots']['robot_z'] = dict(data['robots']['robot_b'], namespace='/robot9')
        view = self.view(data)
        self.assertIn("배치 밖 로봇 직접 관측 무시: 'robot_z'", ' '.join(view['warnings']))   # REV-251-03

    def test_missing_or_broken_file_is_a_warning_not_a_crash(self):
        view = self.view(None, problem='로봇 직접 관측 파일 없음')
        self.assertIn('로봇 직접 관측 파일 없음', ' '.join(view['warnings']))
        self.assertTrue(all(r['watch'] is None for r in view['robots']))

    def test_off_by_default(self):
        view = build_view(load_ledger(self.db), self.status, built_at_ns=NOW, robot_names=NAMES)
        self.assertTrue(all(r['watch'] is None for r in view['robots']))

    def test_load_robot_watch(self):
        path = self.dir / 'robot_watch.json'
        self.assertEqual((None, '로봇 직접 관측 파일 없음'), load_robot_watch(path))
        path.write_text('{not json', encoding='utf-8')
        self.assertIsNone(load_robot_watch(path)[0])
        self.assertIn('읽기 실패', load_robot_watch(path)[1])
        path.write_text(json.dumps(watch()), encoding='utf-8')
        self.assertEqual((watch(), None), load_robot_watch(path))

    def test_current_view_reads_the_file(self):
        path = self.dir / 'robot_watch.json'
        path.write_text(json.dumps(watch()), encoding='utf-8')
        status_path = self.dir / 'status.json'
        status = dict(self.status, robot_status={'schema_version': '1.1', 'robots': []})
        status_path.write_text(json.dumps(status), encoding='utf-8')
        view = current_view(self.db, status_path, clock=lambda: NOW, robot_names=NAMES,
                            robot_watch_path=path, robot_watch_stale_s=5.0, robot_watch_map_robot='robot_a')
        self.assertEqual(81.0, {r['robot_id']: r for r in view['robots']}['robot_b']['watch']['battery_percent'])


if __name__ == '__main__':
    unittest.main()
