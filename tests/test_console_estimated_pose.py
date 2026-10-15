"""1차 지도 고정(R1) 전 TB-5 추정 위치(재범 결정 2026-10-15 17:15, 작전 설계): TB-6 출발 자세 + 현장 파일
robots.<id>.initial_pose_xy_yaw 를 compose_planar 로 더한 한 점을 「추정」으로 그린다. 화면 전용이고 odom 누적은 없다.
값은 저장소 현장 파일에 아직 없어서 시험은 그 키를 넣은 사본으로 한다(작전 2026-10-15)."""
import json
import math
import unittest
from types import SimpleNamespace

from rescue_console.app import current_view
from rescue_console.estimate import EstimateLog, load_start_offsets
from rescue_console.mapimage import map_overlay
from rescue_console.render import render_fragment
from rescue_console.sources import load_ledger
from rescue_console.view import ConsoleError, build_view
from test_console_robot_watch import NAMES, NOW, watch
from test_console_view import SECOND, LedgerFixture, robot_status

START = {'x': -0.116, 'y': -0.057, 'yaw': 2.950, 'frame_id': 'map'}   # 이번 판의 TB-6 출발 자세(작전 2026-10-15)
OFFSETS = {'robot_b': (0.0, -1.86, 0.0)}                            # 현장 파일 robots.robot_b.initial_pose_xy_yaw
META = {'origin': [-1.0, -1.0, 0.0], 'resolution': 0.05, 'width_px': 100, 'height_px': 100, 'source': 'live',
        'url': 'map.png', 'frame_id': 'map'}


def watch_with_start(start=START, **kwargs):
    data = watch(**kwargs)
    data['robots']['robot_a']['start_pose'] = start
    return data


class EstimateTests(LedgerFixture):
    def view(self, data, *, offsets=OFFSETS, status_robots=(), written_ok=True):
        status = dict(self.status, robot_status={'schema_version': '1.1', 'robots': list(status_robots)})
        return build_view(load_ledger(self.db), status, built_at_ns=NOW, robot_names=NAMES,
                          robot_watch={'stale_s': 5.0, 'data': data, 'problem': None, 'map_robot': 'robot_a',
                                       'start_offsets': offsets})

    def tb5(self, view):
        return {r['robot_id']: r for r in view['robots']}['robot_b']

    def test_start_pose_plus_offset_is_drawn_as_estimated(self):
        view = self.view(watch_with_start())
        pose = self.tb5(view)['pose']
        self.assertEqual((0.238, 1.769, 2.950), (round(pose['x'], 3), round(pose['y'], 3), round(pose['yaw'], 3)))
        self.assertEqual(('map', True), (pose['frame_id'], pose['estimated']))
        layer = map_overlay(META, view['robots'], [])
        drawn = {r['id']: r for r in layer['robots']}
        self.assertTrue(drawn['robot_b']['estimated'])
        self.assertFalse(drawn['robot_a']['estimated'])

    def test_nothing_without_the_start_pose_or_the_offset(self):
        for data, offsets in ((watch(), OFFSETS), (watch_with_start(), {}), (watch_with_start(), None),
                              (watch_with_start(dict(START, frame_id='robot6/odom')), OFFSETS)):
            with self.subTest(offsets=offsets, start=data['robots']['robot_a'].get('start_pose')):
                self.assertIsNone(self.tb5(self.view(data, offsets=offsets))['pose'])

    def test_a_dead_watch_file_draws_no_estimate(self):
        self.assertIsNone(self.tb5(self.view(watch_with_start(written=NOW - 10 * SECOND)))['pose'])

    def test_a_real_map_pose_wins(self):
        real = dict(robot_status('robot_b'), pose={'x': 1.0, 'y': 2.0, 'yaw': 0.5, 'frame_id': 'map'})
        pose = self.tb5(self.view(watch_with_start(), status_robots=[real]))['pose']
        self.assertEqual({'x': 1.0, 'y': 2.0, 'yaw': 0.5, 'frame_id': 'map'}, pose)

    def test_a_pose_in_its_own_map_is_not_real_on_this_map(self):
        data = watch_with_start()
        data['robots']['robot_b'].update(pose={'x': -0.13, 'y': 0.0, 'yaw': -3.03, 'frame_id': 'map'},
                                         pose_seen_ns=NOW - SECOND)
        self.assertTrue(self.tb5(self.view(data))['pose']['estimated'])

    def test_the_map_robot_is_never_estimated(self):
        data = watch_with_start(pose=False)
        view = self.view(data, offsets={'robot_a': (1.0, 0.0, 0.0), **OFFSETS})
        self.assertIsNone({r['robot_id']: r for r in view['robots']}['robot_a']['pose'])


class FieldFileTests(LedgerFixture):
    def write(self, robots):
        path = self.dir / 'search_area.field.json'
        path.write_text(json.dumps({'robots': robots}), encoding='utf-8')
        return path

    def test_reads_only_robots_with_the_key(self):
        path = self.write({'robot_a': {'task': {}}, 'robot_b': {'initial_pose_xy_yaw': [0.0, -1.86, 0.0]}})
        self.assertEqual({'robot_b': (0.0, -1.86, 0.0)}, load_start_offsets(path))

    def test_bad_values_stop_the_start(self):
        for value in ([0.0, 1.0], [0.0, 'x', 0.0], [0.0, math.nan, 0.0], [True, 0.0, 0.0], '0,0,0'):
            with self.subTest(value=value), self.assertRaises(ConsoleError):
                load_start_offsets(self.write({'robot_b': {'initial_pose_xy_yaw': value}}))
        with self.assertRaises(ConsoleError):
            load_start_offsets(self.dir / 'missing.json')

    def test_current_view_uses_the_offsets(self):
        path = self.dir / 'robot_watch.json'
        path.write_text(json.dumps(watch_with_start()), encoding='utf-8')
        status_path = self.dir / 'status.json'
        status_path.write_text(json.dumps(dict(self.status, robot_status={'schema_version': '1.1', 'robots': []})),
                               encoding='utf-8')
        view = current_view(self.db, status_path, clock=lambda: NOW, robot_names=NAMES,
                            map_image=SimpleNamespace(meta=META), robot_watch_path=path, robot_watch_stale_s=5.0,
                            robot_watch_map_robot='robot_a', start_offsets=OFFSETS)
        self.assertEqual(['robot_a', 'robot_b'], sorted(r['id'] for r in view['map']['robots']))
        self.assertNotIn('TB-5: 위치 미확정', ' '.join(view['warnings']))
        fragment = render_fragment(view)
        self.assertRegex(fragment, r'<g class="robot [a-z]+ estimated">')   # 점선·흐린 색
        self.assertIn('TB-5 추정</text>', fragment)
        self.assertIn('추정(TB-6 출발 자세 + 현장 파일 시작 값)', fragment)   # 카드


class EstimateLogTests(unittest.TestCase):
    def test_says_when_the_estimate_turns_real(self):
        log = EstimateLog()
        estimated = [{'robot_id': 'robot_b', 'name': 'TB-5', 'pose': {'x': 0, 'y': 0, 'frame_id': 'map', 'estimated': True}}]
        real = [{'robot_id': 'robot_b', 'name': 'TB-5', 'pose': {'x': 0, 'y': 0, 'frame_id': 'map'}}]
        self.assertEqual(['TB-5 지도 위치: 없음 -> 추정'], log.changes(estimated))
        self.assertEqual([], log.changes(estimated))
        self.assertEqual(['TB-5 지도 위치: 추정 -> 실제'], log.changes(real))
        self.assertEqual([], log.changes(real))


class StartPoseTopicTests(unittest.TestCase):
    def test_robot_watch_listens_where_the_coordinator_does(self):
        from rescue_coordination.ros_node import START_POSE_TOPIC
        from rescue_operations.robot_watch import START_POSE_PREFIX
        self.assertEqual(START_POSE_TOPIC, START_POSE_PREFIX)


if __name__ == '__main__':
    unittest.main()
