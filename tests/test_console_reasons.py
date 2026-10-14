"""SystemState.reason in plain Korean on the console (2026-10-13).

The coordinator puts the support robot's exit state and refusal reasons in SystemState.reason
(operations stores it in the mission row). The console names the support robot with --robot-name
(#168), shows the code next to the words, keeps unknown codes as they are, and marks a failed exit red.
"""
import re
import unittest

from test_console_commands import dispatch, mission_state
from test_console_view import LedgerFixture, SECOND, T0, uid
from rescue_console.render import render_fragment, render_page
from rescue_console.view import build_view, mission_reason
from rescue_console.sources import load_ledger

NAMES = {'robot_a': 'TB-6', 'robot_b': 'TB-5'}


class ReasonWordTests(unittest.TestCase):
    def test_support_exit_states(self):
        cases = {
            'SUPPORT_EXIT_IN_12S': ('TB-5 출구 출발까지 12초', 'info'),
            'SUPPORT_EXITING': ('TB-5 출구로 이동 중', 'info'),
            'SUPPORT_AT_EXIT': ('TB-5 출구 도착', 'info'),
            'SUPPORT_EXIT_CANCELED': ('TB-5 출구 이동 취소(판정 종료·재난 해제)', 'info'),
            'SUPPORT_EXIT_FAILED': ('TB-5 출구 이동 실패 — 다시 보내지 않음', 'alert'),
        }
        for code, (text, level) in cases.items():
            with self.subTest(code):
                self.assertEqual({'code': code, 'text': text, 'level': level, 'scope': 'support'},
                                 mission_reason(code, 'TB-5', 'TB-6'))

    def test_automatic_approach_codes_of_187(self):   # TB-5 automatic dispatch (#187, head 4528226)
        cases = {
            'SUPPORT_POSE_BASIS_MISMATCH': ('TB-5 위치의 지도 기준이 다름 — 자동 출동 보류', 'caution'),
            'SUPPORT_AT_TARGET': ('TB-5 가 이미 대상 바로 옆 — 접근 지점을 정할 수 없음', 'caution'),
            'SUPPORT_NOT_READY': ('TB-5 준비 안 됨 — arm 필요', 'caution'),   # #196
            'APPROACH_FAILED': ('TB-5 대상 접근 실패 — 다시 보내지 않음', 'alert'),
            'MODE_NOT_DISASTER': ('재난 모드가 아님 — TB-5 출동 안 함', 'info'),
            'ALREADY_DISPATCHED': ('TB-5 이미 출동함', 'info'),
            'UNKNOWN_TARGET': ('모르는 대상 — TB-5 출동 안 함', 'caution'),
            'TARGET_NOT_CONFIRMED': ('대상 미확정 — TB-5 출동 대기', 'caution'),
            'TARGET_REVISION_STALE': ('대상 정보가 바뀜 — TB-5 출동 다시 판단', 'caution'),
            'TARGET_ALREADY_JUDGED': ('이미 판정한 대상 — TB-5 출동 안 함', 'info'),
        }
        for code, (text, level) in cases.items():
            with self.subTest(code):
                r = mission_reason(code, 'TB-5', 'TB-6')
                self.assertEqual((text, level, 'support'), (r['text'], r['level'], r['scope']))

    def test_approach_failed_with_a_reason_of_216(self):   # APPROACH_FAILED_<executor terminal reason> (OPS-127)
        cases = {
            'APPROACH_FAILED_PASSAGE_BLOCKED': ('TB-5 접근 실패: 통로 막힘 — 다시 보내지 않음', 'alert'),
            'APPROACH_FAILED_UNREACHABLE': ('TB-5 접근 실패: 갈 수 없는 위치 — 다시 보내지 않음', 'alert'),
            'APPROACH_FAILED_FAILED': ('TB-5 접근 실패: 실행 실패 — 다시 보내지 않음', 'alert'),
            # an unknown reason is shown as its code, never hidden
            'APPROACH_FAILED_WHEEL_DROP': ('TB-5 접근 실패: WHEEL_DROP — 다시 보내지 않음', 'alert'),
        }
        for code, (text, level) in cases.items():
            with self.subTest(code):
                self.assertEqual({'code': code, 'text': text, 'level': level, 'scope': 'support'},
                                 mission_reason(code, 'TB-5', 'TB-6'))

    def test_map_freeze_gates_of_214(self):   # TB-5 dispatch gates on the frozen map (#214, OPS-127)
        cases = {
            'MAP_FREEZE_WAITING_SCOUT_STOP': ('TB-6 이 멈추기를 기다림 — 멈추면 지도 고정', 'info', None),
            'MAP_FREEZE_REQUESTED': ('지도 고정 요청함 — 고정 지도 확인 중', 'info', None),
            'MAP_NOT_FROZEN': ('지도가 아직 고정되지 않음 — TB-5 출동 대기', 'caution', None),
            'MAP_FREEZE_FAILED': ('지도 고정 실패 — 다시 요청하지 않음', 'alert', None),
            'TARGET_NOT_ON_FROZEN_MAP': ('고정 지도 기준 대상 위치가 아직 없음 — TB-5 출동 대기', 'caution', None),
            'SCOUT_NOT_LOCALIZED': ('TB-6 이 고정 지도에서 위치를 못 잡음 — TB-5 출동 대기', 'caution', 'scout'),
            'SUPPORT_NOT_LOCALIZED': ('TB-5 가 고정 지도에서 위치를 못 잡음 — 출동 대기', 'caution', 'support'),
            'SUPPORT_APPROACH_NOT_FREE': ('TB-5 접근 지점이 막힘 — 출동 보류', 'caution', 'support'),
            'SUPPORT_EXIT_NOT_FREE': ('TB-5 출구 지점이 막힘 — 출구 이동 보류(대상 옆 대기)', 'caution', 'support'),
            'SCOUT_HOME_NOT_FREE': ('TB-6 도킹 접근 지점이 막힘 — TB-6 복귀 보류', 'caution', 'scout'),
            'NO_HOME_POSITION': ('복귀 위치(출구·도킹 접근) 설정 없음 — 복귀 이동 안 함', 'caution', None),
        }
        for code, (text, level, scope) in cases.items():
            with self.subTest(code):
                self.assertEqual({'code': code, 'text': text, 'level': level, 'scope': scope},
                                 mission_reason(code, 'TB-5', 'TB-6'))

    def test_auto_search_start_of_228(self):   # search_start AUTO (#228, OPS-140)
        cases = {
            'SEARCH_GOAL_NOT_SELECTED': ('TB-6 수색 목표 선택 대기', 'caution', None),
            'AUTO_SEARCH_ENABLED': ('자동 수색 — 지진 감지로 시작, 수색 시작 버튼은 받지 않음', 'info', None),
            'MAP_FREEZE_WAITING_EXIT': ('출구 QR 찾는 중 — TB-5 출동 대기', 'info', None),
        }
        for code, (text, level, scope) in cases.items():
            with self.subTest(code):
                self.assertEqual({'code': code, 'text': text, 'level': level, 'scope': scope},
                                 mission_reason(code, 'TB-5', 'TB-6'))

    def test_exit_patrol_and_start_pose_of_234(self):   # QR exit, re-base at the freeze, patrol, start pose (OPS-147)
        cases = {
            'TARGET_REBASED_ON_FREEZE': ('대상 위치를 고정 지도로 옮김 — 보고 저장 대기', 'info', None),
            'EXIT_NOT_ON_FROZEN_MAP': ('출구(QR) 위치가 고정 지도 기준에 없음 — TB-5 출동·출구 이동 보류', 'caution', None),
            'SUPPORT_EXIT_UNREGISTERED': ('출구(QR) 미등록 — TB-5 출동·출구 이동 보류', 'caution', 'support'),
            'START_POSE_UNKNOWN': ('TB-6 출발 자세 모름 — 순찰·TB-6 복귀 보류', 'caution', None),
            'START_POSE_CONFLICT': ('TB-6 출발 자세가 다른 값으로 다시 옴 — 처음 값 유지, 확인 필요', 'alert', None),
            'PATROL_ROUND_2': ('TB-6 순찰 2바퀴째', 'info', None),
            'PATROL_EXHAUSTED': ('TB-6 순찰 끝(정한 바퀴 다 돎) — 더 돌지 않음', 'caution', None),
        }
        for code, (text, level, scope) in cases.items():
            with self.subTest(code):
                self.assertEqual({'code': code, 'text': text, 'level': level, 'scope': scope},
                                 mission_reason(code, 'TB-5', 'TB-6'))
        self.assertEqual('PATROL_ROUND_X', mission_reason('PATROL_ROUND_X', 'TB-5', 'TB-6')['text'])   # unknown stays

    def test_traffic_codes_of_272(self):   # 두 로봇 교통 관제(#272, OPS-235). 두 로봇이 다 멈추는 둘은 빨강
        cases = {
            'TRAFFIC_SUPPORT_WAITING_SCOUT_STOP': ('TB-6 정지 확인 기다림 — TB-5 대기', 'caution', None),
            'TRAFFIC_SCOUT_STOP_TIMEOUT': ('TB-6 정지 확인 시간 초과 — 두 로봇 모두 대기', 'alert', None),
            'TRAFFIC_SUPPORT_WAITING_SCOUT_YIELD': ('TB-6 이 TB-5 경로 밖으로 비키는 중 — TB-5 대기', 'caution', None),
            'TRAFFIC_SCOUT_YIELD_NO_SPACE': ('TB-6 이 비킬 자리 없음 — 두 로봇 모두 정지', 'alert', None),
            'TRAFFIC_SCOUT_HELD_FOR_SUPPORT': ('TB-5 경로와 겹쳐 TB-6 명령 보류', 'caution', None),
            'TRAFFIC_ROBOTS_TOO_CLOSE': ('두 로봇이 너무 가까움 — 새 명령 보류', 'caution', None),
            'TRAFFIC_SUPPORT_PATH_UNKNOWN': ('고정 지도에서 TB-5 경로를 찾지 못함 — 대기', 'caution', None),
            'TRAFFIC_SCOUT_POSE_UNKNOWN': ('TB-6 위치 확인 안 됨 — 대기', 'caution', None),
            'TRAFFIC_SUPPORT_POSE_UNKNOWN': ('TB-5 위치 확인 안 됨 — 대기', 'caution', None),
            'TRAFFIC_SEPARATION_UNCHECKED': ('TB-5 출발 자리 미측정 — 두 로봇 간격 검사 꺼짐', 'info', None),
        }
        for code, (text, level, scope) in cases.items():
            with self.subTest(code):
                self.assertEqual({'code': code, 'text': text, 'level': level, 'scope': scope},
                                 mission_reason(code, 'TB-5', 'TB-6'))

    def test_target_time_refusals_and_unknown_codes(self):
        self.assertEqual('caution', mission_reason('TARGET_TIME_BASIS_UNUSABLE', 'TB-5', 'TB-6')['level'])
        self.assertIn('시각', mission_reason('TARGET_TIME_BASIS_UNUSABLE', 'TB-5', 'TB-6')['text'])
        self.assertIsNone(mission_reason('TARGET_OBSERVATION_STALE', 'TB-5', 'TB-6')['scope'])
        self.assertEqual({'code': 'SOMETHING_NEW', 'text': 'SOMETHING_NEW', 'level': None, 'scope': None},
                         mission_reason('SOMETHING_NEW', 'TB-5', 'TB-6'))
        self.assertIsNone(mission_reason(None, 'TB-5', 'TB-6'))

    def test_without_a_support_name(self):
        self.assertEqual('SUPPORT 출구로 이동 중', mission_reason('SUPPORT_EXITING', None, None)['text'])
        self.assertEqual('SCOUT 도킹 접근 지점으로 이동 중', mission_reason('SCOUT_RETURNING', None, None)['text'])

    def test_recheck_needed_reasons(self):   # coordinator recheck -> judgement (C-11 rows 7-11)
        cases = {'CANDIDATE_NOT_LINKED': '후보 연결 안 됨', 'FRAME_REVISION_MISMATCH': '지도 기준 다름',
                 'CANDIDATE_TIME_BASIS_UNUSABLE': '시각 기준 못 씀', 'ORDER_VIOLATION': '시각 순서 틀림',
                 'CAMERA_NOT_ABSENT': '인형이 아직 보임', 'LIDAR_NOT_CLEAR': '자리가 막힘',
                 'COVERAGE_NOT_VALID': '관측 범위 부족'}
        for why, words in cases.items():
            with self.subTest(why):
                r = mission_reason(f'RECHECK_NEEDED_{why}', 'TB-5', 'TB-6')
                self.assertEqual((f'재확인 필요: {words}', 'caution', 'target'), (r['text'], r['level'], r['scope']))
        missing = mission_reason('RECHECK_NEEDED_RECHECK_OBSERVATION_MISSING', 'TB-5', 'TB-6')
        self.assertIn('재확인 관측 없음', missing['text'])
        self.assertIn('재확인 지시', missing['text'])
        unknown = mission_reason('RECHECK_NEEDED_SOMETHING', 'TB-5', 'TB-6')
        self.assertEqual('재확인 필요: SOMETHING', unknown['text'])

    def test_scout_return(self):
        self.assertEqual(('TB-6 도킹 접근 지점으로 이동 중', 'info', 'scout'),
                         tuple(mission_reason('SCOUT_RETURNING', 'TB-5', 'TB-6')[k] for k in ('text', 'level', 'scope')))
        self.assertEqual('TB-6 도킹 접근 지점 도착 — 도킹은 사람이 합니다',
                         mission_reason('SCOUT_AT_DOCK_APPROACH_DOCK_BY_HAND', 'TB-5', 'TB-6')['text'])
        failed = mission_reason('SCOUT_RETURN_FAILED', 'TB-5', 'TB-6')   # #194: not sent again by itself
        self.assertEqual(('TB-6 도킹 접근 지점 이동 실패 — 다시 보내지 않음', 'alert', 'scout'),
                         (failed['text'], failed['level'], failed['scope']))

    def test_rescue_and_finish_words(self):
        from rescue_console.labels import label
        self.assertEqual('구출 완료', label('rescue', 'RESCUE_CONFIRMED'))
        self.assertEqual('구출 완료', label('phase', 'RESCUE_CONFIRMED'))
        self.assertEqual('임무 종료', label('mode', 'MISSION_FINISHED'))


class ReasonScreenTests(LedgerFixture):
    def view(self, reason, with_dispatch=True):
        status = dict(self.status, mission_state={'schema_version': '1.2', 'mission': mission_state(
            'DISASTER', 'ARRIVED', dispatch=dispatch('STOPPED', 3) if with_dispatch else None, reason=reason)})
        return build_view(load_ledger(self.db), status, built_at_ns=T0 + 30 * SECOND, robot_names=NAMES)

    def test_band_support_card_and_dispatched_target_show_the_exit_countdown(self):
        view = self.view('SUPPORT_EXIT_IN_7S')
        self.assertEqual('TB-5 출구 출발까지 7초', view['mission']['reason_view']['text'])
        html = render_fragment(view)
        band = re.search(r'<section id="mission".*?</section>', html, re.S).group(0)
        self.assertIn('TB-5 출구 출발까지 7초', band)
        self.assertIn('SUPPORT_EXIT_IN_7S', band)
        card = re.search(r'<section class="unit[^"]*" data-robot="robot_b".*?</section>', html, re.S).group(0)
        self.assertIn('TB-5 출구 출발까지 7초', card)
        row = re.search(rf'<li class="victim-row[^"]*"[^>]*>(?:(?!</li>).)*{uid("waiting")[:8]}.*?</li>', html, re.S)
        self.assertIsNotNone(row)
        self.assertIn('TB-5 출구 출발까지 7초', row.group(0))
        scout = re.search(r'<section class="unit[^"]*" data-robot="robot_a".*?</section>', html, re.S).group(0)
        self.assertNotIn('출구', scout)

    def test_failed_exit_is_red(self):
        html = render_fragment(self.view('SUPPORT_EXIT_FAILED'))
        self.assertRegex(html, r'class="reason alert"[^>]*>TB-5 출구 이동 실패 — 다시 보내지 않음')

    def test_scout_return_goes_on_the_scout_card(self):
        html = render_fragment(self.view('SCOUT_RETURNING'))
        scout = re.search(r'<section class="unit[^"]*" data-robot="robot_a".*?</section>', html, re.S).group(0)
        self.assertIn('TB-6 도킹 접근 지점으로 이동 중', scout)
        support = re.search(r'<section class="unit[^"]*" data-robot="robot_b".*?</section>', html, re.S).group(0)
        self.assertNotIn('도킹', support)

    def test_recheck_needed_goes_on_the_dispatched_target_row(self):
        html = render_fragment(self.view('RECHECK_NEEDED_CAMERA_NOT_ABSENT'))
        row = re.search(rf'<li class="victim-row[^"]*"[^>]*>(?:(?!</li>).)*{uid("waiting")[:8]}.*?</li>', html, re.S)
        self.assertIn('재확인 필요: 인형이 아직 보임', row.group(0))

    def test_rescued_target_is_green(self):
        html = render_fragment(self.view(None))
        self.assertIn('구출 완료', html)
        self.assertRegex(html, r'class="victim-row rescued')

    def test_target_refusal_stays_on_the_band_only(self):
        html = render_fragment(self.view('TARGET_TIME_BASIS_UNUSABLE', with_dispatch=False))
        band = re.search(r'<section id="mission".*?</section>', html, re.S).group(0)
        self.assertIn('TARGET_TIME_BASIS_UNUSABLE', band)
        card = re.search(r'<section class="unit[^"]*" data-robot="robot_b".*?</section>', html, re.S).group(0)
        self.assertNotIn('TARGET_TIME_BASIS_UNUSABLE', card)


class SupportDispatchModeTests(LedgerFixture):
    """--support-dispatch AUTO (owner decision 2026-10-13): the coordinator sends the support robot itself
    and refuses DISPATCH_SUPPORT with NOT_AUTHORIZED, so there is no button, only the state."""

    def view(self, mode, reason=None, with_dispatch=False):
        status = dict(self.status, mission_state={'schema_version': '1.2', 'mission': mission_state(
            'DISASTER', 'TARGET_FOUND', dispatch=dispatch('STOPPED', 1) if with_dispatch else None, reason=reason)})
        return build_view(load_ledger(self.db), status, built_at_ns=T0 + 30 * SECOND, robot_names=NAMES,
                          support_dispatch=mode)

    def test_auto_shows_the_state_instead_of_the_button(self):
        view = self.view('AUTO')
        gate = {c['name']: c for c in view['commands']}['DISPATCH_SUPPORT']
        self.assertEqual((False, 'TB-5 자동 출동', True), (gate['enabled'], gate['label'], gate['auto']))
        html = render_page(view, command_access=(True, 'ADMIN'))
        self.assertNotIn('data-command="DISPATCH_SUPPORT"', re.sub(r'<script>.*?</script>', '', html, flags=re.S)
                         .replace('class="cmd-auto" data-command="DISPATCH_SUPPORT"', ''))
        self.assertIn('class="cmd-auto"', html)
        self.assertIn('TB-5 자동 출동', html)

    def test_operator_or_unset_keeps_the_button(self):
        for mode in ('OPERATOR', None):
            with self.subTest(mode=mode):
                html = render_page(self.view(mode), command_access=(True, 'ADMIN'))
                self.assertIn('<button type="button" class="cmd approve"', html)
                self.assertRegex(html, r'<button[^>]*data-command="DISPATCH_SUPPORT"')

    def test_auto_reason_goes_on_the_targets_waiting_for_support(self):
        html = render_fragment(self.view('AUTO', reason='SUPPORT_POSE_UNKNOWN'))
        rows = re.findall(r'<li class="victim-row not_rescued".*?</li>', html, re.S)
        self.assertTrue(rows)
        self.assertTrue(all('TB-5 위치 모름' in row for row in rows))

    def test_mode_value_is_checked(self):
        from rescue_console.view import ConsoleError
        with self.assertRaises(ConsoleError):
            self.view('SOMETIMES')


class SearchStartModeTests(LedgerFixture):
    """--search-start AUTO (#228, OPS-140): the disaster sign starts TB-6's search and the coordinator refuses
    START_SEARCH (NOT_AUTHORIZED, AUTO_SEARCH_ENABLED), so there is no button, only the state. A console
    started without AUTO against an AUTO coordinator is said, from the refusal it gets back."""

    def view(self, mode, phase='DISASTER', reason=None, commands=()):
        status = dict(self.status, mission_state={'schema_version': '1.2', 'mission': mission_state(
            'DISASTER', phase, reason=reason)})
        if commands:
            status['operator_commands'] = {'schema_version': '1.0', 'commands': list(commands)}
        return build_view(load_ledger(self.db), status, built_at_ns=T0 + 30 * SECOND, robot_names=NAMES,
                          search_start=mode)

    def test_auto_shows_the_state_instead_of_the_button(self):
        view = self.view('AUTO')
        gate = {c['name']: c for c in view['commands']}['START_SEARCH']
        self.assertEqual((False, 'TB-6 자동 출발', True), (gate['enabled'], gate['label'], gate['auto']))
        self.assertIn('지진 감지', gate['reason'])
        html = re.sub(r'<script>.*?</script>', '', render_page(view, command_access=(True, 'ADMIN')), flags=re.S)
        self.assertNotIn('data-command="START_SEARCH"', html)
        self.assertIn('TB-6 자동 출발', html)

    def test_operator_or_unset_keeps_the_button(self):
        for mode in ('OPERATOR', None):
            with self.subTest(mode=mode):
                html = render_page(self.view(mode), command_access=(True, 'ADMIN'))
                self.assertRegex(html, r'<button[^>]*data-command="START_SEARCH"')

    def test_a_coordinator_in_auto_against_a_button_console_is_said(self):
        refused = {'command_id': uid('cmd-auto'), 'kind': 'START_SEARCH', 'requested_at_ns': T0 + SECOND, 'accepted': False,
                   'result': 'NOT_AUTHORIZED', 'reason': 'AUTO_SEARCH_ENABLED', 'target_id': None,
                   'operator_id': 'admin'}
        for mode, kwargs in ((None, {'commands': [refused]}), ('OPERATOR', {'reason': 'AUTO_SEARCH_ENABLED'})):
            with self.subTest(mode=mode):
                warnings = ' '.join(self.view(mode, **kwargs)['warnings'])
                self.assertIn('--search-start AUTO', warnings)
        self.assertNotIn('--search-start', ' '.join(self.view('AUTO', reason='AUTO_SEARCH_ENABLED')['warnings']))

    def test_mode_value_is_checked(self):
        from rescue_console.view import ConsoleError
        with self.assertRaises(ConsoleError):
            self.view('auto')

    def test_command_line_passes_the_mode(self):
        import os
        from unittest import mock
        from rescue_console import __main__ as cli
        from rescue_console.view import ConsoleError
        os.environ['RESCUE_CONSOLE_TEST_SECRET'] = 'x' * 16
        self.addCleanup(os.environ.pop, 'RESCUE_CONSOLE_TEST_SECRET', None)
        seen = {}

        def create_app(*args, **kwargs):   # stop before the server starts
            seen.update(kwargs)
            raise ConsoleError('INVALID_INPUT', 'stop')
        with mock.patch.object(cli, 'create_app', create_app), mock.patch('sys.stderr'):
            cli.main([str(self.db), '--accounts', str(self.dir / 'accounts.sqlite3'), '--status', 'status.json',
                      '--search-start', 'AUTO', '--secret-env', 'RESCUE_CONSOLE_TEST_SECRET'])
        self.assertEqual('AUTO', seen.get('search_start'))


if __name__ == '__main__':
    unittest.main()
