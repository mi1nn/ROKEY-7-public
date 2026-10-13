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


if __name__ == '__main__':
    unittest.main()
