"""SystemState.reason in plain Korean on the console (2026-10-13).

The coordinator puts the support robot's exit state and refusal reasons in SystemState.reason
(operations stores it in the mission row). The console names the support robot with --robot-name
(#168), shows the code next to the words, keeps unknown codes as they are, and marks a failed exit red.
"""
import re
import unittest

from test_console_commands import dispatch, mission_state
from test_console_view import LedgerFixture, SECOND, T0, uid
from rescue_console.render import render_fragment
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
                self.assertEqual({'code': code, 'text': text, 'level': level, 'support': True},
                                 mission_reason(code, 'TB-5'))

    def test_target_time_refusals_and_unknown_codes(self):
        self.assertEqual('caution', mission_reason('TARGET_TIME_BASIS_UNUSABLE', 'TB-5')['level'])
        self.assertIn('시각', mission_reason('TARGET_TIME_BASIS_UNUSABLE', 'TB-5')['text'])
        self.assertFalse(mission_reason('TARGET_OBSERVATION_STALE', 'TB-5')['support'])
        self.assertEqual({'code': 'SOMETHING_NEW', 'text': 'SOMETHING_NEW', 'level': None, 'support': False},
                         mission_reason('SOMETHING_NEW', 'TB-5'))
        self.assertIsNone(mission_reason(None, 'TB-5'))

    def test_without_a_support_name(self):
        self.assertEqual('지원 로봇 출구로 이동 중', mission_reason('SUPPORT_EXITING', None)['text'])


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

    def test_target_refusal_stays_on_the_band_only(self):
        html = render_fragment(self.view('TARGET_TIME_BASIS_UNUSABLE', with_dispatch=False))
        band = re.search(r'<section id="mission".*?</section>', html, re.S).group(0)
        self.assertIn('TARGET_TIME_BASIS_UNUSABLE', band)
        card = re.search(r'<section class="unit[^"]*" data-robot="robot_b".*?</section>', html, re.S).group(0)
        self.assertNotIn('TARGET_TIME_BASIS_UNUSABLE', card)


if __name__ == '__main__':
    unittest.main()
