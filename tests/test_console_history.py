"""Command and error history: the console shows what operations recorded for each command, rejections included.

On 10-13 the operator presses a button and the coordinator may refuse it. The answer shows once in the
command panel of that page; this table keeps it (and operations' error_log) for every viewer and after a reload.
"""
import re
import unittest

from rescue_operations.site_state import SiteStateLedger
from rescue_console.render import render_fragment, render_page
from rescue_console.sources import load_site_state
from rescue_console.view import HISTORY_ROWS, build_view
from rescue_console.sources import load_ledger
from test_console_view import ROBOTS, SECOND, T0, LedgerFixture, uid, write_site_state


def command(label, *, kind='DISPATCH_SUPPORT', offset=0, accepted=False, result='REJECTED',
            reason='PENDING_EVIDENCE', target=True):
    return dict(command_id=uid('command/' + label), kind=kind, operator_id='admin', robot_id=None,
                target_id=uid('waiting') if target else None, segment_index=None, route_revision=None,
                requested_at_ns=T0 + offset * SECOND, accepted=accepted, result=result, reason=reason)


class HistoryFixture(LedgerFixture):
    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        write_site_state(self.site_db, self.status)
        with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            ledger.record_operator_command(command('start', kind='START_SEARCH', offset=40, accepted=True,
                                                   result='ACCEPTED', reason=None, target=False))
            ledger.record_operator_command(command('dispatch', offset=50))
            ledger.record_error(dict(at_ns=T0 + 45 * SECOND, robot_id=None, task_id=None, code='ACCEPT_TIMEOUT',
                                     detail='operator_command <b>x</b>: no coordinator answer (503)'))

    def site_view(self):
        return build_view(load_ledger(self.db), load_site_state(self.site_db), built_at_ns=T0 + 60 * SECOND)


class HistorySourceTests(HistoryFixture):
    def test_site_state_read_includes_commands_and_errors_newest_first(self):
        status = load_site_state(self.site_db)
        self.assertEqual([c['kind'] for c in status['operator_commands']['commands']],
                         ['DISPATCH_SUPPORT', 'START_SEARCH'])
        self.assertEqual([e['code'] for e in status['error_log']['errors']], ['ACCEPT_TIMEOUT'])


class HistoryViewTests(HistoryFixture):
    def test_commands_and_errors_merge_newest_first_with_their_answer(self):
        rows = self.site_view()['history']['rows']
        self.assertEqual([(r['kind'], r['what']) for r in rows],
                         [('COMMAND', 'DISPATCH_SUPPORT'), ('ERROR', 'ACCEPT_TIMEOUT'), ('COMMAND', 'START_SEARCH')])
        refused, error, started = rows
        self.assertEqual((refused['accepted'], refused['reason'], refused['target_id']),
                         (False, 'PENDING_EVIDENCE', uid('waiting')))
        self.assertEqual((started['accepted'], started['reason']), (True, None))
        self.assertIn('no coordinator answer', error['detail'])

    def test_rows_are_capped_newest_kept(self):
        with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            for i in range(HISTORY_ROWS + 5):
                ledger.record_operator_command(command(f'bulk-{i}', offset=100 + i))
        rows = self.site_view()['history']['rows']
        self.assertEqual(len(rows), HISTORY_ROWS)
        self.assertEqual(rows[0]['ref_id'], uid(f'command/bulk-{HISTORY_ROWS + 4}'))

    def test_without_the_inputs_history_is_missing_not_empty(self):
        self.assertIsNone(self.view()['history'])        # status fixture without the two sections


class HistoryPageTests(HistoryFixture):
    def test_refused_command_shows_its_reason_in_korean_and_code(self):
        html = render_fragment(self.site_view())
        section = re.search(r'<section id="history".*?</section>', html, re.S).group(0)
        self.assertIn('AMR2 출동 승인', section)
        self.assertIn('거절', section)
        self.assertIn('확정·경로·판정이 아직 없음', section)
        self.assertIn('PENDING_EVIDENCE', section)
        self.assertIn('수색 시작', section)
        self.assertIn('접수', section)
        self.assertIn('ACCEPT_TIMEOUT', section)

    def test_history_rows_stay_out_of_the_kpi04_event_rows(self):
        view = self.site_view()
        html = render_fragment(view)
        self.assertEqual(len(re.findall(r'<tr class="event ', html)), len(view['events']))

    def test_error_detail_is_escaped(self):
        html = render_page(self.site_view())
        self.assertNotIn('<b>x</b>', html)
        self.assertIn('&lt;b&gt;x&lt;/b&gt;', html)

    def test_missing_history_says_so(self):
        html = render_fragment(self.view())
        self.assertIn('id="history"', html)
        self.assertIn('기록 입력 없음', html)


if __name__ == '__main__':
    unittest.main()
