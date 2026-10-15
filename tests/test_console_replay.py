"""Replay page model: a past run is drawn only from the site-state tables that keep their history.

Events, operator commands, errors and exit sightings are stored with their time, so the console can say what
was known at any moment of the run. Robot status, target stages and the map keep only their latest row, so
the replay must show them as "no history" rather than draw today's value at an earlier time.
"""
import unittest

from rescue_operations.site_state import SiteStateLedger
from rescue_contracts import SiteEvent
from rescue_console.replay import REPLAY_READ_LIMIT, REPLAY_ROWS, build_timeline, replay_at
from rescue_console.sources import load_replay
from rescue_console.view import ConsoleError
from test_console_view import ROBOTS, SECOND, T0, LedgerFixture, digest_dir, site_event, uid


def command(label, *, kind='START_SEARCH', offset=0, accepted=True, result='ACCEPTED', reason=None):
    return dict(command_id=uid('replay-command/' + label), kind=kind, operator_id='admin', robot_id=None,
                target_id=None, segment_index=None, route_revision=None, requested_at_ns=T0 + offset * SECOND,
                accepted=accepted, result=result, reason=reason)


def exit_location(label, *, offset, x=1.5):
    return dict(location_id=uid('replay-exit/' + label), observed_at_ns=T0 + offset * SECOND,
                received_at_ns=T0 + offset * SECOND + 1, map_session_id=uid('map'), map_revision=2,
                transform_revision=1, mission_frame='map', x=x, y=0.5, class_label='exit_qr')


class ReplayFixture(LedgerFixture):
    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            ledger.record_site_event(SiteEvent(**site_event('sign', kind='DISASTER_SIGN', offset=10)))
            ledger.record_operator_command(command('start', offset=20))
            ledger.record_error(dict(at_ns=T0 + 30 * SECOND, robot_id='robot_b', task_id=None,
                                     code='ACCEPT_TIMEOUT', detail='no coordinator answer'))
            ledger.record_exit_location(exit_location('first', offset=40))
            ledger.record_site_event(SiteEvent(**site_event('pass', kind='EXIT_PASS', offset=50)))
            ledger.record_operator_command(command('home', kind='RETURN_HOME', offset=60, accepted=False,
                                                   result='REJECTED', reason='NOT_VERDICT'))
            ledger.record_exit_location(exit_location('second', offset=70, x=2.5))

    def timeline(self):
        return build_timeline(load_replay(self.site_db))


class ReplayTimelineTests(ReplayFixture):
    def test_timeline_spans_the_first_and_last_record_in_time_order(self):
        timeline = self.timeline()
        self.assertEqual((T0 + 10 * SECOND, T0 + 70 * SECOND), (timeline['start_ns'], timeline['end_ns']))
        self.assertEqual(['EVENT', 'COMMAND', 'ERROR', 'EXIT', 'EVENT', 'COMMAND', 'EXIT'],
                         [item['kind'] for item in timeline['items']])
        self.assertEqual(([], []), (timeline['missing'], timeline['truncated']))

    def test_rows_match_the_stored_records(self):
        items = {item['ref_id']: item for item in self.timeline()['items']}
        home = items[uid('replay-command/home')]
        self.assertEqual(('RETURN_HOME', '복귀', 0, 'NOT_VERDICT', 'REJECTED'),
                         (home['what'], home['label'], home['accepted'], home['reason'], home['detail']))
        sign = items[uid('sign')]
        self.assertEqual(('DISASTER_SIGN', 'cam_a', T0 + 10 * SECOND), (sign['what'], sign['source'], sign['at_ns']))

    def test_tables_without_history_are_named_not_filled(self):
        keys = [row['key'] for row in self.timeline()['no_history']]
        self.assertEqual(['robots', 'targets', 'map'], keys)

    def test_reading_leaves_the_database_untouched(self):
        before = digest_dir(self.dir)
        self.timeline()
        self.assertEqual(before, digest_dir(self.dir))


class ReplayAtTests(ReplayFixture):
    def test_state_at_a_moment_holds_only_what_was_known_then(self):
        state = replay_at(self.timeline(), T0 + 45 * SECOND)
        self.assertEqual(4, state['seen_count'])
        self.assertEqual(['EXIT', 'ERROR', 'COMMAND', 'EVENT'], [row['kind'] for row in state['rows']])
        self.assertEqual(1.5, state['exit']['x'])
        self.assertEqual(0, state['exit_pass_count'])
        self.assertEqual(35 * SECOND, state['offset_ns'])

    def test_later_moment_counts_the_exit_pass_and_takes_the_newer_exit(self):
        state = replay_at(self.timeline(), T0 + 70 * SECOND)
        self.assertEqual((7, 1, 2.5), (state['seen_count'], state['exit_pass_count'], state['exit']['x']))

    def test_moment_outside_the_run_is_refused_not_clamped(self):
        timeline = self.timeline()
        for t_ns in (T0, T0 + 71 * SECOND):
            with self.subTest(t_ns=t_ns), self.assertRaises(ConsoleError) as caught:
                replay_at(timeline, t_ns)
            self.assertEqual('INVALID_INPUT', caught.exception.detail_code)
        with self.assertRaises(ConsoleError):
            replay_at(timeline, float(T0 + 20 * SECOND))

    def test_rows_are_capped_newest_first(self):
        items = [{'at_ns': T0 + n, 'kind': 'ERROR', 'what': 'X', 'ref_id': str(n)} for n in range(REPLAY_ROWS + 5)]
        timeline = {'start_ns': T0, 'end_ns': T0 + REPLAY_ROWS + 4, 'items': items}
        state = replay_at(timeline, T0 + REPLAY_ROWS + 4)
        self.assertEqual(REPLAY_ROWS, len(state['rows']))
        self.assertEqual(str(REPLAY_ROWS + 4), state['rows'][0]['ref_id'])


class ReplayEdgeTests(unittest.TestCase):
    def test_empty_run_has_no_time_axis(self):
        empty = {key: {'schema_version': '1.4', name: []} for key, name in
                 (('site_events', 'site_events'), ('operator_commands', 'commands'), ('error_log', 'errors'),
                  ('exit_locations', 'exits'))}
        timeline = build_timeline(empty)
        self.assertEqual((None, None, []), (timeline['start_ns'], timeline['end_ns'], timeline['items']))
        with self.assertRaises(ConsoleError):
            replay_at(timeline, T0)

    def test_missing_and_full_sections_are_reported(self):
        errors = [{'seq': n, 'at_ns': T0 + n, 'robot_id': None, 'task_id': None, 'code': 'X', 'detail': ''}
                  for n in range(REPLAY_READ_LIMIT)]
        timeline = build_timeline({'error_log': {'schema_version': '1.4', 'errors': errors}})
        self.assertEqual(['site_events', 'operator_commands', 'exit_locations'], timeline['missing'])
        self.assertEqual(['error_log'], timeline['truncated'])


if __name__ == '__main__':
    unittest.main()
