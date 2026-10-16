"""Replay page: a past run (schema 1.5 runs, #318) drawn as it was known at each moment.

Events, operator commands, errors, exit sightings and target stage changes come from the run's range reads;
robot and target values at a moment come from the history tables (the latest row at or before it), never from
today's latest row. The run's maps are drawn in test_console_replay_map.py.
"""
import unittest
from unittest import mock

from rescue_operations.site_state import SiteStateLedger
from rescue_contracts import SiteEvent
from rescue_console.render import render_page
from rescue_console.replay import REPLAY_ROWS, build_timeline, replay_at, run_choices
from rescue_console.replay_page import render_replay_fragment, render_replay_page
from rescue_console.sources import REPLAY_PAGES, _range_all, load_ledger, load_replay, load_runs
from rescue_console.view import ConsoleError, build_view
from test_console_web import ADMIN, CUSTOMER, HAS_FLASK, SECRET, AccountsFixture
from test_console_view import (ROBOTS, SECOND, T0, LedgerFixture, digest_dir, robot_status, site_event,
                               target_state, uid)


def command(label, *, kind='START_SEARCH', offset=0, accepted=True, result='ACCEPTED', reason=None):
    return dict(command_id=uid('replay-command/' + label), kind=kind, operator_id='admin', robot_id=None,
                target_id=None, segment_index=None, route_revision=None, requested_at_ns=T0 + offset * SECOND,
                accepted=accepted, result=result, reason=reason)


def exit_location(label, *, offset, x=1.5):
    return dict(location_id=uid('replay-exit/' + label), observed_at_ns=T0 + offset * SECOND,
                received_at_ns=T0 + offset * SECOND + 1, map_session_id=uid('map'), map_revision=2,
                transform_revision=1, mission_frame='map', x=x, y=0.5, class_label='exit_qr')


def robot(sequence, x, offset):
    status = robot_status('robot_a', stamp_offset=offset)
    status.pop('received_at_ns')
    return status | {'state_sequence': sequence, 'pose': {'x': x, 'y': -1.0, 'yaw': 0.0, 'frame_id': 'map'}}


def target(offset, **changes):
    return target_state('doll', **changes) | {'updated_at_ns': T0 + offset * SECOND}


class ReplayFixture(LedgerFixture):
    """Run 1 (closed, 0 ~ 80 s) holds the story; run 2 (open from 100 s) holds one later event.

    The ledger stamps robot history and run start with its own clock, so the clock is pinned per write.
    """

    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        self.now = T0
        with mock.patch('rescue_operations.site_state.time.time_ns', lambda: self.now):
            with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
                self.run1 = ledger.run_id
                writes = [
                    (10, lambda: ledger.record_site_event(SiteEvent(**site_event('sign', kind='DISASTER_SIGN', offset=10)))),
                    (15, lambda: ledger.record_robot_status(robot(1, 0.5, 15))),
                    (20, lambda: ledger.record_operator_command(command('start', offset=20))),
                    (25, lambda: ledger.record_target_state(target(25))),
                    (30, lambda: ledger.record_error(dict(at_ns=T0 + 30 * SECOND, robot_id='robot_b', task_id=None,
                                                          code='ACCEPT_TIMEOUT', detail='no coordinator answer'))),
                    (40, lambda: ledger.record_robot_status(robot(2, 1.0, 40))),
                    (40, lambda: ledger.record_exit_location(exit_location('first', offset=40))),
                    (45, lambda: ledger.record_target_state(target(45, support='ASSIGNED', robot='robot_b'))),
                    (50, lambda: ledger.record_site_event(SiteEvent(**site_event('pass', kind='EXIT_PASS', offset=50)))),
                    (60, lambda: ledger.record_operator_command(command('home', kind='RETURN_HOME', offset=60,
                                                                        accepted=False, result='REJECTED',
                                                                        reason='NOT_VERDICT'))),
                    (70, lambda: ledger.record_exit_location(exit_location('second', offset=70, x=2.5))),
                ]
                for offset, write in writes:
                    self.now = T0 + offset * SECOND
                    write()
                ledger.close_run(at_ns=T0 + 80 * SECOND)
                self.run2 = ledger.open_run(at_ns=T0 + 100 * SECOND)
                ledger.record_site_event(SiteEvent(**site_event('later', kind='DISASTER_SIGN', offset=110)))

    def timeline(self, run_id=None):
        runs, _default = run_choices(load_runs(self.site_db))
        run = next(r for r in runs if r['run_id'] == (run_id or self.run1))
        return build_timeline(load_replay(self.site_db, run['run_id']), run=run, robot_names={'robot_a': 'TB-6'})


class ReplayRunTests(ReplayFixture):
    def test_runs_are_numbered_oldest_first_and_the_open_one_is_chosen(self):
        runs, default = run_choices(load_runs(self.site_db))
        self.assertEqual([(1, self.run1, False), (2, self.run2, True)],
                         [(r['number'], r['run_id'], r['open']) for r in runs])
        self.assertEqual(self.run2, default)

    def test_without_an_open_run_the_last_run_is_chosen(self):
        runs = {'open_run_id': None, 'runs': [{'run_id': uid('a'), 'started_at_ns': T0, 'ended_at_ns': T0 + 1,
                                               'note': None}, {'run_id': uid('b'), 'started_at_ns': T0 + 2,
                                                               'ended_at_ns': T0 + 3, 'note': None}]}
        self.assertEqual(uid('b'), run_choices(runs)[1])
        self.assertEqual(([], None), run_choices({'open_run_id': None, 'runs': []}))

    def test_a_run_holds_only_its_own_rows(self):
        later = self.timeline(self.run2)
        self.assertEqual([uid('later')], [item['ref_id'] for item in later['items']])
        self.assertNotIn(uid('later'), [item['ref_id'] for item in self.timeline()['items']])


class ReplayTimelineTests(ReplayFixture):
    def test_timeline_spans_the_run_and_lists_records_in_time_order(self):
        timeline = self.timeline()
        self.assertEqual((T0, T0 + 80 * SECOND), (timeline['start_ns'], timeline['end_ns']))
        self.assertEqual(['EVENT', 'COMMAND', 'TARGET', 'ERROR', 'EXIT', 'TARGET', 'EVENT', 'COMMAND', 'EXIT'],
                         [item['kind'] for item in timeline['items']])
        self.assertEqual(([], []), (timeline['missing'], timeline['truncated']))

    def test_rows_match_the_stored_records(self):
        items = {item['ref_id']: item for item in self.timeline()['items']}
        home = items[uid('replay-command/home')]
        self.assertEqual(('RETURN_HOME', '복귀', 0, 'NOT_VERDICT', 'REJECTED'),
                         (home['what'], home['label'], home['accepted'], home['reason'], home['detail']))
        stages = [item['label'] for item in self.timeline()['items'] if item['kind'] == 'TARGET']
        self.assertEqual(['인형 단계 → 미구조', '인형 단계 → 지원 중'], stages)

    def test_the_run_maps_are_read_too(self):
        """지도도 회차 범위 읽기로 받는다(이 픽스처 회차에는 지도가 없다: test_console_replay_map)."""
        self.assertEqual([], self.timeline()['map_rows'])
        self.assertNotIn('map_snapshots', self.timeline()['missing'])

    def test_reading_leaves_the_database_untouched(self):
        before = digest_dir(self.dir)
        self.timeline()
        self.assertEqual(before, digest_dir(self.dir))


class ReplayAtTests(ReplayFixture):
    def test_robot_and_target_are_their_values_at_that_moment_not_the_latest(self):
        state = replay_at(self.timeline(), T0 + 30 * SECOND)
        (tb6,) = state['robots']
        self.assertEqual(('TB-6', 0.5, 15 * SECOND), (tb6['name'], tb6['pose']['x'], tb6['age_ns']))
        self.assertTrue(tb6['stale'])   # 15 s old > REPLAY_STALE_NS
        self.assertEqual(['NOT_RESCUED'], [t['stage'] for t in state['targets']])
        later = replay_at(self.timeline(), T0 + 45 * SECOND)
        self.assertEqual((1.0, False), (later['robots'][0]['pose']['x'], later['robots'][0]['stale']))
        self.assertEqual(['SUPPORTING'], [t['stage'] for t in later['targets']])

    def test_before_the_first_history_row_nothing_is_drawn(self):
        state = replay_at(self.timeline(), T0 + 5 * SECOND)
        self.assertEqual(([], [], 0), (state['robots'], state['targets'], state['seen_count']))

    def test_state_counts_exits_and_passes_known_by_then(self):
        state = replay_at(self.timeline(), T0 + 45 * SECOND)
        self.assertEqual((1.5, 0), (state['exit']['x'], state['exit_pass_count']))
        state = replay_at(self.timeline(), T0 + 80 * SECOND)
        self.assertEqual((2.5, 1), (state['exit']['x'], state['exit_pass_count']))

    def test_moment_outside_the_run_is_refused_not_clamped(self):
        timeline = self.timeline()
        for t_ns in (T0 - 1, T0 + 81 * SECOND):
            with self.subTest(t_ns=t_ns), self.assertRaises(ConsoleError) as caught:
                replay_at(timeline, t_ns)
            self.assertEqual('INVALID_INPUT', caught.exception.detail_code)
        with self.assertRaises(ConsoleError):
            replay_at(timeline, float(T0 + 20 * SECOND))

    def test_rows_are_capped_newest_first(self):
        items = [{'at_ns': T0 + n, 'kind': 'ERROR', 'what': 'X', 'ref_id': str(n)} for n in range(REPLAY_ROWS + 5)]
        timeline = {'start_ns': T0, 'end_ns': T0 + REPLAY_ROWS + 4, 'items': items, 'robot_rows': [],
                    'target_rows': [], 'names': {}}
        state = replay_at(timeline, T0 + REPLAY_ROWS + 4)
        self.assertEqual(REPLAY_ROWS, len(state['rows']))
        self.assertEqual(str(REPLAY_ROWS + 4), state['rows'][0]['ref_id'])


class RangePagingTests(unittest.TestCase):
    """operations returns at most `limit` rows oldest first and cuts the newest; the console reads on."""

    @staticmethod
    def reader(times, limit=3):
        def read(_db, *, run_id, start_ns=None):
            rows = [{'at_ns': t, 'n': n} for n, t in enumerate(times) if start_ns is None or t >= start_ns]
            return {'truncated': len(rows) > limit, 'history': rows[:limit]}
        return read

    def test_pages_join_without_gaps_or_repeats_at_a_shared_time(self):
        times = [1, 2, 2, 3, 3, 4, 5, 5]   # a shared time never fills a whole page here
        got = _range_all(self.reader(times), 'history', 'at_ns', None, 'run')
        self.assertEqual((False, list(range(len(times)))), (got['truncated'], [row['n'] for row in got['history']]))

    def test_one_time_fuller_than_a_page_is_truncated_not_looped(self):
        got = _range_all(self.reader([1, 1, 1, 1, 2]), 'history', 'at_ns', None, 'run')
        self.assertTrue(got['truncated'])

    def test_page_cap_marks_truncated(self):
        times = list(range(3 * REPLAY_PAGES + 10))
        got = _range_all(self.reader(times), 'history', 'at_ns', None, 'run')
        self.assertTrue(got['truncated'])
        self.assertEqual(list(range(len(got['history']))), [row['n'] for row in got['history']])


class ReplayRenderTests(ReplayFixture):
    def page(self):
        timeline = self.timeline()
        runs, _default = run_choices(load_runs(self.site_db))
        return render_replay_page(timeline, replay_at(timeline, timeline['end_ns']), runs)

    def test_page_draws_the_end_of_the_run_with_a_mark_per_record_and_no_command_buttons(self):
        html = self.page()
        self.assertEqual(9, html.count('data-offset="'))
        self.assertIn(f'max="{80 * SECOND}"', html)
        self.assertIn('이 회차에는 남은 지도가 없습니다', html)
        self.assertIn(f'<option value="{self.run1}" selected>회차 1', html)
        self.assertIn('진행 중', html)
        self.assertNotIn('action="/commands"', html)
        self.assertNotIn('name="kind"', html)

    def test_fragment_shows_robot_target_and_refusal_at_that_moment(self):
        timeline = self.timeline()
        html = render_replay_fragment(replay_at(timeline, T0 + 60 * SECOND), timeline)
        self.assertIn('<td>TB-6</td><td>(1.00, -1.00) map</td><td>81%</td>', html)
        self.assertIn('낡음', html)   # 20 s since the last robot row
        self.assertIn('지원 중', html)
        self.assertIn('거절', html)
        self.assertIn('NOT_VERDICT', html)
        self.assertIn('<dt>출구 통과</dt><dd>1</dd>', html)

    def test_record_text_is_escaped(self):
        with mock.patch('rescue_operations.site_state.time.time_ns', lambda: T0 + 120 * SECOND), \
                SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            ledger.record_error(dict(at_ns=T0 + 120 * SECOND, robot_id=None, task_id=None, code='ACCEPT_TIMEOUT',
                                     detail='<script>alert(1)</script>'))
        timeline = self.timeline(self.run2)
        html = render_replay_fragment(replay_at(timeline, timeline['end_ns']), timeline)
        self.assertNotIn('<script>alert', html)
        self.assertIn('&lt;script&gt;', html)

    def test_no_run_page_says_so(self):
        html = render_replay_page(None, None)
        self.assertIn('기록 없음', html)
        self.assertNotIn('id="replay-slider"', html)

    def test_console_page_links_to_the_replay(self):
        html = render_page(build_view(load_ledger(self.db), {}, built_at_ns=T0))
        self.assertIn('href="/replay"', html)


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: replay routes NOT_RUN')
class ReplayWebTests(ReplayFixture, AccountsFixture):
    def client(self, site_state=True):
        from rescue_console.app import create_app
        app = create_app(None if site_state else self.db, accounts_path=self.accounts, secret_key=SECRET,
                         site_state_path=self.site_db if site_state else None, robot_names={'robot_a': 'TB-6'})
        return app.test_client()

    def test_any_logged_in_role_reads_the_replay_and_nothing_is_written(self):
        client = self.client()
        self.assertEqual(302, client.get('/replay').status_code)
        self.assertEqual(401, client.get('/replay/fragment?offset_ns=0').status_code)
        self.login(client, *CUSTOMER)
        before = digest_dir(self.dir)
        page = client.get('/replay').get_data(as_text=True)
        self.assertIn(f'<option value="{self.run2}" selected>', page)   # the open run first
        page = client.get(f'/replay?run={self.run1}')
        self.assertEqual(200, page.status_code)
        self.assertIn('id="replay-slider"', page.get_data(as_text=True))
        part = client.get(f'/replay/fragment?run={self.run1}&offset_ns={45 * SECOND}')
        self.assertEqual(200, part.status_code)
        self.assertIn('이 시점까지 6건', part.get_data(as_text=True))
        self.assertEqual(before, digest_dir(self.dir))

    def test_unknown_run_and_bad_offset_are_a_503_line(self):
        client = self.client()
        self.login(client, *ADMIN)
        for path in (f'/replay?run={uid("nope")}', '/replay?run=not-a-uuid',
                     f'/replay/fragment?run={uid("nope")}&offset_ns=0'):
            with self.subTest(path=path):
                self.assertEqual(503, client.get(path).status_code)
        # '²' passes str.isdigit() but int() refuses it; '١' (Arabic-Indic one) would be read as 1 (REV-06).
        for query in ('offset_ns=-1', 'offset_ns=abc', 'offset_ns=', f'offset_ns={81 * SECOND}',
                      'offset_ns=%C2%B2', 'offset_ns=%D9%A1', 'offset_ns=%EF%BC%91', 'offset_ns=+1', 'offset_ns=1_0'):
            with self.subTest(query=query):
                self.assertEqual(503, client.get(f'/replay/fragment?run={self.run1}&' + query).status_code)

    def test_closed_run_is_read_once_and_open_run_every_time(self):
        client = self.client()
        self.login(client, *CUSTOMER)
        with mock.patch('rescue_console.app.load_replay', side_effect=load_replay) as reads:
            for _ in range(3):
                client.get(f'/replay/fragment?run={self.run1}&offset_ns=0')
                client.get(f'/replay/fragment?run={self.run2}&offset_ns=0')
        self.assertEqual([self.run1, self.run2, self.run2, self.run2], [c.args[1] for c in reads.call_args_list])

    def test_replay_without_site_state_is_refused(self):
        client = self.client(site_state=False)
        self.login(client, *CUSTOMER)
        self.assertEqual(503, client.get('/replay').status_code)
        self.assertEqual(503, client.get('/replay/fragment?offset_ns=0').status_code)

    def test_replay_routes_take_no_post(self):
        client = self.client()
        self.login(client, *ADMIN)
        self.assertEqual(405, client.post('/replay').status_code)
        self.assertEqual(405, client.post('/replay/fragment').status_code)


if __name__ == '__main__':
    unittest.main()
