"""리플레이 지도 칸(재범 2026-10-16 「지도 무조건 나와야지」): 고른 시각 이전의 가장 최근 회차 지도 위에 로봇·인형·출구.

operations 의 MapArchive(#map_archive)가 회차마다 `<map_dir>/<run_id>/live-r…` 실시간 SLAM 지도와 `frozen-r…`
고정 지도를 PGM + YAML 로 남기고 map_snapshots 에 한 줄씩 적는다. 웹은 그 줄을 회차 범위 읽기로 받고, 줄의 path(PGM)
옆 같은 이름 YAML 을 읽는다. PGM sha256 이 줄과 다르면 그리지 않는다. 지도가 아직 없는 시각에는 첫 지도를 흐리게
보이고 그렇다고 적는다. 회차에 지도가 하나도 없거나 파일을 못 읽으면 그 사유를 보인다(빈 칸으로 두지 않는다).
"""
import hashlib
import unittest
from unittest import mock

from rescue_operations.map_archive import MapArchive
from rescue_operations.map_snapshot import grid_to_pgm
from rescue_operations.site_state import SiteStateLedger
from rescue_console.mapimage import SnapshotMaps
from rescue_console.replay import build_timeline, replay_at, run_choices
from rescue_console.replay_page import render_replay_fragment, render_replay_page
from rescue_console.sources import load_replay, load_runs
from rescue_console.view import ConsoleError
from test_console_replay import robot, target
from test_console_view import ROBOTS, SECOND, T0, LedgerFixture, digest_dir
from test_console_web import CUSTOMER, HAS_FLASK, SECRET, AccountsFixture


def fields(fill):
    """40 x 40 칸, 0.05 m, 원점 (-1, -2): 픽스처 로봇 (0.5, -1.0)·(1.0, -1.0) 이 지도 안에 든다."""
    return dict(frame='map', width=40, height=40, resolution=0.05, origin=(-1.0, -2.0, 0.0), data=[fill] * 1600)


class ReplayMapFixture(LedgerFixture):
    """회차 1(0 ~ 80 s): 15 s·40 s 로봇 행, 25 s 인형, 20 s 실시간 지도, 50 s 고정 지도. 회차 2: 지도 없음."""

    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        self.archive_dir = self.dir / 'maps'
        self.now = T0
        with mock.patch('rescue_operations.site_state.time.time_ns', lambda: self.now):
            with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
                self.run1 = ledger.run_id
                archive = MapArchive(ledger, out_dir=self.archive_dir, robot_id='robot_a')

                def keep(fill, kind, revision, offset):
                    pgm = grid_to_pgm(40, 40, [fill] * 1600)
                    archive._store(self.run1, fields(fill), pgm, hashlib.sha256(pgm).hexdigest(), kind=kind,
                                   revision=revision, at_ns=T0 + offset * SECOND)
                for offset, write in ((15, lambda: ledger.record_robot_status(robot(1, 0.5, 15))),
                                      (20, lambda: keep(0, 'live', 0, 20)),
                                      (25, lambda: ledger.record_target_state(target(25))),
                                      (40, lambda: ledger.record_robot_status(robot(2, 1.0, 40))),
                                      (50, lambda: keep(-1, 'frozen', 2, 50))):
                    self.now = T0 + offset * SECOND
                    write()
                ledger.close_run(at_ns=T0 + 80 * SECOND)
                self.run2 = ledger.open_run(at_ns=T0 + 100 * SECOND)
                self.now = T0 + 110 * SECOND
                ledger.record_robot_status(robot(3, 1.0, 110))

    def timeline(self, run_id=None):
        runs, _default = run_choices(load_runs(self.site_db))
        run = next(r for r in runs if r['run_id'] == (run_id or self.run1))
        return build_timeline(load_replay(self.site_db, run['run_id']), run=run, robot_names={'robot_a': 'TB-6'})

    def drawn(self, t_ns, run_id=None):
        timeline = self.timeline(run_id)
        state = replay_at(timeline, t_ns)
        maps = SnapshotMaps(4)
        image, problem = None, None
        if state['map'] is not None:
            try:
                image = maps.load(state['map'])
            except ConsoleError as exc:
                problem = exc.detail
        return state, render_replay_fragment(state, timeline, map_image=image, map_problem=problem)


class ReplayMapModelTests(ReplayMapFixture):
    def test_the_run_maps_are_read_and_nothing_is_missing(self):
        timeline = self.timeline()
        self.assertEqual([('live', 0), ('frozen', 2)], [(m['kind'], m['revision']) for m in timeline['map_rows']])
        self.assertEqual([], timeline['missing'])

    def test_the_latest_map_at_or_before_the_moment(self):
        state = replay_at(self.timeline(), T0 + 30 * SECOND)
        self.assertEqual(('live', 10 * SECOND, False), (state['map']['kind'], state['map']['age_ns'],
                                                        state['map']['early']))
        state = replay_at(self.timeline(), T0 + 60 * SECOND)
        self.assertEqual(('frozen', 2), (state['map']['kind'], state['map']['revision']))

    def test_before_the_first_map_the_first_one_is_shown_and_marked_early(self):
        state = replay_at(self.timeline(), T0 + 5 * SECOND)
        self.assertEqual(('live', True, 15 * SECOND), (state['map']['kind'], state['map']['early'],
                                                       -state['map']['age_ns']))

    def test_a_run_without_maps_has_none(self):
        timeline = self.timeline(self.run2)
        self.assertEqual([], timeline['map_rows'])
        self.assertIsNone(replay_at(timeline, timeline['end_ns'])['map'])


class SnapshotMapsTests(ReplayMapFixture):
    def test_reads_the_yaml_beside_the_pgm_once(self):
        row = self.timeline()['map_rows'][0]
        maps = SnapshotMaps(4)
        image = maps.load(row)
        self.assertEqual((40, 40, 0.05, [-1.0, -2.0, 0.0]), (image.meta['width_px'], image.meta['height_px'],
                                                              image.meta['resolution'], image.meta['origin']))
        self.assertTrue(image.png.startswith(b'\x89PNG'))
        with mock.patch('rescue_console.mapimage.load_map') as again:
            self.assertIs(image, maps.load(row))
        again.assert_not_called()

    def test_a_changed_file_is_refused(self):
        row = self.timeline()['map_rows'][0]
        path = self.archive_dir / self.run1 / row['path'].rsplit('/', 1)[-1]
        path.write_bytes(path.read_bytes()[:-1] + b'\x00')
        with self.assertRaises(ConsoleError) as caught:
            SnapshotMaps(4).load(row)
        self.assertIn('sha256', caught.exception.detail)

    def test_a_missing_file_is_refused(self):
        row = dict(self.timeline()['map_rows'][0], path=str(self.dir / 'gone' / 'live-r0-000000000000.pgm'))
        with self.assertRaises(ConsoleError):
            SnapshotMaps(4).load(row)

    def test_the_cache_keeps_size_maps(self):
        rows = self.timeline()['map_rows']
        maps = SnapshotMaps(1)
        first = maps.load(rows[0])
        maps.load(rows[1])
        self.assertIsNot(first, maps.load(rows[0]))


class ReplayMapRenderTests(ReplayMapFixture):
    def test_map_with_robot_target_and_caption(self):
        _state, html = self.drawn(T0 + 30 * SECOND)
        self.assertIn('<svg class="plan"', html)
        self.assertIn(f'/replay/map.png?run={self.run1}&amp;snapshot=', html)
        self.assertIn('class="robot scout"', html)
        self.assertIn('TB-6', html)
        self.assertIn('class="victim not_rescued"', html)
        self.assertIn('실시간 SLAM 지도', html)
        self.assertIn('10초 전', html)
        self.assertNotIn('지도 이력 없음', html)

    def test_frozen_map_is_named(self):
        html = self.drawn(T0 + 60 * SECOND)[1]
        self.assertIn('고정 지도 r2', html)

    def test_early_moment_says_the_map_comes_later(self):
        html = self.drawn(T0 + 5 * SECOND)[1]
        self.assertIn('class="map-early"', html)
        self.assertIn('아직 지도 없음', html)
        self.assertIn('<svg class="plan"', html)

    def test_no_map_in_the_run_says_why(self):
        timeline = self.timeline(self.run2)
        html = self.drawn(timeline['end_ns'], self.run2)[1]
        self.assertIn('이 회차에는 남은 지도가 없습니다', html)
        self.assertNotIn('<svg class="plan"', html)

    def test_unreadable_file_is_shown_not_hidden(self):
        timeline = self.timeline()
        state = replay_at(timeline, T0 + 30 * SECOND)
        html = render_replay_fragment(state, timeline, map_image=None, map_problem='map file unreadable: <x>')
        self.assertIn('지도 파일을 읽지 못함', html)
        self.assertIn('&lt;x&gt;', html)

    def test_page_keeps_the_same_map_image_between_fragments(self):
        timeline = self.timeline()
        state = replay_at(timeline, timeline['end_ns'])
        page = render_replay_page(timeline, state, run_choices(load_runs(self.site_db))[0])
        self.assertIn('replaceWith', page)   # 같은 지도면 이미 받은 <image> 를 옮겨 깜박이지 않는다


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: replay map route NOT_RUN')
class ReplayMapWebTests(ReplayMapFixture, AccountsFixture):
    def client(self):
        from rescue_console.app import create_app
        app = create_app(None, accounts_path=self.accounts, secret_key=SECRET, site_state_path=self.site_db,
                         robot_names={'robot_a': 'TB-6'})
        client = app.test_client()
        self.login(client, *CUSTOMER)
        return client

    def test_fragment_draws_the_map_and_its_image_is_served(self):
        client = self.client()
        before = digest_dir(self.dir)
        part = client.get(f'/replay/fragment?run={self.run1}&offset_ns={30 * SECOND}').get_data(as_text=True)
        self.assertIn('<svg class="plan"', part)
        snapshot = self.timeline()['map_rows'][0]['snapshot_id']
        image = client.get(f'/replay/map.png?run={self.run1}&snapshot={snapshot}')
        self.assertEqual((200, 'image/png'), (image.status_code, image.mimetype))
        self.assertIn('immutable', image.headers['Cache-Control'])
        self.assertEqual(before, digest_dir(self.dir))

    def test_unknown_snapshot_is_refused(self):
        client = self.client()
        for query in (f'run={self.run1}&snapshot=nope', f'run={self.run2}&snapshot='
                      + self.timeline()['map_rows'][0]['snapshot_id']):
            with self.subTest(query=query):
                self.assertNotEqual(200, client.get('/replay/map.png?' + query).status_code)

    def test_map_image_needs_login(self):
        from rescue_console.app import create_app
        app = create_app(None, accounts_path=self.accounts, secret_key=SECRET, site_state_path=self.site_db)
        self.assertEqual(401, app.test_client().get(f'/replay/map.png?run={self.run1}&snapshot=x').status_code)


if __name__ == '__main__':
    unittest.main()
