"""리플레이 회차 목록에 옆 폴더 DB 의 회차도 넣음(재범 2026-10-16 「몇번을 했는데 이거 하나밖에 안뜨냐」).

시뮬(scripts/sim/run_full.sh: ~/sim-runs/full-<시각>/site_state.sqlite3)과 현장(런북: ~/rokey7-field/noweb/<순번>/)
모두 launch 를 띄울 때마다 새 폴더에 현장 상태 DB 를 새로 만든다. 웹은 --site-state 파일 하나만 읽어서 리플레이에
지금 회차 하나만 보였다. 이제 --site-state 폴더의 위 폴더(또는 --replay-dir) 아래 한 단계 폴더에 있는 같은 이름
DB 의 회차도 시작 시각 순으로 함께 보인다. 모두 읽기만 한다. 못 읽은 파일은 건너뛰고 그 사실을 적는다. 지금 DB 가
아닌 파일의 열린 회차는 「끝 기록 없음」이다(진행 중이 아님).
"""
import unittest
from unittest import mock

from rescue_contracts import SiteEvent
from rescue_operations.site_state import SiteStateLedger, list_runs
from test_console_view import ROBOTS, SECOND, T0, digest_dir, site_event
from test_console_web import CUSTOMER, HAS_FLASK, SECRET, AccountsFixture

DB = 'site_state.sqlite3'


def write_run(path, start, event, *, close=None):
    """``path`` 에 회차 하나(start 초에 열림)와 사건 하나. close 초가 있으면 닫는다."""
    path.parent.mkdir(parents=True)
    now = [T0 + start * SECOND]
    with mock.patch('rescue_operations.site_state.time.time_ns', lambda: now[0]), \
            SiteStateLedger(path, allowed_robot_ids=ROBOTS) as ledger:
        run = ledger.run_id
        now[0] += 10 * SECOND
        ledger.record_site_event(SiteEvent(**dict(site_event(event, kind='DISASTER_SIGN'),
                                                  observed_at_ns=now[0])))
        if close is not None:
            ledger.close_run(at_ns=T0 + close * SECOND)
    return run


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: replay run list NOT_RUN')
class AllRunsTests(AccountsFixture):
    def setUp(self):
        super().setUp()
        self.root = self.dir / 'runs'
        self.old = write_run(self.root / 'full-a' / DB, 0, 'a', close=80)
        self.dead = write_run(self.root / 'full-b' / DB, 100, 'b')          # 끝나지 않은 채 남은 지난 launch
        self.live = write_run(self.root / 'full-c' / DB, 200, 'c')          # 지금 웹이 붙은 launch
        (self.root / 'full-x').mkdir()
        (self.root / 'full-x' / DB).write_bytes(b'not a database')
        (self.root / 'web-state').mkdir()                                    # DB 없는 폴더는 그냥 지나감

    def client(self, **options):
        from rescue_console.app import create_app
        app = create_app(None, accounts_path=self.accounts, secret_key=SECRET,
                         site_state_path=self.root / 'full-c' / DB, **options)
        client = app.test_client()
        self.login(client, *CUSTOMER)
        return client

    def options(self, html):
        import re
        return re.findall(r'<option value="([^"]+)"( selected)?>([^<]+)</option>', html)

    def test_runs_of_every_folder_are_listed_oldest_first_and_the_live_one_is_chosen(self):
        html = self.client().get('/replay').get_data(as_text=True)
        options = self.options(html)
        self.assertEqual([self.old, self.dead, self.live], [value for value, _sel, _text in options])
        self.assertEqual([self.live], [value for value, selected, _text in options if selected])
        words = [text for _value, _sel, text in options]
        self.assertTrue(words[0].startswith('회차 1'))
        self.assertTrue(words[2].startswith('회차 3'))
        self.assertIn('끝 기록 없음', words[1])
        self.assertNotIn('진행 중', words[1])
        self.assertIn('진행 중', words[2])
        self.assertIn('full-a', words[0])                                    # 어느 폴더의 회차인지

    def test_an_older_run_is_read_from_its_own_file(self):
        client = self.client()
        part = client.get(f'/replay/fragment?run={self.old}&offset_ns={20 * SECOND}').get_data(as_text=True)
        self.assertIn('이 시점까지 1건', part)
        self.assertEqual(200, client.get(f'/replay?run={self.dead}').status_code)

    def test_an_unreadable_file_is_named_and_skipped(self):
        html = self.client().get('/replay').get_data(as_text=True)
        self.assertIn('읽지 못한 기록 파일 1개', html)
        self.assertIn('full-x', html)

    def test_replay_dir_chooses_where_to_look(self):
        empty = self.dir / 'elsewhere'
        empty.mkdir()
        html = self.client(replay_dir=empty).get('/replay').get_data(as_text=True)
        self.assertEqual([self.live], [value for value, _sel, _text in self.options(html)])

    def test_nothing_is_written_and_other_files_are_listed_once(self):
        before = {name: digest_dir(self.root / name) for name in ('full-a', 'full-b', 'full-c', 'full-x')}
        client = self.client()
        with mock.patch('rescue_console.sources.list_runs', side_effect=list_runs) as reads:
            for _ in range(3):
                client.get('/replay')
        read = sorted(call.args[0].parent.name for call in reads.call_args_list)
        self.assertEqual(['full-a', 'full-b', 'full-c', 'full-c', 'full-c', 'full-x'], read)   # 지금 DB 만 매번
        self.assertEqual(before, {name: digest_dir(self.root / name) for name in before})


    def test_replay_dir_needs_site_state_on_the_command_line(self):
        import contextlib
        import io
        import os
        from rescue_console.__main__ import main
        os.environ['RESCUE_CONSOLE_TEST_SECRET'] = 'x' * 16
        self.addCleanup(os.environ.pop, 'RESCUE_CONSOLE_TEST_SECRET', None)
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            code = main([str(self.db), '--accounts', str(self.accounts), '--status', 'status.json',
                         '--replay-dir', str(self.root), '--secret-env', 'RESCUE_CONSOLE_TEST_SECRET'])
        self.assertEqual(2, code)
        self.assertIn('--replay-dir needs --site-state', err.getvalue())


if __name__ == '__main__':
    unittest.main()
