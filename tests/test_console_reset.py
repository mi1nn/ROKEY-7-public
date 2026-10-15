"""Run reset page (/reset): one request to operations, its answer shown as is, and the restart checklist.

The scope is (나): TB-6 SLAM map and map freeze back to the start, past records kept per run (2026-10-15).
The console sends POST /api/runs/reset once (#323), never resends, and shows a refusal in red with the reason.
The checklist's automatic items come from operations' read_reset_checklist; the person items are kept in
the console's own accounts DB, tied to the run the reset opened, so the next reset starts empty.
"""
import json
import sqlite3
import unittest

from rescue_operations.site_state import EARLIER_RUN_NOTE, SiteStateLedger
from rescue_console.accounts import AccountStore
from rescue_console.commands import CommandSendError
from rescue_console.render import render_page
from rescue_console.replay import run_choices
from rescue_console.reset import (PERSON_ITEMS, REFUSAL_HINT, RESET_PATH, build_reset_request, checklist,
                                  reset_notice, robot_precheck, send_reset)
from rescue_console.sources import load_ledger, load_reset_state
from rescue_console.view import build_view
from test_console_web import ADMIN, CUSTOMER, HAS_FLASK, SECRET, AccountsFixture
from test_console_view import ROBOTS, SECOND, T0, LedgerFixture, robot_status, uid

NAMES = {'robot_a': 'TB-6', 'robot_b': 'TB-5'}
BASE = 'http://operations.invalid:8090'


def resting(robot, boot='boot-1', sequence=1, stamp_offset=20):
    status = robot_status(robot, stamp_offset=stamp_offset) | {'motion_state': 'STOPPED', 'active_task_id': None, 'generation': None, 'dock_state': 'DOCKED',
                                    'robot_boot_id': uid(robot + '/' + boot), 'state_sequence': sequence}
    status.pop('received_at_ns')
    return status


def reset_row(label, offset):
    return dict(command_id=uid('reset/' + label), kind='RESET_RUN', operator_id='admin', robot_id=None,
                target_id=None, segment_index=None, route_revision=None, requested_at_ns=T0 + offset * SECOND,
                accepted=True, result='ACCEPTED', reason=None)


class PrecheckTests(unittest.TestCase):
    def test_reasons_follow_the_operations_order(self):
        now = T0 + 10 * SECOND
        ok = {'received_at_ns': now, 'motion_state': 'STOPPED', 'active_task_id': None, 'dock_state': 'DOCKED'}
        cases = [
            ([('a', ok), ('b', ok)], None),
            ([('a', ok), ('b', None)], 'ROBOT_STATUS_STALE'),
            ([('a', ok | {'received_at_ns': now - 6 * SECOND}), ('b', ok)], 'ROBOT_STATUS_STALE'),
            ([('a', ok | {'motion_state': 'MOVING', 'dock_state': 'UNDOCKED'}), ('b', ok)], 'ROBOT_MOVING'),
            ([('a', ok | {'active_task_id': 't'}), ('b', ok | {'dock_state': 'UNKNOWN'})], 'TASK_ACTIVE'),
            ([('a', ok | {'dock_state': None}), ('b', ok)], 'ROBOT_NOT_DOCKED'),
        ]
        for robots, expected in cases:
            with self.subTest(expected=expected):
                rows, first = robot_precheck(robots, now)
                self.assertEqual(expected, first)
        rows, _first = robot_precheck([('a', None)], now)
        self.assertEqual(('ROBOT_STATUS_STALE', None), (rows[0]['reason'], rows[0]['age_ns']))


class SendTests(unittest.TestCase):
    payload = build_reset_request(operator_id='admin', command_id=uid('send'))

    def send(self, status, body):
        calls = []

        def transport(url, payload, timeout):
            calls.append((url, payload))
            return status, json.dumps(body).encode() if isinstance(body, dict) else body
        result = send_reset(BASE, self.payload, transport=transport)
        self.assertEqual([(BASE + RESET_PATH, self.payload)], calls)   # once, never again
        return result

    def test_accepted_answer_carries_the_new_run(self):
        result = self.send(200, {'accepted': True, 'request_id': uid('send'), 'reason': None,
                                 'request_state': 'ACCEPTED', 'created_task_id': None, 'run_id': uid('new')})
        self.assertEqual((True, True, uid('new')), (result['sent'], result['accepted'], result['run_id']))
        self.assertEqual('success', reset_notice(result)[0])

    def test_refusal_is_red_with_reason_word_code_and_the_hint(self):
        result = self.send(200, {'accepted': False, 'request_id': uid('send'), 'reason': 'ROBOT_STATUS_STALE',
                                 'request_state': 'REJECTED', 'created_task_id': None, 'run_id': None})
        category, text = reset_notice(result)
        self.assertEqual('danger', category)
        self.assertIn('ROBOT_STATUS_STALE', text)
        self.assertIn('로봇이 꺼져 있음', text)
        self.assertIn(REFUSAL_HINT, text)

    def test_db_failure_503_is_unknown_not_refused(self):
        result = self.send(503, {'error': 'STORAGE_ERROR', 'request_id': uid('send'), 'detail': 'x'})
        self.assertEqual((False, 'UNKNOWN'), (result['sent'], result['delivery']))
        category, text = reset_notice(result)
        self.assertEqual('warning', category)
        self.assertIn('결과 모름', text)

    def test_bad_run_id_is_a_bad_response(self):
        result = self.send(200, {'accepted': True, 'request_id': uid('send'), 'reason': None,
                                 'request_state': 'ACCEPTED', 'created_task_id': None, 'run_id': ''})
        self.assertEqual((False, 'BAD_RESPONSE'), (result['sent'], result['error']))

    def test_unreachable_is_not_sent(self):
        def transport(url, payload, timeout):
            raise CommandSendError('UNREACHABLE', 'refused', delivery='NOT_SENT')
        result = send_reset(BASE, self.payload, transport=transport)
        self.assertEqual('danger', reset_notice(result)[0])

    def test_note_only_when_given(self):
        self.assertNotIn('note', build_reset_request(operator_id='a', command_id='c', note=''))
        self.assertEqual('x', build_reset_request(operator_id='a', command_id='c', note='x')['note'])


class ResetFixture(LedgerFixture):
    """Two resting robots in run 1; ``reset()`` makes operations open the next run (as RunReset would)."""

    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            for robot in ROBOTS:
                ledger.record_robot_status(resting(robot))

    def reset(self, label, offset):
        with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            return ledger.reset_run(at_ns=T0 + offset * SECOND, command=reset_row(label, offset))[1]

    def restart(self, robot):
        with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            ledger.record_robot_status(resting(robot, boot='boot-2', stamp_offset=30))


class ChecklistTests(ResetFixture):
    def model(self, person=None):
        return checklist(load_reset_state(self.site_db)['checklist'], person or {}, NAMES)

    def test_no_checklist_before_any_reset(self):
        self.assertIsNone(self.model())

    def test_steps_follow_the_operations_order_and_automatic_items_fill_in(self):
        self.reset('one', 10)
        model = self.model()
        self.assertEqual([1, 2, 3, 4, 5, 6], [step['number'] for step in model['steps']])
        self.assertEqual((1, 9), (model['done'], model['total']))   # only the reset answer so far
        self.restart('robot_a')   # TB-6's executor restarted
        items = {row['key']: row for step in self.model()['steps'] for row in step['items']}
        self.assertTrue(items['TB-6']['done'])
        self.assertFalse(items['TB-5']['done'])
        self.assertFalse(items['coordinator_up']['done'])

    def test_person_items_show_who_checked(self):
        self.reset('one', 10)
        items = {row['key']: row for step in self.model({'robots_down': {'checked': True, 'by_username': 'admin',
                                                                          'at_ns': T0}})['steps']
                 for row in step['items']}
        self.assertEqual((True, 'admin'), (items['robots_down']['done'], items['robots_down']['by_username']))

    def test_robot_without_that_display_name_is_unknown_not_checked(self):
        self.reset('one', 10)
        model = checklist(load_reset_state(self.site_db)['checklist'], {}, {})
        items = {row['key']: row for step in model['steps'] for row in step['items']}
        self.assertEqual((True, False), (items['TB-6']['unknown'], items['TB-6']['done']))


class AccountsUpgradeTests(LedgerFixture):
    def test_a_1_0_accounts_file_gains_the_checks_table_and_keeps_its_accounts(self):
        path = self.dir / 'accounts.sqlite3'
        AccountStore.create(path, *ADMIN)
        with sqlite3.connect(path) as db:   # make it the 1.0 file the field PC has today
            db.execute('DROP TABLE reset_checks')
            db.execute("UPDATE metadata SET value = '1.0' WHERE key = 'schema_version'")
        store = AccountStore.open(path)
        self.assertEqual(['admin'], [a['username'] for a in store.list_accounts()])
        with sqlite3.connect(path) as db:
            self.assertEqual('1.1', db.execute("SELECT value FROM metadata WHERE key = 'schema_version'").fetchone()[0])
        store.set_reset_check(uid('run'), 'robots_down', True, by_username='admin', at_ns=T0)
        self.assertEqual({'robots_down'}, set(store.reset_checks(uid('run'))))
        self.assertEqual({}, store.reset_checks(uid('other-run')))


class RunNumberTests(unittest.TestCase):
    def test_earlier_records_run_has_no_number(self):
        runs = {'open_run_id': uid('b'), 'runs': [
            {'run_id': uid('e'), 'started_at_ns': T0, 'ended_at_ns': T0 + 1, 'note': EARLIER_RUN_NOTE},
            {'run_id': uid('a'), 'started_at_ns': T0 + 2, 'ended_at_ns': T0 + 3, 'note': None},
            {'run_id': uid('b'), 'started_at_ns': T0 + 4, 'ended_at_ns': None, 'note': None}]}
        rows, default = run_choices(runs)
        self.assertEqual([None, 1, 2], [row['number'] for row in rows])
        self.assertEqual(uid('b'), default)


class MastheadTests(LedgerFixture):
    def test_reset_link_only_for_admin(self):
        view = build_view(load_ledger(self.db), {}, built_at_ns=T0)
        admin = {'display_name': 'a', 'role': 'ADMIN'}
        customer = {'display_name': 'c', 'role': 'CUSTOMER'}
        self.assertIn('href="/reset"', render_page(view, admin, csrf='x'))
        self.assertNotIn('href="/reset"', render_page(view, customer, csrf='x'))


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: reset routes NOT_RUN')
class ResetWebTests(ResetFixture, AccountsFixture):
    def client(self, answer=None, operations=True):
        from rescue_console.app import create_app
        self.sent = []

        def transport(url, payload, timeout):
            self.sent.append((url, payload))
            return 200, json.dumps(answer).encode()
        app = create_app(None, accounts_path=self.accounts, secret_key=SECRET, site_state_path=self.site_db,
                         robot_names=NAMES, operations_url=BASE if operations else None, command_transport=transport)
        return app.test_client()

    def post(self, client, path, **form):
        return client.post(path, data={'csrf': self.csrf(client, '/reset'), **form})

    def test_customer_sees_the_page_but_cannot_send_or_check(self):
        client = self.client()
        self.login(client, *CUSTOMER)
        page = client.get('/reset').get_data(as_text=True)
        self.assertIn('관리자만 보낼 수 있습니다', page)
        self.assertNotIn('action="/reset/send"', page)
        self.assertEqual(403, self.post(client, '/reset/send', confirm='yes').status_code)
        self.assertEqual(403, self.post(client, '/reset/check', item='robots_down', checked='1').status_code)
        self.assertEqual([], self.sent)

    def test_without_the_confirm_box_nothing_is_sent(self):
        client = self.client()
        self.login(client, *ADMIN)
        page = self.post(client, '/reset/send', note='x')
        self.assertEqual(302, page.status_code)
        self.assertEqual([], self.sent)
        self.assertIn('확인 칸을 체크해야', client.get('/reset').get_data(as_text=True))

    def test_confirmed_send_goes_once_and_the_refusal_shows_red(self):
        client = self.client({'accepted': False, 'request_id': uid('x'), 'reason': 'ROBOT_NOT_DOCKED',
                              'request_state': 'REJECTED', 'created_task_id': None, 'run_id': None})
        self.login(client, *ADMIN)
        site_before = self.site_db.read_bytes()
        self.post(client, '/reset/send', confirm='yes', note='  다시 하기  ')
        self.assertEqual(1, len(self.sent))
        url, payload = self.sent[0]
        self.assertEqual(BASE + RESET_PATH, url)
        self.assertEqual({'command_id', 'operator_id', 'note'}, set(payload))
        self.assertEqual(('admin', '다시 하기'), (payload['operator_id'], payload['note']))
        page = client.get('/reset').get_data(as_text=True)
        self.assertIn('class="flash reset-danger"', page)
        self.assertIn('ROBOT_NOT_DOCKED', page)
        self.assertEqual(site_before, self.site_db.read_bytes())   # the console never writes the site DB

    def test_no_operations_link_means_no_send(self):
        client = self.client(operations=False)
        self.login(client, *ADMIN)
        self.assertIn('명령 연결 없음', client.get('/reset').get_data(as_text=True))
        self.post(client, '/reset/send', confirm='yes')
        self.assertEqual([], self.sent)

    def test_person_checks_belong_to_the_run_the_reset_opened(self):
        client = self.client()
        self.login(client, *ADMIN)
        self.post(client, '/reset/check', item='robots_down', checked='1')   # before any reset: refused
        self.assertEqual({}, AccountStore.open(self.accounts).reset_checks(load_reset_state(self.site_db)['checklist']['run_id']))
        run2 = self.reset('one', 10)
        self.post(client, '/reset/check', item='robots_down', checked='1')
        self.assertEqual({'robots_down'}, set(AccountStore.open(self.accounts).reset_checks(run2)))
        part = client.get('/reset/fragment').get_data(as_text=True)
        self.assertIn('(2/9)', part)
        self.reset('two', 20)   # the next reset starts with nothing checked
        self.assertIn('(1/9)', client.get('/reset/fragment').get_data(as_text=True))

    def test_unknown_item_is_refused(self):
        client = self.client()
        self.login(client, *ADMIN)
        self.reset('one', 10)
        self.assertEqual(503, self.post(client, '/reset/check', item='drop_table', checked='1').status_code)
        self.assertEqual(503, self.post(client, '/reset/check', item='robots_down', checked='yes').status_code)
        self.assertTrue(PERSON_ITEMS)

    def test_reset_routes_need_site_state(self):
        from rescue_console.app import create_app
        app = create_app(self.db, accounts_path=self.accounts, secret_key=SECRET)
        client = app.test_client()
        self.login(client, *ADMIN)
        self.assertEqual(503, client.get('/reset').status_code)
        self.assertEqual(503, client.get('/reset/fragment').status_code)
        self.assertEqual(401, app.test_client().get('/reset/fragment').status_code)


if __name__ == '__main__':
    unittest.main()
