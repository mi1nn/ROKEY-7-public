"""Operator commands: the console sends the five enabled kinds to operations once, shows the answer or
the failure as it came, and only ADMIN sends. Buttons mirror the coordinator gate; they are not the gate."""
import html
import http.server
import json
import re
import socket
import threading
import time
import unittest

from rescue_operations.site_state import SiteStateLedger
from rescue_console.commands import (COMMAND_FIELDS, COMMAND_PATH, COMMAND_TIMEOUT_S, SENDABLE, CommandSendError,
                                     build_request, command_access, parse_operations_url, post_json, read_reply,
                                     send_command)
from rescue_console.render import render_fragment, render_page
from rescue_console.sources import load_ledger
from rescue_console.view import COMMANDS, ConsoleError, build_view
from test_console_view import (ROBOTS, SECOND, T0, LedgerFixture, digest_dir, target_state, uid,
                               write_site_state)
from test_console_web import ADMIN, CUSTOMER, HAS_FLASK, SECRET, AccountsFixture

HELD = ('PAUSE', 'RESUME', 'CANCEL', 'STOP_ROBOT')
COMMAND_ID = uid('command-1')
OPS = 'http://ops.invalid:8000'
NO_DISPATCH = {'target_id': None, 'target_revision': None, 'route_revision': None, 'segment_index': None,
               'segment_count': None, 'segment_phase': None}


def mission_state(mode='DISASTER', phase='DISASTER', *, dispatch=None, reason=None, updated=25):
    """One site_state mission row (read_mission_state()['mission'] shape, schema 1.2)."""
    return {'mode': mode, 'phase': phase, 'trigger_event_id': uid('quake'), 'started_at_ns': T0,
            'generation': None, 'updated_at_ns': T0 + updated * SECOND, **NO_DISPATCH, **(dispatch or {}),
            'reason': reason}


def dispatch(segment_phase='AWAITING_APPROVAL', segment_index=1):
    return {'target_id': uid('waiting'), 'target_revision': 4, 'route_revision': 2, 'segment_index': segment_index,
            'segment_count': 3, 'segment_phase': segment_phase}


def answer(accepted=True, reason=None, **extra):
    return json.dumps({'accepted': accepted, 'request_id': COMMAND_ID, 'reason': reason, **extra}).encode()


class CommandRequestTests(unittest.TestCase):
    FORMS = {
        'START_SEARCH': {},
        'DISPATCH_SUPPORT': {'target_id': 't-1', 'target_revision': '4'},
        'CONTINUE_SEGMENT': {'target_id': 't-1', 'target_revision': '4', 'route_revision': '2', 'segment_index': '1'},
        'REQUEST_RECHECK': {'target_id': 't-1'},
        'RETURN_HOME': {},
    }

    def build(self, kind, **form):
        return build_request({'csrf': 'token', 'kind': kind, **form}, operator_id='admin', command_id=COMMAND_ID)

    def test_each_kind_carries_exactly_the_fields_the_coordinator_reads(self):
        self.assertEqual(set(self.FORMS), set(SENDABLE))
        for kind, form in self.FORMS.items():
            with self.subTest(kind=kind):
                payload = self.build(kind, **form)
                self.assertEqual(set(payload), {'command_id', 'kind', 'operator_id'} | set(COMMAND_FIELDS[kind]))
                self.assertEqual((payload['command_id'], payload['kind'], payload['operator_id']),
                                 (COMMAND_ID, kind, 'admin'))
        self.assertEqual(self.build('CONTINUE_SEGMENT', **self.FORMS['CONTINUE_SEGMENT']),
                         {'command_id': COMMAND_ID, 'kind': 'CONTINUE_SEGMENT', 'operator_id': 'admin',
                          'target_id': 't-1', 'target_revision': 4, 'route_revision': 2, 'segment_index': 1})

    def test_fields_match_the_coordinator_gate(self):
        from rescue_coordination.mission import COMMAND_FIELDS as GATE
        self.assertEqual({kind: set(fields) for kind, fields in COMMAND_FIELDS.items()},
                         {kind: set(fields) for kind, fields in GATE.items()})

    def test_every_value_the_operator_saw_must_be_there(self):
        # An unknown revision or segment is never sent as null for operations to fill (작전 #115).
        for kind, name in (('DISPATCH_SUPPORT', 'target_revision'), ('CONTINUE_SEGMENT', 'target_revision'),
                           ('CONTINUE_SEGMENT', 'route_revision'), ('CONTINUE_SEGMENT', 'segment_index')):
            with self.subTest(kind=kind, field=name), self.assertRaises(ConsoleError):
                self.build(kind, **dict(self.FORMS[kind], **{name: ''}))

    def test_held_and_unknown_kinds_are_refused_before_anything_is_sent(self):
        for kind in HELD + ('FINISH_MISSION', 'RESET_STOP', 'SET_CAMERA_WATCH', '', 'start_search'):
            with self.subTest(kind=kind), self.assertRaises(ConsoleError) as caught:
                self.build(kind)
            self.assertEqual(caught.exception.detail_code, 'INVALID_INPUT')

    def test_bad_or_extra_values_are_refused(self):
        cases = [
            ('REQUEST_RECHECK', {}),                                          # target missing
            ('REQUEST_RECHECK', {'target_id': 'two words'}),
            ('REQUEST_RECHECK', {'target_id': 'x' * 129}),
            ('CONTINUE_SEGMENT', dict(self.FORMS['CONTINUE_SEGMENT'], segment_index='-1')),
            ('CONTINUE_SEGMENT', dict(self.FORMS['CONTINUE_SEGMENT'], route_revision='1.5')),
            ('CONTINUE_SEGMENT', dict(self.FORMS['CONTINUE_SEGMENT'], target_revision='abc')),
            ('START_SEARCH', {'target_id': 't-1'}),                           # a field the kind does not use
            ('START_SEARCH', {'robot_id': 'robot_a'}),
            ('START_SEARCH', {'operator_id': 'someone-else'}),                # who sends comes from the login
        ]
        for kind, form in cases:
            with self.subTest(kind=kind, form=form), self.assertRaises(ConsoleError) as caught:
                self.build(kind, **form)
            self.assertEqual(caught.exception.detail_code, 'INVALID_INPUT')

    def test_operations_url_is_http_only_and_has_no_default(self):
        self.assertEqual(parse_operations_url('http://ops.invalid:8000/'), OPS)
        self.assertEqual(parse_operations_url('https://ops.invalid/base'), 'https://ops.invalid/base')
        for bad in ('', 'ops.invalid:8000', 'ftp://ops.invalid', 'http://', 'http://ops.invalid/?x=1', None):
            with self.subTest(url=bad), self.assertRaises(ConsoleError):
                parse_operations_url(bad)


class ReplyTests(unittest.TestCase):
    def test_answer_is_passed_on_as_given(self):
        self.assertEqual(read_reply(200, answer()),
                         {'accepted': True, 'request_id': COMMAND_ID, 'reason': None, 'request_state': None,
                          'created_task_id': None})
        extra = read_reply(200, answer(True, None, request_state='DUPLICATE', created_task_id='task-9'))
        self.assertEqual((extra['request_state'], extra['created_task_id']), ('DUPLICATE', 'task-9'))

    def test_an_error_status_that_carries_an_answer_is_still_an_answer(self):
        reply = read_reply(409, answer(False, 'INVALID_INPUT'))
        self.assertEqual((reply['accepted'], reply['reason']), (False, 'INVALID_INPUT'))

    def test_what_is_not_an_answer_is_a_failure_not_a_rejection(self):
        cases = [
            (200, answer(False, None), 'BAD_RESPONSE'),                      # a rejection needs its reason
            (500, answer(True), 'BAD_RESPONSE'),                             # "accepted" with an error status
            (200, b'<html>login</html>', 'BAD_RESPONSE'),
            (200, b'{"accepted": "yes", "request_id": "x", "reason": null}', 'BAD_RESPONSE'),
            (200, b'[]', 'BAD_RESPONSE'),
            (502, b'Bad gateway', 'HTTP_ERROR'),
        ]
        for status, body, code in cases:
            with self.subTest(status=status, body=body), self.assertRaises(CommandSendError) as caught:
                read_reply(status, body)
            self.assertEqual(caught.exception.code, code)
            self.assertEqual(caught.exception.delivery, 'UNKNOWN')   # it reached operations; the outcome is not known


class SendTests(unittest.TestCase):
    PAYLOAD = {'command_id': COMMAND_ID, 'kind': 'START_SEARCH', 'operator_id': 'admin'}

    def test_one_post_and_the_answer_comes_back(self):
        calls = []

        def transport(url, payload, timeout):
            calls.append((url, payload, timeout))
            return 200, answer(False, 'NAV_NOT_READY')
        result = send_command(OPS, self.PAYLOAD, transport=transport)
        self.assertEqual(calls, [(OPS + COMMAND_PATH, self.PAYLOAD, COMMAND_TIMEOUT_S)])
        self.assertEqual(COMMAND_PATH, '/api/operator-commands')
        self.assertTrue(result['sent'])
        self.assertEqual({k: result[k] for k in ('command_id', 'kind', 'accepted', 'request_id', 'reason')},
                         {'command_id': COMMAND_ID, 'kind': 'START_SEARCH', 'accepted': False,
                          'request_id': COMMAND_ID, 'reason': 'NAV_NOT_READY'})
        self.assertTrue(result['reason_label'])        # Korean gloss next to the code, the code is kept

    def test_a_failure_is_reported_and_never_retried(self):
        for code, delivery in (('UNREACHABLE', 'NOT_SENT'), ('TIMEOUT', 'UNKNOWN'), ('HTTP_ERROR', 'UNKNOWN')):
            calls = []

            def transport(url, payload, timeout):
                calls.append(payload)
                raise CommandSendError(code, 'detail', delivery=delivery)
            with self.subTest(code=code):
                result = send_command(OPS, self.PAYLOAD, transport=transport)
                self.assertEqual(len(calls), 1)
                self.assertEqual(result, {'sent': False, 'command_id': COMMAND_ID, 'kind': 'START_SEARCH',
                                          'error': code, 'message': 'detail', 'delivery': delivery})


class FakeOperations(http.server.BaseHTTPRequestHandler):
    """Stands in for operations' POST /api/operator-commands; the test sets `behave`."""
    requests, behave = [], None

    def do_POST(self):
        body = self.rfile.read(int(self.headers['Content-Length']))
        type(self).requests.append((self.path, self.headers['Content-Type'], json.loads(body)))
        status, reply, delay = type(self).behave(json.loads(body))
        time.sleep(delay)
        try:
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(reply)))
            self.end_headers()
            self.wfile.write(reply)
        except OSError:
            pass                                  # the client gave up first (timeout test)

    def log_message(self, *args):
        pass


class FakeOperationsTests(unittest.TestCase):
    PAYLOAD = {'command_id': COMMAND_ID, 'kind': 'RETURN_HOME', 'operator_id': 'admin'}

    def setUp(self):
        FakeOperations.requests = []
        self.server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), FakeOperations)
        self.server.daemon_threads = True
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.url = f'http://127.0.0.1:{self.server.server_address[1]}' + COMMAND_PATH

    def test_one_json_post_and_the_answer_is_read(self):
        FakeOperations.behave = lambda body: (200, answer(), 0)
        status, body = post_json(self.url, self.PAYLOAD, 2.0)
        self.assertEqual((status, json.loads(body)['accepted']), (200, True))
        self.assertEqual(FakeOperations.requests, [(COMMAND_PATH, 'application/json', self.PAYLOAD)])

    def test_an_error_status_body_is_still_read(self):
        FakeOperations.behave = lambda body: (409, answer(False, 'TASK_BUSY'), 0)
        status, body = post_json(self.url, self.PAYLOAD, 2.0)
        self.assertEqual((status, read_reply(status, body)['reason']), (409, 'TASK_BUSY'))

    def test_slow_operations_times_out_once_and_delivery_is_unknown(self):
        FakeOperations.behave = lambda body: (200, answer(), 1.0)
        with self.assertRaises(CommandSendError) as caught:
            post_json(self.url, self.PAYLOAD, 0.2)
        self.assertEqual((caught.exception.code, caught.exception.delivery), ('TIMEOUT', 'UNKNOWN'))
        time.sleep(0.2)
        self.assertEqual(len(FakeOperations.requests), 1)

    def test_nothing_listening_is_unreachable_and_not_sent(self):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        with self.assertRaises(CommandSendError) as caught:
            post_json(f'http://127.0.0.1:{port}' + COMMAND_PATH, self.PAYLOAD, 1.0)
        self.assertEqual((caught.exception.code, caught.exception.delivery), ('UNREACHABLE', 'NOT_SENT'))


class GateTests(LedgerFixture):
    """console-and-db-plan 4절: the buttons mirror the coordinator gate (rescue_coordination.mission)."""

    def gates(self, mission=None, *, targets=None, ledger_reports=True):
        status = dict(self.status, mission_state={'schema_version': '1.2', 'mission': mission})
        if targets is not None:
            status['target_states'] = {'schema_version': '1.2', 'targets': targets}
        ledger = load_ledger(self.db)
        if not ledger_reports:
            ledger = dict(ledger, reports=[])
        view = build_view(ledger, status, built_at_ns=T0 + 30 * SECOND)
        return {command['name']: command for command in view['commands']}

    def assertOff(self, command, words):
        self.assertFalse(command['enabled'], command['name'])
        self.assertIn(words, command['reason'])

    def test_panel_lists_every_command_with_the_flow_gates_first(self):
        commands = self.view()['commands']
        self.assertEqual([c['name'] for c in commands], [name for name, _ in COMMANDS])
        self.assertEqual([c['name'] for c in commands if c['group'] == 'approve'],
                         ['START_SEARCH', 'DISPATCH_SUPPORT', 'CONTINUE_SEGMENT'])
        self.assertEqual([c['name'] for c in commands if c['group'] == 'stop'], ['STOP_ROBOT'])

    def test_without_a_mission_row_nothing_is_enabled(self):
        for gates in ({c['name']: c for c in self.view()['commands']}, self.gates(None)):
            for name in SENDABLE:
                self.assertOff(gates[name], '임무 상태')

    def test_outside_disaster_mode_nothing_is_enabled(self):
        for mode, phase in (('NORMAL', 'IDLE'), ('CLEARING', 'IDLE'), ('MISSION_FINISHED', 'FINISHED')):
            gates = self.gates(mission_state(mode, phase))
            for name in SENDABLE:
                with self.subTest(mode=mode, command=name):
                    self.assertOff(gates[name], '재난 모드 아님')

    def test_start_search_only_before_the_search(self):
        start = self.gates(mission_state('DISASTER', 'DISASTER'))['START_SEARCH']
        self.assertTrue(start['enabled'])
        self.assertEqual(start['args'], {})
        self.assertOff(self.gates(mission_state('DISASTER', 'SEARCHING'))['START_SEARCH'], '이미 수색')

    def with_revisions(self, saved=(), **revisions):
        """Target states with the revision the operator approves; the `saved` ones also carry the report the
        coordinator holds a PERSISTED receipt for (#118 선택 키 persisted_report_id)."""
        rows = []
        for label, state in zip(('unconfirmed', 'waiting', 'supporting', 'rescued'),
                                self.status['target_states']['targets']):
            if label in revisions:
                state = dict(state, target_revision=revisions[label])
            if label in saved:
                state = dict(state, persisted_report_id=uid(f'saved-{label}'))
            rows.append(state)
        return rows

    def test_dispatch_needs_a_confirmed_target_with_its_revision_and_no_dispatch_yet(self):
        targets = self.with_revisions(saved=('waiting', 'supporting', 'rescued'), waiting=4, supporting=7, rescued=2)
        dispatch_gate = self.gates(mission_state('DISASTER', 'TARGET_FOUND'), targets=targets)['DISPATCH_SUPPORT']
        self.assertTrue(dispatch_gate['enabled'])
        # Confirmed, not rescued and with the revision the operator approves. The ledger-only target has no
        # revision on screen, so it is not offered (작전 #115: unknown -> off, never null).
        self.assertEqual({c['target_id']: c['target_revision'] for c in dispatch_gate['choices']},
                         {uid('waiting'): 4, uid('supporting'): 7})
        self.assertOff(self.gates(mission_state('DISASTER', 'TARGET_FOUND'))['DISPATCH_SUPPORT'],
                       '대상 revision 없음')
        self.assertOff(self.gates(mission_state('DISASTER', 'SEGMENT_MOVING', dispatch=dispatch('EXECUTING')),
                                  targets=targets)['DISPATCH_SUPPORT'], '이미 출동')
        lone = self.gates(mission_state('DISASTER', 'SEARCHING'), targets=[target_state('unconfirmed', confirmed=False)],
                          ledger_reports=False)
        self.assertOff(lone['DISPATCH_SUPPORT'], '확정 대상 없음')

    def test_dispatch_waits_for_the_report_receipt(self):
        # mission.py _dispatch_support: confirmed AND report_persisted, else TARGET_NOT_CONFIRMED. The thin
        # TargetSnapshot marks every target CONFIRMED, so "확인됨" alone would offer a press that is refused.
        found = mission_state('DISASTER', 'TARGET_FOUND')
        none_saved = self.gates(found, targets=self.with_revisions(waiting=4, supporting=7))['DISPATCH_SUPPORT']
        self.assertOff(none_saved, '보고 저장 대기')
        self.assertEqual(none_saved['choices'], [])
        one_saved = self.gates(found, targets=self.with_revisions(saved=('supporting',), waiting=4, supporting=7))
        gate = one_saved['DISPATCH_SUPPORT']
        self.assertTrue(gate['enabled'])
        self.assertEqual([c['target_id'] for c in gate['choices']], [uid('supporting')])
        self.assertIn('보고 저장 대기 1명', gate['reason'])

    def test_a_row_without_the_receipt_key_is_not_guessed_from_the_ledger(self):
        # Rows written before #118 have no persisted_report_id. The ledger receipt is not the coordinator's:
        # no silent fallback, the button stays off (PR검토보조 2026-10-12, 소유자 지시 전달).
        view = build_view(load_ledger(self.db), dict(self.status, mission_state={
            'schema_version': '1.2', 'mission': mission_state('DISASTER', 'TARGET_FOUND')},
            target_states={'schema_version': '1.2', 'targets': self.with_revisions(waiting=4)}),
            built_at_ns=T0 + 30 * SECOND)
        waiting = next(t for t in view['targets'] if t['target_id'] == uid('waiting'))
        self.assertEqual(waiting['receipt_status'], 'PERSISTED')
        self.assertIsNone(waiting['persisted_report_id'])
        gate = next(c for c in view['commands'] if c['name'] == 'DISPATCH_SUPPORT')
        self.assertOff(gate, '보고 저장 대기')

    def test_malformed_revision_or_receipt_fails_the_poll(self):
        found = mission_state('DISASTER', 'TARGET_FOUND')
        for field, value in (('target_revision', -1), ('target_revision', '4'), ('target_revision', True),
                             ('persisted_report_id', ''), ('persisted_report_id', 7)):
            with self.subTest(field=field, value=value):
                targets = [dict(state, **{field: value}) for state in self.with_revisions(waiting=4)]
                with self.assertRaises(ConsoleError):
                    self.gates(found, targets=targets)

    def test_continue_only_while_awaiting_approval_and_carries_what_the_screen_shows(self):
        gate = self.gates(mission_state('DISASTER', 'AWAITING_CONTINUE', dispatch=dispatch()))['CONTINUE_SEGMENT']
        self.assertTrue(gate['enabled'])
        self.assertEqual(gate['args'], {'target_id': uid('waiting'), 'target_revision': 4, 'route_revision': 2,
                                        'segment_index': 1})
        self.assertOff(self.gates(mission_state('DISASTER', 'SEGMENT_MOVING', dispatch=dispatch('EXECUTING')))
                       ['CONTINUE_SEGMENT'], '승인 대기 아님')
        self.assertOff(self.gates(mission_state('DISASTER', 'TARGET_FOUND'))['CONTINUE_SEGMENT'], '출동 전')

    def test_recheck_leaves_out_rescued_targets(self):
        gate = self.gates(mission_state('DISASTER', 'CANDIDATE'))['REQUEST_RECHECK']
        self.assertTrue(gate['enabled'])
        ids = {c['target_id'] for c in gate['choices']}
        self.assertNotIn(uid('rescued'), ids)
        self.assertIn(uid('waiting'), ids)

    def test_return_home_only_after_a_verdict(self):
        for phase, enabled in (('RESCUE_CONFIRMED', True), ('RECHECK_NEEDED', True), ('SEARCHING', False),
                               ('RETURNING', False)):
            with self.subTest(phase=phase):
                self.assertEqual(self.gates(mission_state('DISASTER', phase))['RETURN_HOME']['enabled'], enabled)

    def test_held_commands_stay_off_whatever_the_state(self):
        for mission in (None, mission_state('DISASTER', 'AWAITING_CONTINUE', dispatch=dispatch()),
                        mission_state('DISASTER', 'RESCUE_CONFIRMED')):
            gates = self.gates(mission)
            for name in HELD:
                self.assertFalse(gates[name]['enabled'], name)
        gates = self.gates(mission_state())
        self.assertIn('Q-22', gates['RESUME']['reason'])
        self.assertIn('현장', gates['STOP_ROBOT']['reason'])


class MissionBadgeTests(LedgerFixture):
    def view_with(self, mission):
        return self.view(dict(self.status, mission_state={'schema_version': '1.2', 'mission': mission}))

    def test_badge_reads_the_mission_state_row(self):
        mission = self.view_with(mission_state('DISASTER', 'AWAITING_CONTINUE', dispatch=dispatch(),
                                               reason='PASSAGE_BLOCKED'))['mission']
        self.assertEqual((mission['source'], mission['mode'], mission['label']), ('mission_state', 'DISASTER', '재난'))
        self.assertEqual((mission['phase'], mission['reason']), ('AWAITING_CONTINUE', 'PASSAGE_BLOCKED'))
        self.assertEqual(mission['dispatch']['segment_index'], 1)
        self.assertEqual(mission['trigger_ns'], T0)
        band = render_fragment(self.view_with(mission_state('DISASTER', 'AWAITING_CONTINUE', dispatch=dispatch(),
                                                            reason='PASSAGE_BLOCKED')))
        band = band[:band.index('</section>')]
        for words in ('계속 승인 대기', '구간', '1/3', 'PASSAGE_BLOCKED'):
            self.assertIn(words, band)

    def test_every_mission_mode_has_a_word(self):
        for mode, phase, word in (('NORMAL', 'IDLE', '평시'), ('CLEARING', 'IDLE', '해제 중'),
                                  ('MISSION_FINISHED', 'FINISHED', '임무 종료')):
            mission = self.view_with(mission_state(mode, phase))['mission']
            self.assertEqual((mission['mode'], mission['label']), (mode, word))

    def test_no_mission_row_yet_falls_back_to_the_signs_and_says_so(self):
        view = self.view_with(None)
        self.assertEqual((view['mission']['source'], view['mission']['mode']), ('site_events', 'DISASTER'))
        self.assertIn('임무 상태 기록 전', render_fragment(view))


class PanelRenderTests(LedgerFixture):
    USER = {'username': 'admin', 'display_name': 'admin', 'role': 'ADMIN'}

    def view_with(self, mission):
        return self.view(dict(self.status, mission_state={'schema_version': '1.2', 'mission': mission}))

    @staticmethod
    def buttons(page):
        return {re.search(r'data-command="([A-Z_]+)"', b).group(1): b
                for b in re.findall(r'<button[^>]*data-command=[^>]*>', page)}

    def test_fragment_carries_the_gates_for_the_buttons(self):
        fragment = render_fragment(self.view_with(mission_state('DISASTER', 'AWAITING_CONTINUE', dispatch=dispatch())))
        gates = json.loads(html.unescape(re.search(r'<div id="gates" hidden data-gates="([^"]*)"', fragment).group(1)))
        self.assertTrue(gates['CONTINUE_SEGMENT']['enabled'])
        self.assertEqual(gates['CONTINUE_SEGMENT']['args']['segment_index'], 1)
        self.assertFalse(gates['PAUSE']['enabled'])

    def test_buttons_need_both_the_access_and_the_gate(self):
        view = self.view_with(mission_state('DISASTER', 'DISASTER'))
        page = render_page(view, self.USER, csrf='tok', command_access=command_access('ADMIN', True))
        buttons = self.buttons(page)
        self.assertEqual(set(buttons), {name for name, _ in COMMANDS})
        self.assertNotIn(' disabled', buttons['START_SEARCH'])
        for name in HELD + ('CONTINUE_SEGMENT', 'RETURN_HOME'):
            self.assertIn(' disabled', buttons[name])
        self.assertIn('data-csrf="tok"', page)
        for role, connected, words in (('CUSTOMER', True, '보기 전용'), ('ADMIN', False, '--operations')):
            with self.subTest(role=role, connected=connected):
                page = render_page(view, dict(self.USER, role=role), csrf='tok',
                                   command_access=command_access(role, connected))
                self.assertTrue(all(' disabled' in b for b in self.buttons(page).values()))
                self.assertNotIn('data-csrf=', page)
                self.assertIn(words, page[page.index('id="commands"'):])

    def test_the_site_stop_stays_on_the_panel(self):
        page = render_page(self.view(), self.USER, csrf='tok', command_access=command_access('ADMIN', True))
        self.assertIn('현장 비상 정지', page[page.index('id="commands"'):])


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
class CommandRouteTests(AccountsFixture):
    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        write_site_state(self.site_db, self.status)
        with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            ledger.record_mission_state(mission_state('DISASTER', 'DISASTER'))
        self.calls = []
        self.reply = lambda payload: (200, answer())

    def transport(self, url, payload, timeout):
        self.calls.append((url, payload, timeout))
        return self.reply(payload)

    def client(self, account=ADMIN, operations=OPS):
        from rescue_console.app import create_app
        app = create_app(self.db, accounts_path=self.accounts, secret_key=SECRET, site_state_path=self.site_db,
                         operations_url=operations, command_transport=self.transport)
        client = app.test_client()
        if account is not None:
            self.login(client, *account)
        return client

    def post(self, client, csrf=True, **form):
        if csrf:
            form['csrf'] = self.csrf(client, '/')
        return client.post('/commands', data=form)

    def test_admin_sends_once_and_gets_the_answer(self):
        client = self.client()
        before = digest_dir(self.dir)
        response = self.post(client, kind='START_SEARCH')
        self.assertEqual(response.status_code, 200)
        result = response.get_json()
        self.assertEqual((result['sent'], result['accepted'], result['request_id']), (True, True, COMMAND_ID))
        self.assertEqual(len(self.calls), 1)
        url, payload, timeout = self.calls[0]
        self.assertEqual(url, OPS + COMMAND_PATH)
        self.assertEqual((payload['kind'], payload['operator_id']), ('START_SEARCH', 'admin'))
        self.assertRegex(payload['command_id'], r'^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$')
        self.assertEqual(result['command_id'], payload['command_id'])
        self.assertEqual(digest_dir(self.dir), before)        # the console writes nothing; operations records it

    def test_rejection_is_shown_as_given(self):
        self.reply = lambda payload: (200, answer(False, 'NAV_NOT_READY'))
        result = self.post(self.client(), kind='START_SEARCH').get_json()
        self.assertEqual((result['sent'], result['accepted'], result['reason']), (True, False, 'NAV_NOT_READY'))

    def test_customer_cannot_send(self):
        client = self.client(CUSTOMER)
        response = self.post(client, kind='START_SEARCH')
        self.assertEqual(response.status_code, 403)
        self.assertEqual(response.get_json()['error'], 'NOT_AUTHORIZED')
        self.assertIn('보기 전용', client.get('/').get_data(as_text=True))
        self.assertEqual(self.calls, [])

    def test_login_and_csrf_are_required(self):
        self.assertEqual(self.client(account=None).post('/commands', data={'kind': 'START_SEARCH'}).status_code, 401)
        self.assertEqual(self.post(self.client(), csrf=False, kind='START_SEARCH').status_code, 400)
        self.assertEqual(self.calls, [])

    def test_without_an_operations_address_nothing_is_sent(self):
        client = self.client(operations=None)
        response = self.post(client, kind='START_SEARCH')
        self.assertEqual((response.status_code, response.get_json()['error']), (503, 'NOT_CONNECTED'))
        self.assertIn('--operations', client.get('/').get_data(as_text=True))
        self.assertEqual(self.calls, [])

    def test_held_or_malformed_commands_are_refused_here(self):
        client = self.client()
        for form in ({'kind': 'PAUSE'}, {'kind': 'STOP_ROBOT'}, {'kind': 'CONTINUE_SEGMENT', 'target_id': 'x'}):
            with self.subTest(form=form):
                response = self.post(client, **form)
                self.assertEqual((response.status_code, response.get_json()['error']), (400, 'INVALID_INPUT'))
        self.assertEqual(self.calls, [])

    def test_a_failed_send_is_shown_and_not_retried(self):
        def timeout(payload):
            raise CommandSendError('TIMEOUT', 'timed out', delivery='UNKNOWN')
        self.reply = timeout
        response = self.post(self.client(), kind='RETURN_HOME')
        self.assertEqual(response.status_code, 502)
        result = response.get_json()
        self.assertEqual((result['sent'], result['error'], result['delivery']), (False, 'TIMEOUT', 'UNKNOWN'))
        self.assertEqual(len(self.calls), 1)

    def test_commands_is_the_only_new_write_route(self):
        from rescue_console.app import create_app
        app = create_app(self.db, accounts_path=self.accounts, secret_key=SECRET, operations_url=OPS)
        rules = {rule.rule: rule.methods - {'HEAD', 'OPTIONS'} for rule in app.url_map.iter_rules()
                 if not rule.rule.startswith(('/login', '/logout', '/admin', '/static'))}
        self.assertEqual(rules.pop('/commands'), {'POST'})
        self.assertEqual(rules.pop('/webcam'), {'POST'})   # 관리자 카메라 고르기(OPS-183)
        self.assertEqual(rules.pop('/robot-camera'), {'POST'})   # 로봇 카메라 보기 설정(OPS-191)
        self.assertEqual({frozenset(methods) for methods in rules.values()}, {frozenset({'GET'})})


if __name__ == '__main__':
    unittest.main()
