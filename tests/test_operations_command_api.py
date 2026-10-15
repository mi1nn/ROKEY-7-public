"""Operator command API (#115 operations side): HTTP JSON -> coordinator SubmitOperatorCommand (E2).

The request shape, reply shape and status codes are the ones agreed on #115
(issuecomment-6118222924). The coordinator is a plain function here, so this runs
without ROS; the rclpy node only wires the HTTP server and the service client.
"""
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from uuid import UUID

from rescue_operations import site_state
from rescue_operations.schema import OperationsError

ROBOTS = ('robot_a', 'robot_b')
T0 = 1_700_000_000_000_000_000


def uid(n):
    return str(UUID(int=n))


def request(kind='START_SEARCH', **changes):
    extra = {'START_SEARCH': {}, 'RETURN_HOME': {},
             'DISPATCH_SUPPORT': dict(target_id=uid(500), target_revision=2),
             'CONTINUE_SEGMENT': dict(target_id=uid(500), target_revision=2, route_revision=1, segment_index=1),
             'REQUEST_RECHECK': dict(target_id=uid(500)), 'SET_CAMERA_WATCH': dict(target_id=uid(500))}.get(kind, {})
    return dict(command_id=uid(77), kind=kind, operator_id='admin') | extra | changes


def body(value):
    return json.dumps(value).encode()


class ParseCommand(unittest.TestCase):
    def parse(self, raw):
        from rescue_operations.command_api import parse_command
        return parse_command(raw)

    def test_each_kind_takes_exactly_its_fields(self):
        """Same per-kind fields as the coordinator's mission.COMMAND_FIELDS (#115 table)."""
        for kind in ('START_SEARCH', 'DISPATCH_SUPPORT', 'CONTINUE_SEGMENT', 'REQUEST_RECHECK', 'RETURN_HOME',
                     'SET_CAMERA_WATCH', 'DECLARE_DISASTER'):
            with self.subTest(kind=kind):
                self.assertEqual(request(kind), self.parse(body(request(kind))))

    def test_fields_match_the_coordinator_gate(self):
        from rescue_coordination.mission import ALL_COMMAND_FIELDS as GATE
        from rescue_operations.command_api import COMMAND_FIELDS
        self.assertEqual({kind: set(fields) for kind, fields in GATE.items()},
                         {kind: set(fields) for kind, fields in COMMAND_FIELDS.items()})

    def test_declare_disaster_takes_only_the_id_and_the_operator(self):
        """The manual disaster declaration carries nothing but who asked (재범님 결정 2026-10-15)."""
        self.assertEqual(dict(command_id=uid(77), kind='DECLARE_DISASTER', operator_id='admin'),
                         self.parse(body(request('DECLARE_DISASTER'))))
        for extra in (dict(target_id=uid(500)), dict(robot_id='robot_a'), dict(note='x')):
            with self.subTest(extra=extra), self.assertRaises(OperationsError) as caught:
                self.parse(body(request('DECLARE_DISASTER', **extra)))
            self.assertEqual('INVALID_INPUT', caught.exception.code)

    def test_malformed_requests_are_invalid_input(self):
        bad = [b'not json', b'[]', body(request(kind='PAUSE')), body(request(kind='STOP_ROBOT')),
               body(request(kind='FINISH_MISSION')), body(request(robot_id='robot_a')),
               body(request('DISPATCH_SUPPORT', target_revision=None)),
               body({k: v for k, v in request('CONTINUE_SEGMENT').items() if k != 'segment_index'}),
               body(request(command_id='not-a-uuid')), body(request(operator_id='')),
               body(request('REQUEST_RECHECK', target_id='x')),
               body(request('CONTINUE_SEGMENT', segment_index=-1)),
               body(request('CONTINUE_SEGMENT', route_revision=True)),
               body(request('DISPATCH_SUPPORT', target_revision='2')),
               b'{"command_id": "' + b'a' * 9000 + b'"}']
        for raw in bad:
            with self.subTest(raw=raw[:60]), self.assertRaises(OperationsError) as caught:
                self.parse(raw)
            self.assertEqual('INVALID_INPUT', caught.exception.code)


class ServiceFields(unittest.TestCase):
    def test_set_camera_watch_sends_no_roi(self):
        """The console has no ROI picker: zeros = "not given", the coordinator uses its deployment ROI (C-11 row 2)."""
        from rescue_operations.command_api import service_fields
        fields = service_fields(request('SET_CAMERA_WATCH'), issued_at_ns=T0, timeout_ns=1)
        self.assertEqual(('SET_CAMERA_WATCH', uid(500), [0.0] * 4, 0),
                         (fields['operation'], fields['target_id'], fields['roi_xyxy_normalized'],
                          fields['expected_target_revision']))

    def test_request_maps_to_the_idl_fields_and_unused_ones_stay_empty(self):
        """#115 「operations 가 채우는 것」: target_revision -> expected_target_revision, the rest empty."""
        from rescue_operations.command_api import service_fields
        fields = service_fields(request('CONTINUE_SEGMENT'), issued_at_ns=T0, timeout_ns=3_000_000_000)
        self.assertEqual(dict(command_id=uid(77), operation='CONTINUE_SEGMENT', operator_id='admin', robot_id='',
                              target_id=uid(500), expected_task_id='', expected_generation=0,
                              expected_target_revision=2, route_id='', route_revision=1, segment_index=1,
                              issued_at_ns=T0, deadline_ns=T0 + 3_000_000_000, roi_xyxy_normalized=[0.0] * 4,
                              association_evidence_id=''), fields)
        start = service_fields(request(), issued_at_ns=T0, timeout_ns=1)
        declare = service_fields(request('DECLARE_DISASTER'), issued_at_ns=T0, timeout_ns=1)
        self.assertEqual(start | dict(operation='DECLARE_DISASTER'), declare)  # same .srv, only the operation differs
        self.assertEqual(('', 0, 0, 0), (start['target_id'], start['expected_target_revision'],
                                         start['route_revision'], start['segment_index']))


def reply(accepted=True, state='ACCEPTED', reason='', task=''):
    return dict(accepted=accepted, request_state=state, reason_code=reason, created_task_id=task)


class CommandGate(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site_state.sqlite3'
        self.ledger = site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)
        self.addCleanup(self.ledger.close)
        self.sent = []

    def gate(self, answer):
        from rescue_operations.command_api import CommandGate

        def send(fields):
            self.sent.append(fields)
            if isinstance(answer, Exception):
                raise answer
            return answer
        return CommandGate(self.ledger, send, timeout_ns=3_000_000_000)

    def commands(self):
        return site_state.read_operator_commands(self.path)['commands']

    def errors(self):
        return site_state.read_error_log(self.path)['errors']

    def test_accepted_command_is_answered_and_recorded(self):
        status, answer = self.gate(reply(task=uid(9))).handle(body(request('DISPATCH_SUPPORT')),
                                                              received_at_ns=T0)
        self.assertEqual((200, dict(accepted=True, request_id=uid(77), reason=None, request_state='ACCEPTED',
                                    created_task_id=uid(9))), (status, answer))
        self.assertEqual(2, self.sent[0]['expected_target_revision'])
        (row,) = self.commands()
        self.assertEqual((uid(77), 'DISPATCH_SUPPORT', 'admin', None, uid(500), None, None, T0, True, 'ACCEPTED', None),
                         tuple(row[k] for k in ('command_id', 'kind', 'operator_id', 'robot_id', 'target_id',
                                                'segment_index', 'route_revision', 'requested_at_ns', 'accepted',
                                                'result', 'reason')))

    def test_declare_disaster_is_sent_and_kept_in_the_command_history(self):
        """The row is what the replay shows of the declaration (재범님 결정 2026-10-15)."""
        status, answer = self.gate(reply()).handle(body(request('DECLARE_DISASTER')), received_at_ns=T0)
        self.assertEqual((200, True, 'ACCEPTED'), (status, answer['accepted'], answer['request_state']))
        self.assertEqual(['DECLARE_DISASTER'], [fields['operation'] for fields in self.sent])
        self.assertEqual([('DECLARE_DISASTER', True, 'ACCEPTED')],
                         [(r['kind'], r['accepted'], r['result']) for r in self.commands()])
        self.assertEqual([], self.errors())

    def test_coordinator_refusal_is_an_answer_and_recorded_with_its_reason(self):
        status, answer = self.gate(reply(False, 'REJECTED', 'NAV_NOT_READY')).handle(body(request()),
                                                                                      received_at_ns=T0)
        self.assertEqual((200, False, 'NAV_NOT_READY', 'REJECTED'),
                         (status, answer['accepted'], answer['reason'], answer['request_state']))
        self.assertEqual([(False, 'REJECTED', 'NAV_NOT_READY')],
                         [(r['accepted'], r['result'], r['reason']) for r in self.commands()])

    def test_a_resend_returns_the_first_answer_without_a_conflict(self):
        self.gate(reply()).handle(body(request()), received_at_ns=T0)
        status, answer = self.gate(reply(state='DUPLICATE')).handle(body(request()), received_at_ns=T0 + 9)
        self.assertEqual((200, True, 'DUPLICATE'), (status, answer['accepted'], answer['request_state']))
        self.assertEqual(['ACCEPTED'], [r['result'] for r in self.commands()])  # the first answer stays
        self.assertEqual([], self.errors())

    def test_a_resend_carries_the_first_issue_time(self):
        """Local ROS run with the real coordinator (2026-10-12): it compares the whole command,
        issued_at_ns included, so a resend stamped anew got ID_CONTENT_CONFLICT, not DUPLICATE."""
        gate = self.gate(reply())
        gate.handle(body(request()), received_at_ns=T0)
        gate.handle(body(request()), received_at_ns=T0 + 9)
        self.assertEqual([T0, T0], [fields['issued_at_ns'] for fields in self.sent])
        self.assertEqual([T0 + 3_000_000_000] * 2, [fields['deadline_ns'] for fields in self.sent])

    def test_a_refused_request_is_not_sent_and_is_recorded_when_it_can_be(self):
        status, answer = self.gate(reply()).handle(body(request('DISPATCH_SUPPORT', target_revision=None)),
                                                   received_at_ns=T0)
        self.assertEqual((400, False, uid(77), 'INVALID_INPUT'),
                         (status, answer['accepted'], answer['request_id'], answer['reason']))
        self.assertEqual([], self.sent)
        self.assertEqual([(uid(77), False, 'INVALID_INPUT')],
                         [(r['command_id'], r['accepted'], r['reason']) for r in self.commands()])
        # Without a usable id there is no command row; the refusal still lands in error_log.
        status, answer = self.gate(reply()).handle(b'not json', received_at_ns=T0 + 1)
        self.assertEqual((400, None), (status, answer['request_id']))
        self.assertEqual(['INVALID_INPUT'], [e['code'] for e in self.errors()])

    def test_no_coordinator_answer_is_not_an_answer_shape_and_not_a_command_row(self):
        """503/504 bodies carry no `accepted`, so the console shows 「전달됐는지 모름」 (#115 read_reply)."""
        from rescue_operations.command_api import CommandTimeout, CommandUnavailable
        for error, code in ((CommandUnavailable('no service'), 503), (CommandTimeout('3s'), 504)):
            with self.subTest(code=code):
                status, answer = self.gate(error).handle(body(request()), received_at_ns=T0)
                self.assertEqual(code, status)
                self.assertNotIn('accepted', answer)
                self.assertEqual(('ACCEPT_TIMEOUT', uid(77)), (answer['error'], answer['request_id']))
        self.assertEqual([], self.commands())
        self.assertEqual(['ACCEPT_TIMEOUT', 'ACCEPT_TIMEOUT'], [e['code'] for e in self.errors()])
        self.assertIn(uid(77), self.errors()[0]['detail'])


class HttpServer(unittest.TestCase):
    def setUp(self):
        from rescue_operations.command_api import make_server

        class Gate:
            def handle(self, raw, *, received_at_ns):
                return 200, {'echo': json.loads(raw)['kind']}
        self.server = make_server('127.0.0.1', 0, Gate())
        self.addCleanup(self.server.server_close)
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(self.server.shutdown)
        self.base = f'http://127.0.0.1:{self.server.server_address[1]}'

    def post(self, path, data, headers=None):
        req = urllib.request.Request(self.base + path, data=data, method='POST',
                                     headers={'Content-Type': 'application/json'} | (headers or {}))
        try:
            with urllib.request.urlopen(req, timeout=5) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read() or b'null')

    def test_post_reaches_the_gate_and_returns_its_json(self):
        self.assertEqual((200, {'echo': 'START_SEARCH'}), self.post('/api/operator-commands', body(request())))

    def test_other_paths_and_oversized_bodies_are_refused(self):
        self.assertEqual(404, self.post('/api/other', body(request()))[0])
        self.assertEqual(413, self.post('/api/operator-commands', b'{' + b' ' * 9000 + b'}')[0])


if __name__ == '__main__':
    unittest.main()
