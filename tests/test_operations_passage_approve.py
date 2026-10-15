"""Tunnel pass approval, operations side: POST /api/passages/approve (console-spec I-8, #412; E2).

The console's 「터널 통과 승인」 button reaches db_writer's HTTP API. Operations asks the keepout node once over
/rescue/passage_keepout/approve (ApproveOverheadPassage, #400/#406) with approved_by = the console's login
name and issued_at_ns = the time the request came in, answers in the operator command shape plus item_id and
the node's message, and keeps one operator_commands row (kind APPROVE_PASSAGE, target_id = item_id). The
service is a plain function here, so this runs without ROS.
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

ROBOTS = ('robot_a', 'robot_b')
T0 = 1_700_000_000_000_000_000


def uid(n):
    return str(UUID(int=n))


def request(**changes):
    return dict(command_id=uid(77), kind='APPROVE_PASSAGE', operator_id='admin', item_id=uid(500)) | changes


def body(value):
    return json.dumps(value).encode()


def reply(accepted=True, code='ACCEPTED', message='PASSAGE_APPROVED', item=uid(500)):
    """The node's ApproveOverheadPassage response as the send function gives it."""
    return dict(accepted=accepted, item_id=item, reason_code=code, message=message)


class Case(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site_state.sqlite3'
        self.ledger = site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)
        self.addCleanup(self.ledger.close)
        self.sent = []

    def approval(self, answer=None):
        from rescue_operations.command_api import PassageApproval

        def send(fields):
            self.sent.append(fields)
            if isinstance(answer, Exception):
                raise answer
            return answer or reply()
        return PassageApproval(self.ledger, send)

    def commands(self):
        return site_state.read_operator_commands(self.path)['commands']

    def errors(self):
        return site_state.read_error_log(self.path)['errors']


class Approve(Case):
    def test_the_service_is_called_once_with_the_login_name_and_the_time_it_came_in(self):
        self.approval().handle(body(request()), received_at_ns=T0)
        self.assertEqual([dict(item_id=uid(500), approved_by='admin', issued_at_ns=T0)], self.sent)

    def test_accepted_is_answered_in_the_command_shape_and_kept_as_a_command_row(self):
        status, answer = self.approval().handle(body(request()), received_at_ns=T0)
        self.assertEqual((200, dict(accepted=True, request_id=uid(77), reason=None, request_state='ACCEPTED',
                                    item_id=uid(500), message='PASSAGE_APPROVED')), (status, answer))
        (row,) = self.commands()
        self.assertEqual((uid(77), 'APPROVE_PASSAGE', 'admin', None, uid(500), T0, True, 'ACCEPTED', None),
                         tuple(row[k] for k in ('command_id', 'kind', 'operator_id', 'robot_id', 'target_id',
                                                'requested_at_ns', 'accepted', 'result', 'reason')))
        self.assertEqual([], self.errors())

    def test_a_node_refusal_carries_its_reason_code_and_message(self):
        status, answer = self.approval(reply(False, 'NO_SUCH_ITEM', 'PASSAGE_APPROVE_REFUSED x: no such item')) \
            .handle(body(request()), received_at_ns=T0)
        self.assertEqual((200, False, 'NO_SUCH_ITEM', 'REJECTED', 'PASSAGE_APPROVE_REFUSED x: no such item'),
                         (status, answer['accepted'], answer['reason'], answer['request_state'], answer['message']))
        self.assertEqual([(False, 'REJECTED', 'NO_SUCH_ITEM')],
                         [(r['accepted'], r['result'], r['reason']) for r in self.commands()])

    def test_a_refusal_without_a_code_still_names_a_reason(self):
        """A refused row needs a reason (DB rule), and the console shows one."""
        status, answer = self.approval(reply(False, '', '')).handle(body(request()), received_at_ns=T0)
        self.assertEqual((False, 'UNKNOWN', None), (answer['accepted'], answer['reason'], answer['message']))

    def test_a_resend_gets_the_first_answer_and_does_not_approve_again(self):
        approval = self.approval()
        approval.handle(body(request()), received_at_ns=T0)
        status, answer = approval.handle(body(request()), received_at_ns=T0 + 9)
        self.assertEqual((200, True, 'DUPLICATE', uid(500)),
                         (status, answer['accepted'], answer['request_state'], answer['item_id']))
        self.assertEqual(1, len(self.sent))
        self.assertEqual(['ACCEPTED'], [r['result'] for r in self.commands()])

    def test_the_same_id_for_another_item_is_refused_and_not_sent(self):
        approval = self.approval()
        approval.handle(body(request()), received_at_ns=T0)
        status, answer = approval.handle(body(request(item_id=uid(501))), received_at_ns=T0 + 9)
        self.assertEqual((400, False, 'ID_CONTENT_CONFLICT'), (status, answer['accepted'], answer['reason']))
        self.assertEqual(1, len(self.sent))
        self.assertEqual([uid(500)], [r['target_id'] for r in self.commands()])


class Refuse(Case):
    def test_malformed_requests_are_not_sent(self):
        for value in (request(kind='START_SEARCH'), request(item_id='not-a-uuid'), request(operator_id=''),
                      request(approved_by='someone'), {k: v for k, v in request().items() if k != 'item_id'}):
            with self.subTest(value=value):
                status, answer = self.approval().handle(body(value), received_at_ns=T0)
                self.assertEqual((400, False, 'INVALID_INPUT'), (status, answer['accepted'], answer['reason']))
        status, _ = self.approval().handle(b'not json', received_at_ns=T0)
        self.assertEqual(400, status)
        self.assertEqual([], self.sent)

    def test_a_refusal_with_a_usable_id_is_kept_as_a_row(self):
        self.approval().handle(body(request(item_id='not-a-uuid')), received_at_ns=T0)
        self.assertEqual([(uid(77), 'APPROVE_PASSAGE', False, 'REFUSED_BY_OPERATIONS', 'INVALID_INPUT', None)],
                         [(r['command_id'], r['kind'], r['accepted'], r['result'], r['reason'], r['target_id'])
                          for r in self.commands()])

    def test_no_node_answer_has_no_accepted_and_no_row(self):
        """503/504 bodies carry no `accepted`; the console shows 「전달됐는지 모름」 and does not resend."""
        from rescue_operations.command_api import CommandTimeout, CommandUnavailable
        for error, code in ((CommandUnavailable('no service'), 503), (CommandTimeout('3s'), 504)):
            with self.subTest(code=code):
                status, answer = self.approval(error).handle(body(request()), received_at_ns=T0)
                self.assertEqual((code, 'ACCEPT_TIMEOUT', uid(77)), (status, answer['error'], answer['request_id']))
                self.assertNotIn('accepted', answer)
        self.assertEqual([], self.commands())
        self.assertEqual(['ACCEPT_TIMEOUT', 'ACCEPT_TIMEOUT'], [e['code'] for e in self.errors()])


class Wiring(unittest.TestCase):
    def test_the_kind_can_be_kept(self):
        self.assertIn('APPROVE_PASSAGE', site_state.OPERATOR_COMMAND_KINDS)

    def test_the_path_is_the_one_the_console_posts_to(self):
        from rescue_console import commands
        from rescue_operations.command_api import PASSAGE_PATH
        self.assertEqual('/api/passages/approve', PASSAGE_PATH)
        CONSOLE = getattr(commands, 'PASSAGE_PATH', None)  # the console's side is #412
        if CONSOLE is not None:
            self.assertEqual(CONSOLE, PASSAGE_PATH)

    def test_launch_gives_the_keepout_node_s_service(self):
        from rescue_bringup.operations_launch import db_writer_parameters
        from rescue_bringup.profile import load_deployment
        from rescue_operations.ros_node import node_config
        from rescue_robot.passage_keepout import APPROVE_SERVICE
        profile = Path(__file__).resolve().parents[1] / 'config' / 'deployment.v2.example.json'
        params = db_writer_parameters(load_deployment(profile), db_path='/var/lib/rescue/site_state.sqlite3',
                                      api_port=8090, command_timeout_s=3.0, webcam_node='', map_topic='',
                                      map_dir='')
        self.assertEqual(APPROVE_SERVICE, params['passage_approve_service'])
        self.assertEqual(APPROVE_SERVICE, node_config(params)['passage_approve_service'])

    def test_the_server_routes_the_path_to_the_approval(self):
        from rescue_operations.command_api import make_server

        class Echo:
            def __init__(self, name):
                self.name = name

            def handle(self, raw, *, received_at_ns):
                return 200, {'by': self.name}
        server = make_server('127.0.0.1', 0, Echo('gate'), passages=Echo('passages'))
        self.addCleanup(server.server_close)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.shutdown)
        req = urllib.request.Request(f'http://127.0.0.1:{server.server_address[1]}/api/passages/approve',
                                     data=body(request()), method='POST', headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=5) as response:
            self.assertEqual({'by': 'passages'}, json.loads(response.read()))


if __name__ == '__main__':
    unittest.main()
