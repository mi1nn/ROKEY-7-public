"""scripts/field/noweb/operator_command.py: the console's buttons from a terminal (웹 없는 트랙, 2026-10-15).

It posts to the same operations API the console uses (a real command_api server here, with stand-in gates),
makes a new command_id each time, prints the answer JSON as it came and exits non-zero on a refusal. The
revisions DISPATCH_SUPPORT and CONTINUE_SEGMENT need are read from the operations DB, never typed by hand.
"""
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import socket
import tempfile
import threading
import unittest
from uuid import UUID

from rescue_operations import site_state
from rescue_operations.command_api import COMMAND_FIELDS, COMMAND_PATH, RESET_PATH, make_server
from test_operations_site_state import dispatch, mission, target_state, uid

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / 'scripts' / 'field' / 'noweb' / 'operator_command.py'
if not SCRIPT.is_file():
    # as tests/test_field_wiring_check.py: a repo script, not part of the installed-wheel checkout
    raise unittest.SkipTest('scripts/field/noweb/operator_command.py absent (installed-wheel checkout)')
SPEC = importlib.util.spec_from_file_location('noweb_operator_command', SCRIPT)
oc = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(oc)

ROBOTS = ('robot_a', 'robot_b')
ACCEPTED = {'accepted': True, 'request_id': None, 'reason': None, 'request_state': 'ACCEPTED',
            'created_task_id': None}


class Gate:
    """Stand-in for CommandGate / RunReset: keeps each body and gives back ``answer``."""

    def __init__(self, status=200, answer=None):
        self.status, self.answer, self.bodies = status, answer or ACCEPTED, []

    def handle(self, raw, *, received_at_ns):
        self.bodies.append(json.loads(raw))
        return self.status, self.answer


class Case(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.db = Path(self.temp.name) / 'site_state.sqlite3'

    def serve(self, gate=None, reset=None):
        self.gate, self.reset = gate or Gate(), reset or Gate()
        server = make_server('127.0.0.1', 0, self.gate, reset=self.reset)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return server.server_address[1]

    def run_tool(self, *args, port=None):
        out, err = io.StringIO(), io.StringIO()
        argv = ['--port', str(port or self.serve()), '--operator', 'admin', *args]
        with redirect_stdout(out), redirect_stderr(err):
            try:
                code = oc.main(argv)
            except SystemExit as exc:  # argparse usage errors
                code = exc.code
        return code, out.getvalue(), err.getvalue()

    def ledger(self):
        ledger = site_state.SiteStateLedger(self.db, allowed_robot_ids=ROBOTS)
        self.addCleanup(ledger.close)
        return ledger


class CommandTests(Case):
    def test_the_kinds_are_the_api_s_commands_and_the_reset(self):
        self.assertEqual(set(COMMAND_FIELDS) | {'RESET_RUN'}, set(oc.KINDS))

    def test_declare_disaster_sends_a_new_id_each_time_and_prints_the_answer(self):
        port = self.serve()
        first = self.run_tool('DECLARE_DISASTER', port=port)
        second = self.run_tool('DECLARE_DISASTER', port=port)
        self.assertEqual((0, 0), (first[0], second[0]))
        self.assertEqual(ACCEPTED, json.loads(first[1]))
        bodies = self.gate.bodies
        self.assertEqual([{'command_id', 'kind', 'operator_id'}] * 2, [set(body) for body in bodies])
        self.assertEqual(('DECLARE_DISASTER', 'admin'), (bodies[0]['kind'], bodies[0]['operator_id']))
        self.assertNotEqual(bodies[0]['command_id'], bodies[1]['command_id'])
        self.assertEqual(4, UUID(bodies[0]['command_id']).version)

    def test_a_target_command_takes_the_target_id(self):
        for kind in ('REQUEST_RECHECK', 'SET_CAMERA_WATCH'):
            with self.subTest(kind=kind):
                code, _, _ = self.run_tool(kind, '--target-id', uid(500))
                self.assertEqual(0, code)
                self.assertEqual(dict(kind=kind, operator_id='admin', target_id=uid(500)),
                                 {k: v for k, v in self.gate.bodies[-1].items() if k != 'command_id'})

    def test_a_missing_or_unused_target_id_is_refused_before_sending(self):
        for args in (('REQUEST_RECHECK',), ('RETURN_HOME', '--target-id', uid(500)),
                     ('REQUEST_RECHECK', '--target-id', 'not-a-uuid')):
            with self.subTest(args=args):
                code, _, err = self.run_tool(*args)
                self.assertEqual(2, code)
                self.assertTrue(err)
        self.assertEqual([], self.gate.bodies)

    def test_a_refusal_prints_the_answer_and_exits_one(self):
        refused = dict(ACCEPTED, accepted=False, reason='TASK_BUSY', request_state='REJECTED')
        for gate in (Gate(answer=refused), Gate(400, {'accepted': False, 'request_id': None,
                                                      'reason': 'INVALID_INPUT', 'detail': 'x'})):
            with self.subTest(status=gate.status):
                code, out, _ = self.run_tool('DECLARE_DISASTER', port=self.serve(gate))
                self.assertEqual((1, gate.answer), (code, json.loads(out)))

    def test_no_answer_exits_three(self):
        timeout = {'error': 'ACCEPT_TIMEOUT', 'request_id': uid(1), 'detail': '3s'}
        code, out, _ = self.run_tool('RETURN_HOME', port=self.serve(Gate(504, timeout)))
        self.assertEqual((3, timeout), (code, json.loads(out)))
        with socket.socket() as probe:  # a port nobody listens on once the probe closes
            probe.bind(('127.0.0.1', 0))
            free = probe.getsockname()[1]
        code, out, err = self.run_tool('RETURN_HOME', port=free)
        self.assertEqual((3, ''), (code, out))
        self.assertTrue(err)

    def test_reset_goes_to_the_reset_path_with_its_note(self):
        code, out, _ = self.run_tool('RESET_RUN', '--note', '두 번째')
        self.assertEqual(0, code)
        (body,) = self.reset.bodies
        self.assertEqual(dict(operator_id='admin', note='두 번째'),
                         {k: v for k, v in body.items() if k != 'command_id'})
        self.assertEqual([], self.gate.bodies)
        self.assertEqual((COMMAND_PATH, RESET_PATH), (oc.COMMAND_PATH, oc.RESET_PATH))


class RevisionFromDbTests(Case):
    def test_dispatch_support_reads_the_target_revision(self):
        self.ledger().record_target_state(target_state(target_revision=4))
        code, _, err = self.run_tool('DISPATCH_SUPPORT', '--target-id', uid(500), '--db', str(self.db))
        self.assertEqual(0, code, err)
        self.assertEqual(4, self.gate.bodies[0]['target_revision'])

    def test_continue_segment_reads_the_dispatch_from_the_mission(self):
        self.ledger().record_mission_state(mission(phase='SEGMENT_MOVING') | dispatch(
            target_revision=4, route_revision=2, segment_index=1, segment_phase='AWAITING_APPROVAL'))
        code, _, err = self.run_tool('CONTINUE_SEGMENT', '--db', str(self.db))
        self.assertEqual(0, code, err)
        self.assertEqual(dict(target_id=uid(500), target_revision=4, route_revision=2, segment_index=1),
                         {k: self.gate.bodies[0][k] for k in ('target_id', 'target_revision', 'route_revision',
                                                               'segment_index')})

    def test_without_the_db_or_its_value_nothing_is_sent(self):
        ledger = self.ledger()
        ledger.record_target_state(target_state(target_revision=None))
        ledger.record_mission_state(mission())  # no dispatch
        for args in (('DISPATCH_SUPPORT', '--target-id', uid(500)),
                     ('DISPATCH_SUPPORT', '--target-id', uid(500), '--db', str(self.db)),
                     ('DISPATCH_SUPPORT', '--target-id', uid(501), '--db', str(self.db)),
                     ('CONTINUE_SEGMENT', '--db', str(self.db)),
                     ('CONTINUE_SEGMENT', '--db', str(Path(self.temp.name) / 'none.sqlite3'))):
            with self.subTest(args=args):
                code, _, err = self.run_tool(*args)
                self.assertEqual(2, code)
                self.assertTrue(err)
        self.assertEqual([], self.gate.bodies)
        self.assertFalse((Path(self.temp.name) / 'none.sqlite3').exists())


if __name__ == '__main__':
    unittest.main()
