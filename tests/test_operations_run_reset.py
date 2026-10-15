"""Scenario reset (초기화), operations side: POST /api/runs/reset (작전·웹개발 2026-10-15; E2).

The console's reset button reaches db_writer's HTTP API, the same process and the same SiteStateLedger
that writes every row. It is refused, with a reason code and a row in operator_commands, while a robot
is moving, has a task, is not docked or has no fresh status. When accepted, one transaction closes the run,
empties the latest-value tables (targets, mission, map state, tunnel lists) and opens a new run that
remembers the coordinator boot it must no longer record. The robot side is restarted by people following
the runbook; the checklist reader says which restarts the DB has already seen.
The answer has the operator command shape ({accepted, request_id, reason, request_state, created_task_id})
so the console reads it with the same code.
"""
import json
from pathlib import Path
from types import SimpleNamespace as Msg
import tempfile
import time
import unittest

from rescue_operations import site_state
from rescue_operations.ros_ingest import SiteStateIngest
from test_operations_ros_ingest import ROLES, system_state_msg, target_msg
from test_operations_site_state import robot_status, target_state, uid

ROBOTS = ('robot_a', 'robot_b')


def docked(robot='robot_a', boot=1, **changes):
    """A robot at rest on its dock with no task: the state a reset is accepted in."""
    role = 'SCOUT' if robot == 'robot_a' else 'SUPPORT'
    return robot_status(robot_id=robot, role=role, robot_boot_id=uid(boot), motion_state='STOPPED',
                        active_task_id=None, generation=None, dock_state='DOCKED', stamp_ns=time.time_ns(),
                        **changes)


def request(n=70, **changes):
    return json.dumps(dict(command_id=uid(n), operator_id='admin') | changes).encode()


def coordinator_envelope(boot):
    return Msg(schema_version='1.0', publisher_id='coordinator', publisher_boot_id=boot, sequence=1,
               published_at_ns=time.time_ns())


class Case(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site_state.sqlite3'
        self.ledger = site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)
        self.addCleanup(self.ledger.close)
        self.ingest = SiteStateIngest(self.ledger, roles=ROLES)

    def reset(self):
        from rescue_operations.command_api import RunReset
        return RunReset(self.ledger, coordinator_boot_id=lambda: self.ingest.coordinator_boot_id)

    def both_docked(self):
        self.ledger.record_robot_status(docked('robot_a', 1))
        self.ledger.record_robot_status(docked('robot_b', 2))

    def press(self, n=70, **changes):
        return self.reset().handle(request(n, **changes), received_at_ns=time.time_ns())

    def commands(self):
        return site_state.read_operator_commands(self.path)['commands']


class RefusalTests(Case):
    def assert_refused(self, reason):
        old = self.ledger.run_id
        status, answer = self.press()
        self.assertEqual((200, False, reason, 'REJECTED', None),
                         (status, answer['accepted'], answer['reason'], answer['request_state'],
                          answer['created_task_id']))
        self.assertEqual(old, self.ledger.run_id)  # nothing reset
        (row,) = self.commands()
        self.assertEqual(('RESET_RUN', False, reason), (row['kind'], row['accepted'], row['reason']))

    def test_moving_robot_refuses(self):
        self.both_docked()
        self.ledger.record_robot_status(docked('robot_b', 2, state_sequence=6, motion_state='MOVING'))
        self.assert_refused('ROBOT_MOVING')

    def test_stop_unconfirmed_counts_as_moving(self):
        self.both_docked()
        self.ledger.record_robot_status(docked('robot_a', 1, state_sequence=6, motion_state='STOP_UNCONFIRMED'))
        self.assert_refused('ROBOT_MOVING')

    def test_active_task_refuses(self):
        self.both_docked()
        self.ledger.record_robot_status(docked('robot_a', 1, state_sequence=6, active_task_id=uid(9), generation=4))
        self.assert_refused('TASK_ACTIVE')

    def test_undocked_or_unknown_dock_refuses(self):
        self.both_docked()
        self.ledger.record_robot_status(docked('robot_b', 2, state_sequence=6, dock_state='UNDOCKED'))
        self.assert_refused('ROBOT_NOT_DOCKED')

    def test_missing_robot_refuses(self):
        self.ledger.record_robot_status(docked('robot_a', 1))
        self.assert_refused('ROBOT_STATUS_STALE')

    def test_stale_status_refuses(self):
        self.both_docked()
        from rescue_operations.command_api import RESET_STATUS_STALE_NS
        status, answer = self.reset().handle(request(), received_at_ns=time.time_ns() + RESET_STATUS_STALE_NS + 1)
        self.assertEqual((False, 'ROBOT_STATUS_STALE'), (answer['accepted'], answer['reason']))

    def test_bad_body_is_400_with_no_reset(self):
        old = self.ledger.run_id
        status, answer = self.reset().handle(b'{"command_id": "x"}', received_at_ns=time.time_ns())
        self.assertEqual((400, False, 'INVALID_INPUT'), (status, answer['accepted'], answer['reason']))
        self.assertEqual(old, self.ledger.run_id)


class AcceptTests(Case):
    def test_accepted_reset_opens_a_new_run_and_empties_latest_tables(self):
        self.both_docked()
        self.ledger.record_target_state(target_state())
        self.ingest.on_system_state(system_state_msg(envelope=coordinator_envelope(uid(41))), received_at_ns=1)
        old = self.ledger.run_id
        status, answer = self.press(note='두 번째')
        self.assertEqual((200, True, None, 'ACCEPTED'),
                         (status, answer['accepted'], answer['reason'], answer['request_state']))
        new = answer['run_id']
        self.assertNotEqual(old, new)
        self.assertEqual(new, self.ledger.run_id)
        runs = site_state.list_runs(self.path)['runs']
        self.assertEqual([old, new], [run['run_id'] for run in runs])
        self.assertIsNotNone(runs[0]['ended_at_ns'])
        self.assertEqual((uid(41), '두 번째'), (runs[1]['previous_coordinator_boot_id'], runs[1]['note']))
        self.assertEqual([], site_state.read_target_states(self.path)['targets'])
        self.assertIsNone(site_state.read_mission_state(self.path)['mission'])
        self.assertEqual(2, len(site_state.read_robot_status(self.path)['robots']))  # kept for the checklist
        # the command row belongs to the old run: it is the last thing that run saw
        reset_rows = site_state.read_operator_commands_range(self.path, run_id=old)['commands']
        self.assertEqual([('RESET_RUN', True)], [(row['kind'], row['accepted']) for row in reset_rows])

    def test_a_resend_gets_the_first_answer_and_resets_once(self):
        self.both_docked()
        first = self.press()[1]
        again = self.press()[1]
        self.assertEqual((True, 'DUPLICATE', first['run_id']), (again['accepted'], again['request_state'],
                                                                again['run_id']))
        self.assertEqual(2, len(site_state.list_runs(self.path)['runs']))


class PreviousCoordinatorTests(Case):
    def test_old_coordinator_messages_are_not_recorded_in_the_new_run(self):
        self.both_docked()
        old_boot = uid(41)
        self.ingest.on_system_state(system_state_msg(envelope=coordinator_envelope(old_boot)), received_at_ns=1)
        self.assertTrue(self.press()[1]['accepted'])
        late = system_state_msg(envelope=coordinator_envelope(old_boot), changed=time.time_ns())
        self.assertIsNone(self.ingest.on_system_state(late, received_at_ns=2))
        self.assertIsNone(self.ingest.on_target_snapshot(
            Msg(envelope=coordinator_envelope(old_boot), snapshot_id=uid(99), targets=[target_msg()]),
            received_at_ns=3))
        self.assertIsNone(site_state.read_mission_state(self.path)['mission'])
        self.assertEqual([], site_state.read_target_states(self.path)['targets'])
        errors = site_state.read_error_log(self.path)['errors']
        self.assertTrue(errors and all('PREVIOUS_RUN_SOURCE' in error['detail'] for error in errors))
        # the restarted coordinator (new boot) is recorded again
        fresh = system_state_msg(envelope=coordinator_envelope(uid(42)), changed=time.time_ns())
        self.assertTrue(self.ingest.on_system_state(fresh, received_at_ns=4))


class OtherProcessTests(Case):
    """Run ids live in the DB, not in one writer's memory (머지 담당 지적, 2026-10-15)."""

    def test_a_run_closed_and_opened_elsewhere_is_what_this_writer_writes_under(self):
        other = site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)
        self.addCleanup(other.close)
        other.close_run(at_ns=50_000)
        new = other.open_run(at_ns=60_000)
        self.assertEqual(new, self.ledger.run_id)
        self.ledger.record_error(dict(at_ns=70_000, robot_id=None, task_id=None, code='INVALID_INPUT', detail='x'))
        errors = site_state.read_error_log_range(self.path, run_id=new)['errors']
        self.assertEqual(['x'], [error['detail'] for error in errors])

    def test_closed_elsewhere_means_this_writer_writes_between_runs(self):
        other = site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)
        self.addCleanup(other.close)
        old = other.close_run(at_ns=50_000)
        self.assertIsNone(self.ledger.run_id)
        self.ledger.record_error(dict(at_ns=70_000, robot_id=None, task_id=None, code='INVALID_INPUT', detail='y'))
        self.assertEqual([], site_state.read_error_log_range(self.path, run_id=old)['errors'])


class ChecklistTests(Case):
    def test_checklist_sees_coordinator_and_each_robot_restart(self):
        self.both_docked()
        self.ingest.on_system_state(system_state_msg(envelope=coordinator_envelope(uid(41))), received_at_ns=1)
        self.press()
        before = site_state.read_reset_checklist(self.path)
        self.assertFalse(before['coordinator_restarted'])
        self.assertEqual({'robot_a': False, 'robot_b': False},
                         {robot: item['restarted'] for robot, item in before['robots'].items()})
        self.ledger.record_robot_status(docked('robot_a', 11, dock_state='UNDOCKED'))  # new executor boot
        self.ingest.on_system_state(system_state_msg(envelope=coordinator_envelope(uid(42)), changed=time.time_ns()),
                                    received_at_ns=5)
        after = site_state.read_reset_checklist(self.path)
        self.assertTrue(after['coordinator_restarted'])
        self.assertEqual({'robot_a': True, 'robot_b': False},
                         {robot: item['restarted'] for robot, item in after['robots'].items()})
        self.assertEqual((uid(1), uid(11)), (after['robots']['robot_a']['boot_id_before'],
                                             after['robots']['robot_a']['boot_id_now']))

    def test_no_reset_yet_means_no_checklist(self):
        self.assertIsNone(site_state.read_reset_checklist(self.path)['previous_run_id'])


class ServerRouteTests(Case):
    def test_reset_path_is_served(self):
        import threading
        import urllib.request
        from rescue_operations.command_api import RESET_PATH, make_server

        class Gate:
            def handle(self, raw, *, received_at_ns):
                return 200, {}
        self.both_docked()
        server = make_server('127.0.0.1', 0, Gate(), reset=self.reset())
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        url = f'http://127.0.0.1:{server.server_address[1]}{RESET_PATH}'
        with urllib.request.urlopen(urllib.request.Request(url, data=request(), method='POST')) as answer:
            self.assertTrue(json.loads(answer.read())['accepted'])


if __name__ == '__main__':
    unittest.main()
