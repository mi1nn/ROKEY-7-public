"""Offline rehearsal: the BRD flow written into the site-state DB step by step (E2, no robot, no ROS).

The flow is console-and-db-plan 1절 (one fixed webcam). Decisions — when the site is in disaster,
whether a dispatch is accepted, the rescue verdict — belong to coordination; here they are given as
fixture results, and what is checked is that the DB and its read API hold and show the whole flow.
`FLOW` is data and `apply` is the only writer, so the same steps can later move into `offline_demo`.
"""
from pathlib import Path
import sqlite3
import tempfile
import unittest
from uuid import UUID, uuid5

from rescue_contracts import SYSTEM_MODES, SiteEvent
from rescue_operations import site_state
from rescue_operations.schema import OperationsError

ROBOTS = ('robot_a', 'robot_b')  # robot_a = AMR1 (SCOUT), robot_b = AMR2 (SUPPORT)
T0 = 1_700_000_000_000_000_000
NAMESPACE = UUID(int=0x5EED)


def at(seconds):
    return T0 + int(seconds * 1_000_000_000)


def uid(label):
    return str(uuid5(NAMESPACE, label))


TARGET = uid('target/1')
SIGN = SiteEvent(uid('event/sign'), 'cam_a', 'DISASTER_SIGN', at(10), uid('evidence/sign'), 'a' * 64, 0.92,
                 'site-clock', 'CAPTURE', 'SYNCED', 2_000_000, 'shake reversals 4')
CANDIDATE = SiteEvent(uid('event/candidate'), 'cam_a', 'TARGET_DISAPPEARED_CANDIDATE', at(400),
                      uid('evidence/candidate'), 'b' * 64, 0.81, 'site-clock', 'CAPTURE', 'SYNCED', 2_000_000,
                      'target left watched area', target_id=TARGET, watch_id=uid('watch/1'))


def mission(phase, seconds, mode='DISASTER', generation=1):
    started = (SIGN.event_id, SIGN.observed_at_ns) if mode != 'NORMAL' else (None, None)
    return dict(mode=mode, phase=phase, trigger_event_id=started[0], started_at_ns=started[1],
                generation=generation, updated_at_ns=at(seconds))


def robot(robot_id, sequence, seconds, motion, task=None, role=None):
    return dict(robot_id=robot_id, robot_boot_id=uid('boot/' + robot_id), state_sequence=sequence,
                role=role or ('SCOUT' if robot_id == 'robot_a' else 'SUPPORT'), stamp_ns=at(seconds),
                pose=dict(x=1.0 + sequence, y=2.0, yaw=0.0, frame_id='mission'), motion_state=motion,
                readiness='READY', battery_percent=90.0 - sequence, battery_stamp_ns=at(seconds),
                active_task_id=None if task is None else uid('task/' + task),
                generation=None if task is None else 1)


def target(seconds, confirmed, rescue='NOT_CHECKED', support='NONE', verdict=None, reasons=()):
    return dict(target_id=TARGET,
                position=dict(x=3.0, y=4.0, frame_id='mission', map_session_id=uid('map/a'), map_revision=2),
                detection_confirmed=confirmed, rescue_state=rescue, rescue_reasons=list(reasons),
                verdict_id=None if verdict is None else uid(verdict), support_state=support,
                support_robot_id=None if support == 'NONE' else 'robot_b', updated_at_ns=at(seconds))


def command(label, kind, seconds, accepted, reason=None, robot_id='robot_b', target_id=TARGET, segment=None):
    return dict(command_id=uid('command/' + label), kind=kind, operator_id='admin', robot_id=robot_id,
                target_id=target_id, segment_index=segment, route_revision=None if segment is None else 1,
                requested_at_ns=at(seconds), accepted=accepted, result='ACCEPTED' if accepted else None,
                reason=reason)


# (label, expected mission phase after the step, writes). Order is the screen's order of events.
FLOW = (
    ('평시', 'IDLE', [('mission', mission('IDLE', 0, mode='NORMAL', generation=0)),
                    ('robot', robot('robot_a', 1, 0, 'IDLE')), ('robot', robot('robot_b', 1, 0, 'IDLE'))]),
    ('웹캠 진동 → 지진 감지', 'DISASTER', [('event', SIGN), ('mission', mission('DISASTER', 11))]),
    ('AMR1 정찰 시작 (자동)', 'SEARCHING', [('mission', mission('SEARCHING', 12)),
                                      ('robot', robot('robot_a', 2, 12, 'MOVING', task='search'))]),
    ('지도 올라옴', 'SEARCHING', [('map', dict(snapshot_id=uid('map/rev2'), robot_id='robot_a', revision=2,
                                         path='maps/robot_a/rev2.png', sha256='c' * 64, at_ns=at(60)))]),
    ('요구조자 발견 (미확정)', 'TARGET_FOUND', [('target', target(100, confirmed=False)),
                                         ('mission', mission('TARGET_FOUND', 101))]),
    ('확정 전 AMR2 출동 → 거절', 'TARGET_FOUND', [('command', command(
        'dispatch-early', 'DISPATCH_SUPPORT', 110, False, reason='GUARD_INVALID: no confirmed target'))]),
    ('대상 확정', 'TARGET_FOUND', [('target', target(120, confirmed=True))]),
    ('AMR2 출동 승인', 'DISPATCH_APPROVED', [
        ('command', command('dispatch', 'DISPATCH_SUPPORT', 130, True)),
        ('target', target(131, confirmed=True, support='ASSIGNED')),
        ('mission', mission('DISPATCH_APPROVED', 131)),
        ('robot', robot('robot_b', 2, 131, 'MOVING', task='approach'))]),
    ('구간 1 이동 뒤 정지', 'AWAITING_CONTINUE', [('mission', mission('SEGMENT_MOVING', 132)),
                                            ('mission', mission('AWAITING_CONTINUE', 200))]),
    ('Nav2 준비 안 됨 오류', 'AWAITING_CONTINUE', [('error', dict(
        at_ns=at(205), robot_id='robot_b', task_id=uid('task/approach'), code='NAV_NOT_READY',
        detail='rescue_robot.NAV2_INACTIVE: bt_navigator not active'))]),
    ('운영자 "계속"', 'SEGMENT_MOVING', [('command', command('continue-1', 'CONTINUE_SEGMENT', 210, True,
                                                         segment=1)),
                                      ('mission', mission('SEGMENT_MOVING', 211))]),
    ('AMR2 도착', 'ARRIVED', [('target', target(300, confirmed=True, support='ARRIVED')),
                            ('mission', mission('ARRIVED', 300)),
                            ('robot', robot('robot_b', 3, 300, 'STOPPED'))]),
    ('웹캠 소실 → 구조 완료 후보', 'CANDIDATE', [('event', CANDIDATE),
                                         ('mission', mission('CANDIDATE', 401))]),
    ('AMR1 재확인', 'RECHECKING', [('mission', mission('RECHECKING', 402)),
                                ('robot', robot('robot_a', 3, 402, 'MOVING', task='reobserve'))]),
    ('구출 완료 판정', 'RESCUE_CONFIRMED', [
        ('target', target(500, confirmed=True, rescue='RESCUE_CONFIRMED', support='ARRIVED', verdict='verdict/1')),
        ('mission', mission('RESCUE_CONFIRMED', 500))]),
    ('복귀 지시 → 복귀 중', 'RETURNING', [
        ('command', command('return', 'RETURN_HOME', 510, True, robot_id=None, target_id=None)),
        ('mission', mission('RETURNING', 511))]),
    ('종료', 'FINISHED', [('mission', mission('FINISHED', 600)),
                        ('robot', robot('robot_a', 4, 600, 'IDLE')), ('robot', robot('robot_b', 4, 600, 'IDLE'))]),
)

WRITERS = {'mission': 'record_mission_state', 'robot': 'record_robot_status', 'target': 'record_target_state',
           'event': 'record_site_event', 'command': 'record_operator_command', 'error': 'record_error',
           'map': 'record_map_snapshot'}


def apply(ledger, writes):
    """The only writer of the rehearsal: one record_* call per write, results in order."""
    return [getattr(ledger, WRITERS[kind])(value) for kind, value in writes]


def screen(path):
    """What the console can read after a step (operations read API only)."""
    return {'mission': site_state.read_mission_state(path)['mission'],
            'robots': site_state.read_robot_status(path)['robots'],
            'targets': site_state.read_target_states(path)['targets'],
            'events': site_state.read_site_events(path),
            'commands': site_state.read_operator_commands(path)['commands'],
            'errors': site_state.read_error_log(path)['errors'],
            'maps': site_state.read_map_snapshots(path)['maps']}


class SiteFlowRehearsal(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site_state.sqlite3'

    def ledger(self):
        return site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)

    def run_flow(self):
        with self.ledger() as ledger:
            for _, _, writes in FLOW:
                apply(ledger, writes)

    def test_every_step_is_stored_and_shown_in_order(self):
        phases = []
        with self.ledger() as ledger:
            for label, phase, writes in FLOW:
                with self.subTest(step=label):
                    results = apply(ledger, writes)
                    self.assertTrue(all(r is True or type(r) is int for r in results), results)
                    phases.append(screen(self.path)['mission']['phase'])
                    self.assertEqual(phase, phases[-1])
        self.assertEqual([phase for _, phase, _ in FLOW], phases)

    def test_final_screen_links_event_target_commands_and_verdict(self):
        self.run_flow()
        final = screen(self.path)
        mission_row, events = final['mission'], final['events']['site_events']
        self.assertEqual(('DISASTER', 'FINISHED'), (mission_row['mode'], mission_row['phase']))
        # The badge's trigger is a stored DISASTER_SIGN, and its time is that event's time.
        sign = next(e for e in events if e['event_id'] == mission_row['trigger_event_id'])
        self.assertEqual(('DISASTER_SIGN', SIGN.observed_at_ns), (sign['kind'], mission_row['started_at_ns']))
        # The rescue candidate names the target the screen shows as confirmed.
        candidate = next(e for e in events if e['kind'] == 'TARGET_DISAPPEARED_CANDIDATE')
        (shown,) = final['targets']
        self.assertEqual(candidate['target_id'], shown['target_id'])
        self.assertEqual(('RESCUE_CONFIRMED', 'ARRIVED', 'robot_b'),
                         (shown['rescue_state'], shown['support_state'], shown['support_robot_id']))
        self.assertIsNotNone(shown['verdict_id'])
        self.assertEqual(0, final['events']['exit_pass_count'])  # one camera: no EXIT_PASS producer
        # Command history keeps the rejection before the approval, newest first.
        self.assertEqual([('RETURN_HOME', True), ('CONTINUE_SEGMENT', True), ('DISPATCH_SUPPORT', True),
                          ('DISPATCH_SUPPORT', False)], [(c['kind'], c['accepted']) for c in final['commands']])
        self.assertEqual('GUARD_INVALID: no confirmed target', final['commands'][-1]['reason'])
        approved = final['commands'][2]
        self.assertEqual((shown['target_id'], 'robot_b'), (approved['target_id'], approved['robot_id']))
        self.assertEqual([('NAV_NOT_READY', 'robot_b')], [(e['code'], e['robot_id']) for e in final['errors']])
        self.assertEqual([2], [m['revision'] for m in final['maps']])
        self.assertEqual({'robot_a': 'IDLE', 'robot_b': 'IDLE'}, {r['robot_id']: r['motion_state']
                                                                  for r in final['robots']})

    def test_replaying_the_flow_after_a_restart_changes_nothing_but_the_error_log(self):
        self.run_flow()
        before = screen(self.path)
        with self.ledger() as ledger:  # operations restarted and its adapter re-sent everything
            for label, _, writes in FLOW:
                for (kind, _), result in zip(writes, apply(ledger, writes)):
                    with self.subTest(step=label, kind=kind):
                        if kind == 'error':
                            self.assertIs(type(result), int)  # append-only: a replay adds a row
                        else:
                            self.assertIs(False, result)
        after = screen(self.path)
        self.assertEqual(len(before['errors']) * 2, len(after['errors']))
        self.assertEqual({k: v for k, v in before.items() if k != 'errors'},
                         {k: v for k, v in after.items() if k != 'errors'})

    def test_late_snapshot_does_not_move_the_screen_backwards(self):
        self.run_flow()
        with self.ledger() as ledger:
            self.assertFalse(ledger.record_mission_state(mission('SEARCHING', 12)))
            self.assertFalse(ledger.record_target_state(target(120, confirmed=True)))
            self.assertFalse(ledger.record_robot_status(robot('robot_b', 2, 131, 'MOVING', task='approach')))
        final = screen(self.path)
        self.assertEqual('FINISHED', final['mission']['phase'])
        self.assertEqual('RESCUE_CONFIRMED', final['targets'][0]['rescue_state'])

    def test_false_alarm_clears_through_clearing_back_to_normal(self):
        """S-00: DISASTER → CLEARING (stop running tasks, SCOUT returns) → NORMAL."""
        steps = [mission('IDLE', 0, mode='NORMAL', generation=0), mission('DISASTER', 11),
                 mission('SEARCHING', 12), mission('RETURNING', 20, mode='CLEARING'),
                 mission('IDLE', 30, mode='NORMAL', generation=2)]
        modes = []
        with self.ledger() as ledger:
            ledger.record_site_event(SIGN)
            for step in steps:
                self.assertTrue(ledger.record_mission_state(step))
                modes.append(site_state.read_mission_state(self.path)['mission']['mode'])
        self.assertEqual(['NORMAL', 'DISASTER', 'DISASTER', 'CLEARING', 'NORMAL'], modes)
        self.assertLessEqual(set(modes), SYSTEM_MODES)

    def test_operator_arms_the_camera_watch_before_a_candidate_can_exist(self):
        """C-11 2행: the operator's SET_CAMERA_WATCH (1차 IDL since #77) is kept in the command history."""
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_operator_command(command('watch', 'SET_CAMERA_WATCH', 390, True,
                                                                   robot_id=None)))


class SiteFlowStorage(unittest.TestCase):
    """The rehearsal's DB file is a plain site-state 1.1 file the console reads without help."""

    def test_rehearsal_file_is_schema_1_1_and_has_every_table(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'site_state.sqlite3'
            with site_state.SiteStateLedger(path, allowed_robot_ids=ROBOTS) as ledger:
                for _, _, writes in FLOW:
                    apply(ledger, writes)
            db = sqlite3.connect(path)
            try:
                schema = dict(db.execute('SELECT key, value FROM metadata'))['schema']
                tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type = 'table'")}
            finally:
                db.close()
        self.assertEqual(site_state.SCHEMA_VERSION, schema)
        self.assertLessEqual({'robot_status', 'site_events', 'target_states', 'error_log', 'mission_state',
                              'operator_commands', 'map_snapshots'}, tables)

    def test_a_write_outside_the_vocabulary_is_refused_not_stored(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'site_state.sqlite3'
            with site_state.SiteStateLedger(path, allowed_robot_ids=ROBOTS) as ledger:
                with self.assertRaises(OperationsError):
                    ledger.record_mission_state(mission('LOST', 5))
            self.assertIsNone(site_state.read_mission_state(path)['mission'])


if __name__ == '__main__':
    unittest.main()
