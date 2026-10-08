"""Synthetic two-robot contract rehearsal. No ROS, model or physical movement.

Fixture times are exact integer seconds/nanoseconds. This is not a production
float-to-ROS-time adapter. Four hosts describe placement, not four processes.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import struct
import zlib
from uuid import NAMESPACE_URL, uuid5

from rescue_bringup.profile import load_deployment
from rescue_perception import (AlignmentEvidence, CameraIntrinsics, ConservativeEventRegistry,
    DepthFrame, DepthPolicy, Detection, FrameMeta, ModelManifest, ObservationRegistry,
    TimePolicy, locate_detection, make_observation, PerceptionError)
from rescue_mapping import (FrameAlignment, MappingRegistry, RigidTransform, SourceObservation, TimedTransform)
from rescue_coordination.planning import (ApproachPolicy, PathCandidate, Reservation, Allocator,
    Corridor, select_candidate)
from rescue_robot.executor import Policy, Readiness, Task, Executor
from rescue_operations.ledger import ReportLedger, ReportOutbox, LedgerTransport, read_reports


def uid(label):
    return str(uuid5(NAMESPACE_URL, 'rokey7/offline-fixture/' + label))


def to_ns(seconds):
    """Copy one fixture instant into integer nanoseconds.

    The float seconds and the nanosecond fields are not two handwritten copies.
    This is not a ROS sec/nanosec adapter and does not recover precision that a
    float already lost.
    """
    if type(seconds) is not float:
        raise ValueError('fixture seconds must be a float instant')
    scaled = seconds * 1_000_000_000
    if scaled != int(scaled):
        raise ValueError('fixture seconds are not an exact nanosecond')
    return int(scaled)


def _synthetic_evidence(robot):
    """A generated 4x4 solid-colour PNG, never a camera/person photograph."""
    pixel = b'\xff\x00\x00' if robot == 'robot_a' else b'\x00\x00\xff'
    def chunk(kind, data):
        return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data))
    raw = b''.join(b'\x00' + pixel * 4 for _ in range(4))
    return (b'\x89PNG\r\n\x1a\n' +
            chunk(b'IHDR', struct.pack('!IIBBBBB', 4, 4, 8, 2, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


def _observation(robot, clock='offline-fixture'):
    frame = FrameMeta(robot + '/optical', 100., clock, 'CAPTURE', 4, 4, 'fixture-cal')
    detection = Detection(0, 'person', .9, (0, 0, 4, 4), 'same-local-track-7')
    location = locate_detection(frame, DepthFrame(frame, '16UC1', 'mm', [[2000]*4 for _ in range(4)]),
        CameraIntrinsics(frame.frame_id, frame.calibration_id, 4, 4, 2., 2., 1.5, 1.5, True),
        AlignmentEvidence('fixture-only', frame.calibration_id, frame.frame_id, frame.frame_id, True),
        detection, DepthPolicy(.02, .5, 5., 2, .5, .5))
    return make_observation(observation_id=uid(robot+'/observation'), source_robot=robot,
        source_camera='front', source_boot_id=uid(robot+'/boot'), frame=frame,
        model=ModelManifest('synthetic-no-inference', hashlib.sha256(b'synthetic-fixture').hexdigest(), ('person',)),
        detection=detection, localization=location, valid_until=102., clock_state='SYNCED',
        clock_error_bound=.01, evidence_id=uid(robot+'/synthetic-evidence'),
        evidence_sha256=hashlib.sha256(_synthetic_evidence(robot)).hexdigest())


def _search_rehearsal(robot):
    policy = Policy('SYNTHETIC_NOT_LOCKED_FOR_RUN', 1., 3., .2, .11, .5, .01, .01)
    e = Executor(robot, uid(robot+'/boot'), uid('authority'), policy)
    ready = Readiness(.2, True, True, True, True, True, True)
    for t in (0., .1, .2):
        e.observe_motion(sample_steady=t, received_steady=t, linear=0., angular=0.)
    if not e.arm(uid('authority'), now=.2, readiness=ready):
        raise RuntimeError('fixture failed to arm')
    task = Task(uid(robot+'/command'), uid(robot+'/search'), robot, uid(robot+'/boot'),
                uid('authority'), 1, 'SEARCH_AREA', 100., 120., 10., policy.version)
    if not e.admit(task, now_wall=100., now_steady=.2, readiness=ready).accepted:
        raise RuntimeError('fixture task rejected')
    if not e.dispatch(now_wall=100., now_steady=.2, readiness=ready, guards_valid=True):
        raise RuntimeError('fixture dispatch rejected')
    intents = list(e.drain_intents())
    e.goal_response(task.command_id, accepted=True, now=.3)
    e.goal_terminal(task.command_id, succeeded=True, now=.5)
    for t in (.6, .7, .8, .9):
        e.observe_motion(sample_steady=t, received_steady=t, linear=0., angular=0.)
    return {'robot_id': robot, 'task_kind': task.kind, 'synthetic_intents': intents,
            'real_command_sent': False, 'evidence_source': 'SYNTHETIC_ACTION_AND_ODOMETRY',
            'result': asdict(e.results[task.command_id])}


def run_scenario(profile_path, output_directory):
    profile = load_deployment(profile_path)
    if {robot.id for robot in profile.robots} != {'robot_a', 'robot_b'}:
        raise ValueError('offline fixture requires logical robot_a and robot_b profiles')
    output = Path(output_directory)
    # A run is append-only as a directory. Refuse to overwrite/reuse old evidence.
    output.mkdir(parents=True, exist_ok=False)
    clock = 'offline-fixture'
    perception, events = ObservationRegistry(), ConservativeEventRegistry()
    mapping = MappingRegistry(clock_domain_id=clock, alignment_authority='fixture-mapping',
                              max_position_error_m=.2, max_rotation_error_rad=.1)
    observations, projections, event_ids = {}, {}, {}
    snapshots = []
    for robot, offset in (('robot_a', 0.), ('robot_b', 1.)):
        mapping.start_session(robot, 'map', uid(robot+'/map-session'))
        mapping.set_alignment(FrameAlignment(robot, 'map', 'mission', uid(robot+'/map-session'),
            0, 1, 'fixture-mapping', clock, RigidTransform.planar(offset, 0., 0.),
            to_ns(99.), to_ns(103.), .1, .05))
        obs = _observation(robot)
        admission = perception.ingest(obs, now=100.5, received_steady=10.,
            receiver_clock_domain=clock, receiver_clock_error_bound=.01, policy=TimePolicy(2., .02, .04, .01))
        event = events.register(admission, event_id=uid(robot+'/event'))
        observed_at_ns = to_ns(obs.observed_at)
        valid_until_ns = to_ns(obs.valid_until)
        source = SourceObservation(obs.observation_id, robot, obs.source_camera, obs.source_boot_id,
            observed_at_ns, obs.frame.frame_id, obs.localization.point_optical,
            valid_until_ns, clock, uid(robot+'/map-session'), 0)
        transform = TimedTransform(robot, obs.frame.frame_id, 'map', uid(robot+'/map-session'), 0,
            clock, source.observed_at_ns, RigidTransform((0., 0., 0.), (-.5, .5, -.5, .5)),
            'EXACT', (source.observed_at_ns,), 'fixture-observation-time-tf')
        observations[robot] = obs
        projections[robot] = mapping.project(source, transform, now_ns=to_ns(100.5))
        event_ids[robot] = event.event_id

    allocator = Allocator(uid('authority'))
    policy = ApproachPolicy('SYNTHETIC_NOT_LOCKED_FOR_RUN', .6, 1.4, .3, .2, 1., .5)
    proposals = []
    for observer, responder in (('robot_a', 'robot_b'), ('robot_b', 'robot_a')):
        target = projections[observer].mission_point_xyz
        candidate = PathCandidate(responder, (target[0]+1., target[1]), target[:2], 4., .4, .1,
            102., uid(responder+'/map-session'), 0, 1, True, True, True)
        choice = select_candidate([candidate], now=100.5, policy=policy, available_robots=(responder,))
        if choice is None:
            raise RuntimeError('fixture proposal rejected')
        snapshot = mapping.capture_goal_snapshot((observations[observer].observation_id,), responder,
            (choice.goal_xy[0], choice.goal_xy[1], target[2]), now_ns=to_ns(100.5))
        validation = mapping.validate_goal_snapshot(snapshot, now_ns=to_ns(100.5))
        item = Reservation(uid(observer+'/handoff'), event_ids[observer], 0, responder, 2,
                           uid('authority'), choice)
        if not allocator.reserve(item):
            raise RuntimeError('fixture assignment rejected')
        snapshots.append(snapshot)
        proposals.append({'observer': observer, 'responder': responder, 'task_id': item.task_id,
            'coordinates_valid': validation.coordinates_valid,
            'auto_approach_eligible': validation.auto_approach_eligible,
            'approach_command_sent': False, 'reason': 'SLAM_BACKEND_CORRECTION_UNVERIFIED',
            'goal_snapshot': snapshot.snapshot_id,
            'source_observation': observations[observer].observation_id,
            'target_mission_xyz': target, 'candidate_mission_xyz': snapshot.mission_point_xyz,
            'candidate_destination_xyz': snapshot.destination_point_xyz})
    # This rehearsal must stay blocked even though the synthetic geometry is valid.
    if any(p['auto_approach_eligible'] for p in proposals):
        raise RuntimeError('offline backend must not authorize movement')

    stale_rejected = False
    try:
        perception.ingest(observations['robot_a'], now=102., received_steady=12.,
            receiver_clock_domain=clock, receiver_clock_error_bound=.01, policy=TimePolicy(2., .02, .04, .01))
    except PerceptionError as exc:
        if exc.code != 'STALE_OBSERVATION':
            raise
        stale_rejected = True
    invalidation = mapping.advance_map_revision('robot_a', uid('robot_a/map-session'), 1)
    a_status = mapping.projection(observations['robot_a'].observation_id, now_ns=to_ns(101.)).status
    b_status = mapping.projection(observations['robot_b'].observation_id, now_ns=to_ns(101.)).status
    corridor = Corridor()
    corridor.confirm_vacant(expected_revision=0, verified_exit=True, observation_fresh=True, now=0.)
    corridor.acquire(item, now=0., duration=1.)
    occupancy = corridor.check(now=1., holder_pose_fresh=True)

    post_revision_validations = [mapping.validate_goal_snapshot(s, now_ns=to_ns(101.))
                                 for s in snapshots]
    searches = [_search_rehearsal(r) for r in ('robot_a', 'robot_b')]
    report_ids = []
    pending_evidence_states = []
    loss_state = None
    with ReportLedger(output/'operations.sqlite3', evidence_policy='OPTIONAL') as ledger:
        with ReportOutbox(output/'outbox.sqlite3') as outbox:
            for index, p in enumerate(proposals):
                obs = observations[p['observer']]
                payload = {'schema_version': '1.0', 'report_id': uid(p['observer']+'/report'),
                    'target_id': event_ids[p['observer']], 'observation_id': obs.observation_id,
                    'source_robot': obs.source_robot, 'source_camera': obs.source_camera,
                    'source_boot_id': obs.source_boot_id, 'clock_domain_id': clock,
                    'robot_id': p['responder'], 'task_id': p['task_id'], 'generation': 2,
                    'observation_result': 'DETECTED', 'confirmation_result': 'UNCONFIRMED',
                    'confirmation_observation_id': None, 'approach_result': 'NOT_ATTEMPTED',
                    # No current location is claimed after invalidation; discovery is retained.
                    'position': None, 'observed_at_ns': to_ns(obs.observed_at),
                    'confirmed_at_ns': None, 'reported_at_ns': to_ns(101.),
                    'reason': 'SYNTHETIC fixture; generated colour tile, not a camera photo: '+p['reason'],
                    'evidence_refs': [{'evidence_id': obs.evidence_id, 'sha256': obs.evidence_sha256,
                        'media_type': 'image/png', 'observation_id': obs.observation_id,
                        'acquired_at_ns': to_ns(obs.observed_at)}],
                    'model_id': obs.model.model_id, 'model_sha256': obs.model.sha256,
                    'deployment_version': 'offline-fixture-not-a-field-run'}
                envelope = {'schema_version': '1.0', 'publisher_id': 'offline-demo',
                    'publisher_boot_id': uid('publisher'), 'sequence': index, 'published_at_ns': to_ns(101.)}
                outbox.enqueue(payload)
                ledger.receive(payload, envelope)
                pending_evidence_states.append(ledger.persist(payload['report_id']).status)
                ledger.store_evidence(payload['evidence_refs'][0], _synthetic_evidence(p['observer']))
                if index == 0:
                    def lost_receipt(body, headers):
                        LedgerTransport(ledger)(body, headers)
                        raise ConnectionError('synthetic receipt loss after commit')
                    if outbox.deliver(payload['report_id'], envelope, lost_receipt) is not None:
                        raise RuntimeError('expected lost receipt')
                    loss_state = outbox.get(payload['report_id'])['state']
                result = outbox.deliver(payload['report_id'], envelope, LedgerTransport(ledger))
                if result.status != 'PERSISTED':
                    raise RuntimeError('report did not persist')
                report_ids.append(payload['report_id'])
    # Reopen both stores rather than treating a same-process acknowledgement as persistence proof.
    with ReportOutbox(output/'outbox.sqlite3') as reopened:
        outbox_records = [reopened.get(r) for r in report_ids]
        recovered = [row['state'] for row in outbox_records]
    stored = read_reports(output/'operations.sqlite3')
    checks = {
        'two_distinct_candidate_events': len(set(event_ids.values())) == 2,
        'both_roles_proposed': {(p['observer'], p['responder']) for p in proposals}
            == {('robot_a', 'robot_b'), ('robot_b', 'robot_a')},
        'unverified_backend_blocks_approach': all(not p['auto_approach_eligible'] for p in proposals),
        'stale_republication_rejected': stale_rejected,
        'a_revision_invalidates_only_a_observations': a_status == 'NEEDS_REOBSERVATION' and b_status == 'VALID'
            and observations['robot_b'].observation_id not in invalidation.observation_ids,
        'affected_goals_blocked_after_a_revision': all(not v.coordinates_valid for v in post_revision_validations),
        'synthetic_search_action_and_stop_complete': all(
            s['result']['outcome'] == 'COMPLETED' and s['result']['stop_confirmed']
            and not s['real_command_sent'] for s in searches),
        'declared_evidence_waits_for_actual_bytes': pending_evidence_states == ['PENDING_EVIDENCE', 'PENDING_EVIDENCE'],
        'receipt_loss_retries_original_report': loss_state == 'RETRY_PENDING'
            and [row['attempts'] for row in outbox_records] == [2, 1],
        'expired_corridor_unknown': occupancy == 'OCCUPANCY_UNKNOWN',
        'reports_unique_after_receipt_loss_and_reopen': len(stored['reports']) == 2
            and recovered == ['PERSISTED', 'PERSISTED'],
    }
    summary = {'schema_version': 1, 'evidence_level': 'E2_SYNTHETIC_IN_PROCESS',
        'physical_robots_used': 0, 'master_processes_used': 1, 'model_inference_run': False,
        'ros_run': False, 'field_acceptance': 'NOT_RUN', 'slam_backend_integration': 'BLOCKED',
        'deployment_profile': str(Path(profile_path).name),
        'profile_sha256': hashlib.sha256(Path(profile_path).read_bytes()).hexdigest(),
        'configured_deployment': asdict(profile), 'configured_hosts_used_as_processes': False,
        'synthetic_evidence': 'Generated solid-colour PNG bytes; no camera or person photograph',
        'revision_check': {'changed_robot': 'robot_a', 'a_status': a_status, 'b_status': b_status,
            'invalidated_observation_ids': list(invalidation.observation_ids),
            'goal_valid_after_revision': [v.coordinates_valid for v in post_revision_validations]},
        'outbox_recovery': {'states': recovered, 'attempts': [row['attempts'] for row in outbox_records],
            'state_after_lost_receipt': loss_state},
        'proposals': proposals, 'synthetic_search': searches,
        'checks': checks, 'passed': all(checks.values()), 'reports': stored['reports']}
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    if not summary['passed']:
        raise RuntimeError('contract rehearsal failed; inspect summary.json')
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile')
    parser.add_argument('--output', required=True, help='new directory; existing paths are rejected')
    args = parser.parse_args(argv)
    try:
        result = run_scenario(args.profile, args.output)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'OFFLINE REHEARSAL FAILED: {exc}', file=sys.stderr)
        return 2
    print(f"E2 SYNTHETIC PASS: {len(result['checks'])} checks; E3/E4 NOT_RUN; automatic approach BLOCKED")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
