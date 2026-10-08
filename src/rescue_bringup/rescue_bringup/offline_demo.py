"""Synthetic two-robot contract rehearsal. No ROS, model or physical movement.

Fixture times are exact integer seconds/nanoseconds. This is not a production
float-to-ROS-time adapter. Four hosts describe placement, not four processes.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import sys
import struct
import zlib
from uuid import NAMESPACE_URL, uuid5

from rescue_bringup.profile import load_deployment
from rescue_contracts import ros_time_to_ns, seconds_to_ns
from rescue_perception import (AlignmentEvidence, CameraIntrinsics, ConservativeEventRegistry,
    DepthFrame, DepthPolicy, Detection, FrameMeta, ModelManifest, ObservationRegistry,
    TimePolicy, locate_detection, make_observation, PerceptionError)
from rescue_mapping import (FrameAlignment, MappingError, MappingRegistry, RigidTransform, SourceObservation, TimedTransform)
from rescue_coordination.planning import (ApproachPolicy, PathCandidate, Reservation, Allocator,
    Corridor, select_candidate)
from rescue_robot.executor import Policy, Readiness, Task, Executor
from rescue_operations.ledger import ReportLedger, ReportOutbox, LedgerTransport, read_reports


def uid(label):
    return str(uuid5(NAMESPACE_URL, 'rokey7/offline-fixture/' + label))


# One original ROS stamp, never a float epoch or separately handwritten ns copy.
OBSERVED_AT_NS = ros_time_to_ns(1700000000, 123456789)


def fixture_time(offset_seconds):
    """Derive wall instants from the original stamp; local timers remain seconds."""
    return OBSERVED_AT_NS + seconds_to_ns(offset_seconds)


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
    frame = FrameMeta(robot + '/optical', OBSERVED_AT_NS, clock, 'CAPTURE', 4, 4, 'fixture-cal')
    detection = Detection(0, 'person', .9, (0, 0, 4, 4), 'same-local-track-7', 'TRACKING')
    location = locate_detection(frame, DepthFrame(frame, '16UC1', 'mm', [[2000]*4 for _ in range(4)]),
        CameraIntrinsics(frame.frame_id, frame.calibration_id, 4, 4, 2., 2., 1.5, 1.5, True),
        AlignmentEvidence('fixture-only', frame.calibration_id, frame.frame_id, frame.frame_id, True),
        detection, DepthPolicy(seconds_to_ns("0.02"), .5, 5., 2, .5, .5))
    return make_observation(observation_id=uid(robot+'/observation'), source_robot=robot,
        source_camera='front', source_boot_id=uid(robot+'/boot'), frame=frame,
        model=ModelManifest('synthetic-no-inference', hashlib.sha256(b'synthetic-fixture').hexdigest(), ('person',)),
        detection=detection, localization=location, valid_until_ns=fixture_time(2), clock_state='SYNCED',
        clock_error_bound_ns=seconds_to_ns("0.01"), evidence_id=uid(robot+'/synthetic-evidence'),
        evidence_sha256=hashlib.sha256(_synthetic_evidence(robot)).hexdigest())


def _start_search(robot, allocator, allowed_robot_ids):
    policy = Policy('SYNTHETIC_NOT_LOCKED_FOR_RUN', 1., 3., .2, .11, .5, .01, .01)
    executor = Executor(robot, uid(robot+'/boot'), uid('authority'), policy,
                        allowed_robot_ids=allowed_robot_ids)
    ready = Readiness(.2, True, True, True, True, True, True)
    for t in (0., .1, .2):
        executor.observe_motion(sample_steady=t, received_steady=t, linear=0., angular=0.)
    if not executor.arm(uid('authority'), now=.2, readiness=ready):
        raise RuntimeError('fixture failed to arm')
    task = Task(uid(robot+'/command'), uid(robot+'/search'), robot, uid(robot+'/boot'),
                uid('authority'), allocator.issue_generation(robot), 'SEARCH_AREA',
                fixture_time("0.2"), fixture_time(20), 10., policy.version)
    receipt = executor.admit(task, now_wall_ns=fixture_time("0.2"), now_steady=.2, readiness=ready)
    if not receipt.accepted:
        raise RuntimeError('fixture search rejected')
    if not executor.dispatch(now_wall_ns=fixture_time("0.2"), now_steady=.2,
                             readiness=ready, guards_valid=True):
        raise RuntimeError('fixture dispatch rejected')
    intents = list(executor.drain_intents())
    executor.goal_response(task.command_id, accepted=True, now=.3)
    return executor, task, receipt, intents


def _handoff_rehearsal(search, observer, target_id, candidate, validation, allocator):
    """Use the same executor through cancellation, stop, rearm and admission.

    All callbacks and odometry are synthetic. Admission is not SEND permission:
    MappingRegistry's operational gate still prevents the approach dispatch.
    """
    executor, search_task, search_receipt, search_intents = search
    executor.request_stop('ACTION_CANCELED', now=.5)
    search_intents.extend(executor.drain_intents())
    terminal_received = executor.goal_terminal(search_task.command_id, terminal_status='CANCELED', now=.6)
    result_before_stop = search_task.command_id in executor.results
    stop_samples = (.7, .8, .9, 1.)
    for t in stop_samples:
        executor.observe_motion(sample_steady=t, received_steady=t, linear=0., angular=0.)
    result = executor.results[search_task.command_id]
    ready = Readiness(1., True, True, True, True, True, True)
    if not executor.arm(uid('authority'), now=1., readiness=ready):
        raise RuntimeError('fixture did not confirm stop before handoff')
    robot = executor.robot_id
    new_generation = allocator.issue_generation(robot)
    reservation = Reservation(uid(observer+'/handoff'), target_id, 0, robot, new_generation,
        uid('authority'), candidate, uid(observer+'/approach-command'), uid(robot+'/boot'))
    if not allocator.reserve(reservation):
        raise RuntimeError('fixture reservation rejected')
    active_before_admission = reservation.task_id in allocator.active
    task = Task(reservation.command_id, reservation.task_id, robot, reservation.robot_boot_id,
        reservation.authority_epoch, new_generation, 'APPROACH_TARGET', fixture_time(1),
        fixture_time(20), 10., executor.policy.version, target_id, 0)
    receipt = executor.admit(task, now_wall_ns=fixture_time(1), now_steady=1., readiness=ready)
    recorded = allocator.record_admission(receipt)
    if not receipt.accepted or not recorded:
        raise RuntimeError('fixture approach admission rejected')
    dispatched = executor.dispatch(now_wall_ns=fixture_time(1), now_steady=1., readiness=ready,
        guards_valid=validation.coordinates_valid and validation.auto_approach_eligible)
    approach_intents = list(executor.drain_intents())
    return reservation, {'robot_id': robot, 'task_kind': search_task.kind,
        'sequence': ['SEARCH_SEND', 'CANCEL_REQUEST', 'TERMINAL_CANCELED', 'STOP_CONFIRMED',
                     'NEW_GENERATION', 'APPROACH_ADMISSION', 'ALLOCATOR_RECORD_ADMISSION', 'APPROACH_SEND_BLOCKED'],
        'search_admission': asdict(search_receipt), 'synthetic_intents': search_intents,
        'terminal_received': terminal_received, 'search_result_before_stop_samples': result_before_stop,
        'synthetic_stop_sample_times': stop_samples, 'result': asdict(result),
        'approach_admission': asdict(receipt), 'new_generation': new_generation,
        'allocator_active_before_admission': active_before_admission,
        'allocator_recorded_admission': recorded, 'allocator_active_after_admission': reservation.task_id in allocator.active,
        'approach_dispatched': dispatched, 'approach_intents': approach_intents,
        'approach_executor_state': executor.state, 'approach_stop_cause': executor.stop_cause,
        'real_command_sent': False, 'evidence_source': 'SYNTHETIC_ACTION_AND_ODOMETRY'}


def run_scenario(profile_path, output_directory):
    profile = load_deployment(profile_path)
    if {robot.id for robot in profile.robots} != {'robot_a', 'robot_b'}:
        raise ValueError('offline fixture requires logical robot_a and robot_b profiles')
    output = Path(output_directory)
    # A run is append-only as a directory. Refuse to overwrite/reuse old evidence.
    output.mkdir(parents=True, exist_ok=False)
    allowed_robot_ids = tuple(robot.id for robot in profile.robots)
    allocator = Allocator(uid('authority'), allowed_robot_ids=allowed_robot_ids)
    searches = {robot: _start_search(robot, allocator, allowed_robot_ids) for robot in allowed_robot_ids}
    clock = 'offline-fixture'
    perception, events = ObservationRegistry(), ConservativeEventRegistry()
    mapping = MappingRegistry(clock_domain_id=clock, alignment_authority='fixture-mapping',
                              max_position_error_m=.2, max_rotation_error_rad=.1)
    observations, projections, event_ids = {}, {}, {}
    snapshots, timestamp_checks = [], []
    for robot, offset in (('robot_a', 0.), ('robot_b', 1.)):
        mapping.start_session(robot, 'map', uid(robot+'/map-session'))
        mapping.set_alignment(FrameAlignment(robot, 'map', 'mission', uid(robot+'/map-session'),
            0, 1, 'fixture-mapping', clock, RigidTransform.planar(offset, 0., 0.),
            OBSERVED_AT_NS - seconds_to_ns(1), fixture_time(3), .1, .05))
        obs = _observation(robot)
        admission = perception.ingest(obs, now_ns=fixture_time("0.5"), received_steady=10.,
            receiver_clock_domain=clock, receiver_clock_error_bound_ns=seconds_to_ns("0.01"), policy=TimePolicy(*(seconds_to_ns(v) for v in (2, "0.02", "0.04", "0.01"))))
        event = events.register(admission, event_id=uid(robot+'/event'))
        observed_at_ns = obs.observed_at_ns
        valid_until_ns = obs.valid_until_ns
        source = SourceObservation(obs.observation_id, robot, obs.source_camera, obs.source_boot_id,
            observed_at_ns, obs.frame.frame_id, obs.localization.point_optical,
            valid_until_ns, clock, uid(robot+'/map-session'), 0)
        transform = TimedTransform(robot, obs.frame.frame_id, 'map', uid(robot+'/map-session'), 0,
            clock, source.observed_at_ns, RigidTransform((0., 0., 0.), (-.5, .5, -.5, .5)),
            'EXACT', (source.observed_at_ns,), 'fixture-observation-time-tf')
        observations[robot] = obs
        projections[robot] = mapping.project(source, transform, now_ns=fixture_time("0.5"))
        shifted_error = None
        try:
            mapping.project(source, replace(transform, query_time_ns=source.observed_at_ns + 1,
                sample_times_ns=(source.observed_at_ns + 1,)), now_ns=fixture_time("0.5"))
        except MappingError as exc:
            shifted_error = {'code': exc.code, 'detail_code': exc.detail_code}
        timestamp_checks.append({'robot_id': robot, 'perception_observed_at_ns': obs.observed_at_ns,
            'source_observed_at_ns': source.observed_at_ns, 'tf_query_time_ns': transform.query_time_ns,
            'projected_observed_at_ns': projections[robot].original.observed_at_ns,
            'shifted_tf_query_time_ns': source.observed_at_ns + 1, 'shifted_tf_error': shifted_error})
        event_ids[robot] = event.event_id

    policy = ApproachPolicy('SYNTHETIC_NOT_LOCKED_FOR_RUN', .6, 1.4, .3, .2, 1., seconds_to_ns("0.5"))
    proposals, handoffs = [], []
    for observer, responder in (('robot_a', 'robot_b'), ('robot_b', 'robot_a')):
        target = projections[observer].mission_point_xyz
        candidate = PathCandidate(responder, (target[0]+1., target[1]), target[:2], 4., .4, .1,
            fixture_time(2), uid(responder+'/map-session'), 0, 1, True, True, True)
        choice = select_candidate([candidate], now_ns=fixture_time(1), policy=policy, available_robots=(responder,))
        if choice is None:
            raise RuntimeError('fixture proposal rejected')
        snapshot = mapping.capture_goal_snapshot((observations[observer].observation_id,), responder,
            (choice.goal_xy[0], choice.goal_xy[1], target[2]), now_ns=fixture_time(1))
        validation = mapping.validate_goal_snapshot(snapshot, now_ns=fixture_time(1))
        item, handoff = _handoff_rehearsal(searches[responder], observer, event_ids[observer],
                                            choice, validation, allocator)
        handoffs.append(handoff)
        snapshots.append(snapshot)
        proposals.append({'observer': observer, 'responder': responder, 'task_id': item.task_id, 'generation': item.generation,
            'coordinates_valid': validation.coordinates_valid,
            'auto_approach_eligible': validation.auto_approach_eligible,
            'approach_command_sent': any(intent[0] == 'SEND' for intent in handoff['approach_intents']), 'reason': 'SLAM_BACKEND_CORRECTION_UNVERIFIED',
            'goal_snapshot': snapshot.snapshot_id,
            'source_observation': observations[observer].observation_id,
            'target_mission_xyz': target, 'candidate_mission_xyz': snapshot.mission_point_xyz,
            'candidate_destination_xyz': snapshot.destination_point_xyz})
    # This rehearsal must stay blocked even though the synthetic geometry is valid.
    if any(p['auto_approach_eligible'] for p in proposals):
        raise RuntimeError('offline backend must not authorize movement')

    stale_rejected = False
    try:
        perception.ingest(observations['robot_a'], now_ns=fixture_time(2), received_steady=12.,
            receiver_clock_domain=clock, receiver_clock_error_bound_ns=seconds_to_ns("0.01"), policy=TimePolicy(*(seconds_to_ns(v) for v in (2, "0.02", "0.04", "0.01"))))
    except PerceptionError as exc:
        if exc.code != 'STALE_OBSERVATION':
            raise
        stale_rejected = True
    invalidation = mapping.advance_map_revision('robot_a', uid('robot_a/map-session'), 1)
    a_status = mapping.projection(observations['robot_a'].observation_id, now_ns=fixture_time("1.25")).status
    b_status = mapping.projection(observations['robot_b'].observation_id, now_ns=fixture_time("1.25")).status
    corridor = Corridor()
    corridor.confirm_vacant(holder=None, expected_revision=0, verified_exit=True, observation_fresh=True, now=0.)
    corridor.acquire(item, now=0., duration=1.)
    occupancy = corridor.check(now=1., holder_pose_fresh=True)

    post_revision_validations = [mapping.validate_goal_snapshot(s, now_ns=fixture_time("1.25"))
                                 for s in snapshots]
    report_ids = []
    pending_evidence_states = []
    loss_state = None
    with ReportLedger(output/'operations.sqlite3', evidence_policy='OPTIONAL', allowed_robot_ids=allowed_robot_ids) as ledger:
        with ReportOutbox(output/'outbox.sqlite3', allowed_robot_ids=allowed_robot_ids) as outbox:
            for index, p in enumerate(proposals):
                obs = observations[p['observer']]
                payload = {'schema_version': '1.0', 'report_id': uid(p['observer']+'/report'),
                    'target_id': event_ids[p['observer']], 'observation_id': obs.observation_id,
                    'source_robot': obs.source_robot, 'source_camera': obs.source_camera,
                    'source_boot_id': obs.source_boot_id, 'clock_domain_id': clock,
                    'robot_id': p['responder'], 'task_id': p['task_id'], 'generation': p['generation'],
                    'observation_result': 'DETECTED', 'confirmation_result': 'UNCONFIRMED',
                    'confirmation_observation_id': None, 'approach_result': 'NOT_ATTEMPTED',
                    # No current location is claimed after invalidation; discovery is retained.
                    'position': None, 'observed_at_ns': obs.observed_at_ns,
                    'confirmed_at_ns': None, 'reported_at_ns': fixture_time("1.25"),
                    'reason': 'SYNTHETIC fixture; generated colour tile, not a camera photo: '+p['reason'],
                    'evidence_refs': [{'evidence_id': obs.evidence_id, 'sha256': obs.evidence_sha256,
                        'media_type': 'image/png', 'observation_id': obs.observation_id,
                        'acquired_at_ns': obs.observed_at_ns}],
                    'model_id': obs.model.model_id, 'model_sha256': obs.model.sha256,
                    'deployment_version': 'offline-fixture-not-a-field-run'}
                envelope = {'schema_version': '1.0', 'publisher_id': 'offline-demo',
                    'publisher_boot_id': uid('publisher'), 'sequence': index, 'published_at_ns': fixture_time("1.25")}
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
    with ReportOutbox(output/'outbox.sqlite3', allowed_robot_ids=allowed_robot_ids) as reopened:
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
        'synthetic_search_cancel_stop_handoff': all(
            s['terminal_received'] and not s['search_result_before_stop_samples']
            and s['result']['outcome'] == 'CANCELED' and s['result']['action_status'] == 'CANCELED'
            and s['result']['stop_confirmed'] and not s['real_command_sent']
            and s['new_generation'] > s['search_admission']['generation'] for s in handoffs),
        'actual_admission_connects_allocator_without_approach_send': all(
            not s['allocator_active_before_admission'] and s['approach_admission']['accepted']
            and s['allocator_recorded_admission'] and s['allocator_active_after_admission']
            and not s['approach_dispatched'] and not s['approach_intents'] for s in handoffs),
        'epoch_nanoseconds_preserved_through_actual_mapping': all(
            all(c[k] == OBSERVED_AT_NS for k in ('perception_observed_at_ns', 'source_observed_at_ns',
                'tf_query_time_ns', 'projected_observed_at_ns')) for c in timestamp_checks),
        'one_nanosecond_shifted_tf_rejected': all(c['shifted_tf_error'] == {
            'code': 'STALE_TRANSFORM', 'detail_code': 'rescue_mapping.TF_TIME_MISMATCH'} for c in timestamp_checks),
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
        'proposals': proposals, 'synthetic_search': handoffs, 'timestamp_checks': timestamp_checks,
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
