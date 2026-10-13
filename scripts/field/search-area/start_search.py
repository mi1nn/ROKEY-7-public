"""One-shot operator commands through the existing ROS CLI, not an installed node.

This file arms the executor and starts a physical SEARCH_AREA. The only synthetic
input is a clearly named disaster trigger for this isolated field test. It is
not camera evidence or a report of an actual disaster. No input files are read
or written; real clock, map, AMCL, readiness, lease and stop guards remain active.
"""
import argparse
import hashlib
import json
import subprocess
import time
from uuid import uuid4

from rescue_contracts.clock_measurement import collect


def run(argv, expected=None):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        raise SystemExit('Operation timed out; no later command sent: ' + ' '.join(argv[:5]))
    print(result.stdout, end='', flush=True)
    if result.stderr:
        print(result.stderr, end='', flush=True)
    if result.returncode or (expected is not None and expected not in result.stdout):
        raise SystemExit('Operation refused or unavailable; no later command sent.')


def service(name, kind, request, expected=None):
    run(['ros2', 'service', 'call', name, kind, json.dumps(request)], expected)


def main():
    parser = argparse.ArgumentParser(description='Operator arm and physical SEARCH_AREA start')
    parser.add_argument('--synthetic-trigger', action='store_true',
        help='Explicitly inject a marked simulated disaster trigger for this isolated field test')
    args = parser.parse_args()
    print('PHYSICAL SEARCH_AREA START. Synthetic trigger=' + str(args.synthetic_trigger), flush=True)
    # Obtain a real AMCL update on the next real laser scan while stationary.
    service('/robot6/request_nomotion_update', 'std_srvs/srv/Empty', {})
    time.sleep(1.0)
    # Verify the actual local clock before any arm request or synthetic event.
    clock = collect('timesyncd', max_age_ns=3600000000000, drift_ppm=100.0)
    if clock['state'] != 'SYNCED' or clock['error_bound_ns'] > 2000000000:
        raise SystemExit('Measured OS clock unavailable or outside the configured bound; start aborted.')
    # Publish fresh central state after the executor clock startup had time to run.
    service('/mission_coordinator/refresh_state', 'std_srvs/srv/Trigger', {}, 'success=True')
    service('/robot6/rescue_command_adapter/task_arm', 'std_srvs/srv/Trigger', {}, 'success=True')

    boot, publisher = (str(uuid4()) for _ in range(2))
    if args.synthetic_trigger:
        event_id, evidence_id = (str(uuid4()) for _ in range(2))
        stamp = time.time_ns()
        fixture = json.dumps(dict(synthetic=True, purpose='manual SEARCH_AREA integration test',
            event_id=event_id, generated_at_ns=stamp), sort_keys=True, separators=(',', ':')).encode()
        print('SYNTHETIC_TRIGGER_PAYLOAD=' + fixture.decode(), flush=True)
        event = dict(
            envelope=dict(schema_version='rescue-envelope/1', publisher_id=publisher,
                publisher_boot_id=boot, sequence=1, published_at_ns=stamp),
            event_id=event_id, camera_id='cam_manual_search_test', source_boot_id=boot,
            kind='DISASTER_SIGN',
            time=dict(observed_at_ns=stamp, valid_until_ns=stamp + 30000000000,
                stamp_semantics='DRIVER_RECEIVE', clock_domain_id='rokey7-utc-field',
                clock_state=clock['state'], has_clock_error_bound=True,
                clock_error_bound_ns=clock['error_bound_ns']),
            target_id='', watch_id='', confidence=1.0, visibility_state='VALID',
            metrics_name=['SYNTHETIC_MANUAL_TEST'], metrics_value=[1.0],
            policy_version='MANUAL_SEARCH_TEST_SYNTHETIC',
            evidence=dict(evidence_id=evidence_id, sha256=hashlib.sha256(fixture).hexdigest(),
                mime_type='application/json', source_id='cam_manual_search_test',
                source_boot_id=boot, observed_at_ns=stamp))
        run(['ros2', 'topic', 'pub', '--once', '--keep-alive', '3.0', '/rescue/site_events',
            'rescue_interfaces/msg/SiteEvent', json.dumps(event)])
    # Publishing is not confirmation that the mission gate consumed the event.
    # Receive the real retained central mode before sending the start command.
    # The coordinator still performs its own mode/readiness/authority checks.
    run(['ros2', 'topic', 'echo', '/rescue/system_state',
        'rescue_interfaces/msg/SystemState', '--once', '--field', 'mode',
        '--qos-durability', 'transient_local', '--qos-reliability', 'reliable'], 'DISASTER')
    # Only this request authorizes the task and its renewed real lease.
    stamp = time.time_ns()
    command = dict(
        envelope=dict(schema_version='rescue-envelope/1', publisher_id=publisher,
            publisher_boot_id=boot, sequence=2, published_at_ns=stamp),
        command_id=str(uuid4()), operation='START_SEARCH', operator_id='manual_field_test',
        robot_id='robot_a', issued_at_ns=stamp, deadline_ns=stamp + 30000000000)
    service('/rescue/operator_command', 'rescue_interfaces/srv/SubmitOperatorCommand',
        command, 'accepted=True')
    print('START_SEARCH accepted. Completion is reported by the running central/executor, not by this CLI.', flush=True)


if __name__ == '__main__':
    main()
