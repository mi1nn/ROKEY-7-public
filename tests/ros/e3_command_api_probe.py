"""E3 (partial): the operator command API of db_writer against the real mission coordinator node.

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy and a built rescue_interfaces).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    colcon build --packages-select rescue_interfaces   # in a scratch workspace
    source <scratch>/install/setup.bash
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_operations:<REPO>/src/rescue_coordination:<REPO>/src/rescue_bringup:$PYTHONPATH \
      python3 tests/ros/e3_command_api_probe.py <REPO> <SCRATCH_DIR>   # not -I: ROS needs PYTHONPATH

db_writer serves the API on 127.0.0.1 and the coordinator runs with the same example
profile and parameters as e3_coordinator_probe.py. This probe posts like the console does
(#115) and plays the robots' status and the webcam's DISASTER_SIGN. Checks:
1. no coordinator yet -> 503 without `accepted`, error_log ACCEPT_TIMEOUT, no command row;
2. a malformed command -> 400 INVALID_INPUT, never sent, recorded;
3. START_SEARCH before any disaster -> 200 refused with the coordinator's reason, recorded;
4. after DISASTER_SIGN with a READY scout -> START_SEARCH 200 accepted, recorded ACCEPTED;
5. the same command_id again -> 200 DUPLICATE with the first answer, the first row kept;
6. the coordinator stopped -> 503 again; db_writer still answers and exits 0 on SIGINT.
Exit code 0 = all held.
"""
import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rescue_interfaces.msg import RobotStatus, SiteEvent

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from e3_coordinator_probe import NODE_ARGS, SCOUT, SUPPORT, node_command  # noqa: E402

PORT = 18090
URL = f'http://127.0.0.1:{PORT}/api/operator-commands'


def writer_command(db_path):
    code = 'from rescue_operations.ros_node import main; main()'
    params = {'db_path': db_path, 'allowed_robot_ids': f'[{SCOUT},{SUPPORT}]', 'robot_roles': '[SCOUT,SUPPORT]',
              'status_topics': f'[/rescue/robots/{SCOUT}/status,/rescue/robots/{SUPPORT}/status]',
              'pose_topics': f'[/rescue/robots/{SCOUT}/pose,/rescue/robots/{SUPPORT}/pose]',
              'site_events_topic': '/rescue/site_events', 'system_state_topic': '/rescue/system_state',
              'targets_topic': '/rescue/targets', 'api_host': '127.0.0.1', 'api_port': str(PORT),
              'operator_command_service': '/rescue/operator_command', 'command_timeout_s': '3.0'}
    args = [item for name, value in params.items() for item in ('-p', f'{name}:={value}')]
    return [sys.executable, '-c', code, '--ros-args'] + args


def post(payload):
    data = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
    req = urllib.request.Request(URL, data=data, method='POST', headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=5) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


class Field(Node):
    """Robot status of both robots and the webcam's DISASTER_SIGN, as the coordinator probe sends them."""

    def __init__(self):
        super().__init__('e3_command_api_field')
        self.status = {r: self.create_publisher(RobotStatus, f'/rescue/robots/{r}/status', 1) for r in (SCOUT, SUPPORT)}
        self.site = self.create_publisher(SiteEvent, '/rescue/site_events', 10)
        self.boot = {r: str(uuid4()) for r in (SCOUT, SUPPORT)}
        self.sequence = 0
        self.create_timer(0.2, self.publish_status)

    def publish_status(self):
        self.sequence += 1
        for robot, pub in self.status.items():
            pub.publish(RobotStatus(robot_id=robot, robot_boot_id=self.boot[robot], readiness='READY',
                                    state_sequence=self.sequence, observed_at_ns=self.get_clock().now().nanoseconds,
                                    motion_state='STOPPED'))

    def publish_disaster(self):
        msg = SiteEvent(event_id=str(uuid4()), camera_id='cam_fixed', kind='DISASTER_SIGN', confidence=.9,
                        visibility_state='VALID')
        msg.time.observed_at_ns = self.get_clock().now().nanoseconds
        msg.time.stamp_semantics, msg.time.clock_domain_id, msg.time.clock_state = 'CAPTURE', 'e3-clock', 'SYNCED'
        msg.time.has_clock_error_bound, msg.time.clock_error_bound_ns = True, 5_000_000
        msg.evidence.evidence_id, msg.evidence.sha256 = str(uuid4()), 'a' * 64
        self.site.publish(msg)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def command(kind='START_SEARCH', **extra):
    return {'command_id': str(uuid4()), 'kind': kind, 'operator_id': 'e3-admin'} | extra


def main():
    repo, scratch = sys.argv[1], sys.argv[2]
    sys.path.insert(0, os.path.join(repo, 'src', 'rescue_operations'))
    from rescue_operations import site_state

    db_path = os.path.join(scratch, f'site_state-command-api-{uuid4().hex[:8]}.sqlite3')
    checks = {}
    rclpy.init()
    field = Field()
    writer = subprocess.Popen(writer_command(db_path))
    coordinator = None
    try:
        spin_for(field, 3.0)
        first = command()
        status, body = post(first)
        checks['no coordinator -> 503 without accepted'] = status == 503 and 'accepted' not in body
        status, body = post(command('DISPATCH_SUPPORT', target_id=str(uuid4())))  # target_revision missing
        checks['malformed -> 400 INVALID_INPUT'] = (status, body.get('reason')) == (400, 'INVALID_INPUT')

        profile = os.path.join(repo, 'config', 'deployment.v2.example.json')
        coordinator = subprocess.Popen(node_command(repo, ['-p', f'profile_path:={profile}'] + NODE_ARGS))
        spin_for(field, 5.0)
        status, body = post(command())
        checks['before disaster -> 200 refused with a reason'] = (status == 200 and body['accepted'] is False
                                                                  and bool(body['reason']))
        print('before disaster:', status, body)

        field.publish_disaster()
        spin_for(field, 2.0)
        start = command()
        status, body = post(start)
        print('start search:', status, body)
        checks['after disaster -> 200 accepted'] = status == 200 and body['accepted'] is True
        status, again = post(start)
        print('resend:', status, again)
        checks['resend -> DUPLICATE with the first answer'] = (status == 200 and again['request_state'] == 'DUPLICATE'
                                                               and again['accepted'] is True)

        coordinator.send_signal(signal.SIGINT)
        coordinator.wait(timeout=10)
        coordinator = None
        spin_for(field, 2.0)
        status, body = post(command())
        checks['coordinator stopped -> 503 again'] = status == 503 and 'accepted' not in body
    finally:
        if coordinator is not None:
            coordinator.send_signal(signal.SIGINT)
            coordinator.wait(timeout=10)
        writer.send_signal(signal.SIGINT)
        code = writer.wait(timeout=10)
        field.destroy_node()
        rclpy.try_shutdown()
    checks['db_writer exit code 0'] = code == 0

    rows = {r['command_id']: r for r in site_state.read_operator_commands(db_path)['commands']}
    errors = [e['code'] for e in site_state.read_error_log(db_path, limit=1000)['errors']]
    checks['no row for the unanswered command'] = first['command_id'] not in rows
    checks['accepted START_SEARCH recorded once as ACCEPTED'] = rows.get(start['command_id'], {}).get('result') == 'ACCEPTED'
    checks['refusals recorded'] = sum(not r['accepted'] for r in rows.values()) == 2
    checks['two ACCEPT_TIMEOUT in error_log'] = errors.count('ACCEPT_TIMEOUT') == 2
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    print('commands:', [(r['kind'], r['accepted'], r['result'], r['reason']) for r in rows.values()])
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
