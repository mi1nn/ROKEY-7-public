"""E3: scripts/field/wiring_check.py against a fake field graph on a local ROS graph (2026-10-14).

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy). Run on an isolated graph, never a robot's
domain, with no Discovery Server:

    source /opt/ros/jazzy/setup.bash
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \\
      PYTHONPATH=<REPO>/src/rescue_bringup:<REPO>/src/rescue_contracts:<REPO>/src/rescue_coordination:<REPO>/tests:$PYTHONPATH \\
      python3 tests/ros/e3_wiring_check_probe.py <REPO>

The probe builds TB-5's (robot_b) expected connections plus the shared ones from the repository field
file (the field profile of tests/test_bringup_field_launch.py) and makes every one of them with fake
nodes of the expected names: std_msgs/String topics, std_srvs/Trigger services, example_interfaces
Fibonacci actions, one external writer per robot-Pi topic. Checks:
1. the full graph: wiring_check --robot robot_b exits 0, every row PASSED, no duplicate node;
2. while it ran, the wiring_check node had no publisher, subscription, service or client;
3. a second /robot5/amcl node, a BEST_EFFORT publisher on /rescue/robots/robot_b/status and no
   /robot5/rescue_command_watchdog subscription on the executor heartbeat: exit 2, duplicate_nodes
   FAILED naming /robot5/amcl, those rows FAILED with their reasons.
Exit code 0 = all held.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

SPIN = 3.0


def split(full):
    namespace, _, name = full.rpartition('/')
    return name, namespace or '/'


def build(links, *, broken=False):
    """Fake nodes for every expectation; ``broken`` adds the three faults of check 3."""
    nodes = {}

    def get(full):
        if full not in nodes:
            name, namespace = split(full)
            nodes[full] = rclpy.create_node(name, namespace=namespace)
        return nodes[full]
    handles = []
    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    for lk in links:
        writers = ['/robot5/pi_driver' if lk.name.startswith('/robot5/') else '/robot6/pi_driver'] \
            if lk.external_writer else list(lk.writers)
        readers = list(lk.readers) or (['/robot5/create3_driver'] if lk.external_reader else [])
        for w in writers:
            n = get(w)
            if lk.kind == 'topic':
                qos = best_effort if broken and lk.name == '/rescue/robots/robot_b/status' else 10
                handles.append(n.create_publisher(String, lk.name, qos))
            elif lk.kind == 'service':
                handles.append(n.create_service(Trigger, lk.name, lambda req, res: res))
            else:
                handles.append(ActionServer(n, Fibonacci, lk.name, lambda goal: Fibonacci.Result()))
        for r in readers:
            if broken and lk.name == '/robot5/rescue/executor_heartbeat' and r == '/robot5/rescue_command_watchdog':
                continue
            n = get(r)
            if lk.kind == 'topic':
                handles.append(n.create_subscription(String, lk.name, lambda m: None, 10))
            elif lk.kind == 'service':
                handles.append(n.create_client(Trigger, lk.name))
            else:
                handles.append(ActionClient(n, Fibonacci, lk.name))
    extra = []
    if broken:
        extra.append(rclpy.create_node('amcl', namespace='/robot5'))   # same full name as the real one
    return list(nodes.values()) + extra, handles


def run_check(repo, profile, field, out):
    env = dict(os.environ)
    return subprocess.Popen([sys.executable, str(Path(repo) / 'scripts' / 'field' / 'wiring_check.py'),
                             '--profile', str(profile), '--field', str(field), '--robot', 'robot_b',
                             '--spin-time', str(SPIN), '--json', str(out)],
                            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)


def footprint(watcher, deadline):
    """Endpoints of the wiring_check node seen while it runs: (name, pubs, subs, services, clients) or None."""
    while time.monotonic() < deadline:
        for name, namespace in watcher.get_node_names_and_namespaces():
            if name.startswith('wiring_check_'):
                time.sleep(1.0)
                return (name,
                        watcher.get_publisher_names_and_types_by_node(name, namespace),
                        watcher.get_subscriber_names_and_types_by_node(name, namespace),
                        watcher.get_service_names_and_types_by_node(name, namespace),
                        watcher.get_client_names_and_types_by_node(name, namespace))
        time.sleep(0.1)
    return None


def main():
    repo = Path(sys.argv[1]).resolve()
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    sys.path.insert(0, str(repo / 'scripts' / 'field'))
    import test_bringup_field_launch as fixtures
    import wiring_check as wc
    from rescue_bringup.field_launch import load_field
    from rescue_bringup.profile import load_deployment

    failures = []
    with tempfile.TemporaryDirectory() as folder:
        folder = Path(folder)
        profile = fixtures.field_profile(folder)
        links = wc.expected_links(load_deployment(profile), load_field(fixtures.FIELD), robots=['robot_b'])
        rclpy.init()
        for broken in (False, True):
            nodes, handles = build(links, broken=broken)
            watcher = rclpy.create_node('e3_wiring_watcher')
            executor = MultiThreadedExecutor()
            for n in nodes + [watcher]:
                executor.add_node(n)
            spinner = threading.Thread(target=executor.spin, daemon=True)
            spinner.start()
            time.sleep(2.0)
            out = folder / f'result_{broken}.json'
            proc = run_check(repo, profile, fixtures.FIELD, out)
            seen = footprint(watcher, time.monotonic() + SPIN + 10) if not broken else None
            _, err = proc.communicate(timeout=SPIN + 60)
            result = json.loads(out.read_text())
            rows = {r['id']: r for r in result['connections']}
            if not broken:
                bad = {k: r['problems'] for k, r in rows.items() if r['status'] != 'PASSED'}
                if proc.returncode != 0 or bad or result['duplicate_nodes']['status'] != 'PASSED':
                    failures.append(f'1. full graph exit {proc.returncode}, not passed {bad}, '
                                    f'duplicates {result["duplicate_nodes"]}\n{err}')
                if seen is None or any(seen[1:]):
                    failures.append(f'2. wiring_check node endpoints {seen}')
            else:
                want = {'robot_b.status': 'reliability BEST_EFFORT -> RELIABLE',
                        'robot_b.executor_heartbeat': '구독 없음: /robot5/rescue_command_watchdog'}
                got = {k: rows[k]['problems'] for k in want}
                if (proc.returncode != 2 or result['duplicate_nodes']['names'] != ['/robot5/amcl']
                        or any(rows[k]['status'] != 'FAILED' or not any(w in p for p in got[k])
                               for k, w in want.items())):
                    failures.append(f'3. faults exit {proc.returncode}, duplicates {result["duplicate_nodes"]}, '
                                    f'rows {got}\n{err}')
                else:
                    print(err.strip().splitlines()[-1])
            executor.shutdown()
            for h in handles:   # rclpy destroy_node leaves action servers/clients (and their node) behind
                if isinstance(h, (ActionServer, ActionClient)):
                    h.destroy()
            for n in nodes + [watcher]:
                n.destroy_node()
            time.sleep(3.0)
        rclpy.shutdown()
    for f in failures:
        print('FAILED', f)
    print('e3_wiring_check_probe', 'OK' if not failures else 'FAILED')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
