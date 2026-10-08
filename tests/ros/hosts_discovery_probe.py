"""E3 (local only): Fast DDS Discovery Server on 127.0.0.1 — who finds whom, with and without it.

Not collected by scripts/run_tests.py. Needs ROS 2 (Jazzy, rmw_fastrtps_cpp, `fastdds` CLI)
and must start in a shell WITHOUT a robot server in the environment:

    unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
    ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tests/ros/hosts_discovery_probe.py

Every child process is started by this script in its own session and stopped by its
process group id; nothing outside is touched. Cases (docs/network-discovery.md 3절):

  A  server up, plain client lists the graph        -> probe topic NOT listed
  B  server up, super client lists the graph        -> listed
  C  server up, plain client subscribes             -> receives data
  F  server up, publisher is a client, subscriber uses Simple Discovery -> no data
  P  preflight against the local server, two /probe_robot/cmd_vel publishers -> graph check FAILED (2)
  R1 preflight UDP probe while the server is up     -> not FAILED (no_reply is inconclusive)
  H  server killed, new client subscribes           -> no data (new nodes cannot discover)
  R2 preflight UDP probe after the kill             -> FAILED (refused)
  S  no server anywhere, both sides Simple Discovery -> receives data

Exit code 0 means every case matched its expectation.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
PORT = int(os.environ.get('PROBE_DS_PORT', '11899'))   # not the TB4 default 11811
SERVER = f'127.0.0.1:{PORT}'
TOPIC, TYPE = '/hosts_probe', 'std_msgs/msg/String'
SPIN = '4'
ECHO_TIMEOUT = 12.0
FAILED_ = 'FAILED'   # preflight status string


def base_env():
    env = dict(os.environ)
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT'):
        env.pop(key, None)
    return env


def client_env(super_client=False):
    return dict(base_env(), ROS_DISCOVERY_SERVER=SERVER, ROS_SUPER_CLIENT=str(super_client))


def start(cmd, env):
    return subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)


def stop(proc):
    """Stop our own process group; `fastdds discovery` leaves a child server otherwise."""
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)     # leftover children of the group
    except ProcessLookupError:
        pass


def topic_list(env):
    out = subprocess.run(['ros2', 'topic', 'list', '--no-daemon', '--spin-time', SPIN],
                         env=env, capture_output=True, text=True, timeout=60)
    return out.stdout.split()


def echo_once(env):
    """True when one message arrives within ECHO_TIMEOUT."""
    proc = subprocess.Popen(['ros2', 'topic', 'echo', '--once', TOPIC, TYPE], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                            start_new_session=True)
    try:
        out, _ = proc.communicate(timeout=ECHO_TIMEOUT)
        return 'data: hi' in out
    except subprocess.TimeoutExpired:
        stop(proc)
        return False


def preflight(env, *args):
    out = subprocess.run([sys.executable, '-I', str(REPO / 'scripts/field/preflight.py'),
                          '--spin-time', SPIN, '--port-timeout', '1', *args],
                         env=env, capture_output=True, text=True, timeout=90)
    return {c['id']: c for c in json.loads(out.stdout)['checks']}, out.returncode


def main():
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if domain < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        sys.exit('refuse: need ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        sys.exit('refuse: ROS_DISCOVERY_SERVER is set in this shell; unset it (it may point at a robot)')
    results, procs = [], []

    def record(case, expect, got, **extra):
        results.append(dict(case=case, expect=expect, got=got, ok=expect == got, **extra))
        print(json.dumps(results[-1], ensure_ascii=False), flush=True)

    try:
        server = start(['fastdds', 'discovery', '-i', '0', '-l', '127.0.0.1', '-p', str(PORT)], base_env())
        procs.append(server)
        time.sleep(2)
        pub_cmd = ['ros2', 'topic', 'pub', '-r', '5', TOPIC, TYPE, '{data: hi}']
        procs.append(start(pub_cmd, client_env()))
        time.sleep(3)

        record('A', False, TOPIC in topic_list(client_env(False)))
        record('B', True, TOPIC in topic_list(client_env(True)))
        record('C', True, echo_once(client_env(False)))
        record('F', False, echo_once(base_env()))

        twist = ['geometry_msgs/msg/Twist', '{}']
        for _ in range(2):
            procs.append(start(['ros2', 'topic', 'pub', '-r', '2', '/probe_robot/cmd_vel', *twist], client_env()))
        time.sleep(3)
        checks, code = preflight(client_env(), '--namespace', '/probe_robot')
        cmd_vel = checks.get('graph./probe_robot/cmd_vel', {})
        record('P', (FAILED_, 2), (cmd_vel.get('status'), cmd_vel.get('measured', {}).get('publishers')),
               preflight_exit=code)
        reach = checks.get('server.0.reach', {})
        record('R1', True, reach.get('status') != FAILED_, result=reach.get('measured', {}).get('result'))

        stop(server)
        time.sleep(1)
        record('H', False, echo_once(client_env(False)))
        checks, code = preflight(client_env())
        reach = checks.get('server.0.reach', {})
        record('R2', FAILED_, reach.get('status'), result=reach.get('measured', {}).get('result'))
    finally:
        for p in procs:
            stop(p)

    simple_pub = start(['ros2', 'topic', 'pub', '-r', '5', TOPIC, TYPE, '{data: hi}'], base_env())
    try:
        time.sleep(3)
        record('S', True, echo_once(base_env()))
    finally:
        stop(simple_pub)

    ok = all(r['ok'] for r in results)
    print(f'E3 hosts discovery probe: {"PASSED" if ok else "FAILED"} '
          f'({sum(r["ok"] for r in results)}/{len(results)} cases, domain {domain}, server {SERVER})')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
