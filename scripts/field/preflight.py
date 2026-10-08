#!/usr/bin/env python3
"""Pre-run report of this PC's ROS / Discovery Server environment (read-only).

It reads its own environment, looks for installed packages under the sourced
ament prefixes, sends one empty UDP datagram to each discovery server port and,
when ``ros2`` is on PATH, lists the graph once as a super client to count the
publishers of ``<namespace>/cmd_vel`` and ``<namespace>/map``. It never changes
a file, a setting or the robot. The graph listing joins the servers as one
short-lived participant; that is the only footprint.

Result: JSON on stdout, a human table on stderr. Exit code 0 = every check
PASSED, 2 = at least one FAILED, 3 = nothing failed but at least one NOT_RUN.
Statuses follow docs/acceptance-plan.md A-02; an expectation that is unknown is
NOT_RUN, never PASSED.

    python3 -I scripts/field/preflight.py --host-config config/hosts/central.example.json
    python3 -I scripts/field/preflight.py --namespace /robot_a --json <TMP>/preflight.json

Who decided: discovery rules from docs/design/network-discovery.md (field seat,
2026-10-08); duplicate publisher rule from P4 practice L33 (2026-10-06).
"""
import argparse
from datetime import datetime
import errno
import fnmatch
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys

PASSED, FAILED, NOT_RUN = 'PASSED', 'FAILED', 'NOT_RUN'
ENV_KEYS = ('ROS_DISTRO', 'RMW_IMPLEMENTATION', 'ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
            'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')
# Fast DDS is the only RMW with Discovery Server; it is also Jazzy's default
# when RMW_IMPLEMENTATION is unset [문서: ROS 2 Jazzy DS tutorial].
DISCOVERY_RMW = 'rmw_fastrtps_cpp'
DEFAULT_SERVER_PORT = 11811          # TB4 onboard server default [문서: TB4 setup tool]
DEFAULT_PACKAGES = ('nav2_msgs', 'irobot_create_msgs', 'turtlebot4_*', DISCOVERY_RMW)
WATCHED_TOPICS = ('cmd_vel', 'map')
MAX_PUBLISHERS = 1   # P4 2026-10-06: two /robot3/map publishers stopped the run
PLACEHOLDER = re.compile(r'^<.*>$')


def check(check_id, status, detail, **measured):
    return {'id': check_id, 'status': status, 'detail': detail, 'measured': measured}


def discovery_host(value):
    """First host in ROS_DISCOVERY_SERVER (';'-separated, slot = server id).

    Copied from ROKEY_P4 scripts/preflight.py L151-L158 at 1a2d0a9.
    """
    for part in (value or '').split(';'):
        part = part.strip()
        if part:
            host = part.rsplit(':', 1)[0] if part.count(':') == 1 else part
            return host.strip('[]')
    return None


def discovery_servers(value):
    """[{server_id, address, port}] from ROS_DISCOVERY_SERVER (#39 field names); the slot index is the id.

    Same host rule as discovery_host(); a missing port means DEFAULT_SERVER_PORT.
    """
    servers = []
    for server_id, part in enumerate((value or '').split(';')):
        part = part.strip()
        if not part:
            continue
        host, port = part, DEFAULT_SERVER_PORT
        if part.startswith('['):                      # [v6]:port
            host, _, rest = part[1:].partition(']')
            port = rest[1:] if rest.startswith(':') else port
        elif part.count(':') == 1:
            host, port = part.rsplit(':', 1)
        try:
            port = int(port)
        except ValueError:
            port = None                               # reported as is, never guessed
        servers.append({'server_id': server_id, 'address': host, 'port': port})
    return servers


def truthy(value):
    return (value or '').strip().lower() in ('1', 'true', 'yes', 'on')


def summarize(checks):
    """Exit code: 2 if any FAILED, else 3 if any NOT_RUN or no check, else 0."""
    statuses = [c['status'] for c in checks]
    if FAILED in statuses:
        return 2
    if not statuses or any(s != PASSED for s in statuses):
        return 3
    return 0


def is_placeholder(value):
    return value is None or (isinstance(value, str) and bool(PLACEHOLDER.match(value)))


# ---- environment -------------------------------------------------------------

def check_environment(env, config):
    ros = (config or {}).get('ros', {})
    distro, rmw = env.get('ROS_DISTRO'), env.get('RMW_IMPLEMENTATION')
    effective_rmw = rmw or (DISCOVERY_RMW if distro == 'jazzy' else None)
    servers_env = env.get('ROS_DISCOVERY_SERVER')
    checks = []

    if not distro:
        checks.append(check('env.ros_distro', FAILED, 'ROS_DISTRO 없음: ROS 환경을 source 하지 않았거나 미설치'))
    elif ros.get('distro') and ros['distro'] != distro:
        checks.append(check('env.ros_distro', FAILED, f'기대 {ros["distro"]}, 실제 {distro}'))
    else:
        checks.append(check('env.ros_distro', PASSED, distro if ros.get('distro') else f'{distro} (기대값 없음, 설정만 확인)'))

    want_rmw = ros.get('rmw_implementation') or (DISCOVERY_RMW if servers_env else None)
    if want_rmw is None:
        checks.append(check('env.rmw', NOT_RUN, f'기대값 없음 (호스트 설정·ROS_DISCOVERY_SERVER 없음). 실제 {rmw or "미설정"}'))
    elif effective_rmw == want_rmw:
        checks.append(check('env.rmw', PASSED, f'{effective_rmw}' + ('' if rmw else ' (미설정, Jazzy 기본값)')))
    else:
        why = ' — Discovery Server 는 Fast DDS 전용' if want_rmw == DISCOVERY_RMW else ''
        checks.append(check('env.rmw', FAILED, f'기대 {want_rmw}, 실제 {rmw or "미설정"}{why}'))

    expected = (config or {}).get('discovery_servers')
    actual = discovery_servers(servers_env)
    if expected is None:
        checks.append(check('env.discovery_server', NOT_RUN, '기대값 없음 (호스트 설정 없음)', actual=actual))
    elif any(is_placeholder(s.get('address')) or s.get('server_id') is None for s in expected):
        checks.append(check('env.discovery_server', NOT_RUN,
                            '호스트 설정이 예시 값(<...> 또는 server_id null)이라 비교하지 않음', actual=actual))
    else:
        want = sorted((s['server_id'], s['address'], s.get('port', DEFAULT_SERVER_PORT)) for s in expected)
        got = sorted((s['server_id'], s['address'], s['port']) for s in actual)
        status = PASSED if want == got else FAILED
        checks.append(check('env.discovery_server', status, f'기대 {want}, 실제 {got}', actual=actual))

    domain = ros.get('domain_id')
    if domain is None:
        checks.append(check('env.domain_id', NOT_RUN,
                            f'기대값 없음. 실제 {env.get("ROS_DOMAIN_ID", "미설정(0)")} — Discovery Server 는 도메인을 보지 않음'))
    else:
        actual_domain = env.get('ROS_DOMAIN_ID', '0')
        status = PASSED if actual_domain == str(domain) else FAILED
        checks.append(check('env.domain_id', status, f'기대 {domain}, 실제 {actual_domain}'))
    return checks


# ---- installation ------------------------------------------------------------

def ament_prefixes(env):
    return [Path(p) for p in (env.get('AMENT_PREFIX_PATH') or '').split(os.pathsep) if p]


def find_package(prefixes, pattern):
    """Package names under <prefix>/share/<name>/package.xml matching pattern (glob)."""
    found = set()
    for prefix in prefixes:
        share = prefix / 'share'
        if not share.is_dir():
            continue
        for d in share.iterdir():
            if fnmatch.fnmatchcase(d.name, pattern) and (d / 'package.xml').is_file():
                found.add(d.name)
    return sorted(found)


def check_install(env, config, which=shutil.which):
    patterns = (config or {}).get('required_packages', DEFAULT_PACKAGES)
    prefixes = ament_prefixes(env)
    checks = []
    for pattern in patterns:
        if not prefixes:
            checks.append(check(f'pkg.{pattern}', NOT_RUN, 'AMENT_PREFIX_PATH 없음: ROS 환경을 source 하지 않음'))
            continue
        found = find_package(prefixes, pattern)
        checks.append(check(f'pkg.{pattern}', PASSED if found else FAILED,
                            ', '.join(found) if found else '설치 안 됨 (설치는 sudo, 사람이 한다)', found=found))
    tool = which('fastdds', path=env.get('PATH'))
    if tool:
        checks.append(check('tool.fastdds', PASSED, 'fastdds CLI 있음'))
    elif prefixes:
        checks.append(check('tool.fastdds', FAILED, 'fastdds CLI 없음 (ros-<distro>-fastrtps 의 일부)'))
    else:
        checks.append(check('tool.fastdds', NOT_RUN, 'PATH 에 없음, ROS 환경도 source 하지 않음'))
    return checks


# ---- server port -------------------------------------------------------------

def udp_probe(host, port, timeout):
    """Send one empty datagram; 'refused' | 'unreachable' | 'no_reply' | 'reply' | 'error: ...'.

    UDP has no handshake: 'no_reply' is what an open port AND a filtered one look
    like, so it never proves the server is up. Only an ICMP error is conclusive.
    """
    try:
        infos = socket.getaddrinfo(host, port, type=socket.SOCK_DGRAM)
    except OSError as e:
        return f'error: {e}'
    family, kind, proto, _, address = infos[0]
    with socket.socket(family, kind, proto) as s:
        s.settimeout(timeout)
        try:
            s.connect(address)
            s.send(b'')
            s.recv(1)
            return 'reply'
        except ConnectionRefusedError:
            return 'refused'
        except socket.timeout:
            return 'no_reply'
        except OSError as e:
            if e.errno in (errno.EHOSTUNREACH, errno.ENETUNREACH):
                return 'unreachable'
            return f'error: {e}'


def check_servers(env, timeout, probe=udp_probe):
    servers = discovery_servers(env.get('ROS_DISCOVERY_SERVER'))
    if not servers:
        return [check('server.reach', NOT_RUN, 'ROS_DISCOVERY_SERVER 없음')]
    checks = []
    for s in servers:
        result = probe(s['address'], s['port'], timeout) if s['port'] is not None else 'error: 포트를 읽지 못함'
        cid = f'server.{s["server_id"]}.reach'
        where = f'{s["address"]}:{s["port"]}'
        if result in ('refused', 'unreachable'):
            checks.append(check(cid, FAILED, f'{where} {result} (서버가 안 떴거나 경로 없음)', result=result))
        elif result == 'reply':
            checks.append(check(cid, PASSED, f'{where} 응답', result=result))
        else:
            checks.append(check(cid, NOT_RUN, f'{where} {result}: UDP 는 응답이 없어도 열림 증명이 아님 — graph 점검으로 판정',
                                result=result))
    return checks


# ---- graph -------------------------------------------------------------------

PUB_LINE = re.compile(r'^\s*\*\s+(\S+)\s+\[[^\]]*\]\s+(\d+)\s+publishers?\s*$')


def parse_topic_list_verbose(text):
    """{topic: publisher count} from the 'Published topics:' block of `ros2 topic list -v`."""
    counts, in_published = {}, False
    for line in text.splitlines():
        if line.startswith('Published topics:'):
            in_published = True
        elif line.startswith('Subscribed topics:'):
            in_published = False
        elif in_published:
            m = PUB_LINE.match(line)
            if m:
                counts[m.group(1)] = int(m.group(2))
    return counts


def run_ros2(args, env, timeout):
    return subprocess.run(['ros2', *args], env=env, capture_output=True, text=True, timeout=timeout)


def check_graph(env, namespaces, spin_time, which=shutil.which, runner=run_ros2):
    if not namespaces:
        return [check('graph', NOT_RUN, '볼 네임스페이스 없음 (--namespace 또는 호스트 설정 watch_namespaces)')]
    if not which('ros2', path=env.get('PATH')):
        return [check(f'graph.{ns}', NOT_RUN, 'ros2 CLI 없음') for ns in namespaces]
    # Only the child's environment changes: a super client sees every topic on the
    # servers; a plain client would list only its own matches [측정 A·B].
    child_env = dict(env, ROS_SUPER_CLIENT='True')
    try:
        done = runner(['topic', 'list', '-v', '--no-daemon', '--spin-time', str(spin_time)],
                      child_env, spin_time + 30)
    except subprocess.TimeoutExpired:
        return [check(f'graph.{ns}', NOT_RUN, 'ros2 topic list 시간 초과') for ns in namespaces]
    if done.returncode != 0:
        err = (done.stderr or '').strip().splitlines()[-1:] or ['']
        return [check(f'graph.{ns}', NOT_RUN, f'ros2 topic list 종료 코드 {done.returncode}: {err[0]}')
                for ns in namespaces]
    counts = parse_topic_list_verbose(done.stdout)
    checks = []
    for ns in namespaces:
        ns = '/' + ns.strip('/')
        if not any(t.startswith(ns + '/') for t in counts):
            checks.append(check(f'graph.{ns}', NOT_RUN, f'{ns} 아래 발행 토픽이 안 보임 (로봇·서버 미연결 또는 꺼짐)'))
            continue
        for topic in WATCHED_TOPICS:
            name = f'{ns}/{topic}'
            n = counts.get(name, 0)
            status = PASSED if n <= MAX_PUBLISHERS else FAILED
            detail = f'{name} 발행자 {n}' + (' — 중복 발행 (다른 PC 가 같은 로봇에 붙음?)' if status == FAILED else '')
            checks.append(check(f'graph.{name}', status, detail, publishers=n))
    return checks


# ---- host facts (recorded, not judged) ---------------------------------------

def host_facts(os_release=Path('/etc/os-release'), ros_root=Path('/opt/ros')):
    """OS name, Python version and /opt/ros/<distro> dirs: input for picking field PCs."""
    name = None
    try:
        for line in os_release.read_text().splitlines():
            if line.startswith('PRETTY_NAME='):
                name = line.split('=', 1)[1].strip('"')
    except OSError:
        pass
    installs = sorted(d.name for d in ros_root.iterdir() if d.is_dir()) if ros_root.is_dir() else []
    return {'os': name, 'python': sys.version.split()[0], 'ros_installs': installs}


# ---- main --------------------------------------------------------------------

def run(env, config, namespaces, port_timeout=2.0, spin_time=5.0,
        which=shutil.which, runner=run_ros2, probe=udp_probe):
    checks = check_environment(env, config)
    checks += check_install(env, config, which)
    checks += check_servers(env, port_timeout, probe)
    checks += check_graph(env, namespaces, spin_time, which, runner)
    return checks


def main(argv=None, env=None):
    ap = argparse.ArgumentParser(description='실행 전 점검 (보고만, 아무것도 바꾸지 않음). docs/design/network-discovery.md')
    ap.add_argument('--host-config', type=Path, help='config/hosts/<role>.example.json 형식의 기대값')
    ap.add_argument('--namespace', action='append', default=[], help='발행자 수를 볼 로봇 네임스페이스 (여러 번)')
    ap.add_argument('--port-timeout', type=float, default=2.0, help='서버 포트 UDP 응답 대기 (s)')
    ap.add_argument('--spin-time', type=float, default=5.0, help='그래프 발견 대기 (s)')
    ap.add_argument('--json', type=Path, help='JSON 결과를 이 경로에도 쓴다')
    args = ap.parse_args(argv)
    env = dict(os.environ if env is None else env)
    config = json.loads(args.host_config.read_text()) if args.host_config else None
    namespaces = args.namespace or (config or {}).get('watch_namespaces', [])
    started = datetime.now().astimezone()
    checks = run(env, config, namespaces, args.port_timeout, args.spin_time)
    code = summarize(checks)
    result = {
        'tool': 'scripts/field/preflight.py',
        'started': started.isoformat(timespec='seconds'),
        'host_config': str(args.host_config) if args.host_config else None,
        'role': (config or {}).get('role'),
        'host_facts': host_facts(),
        'environment': {k: env.get(k) for k in ENV_KEYS},
        'super_client_runtime_note': 'launch 는 ROS_SUPER_CLIENT=False 로 시작한다 (docs/design/network-discovery.md)',
        'checks': checks,
        'exit_code': code,
    }
    text = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.json:
        args.json.write_text(text)
    for c in checks:
        print(f'{c["status"]:<8} {c["id"]:<34} {c["detail"]}', file=sys.stderr)
    print(f'preflight exit={code} ({ {0: "PASSED", 2: "FAILED", 3: "NOT_RUN 있음"}[code] })', file=sys.stderr)
    sys.stdout.write(text)
    return code


if __name__ == '__main__':
    sys.exit(main())
