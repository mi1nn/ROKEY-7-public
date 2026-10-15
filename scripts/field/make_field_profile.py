#!/usr/bin/env python3
"""Field deployment profile (v2.1) from the repository example, the same on every PC (작전, 2026-10-15).

    python3 scripts/field/make_field_profile.py --out <new folder> --discovery <Discovery PC address>:11811 \
        --domain 6 [--home robot_a=x,y,yaw] [--home robot_b=x,y,yaw]

Until 2026-10-15 the field profile was the example edited by hand on each PC (network blocks, then the
namespaces with sed); the two PCs had to be compared by sha256 afterwards. This writes it from one command:
- config/deployment.v2.example.json and the robot files it names (config/robots/*.v2.json), unchanged but for
- the namespaces: every string starting with /robot_a or /robot_b gets the field namespace (default
  robot_a=/robot6, robot_b=/robot5; TB-6 = robot_a, TB-5 = robot_b);
- every host's network: this domain, the one Discovery server (ID 0, 재범 decision 2026-10-15), not a super client;
- map_basis.frame_id map (the robots' readiness map_frame; robot.launch refuses another frame, #132 field basis);
- deployment_id rokey7_field (the example's other ids and the map basis session stay: the field file and the map
  lifecycle use them);
- home_positions only when given (measured on the frozen map, field-runbook step 5); none is made up.
The files are written with sorted keys and a newline, so the same inputs give the same sha256 on any PC; the
hashes are printed. The out folder must not exist yet (nothing is overwritten). The result is checked with
rescue_bringup.profile.load_deployment before the command ends. Standard library and this checkout only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
try:
    import rescue_bringup.profile  # noqa: F401  (installed or on PYTHONPATH)
except ImportError:  # run straight from the checkout
    sys.path.insert(0, str(REPO / 'src' / 'rescue_bringup'))
from rescue_bringup.profile import ProfileError, load_deployment  # noqa: E402

EXAMPLE = REPO / 'config' / 'deployment.v2.example.json'
DEPLOYMENT_ID = 'rokey7_field'
MAP_FRAME = 'map'   # = config/robots/*.v2.json readiness.map_frame (field_launch refuses a different basis frame)
NAMESPACES = {'robot_a': '/robot6', 'robot_b': '/robot5'}   # TB-6, TB-5


class FieldProfileError(Exception):
    """Nothing was written, or what was written failed the profile check; the message says why."""


def _renamed(value, namespaces):
    if isinstance(value, dict):
        return {key: _renamed(item, namespaces) for key, item in value.items()}
    if isinstance(value, list):
        return [_renamed(item, namespaces) for item in value]
    if isinstance(value, str):
        for robot_id, namespace in namespaces.items():
            example = '/' + robot_id
            if value == example or value.startswith(example + '/'):
                return namespace + value[len(example):]
    return value


def _dump(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n').encode('utf-8')


def _server(text):
    address, sep, port = text.rpartition(':')
    if not sep or not address or not port.isdigit() or not 0 < int(port) < 65536:
        raise FieldProfileError(f'--discovery must be <address>:<port>, got {text!r}')
    return address, int(port)


def _home(text):
    robot_id, sep, pose = text.partition('=')
    try:
        values = [float(v) for v in pose.split(',')]
    except ValueError:
        values = []
    if not sep or len(values) != 3:
        raise FieldProfileError(f'--home must be <robot_id>=x,y,yaw, got {text!r}')
    return robot_id, values


def build(*, discovery, domain, homes=(), namespaces=None, example=EXAMPLE):
    """{relative path: bytes} of the profile and its robot files."""
    namespaces = dict(NAMESPACES if namespaces is None else namespaces)
    address, port = _server(discovery)
    if not 0 <= domain <= 232:
        raise FieldProfileError(f'--domain must be 0..232, got {domain}')
    profile = json.loads(example.read_text(encoding='utf-8'))
    robot_ids = [robot['id'] for robot in profile['robots']]
    if set(namespaces) != set(robot_ids):
        raise FieldProfileError(f'namespaces must name exactly the example robots {robot_ids}')
    files = {}
    for robot in profile['robots']:
        source = example.parent / robot['parameters_file']
        files[robot['parameters_file']] = _dump(_renamed(json.loads(source.read_text(encoding='utf-8')), namespaces))
    profile = _renamed(profile, namespaces)
    profile['deployment_id'] = DEPLOYMENT_ID
    profile['map_basis']['frame_id'] = MAP_FRAME
    for host in profile['hosts']:
        host['network'] = {'domain_id': domain, 'discovery_servers': [{'server_id': 0, 'address': address,
                                                                       'port': port}], 'super_client': False}
    homes = dict(homes)
    if set(homes) - set(robot_ids):
        raise FieldProfileError(f'--home robot must be one of {robot_ids}')
    if homes:
        profile['home_positions'] = homes
    files['profile.json'] = _dump(profile)
    return files


def write(out, files):
    """Write ``files`` under a new folder ``out`` and check the profile; {path: sha256}."""
    out = Path(out)
    if out.exists():
        raise FieldProfileError(f'{out} already exists; give a new folder (nothing is overwritten)')
    for name, data in files.items():
        path = out / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    try:
        load_deployment(out / 'profile.json')
    except ProfileError as exc:
        raise FieldProfileError(f'written profile fails the profile check: {exc}') from exc
    return {name: hashlib.sha256(data).hexdigest() for name, data in sorted(files.items())}


def main(argv=None):
    parser = argparse.ArgumentParser(description='저장소 예시에서 현장 배포 프로파일을 만든다 (모든 PC 같은 sha256)')
    parser.add_argument('--out', required=True, help='새 폴더 (이미 있으면 거절)')
    parser.add_argument('--discovery', required=True, help='Discovery 전용 PC <주소>:<포트>')
    parser.add_argument('--domain', type=int, required=True, help='ROS_DOMAIN_ID')
    parser.add_argument('--home', action='append', default=[], help='<robot_id>=x,y,yaw (고정 지도에서 잰 값)')
    args = parser.parse_args(argv)
    try:
        hashes = write(args.out, build(discovery=args.discovery, domain=args.domain,
                                       homes=[_home(text) for text in args.home]))
    except FieldProfileError as exc:
        print(f'만들지 않음: {exc}', file=sys.stderr)
        return 2
    for name, digest in hashes.items():
        print(f'{digest}  {name}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
