#!/usr/bin/env python3
"""Pre-run check of one PC role before its launch (read-only, no robot command).

    python3 scripts/field/preflight_role.py --role <profile host id> --profile <profile.json> \
        [--field config/field/search_area.field.json] [--webcam-params <webcam_node.yaml>] \
        [--map <map.yaml>] [--json <out.json>]

Run it after sourcing ROS and the workspace (without -I: it checks what that environment
imports). What the role runs comes from the profile: the operations host, the coordination
host, a site camera host, and a robot's command / SLAM / perception host. For each, it checks
the modules and ament packages that launch needs, the files it reads (field run file, webcam
parameters and model hash, map YAML and image), the profile (load, map_basis), the clock
backend measurement, and ROS_DOMAIN_ID / ROS_DISCOVERY_SERVER against the profile host
network. Discovery server reachability and duplicate publishers stay in preflight.py (#42).

It writes nothing, starts no node and sends nothing to a robot. The clock is read the same
way the nodes read it (systemctl / timedatectl / busctl or chronyc, rescue_contracts).
Result: one line per check, PASSED / FAILED / NOT_RUN and a reason; exit 0 all PASSED,
2 any FAILED, 3 otherwise (preflight.py rule).
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

_spec = importlib.util.spec_from_file_location('field_preflight', Path(__file__).resolve().parent / 'preflight.py')
_pf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_pf)
PASSED, FAILED, NOT_RUN = _pf.PASSED, _pf.FAILED, _pf.NOT_RUN
check, summarize = _pf.check, _pf.summarize

# Modules each duty's launch imports (entry points in each package setup.py).
MODULES = {
    'operations': ('rclpy', 'rescue_interfaces', 'rescue_operations.ros_node', 'rescue_operations.report_node',
                   'rescue_console.app', 'flask'),
    'camera': ('rclpy', 'rescue_perception.webcam_node', 'cv2', 'ultralytics'),
    'coordination': ('rclpy', 'rescue_interfaces', 'rescue_coordination.ros_node'),
    'command': ('rclpy', 'rescue_interfaces', 'nav2_msgs', 'rescue_robot.task_input_node',
                'rescue_robot.watchdog_node'),
    'slam': ('nav_msgs',),
    'perception': ('rescue_perception.amr_target', 'cv2', 'ultralytics'),
}
PACKAGES = {'command': ('turtlebot4_navigation',), 'slam': ('turtlebot4_navigation',)}  # launch includes
PLACEHOLDER_GOAL = '[0.0, 0.0, 0.0]'


def _duties(deployment, role):
    duties = []
    if deployment.operations_host == role:
        duties.append(('operations', None))
    if deployment.coordination_host == role:
        duties.append(('coordination', None))
    if any(camera.host == role for camera in deployment.site_cameras):
        duties.append(('camera', None))
    for robot in deployment.robots:
        for duty, host in (('command', robot.command_host), ('slam', robot.slam_host),
                           ('perception', robot.perception_host)):
            if host == role:
                duties.append((duty, robot))
    return duties


def _environment(env):
    checks = [check('ros.distro', PASSED if env.get('ROS_DISTRO') else FAILED,
                    env.get('ROS_DISTRO') or 'ROS_DISTRO 없음: /opt/ros/<distro>/setup.bash 를 source 하지 않음')]
    found = _pf.find_package(_pf.ament_prefixes(env), 'rescue_bringup')
    checks.append(check('ros.workspace', PASSED if found else FAILED,
                        'rescue_bringup 설치 확인' if found else
                        'rescue_bringup 없음: 작업공간 install/setup.bash 를 source 하지 않음'))
    return checks


def _network(env, host):
    network = host.network
    if network is None:
        return [check('env.ROS_DOMAIN_ID', NOT_RUN, f'프로파일 host 에 network 없음; 지금 값={env.get("ROS_DOMAIN_ID")!r}'),
                check('env.ROS_DISCOVERY_SERVER', NOT_RUN,
                      f'프로파일 host 에 network 없음; 지금 값={env.get("ROS_DISCOVERY_SERVER")!r}')]
    domain = env.get('ROS_DOMAIN_ID')
    checks = [check('env.ROS_DOMAIN_ID', PASSED if domain == str(network.domain_id) else FAILED,
                    f'지금 {domain!r}, 프로파일 {network.domain_id}', value=domain)]
    expected = sorted((s.server_id, s.address, s.port) for s in network.discovery_servers)
    actual = sorted((s['server_id'], s['address'], s['port'])
                    for s in _pf.discovery_servers(env.get('ROS_DISCOVERY_SERVER')))
    checks.append(check('env.ROS_DISCOVERY_SERVER', PASSED if actual == expected else FAILED,
                        f'지금 {env.get("ROS_DISCOVERY_SERVER")!r} (자리=server id), 프로파일 {expected}; '
                        f'ROS_AUTOMATIC_DISCOVERY_RANGE={env.get("ROS_AUTOMATIC_DISCOVERY_RANGE")!r}. '
                        f'서버 응답은 preflight.py 로 본다', value=env.get('ROS_DISCOVERY_SERVER')))
    return checks


def _modules(names, find_spec):
    checks = []
    for name in names:
        try:
            present = find_spec(name) is not None
        except (ImportError, ValueError) as exc:  # a missing parent package
            present, why = False, f'{type(exc).__name__}: {exc}'
        else:
            why = '설치 안 됨 또는 이 환경에서 import 경로에 없음'
        checks.append(check('module.' + name, PASSED if present else FAILED, 'import 가능' if present else why))
    return checks


def _packages(names, env):
    prefixes = _pf.ament_prefixes(env)
    return [check('pkg.' + name, PASSED if _pf.find_package(prefixes, name) else FAILED,
                  'ament share 에 있음' if _pf.find_package(prefixes, name) else 'ament share 에 없음 (설치는 사람이 한다)')
            for name in names]


def _clock(clock, backend, max_age_ns, drift_ppm, source):
    check_id = 'clock.' + str(backend)
    try:
        record = clock(backend, max_age_ns=max_age_ns, drift_ppm=drift_ppm)
    except (ValueError, OSError) as exc:
        return check(check_id, FAILED, f'{source}: 측정 실패: {exc}')
    state = record.get('state')
    return check(check_id, PASSED if state == 'SYNCED' else FAILED,
                 f'{source}: state={state} error_bound_ns={record.get("error_bound_ns")}', state=state)


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def _read_settings(path):
    """JSON is a YAML subset; other YAML needs the ROS host's PyYAML dependency."""
    text = Path(path).read_text()
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        import yaml
        try:
            return yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise ValueError(f'invalid YAML: {exc}') from exc


def _webcam(path, clock):
    if path is None:
        return [check('webcam.params', NOT_RUN, '--webcam-params 를 주지 않음 (웹캠을 이 PC 에서 띄우면 필요)')], None
    try:
        from rescue_perception.webcam_node import PARAMS
        values = _read_settings(path)['webcam_node']['ros__parameters']
        if not isinstance(values, dict):
            raise TypeError('webcam_node.ros__parameters is not a mapping')
    except ImportError as exc:
        return [check('webcam.params', NOT_RUN, f'{path}: 필요한 파서/모듈 없음: {exc}')], None
    except (OSError, KeyError, TypeError, ValueError) as exc:
        return [check('webcam.params', FAILED, f'{path}: 읽기 실패: {type(exc).__name__}: {exc}')], None
    missing = sorted(set(PARAMS) - set(values))
    clock_keys = sorted(name for name in PARAMS if name.startswith('clock_'))
    checks = [check('webcam.params', FAILED if missing else PASSED,
                    (f'설치된 webcam_node 가 요구하는 키 없음: {missing}; ' if missing else
                     f'설치된 webcam_node 키 {len(PARAMS)}개 모두 있음; ') + f'시계 키 {clock_keys}',
                    missing=missing)]
    model = Path(str(values.get('model_path', '')))
    if not model.is_file():
        checks.append(check('webcam.model', FAILED, f'모델 파일 없음: {model} (상대 경로는 현재 폴더 {os.getcwd()} 기준)'))
    else:
        actual = _sha256(model)
        checks.append(check('webcam.model', PASSED if actual == values.get('model_sha256') else FAILED,
                            f'{model} sha256={actual}, 설정={values.get("model_sha256")}'))
    backend = values.get('clock_backend')
    if 'clock_backend' not in PARAMS:
        clock_check = check('clock.webcam', FAILED, '설치된 webcam_node 는 시계를 측정하지 않음(고정 clock_state): '
                                                    '측정 안 된 시계는 UNKNOWN 이라 조정기가 재난 징후를 받지 않음 (#146)')
    elif backend in ('timesyncd', 'chrony'):
        max_age, drift = values.get('clock_max_age_ns'), values.get('clock_drift_ppm')
        clock_check = _clock(clock, backend, max_age, drift, 'webcam yaml')
    else:
        clock_check = check('clock.webcam', FAILED, f'clock_backend={backend!r}: 측정하지 않으면 웹캠 시각이 UNKNOWN')
    return checks, clock_check


def _map_files(path):
    if path is None:
        return check('map.files', NOT_RUN, '--map 을 주지 않음 (위치 추정을 이 PC 에서 띄우면 필요)')
    try:
        data = _read_settings(path)
        image = Path(data['image'])
    except ImportError as exc:
        return check('map.files', NOT_RUN, f'{path}: 필요한 YAML 파서 없음: {exc}')
    except (OSError, KeyError, TypeError, ValueError) as exc:
        return check('map.files', FAILED, f'{path}: 읽기 실패: {type(exc).__name__}: {exc}')
    image = image if image.is_absolute() else Path(path).parent / image
    if not image.is_file():
        return check('map.files', FAILED, f'지도 이미지 없음: {image}')
    return check('map.files', PASSED, f'{path} + {image} sha256={_sha256(image)}')


def run_role(role, *, env, profile_path, field_path, webcam_params, map_yaml,
             find_spec=importlib.util.find_spec, clock=None):
    checks = _environment(env)
    try:
        from rescue_bringup.field_launch import (FieldLaunchError, coordinator_parameters,
                                                 frozen_localization_values, load_field, robot_launch_values)
        from rescue_bringup.profile import ProfileError, load_deployment, require_map_basis
    except ImportError as exc:
        return checks + [check('profile.load', NOT_RUN, f'rescue_bringup import 실패: {exc}')]
    if clock is None:
        from rescue_contracts.clock_measurement import collect as clock
    try:
        deployment = load_deployment(profile_path)
    except ProfileError as exc:
        return checks + [check('profile.load', FAILED, f'{profile_path}: {exc}'),
                         check('profile.role', NOT_RUN, '프로파일을 읽지 못해 역할을 모름')]
    checks.append(check('profile.load', PASSED, f'{profile_path} deployment_id={deployment.id}'))
    try:
        basis = require_map_basis(deployment)
        checks.append(check('profile.map_basis', PASSED, f'{basis}'))
    except ProfileError as exc:
        checks.append(check('profile.map_basis', FAILED, str(exc)))
    hosts = {host.id: host for host in deployment.hosts}
    if role not in hosts:
        return checks + [check('profile.role', FAILED, f'{role!r} 는 프로파일 host 가 아님: {sorted(hosts)}')]
    duties = _duties(deployment, role)
    checks.append(check('profile.role', PASSED if duties else FAILED,
                        ', '.join(d if r is None else f'{d}({r.id} {r.namespace})' for d, r in duties)
                        or '이 host 에 맡긴 일이 없음'))
    checks += _network(env, hosts[role])

    names, packages = [], []
    for duty, _robot in duties:
        names += [n for n in MODULES[duty] if n not in names]
        packages += [p for p in PACKAGES.get(duty, ()) if p not in packages]
    checks += _modules(names, find_spec) + _packages(packages, env)

    kinds = {duty for duty, _ in duties}
    clocks = {}
    if kinds & {'coordination', 'command'}:
        if field_path is None:
            checks.append(check('field.load', NOT_RUN, '--field 를 주지 않음 (조정기·로봇 launch 가 읽음)'))
            field = None
        else:
            try:
                field = load_field(field_path)
                checks.append(check('field.load', PASSED, str(field_path)))
            except FieldLaunchError as exc:
                field = None
                checks.append(check('field.load', FAILED, f'{field_path}: {exc}'))
        if field is not None and 'coordination' in kinds:
            try:
                coordinator_parameters(deployment, profile_path=str(profile_path), field=field,
                                       route_segment_goals=PLACEHOLDER_GOAL, observe_goal=PLACEHOLDER_GOAL,
                                       route_segment_passage_free='[false]')
                checks.append(check('field.coordinator', PASSED, '프로파일·현장 파일로 조정기 파라미터를 만들 수 있음 '
                                                                 '(경로·관측 목표 인자는 따로)'))
            except FieldLaunchError as exc:
                checks.append(check('field.coordinator', FAILED, str(exc)))
            c = field.coordinator
            clocks[c['live_inputs.clock_backend']] = (c['live_inputs.clock_max_age_ns'],
                                                      c['live_inputs.clock_drift_ppm'], 'field coordinator')
        for duty, robot in duties:
            if duty != 'command' or field is None:
                continue
            mode = field.robots.get(robot.id, {}).get('task.saved_map.mode')
            try:
                # SAVED_MAP_AMCL: the hash is measured at the site; the SLAM -> frozen map modes take none.
                robot_launch_values(deployment, field=field, robot_id=robot.id, now_ns=1,
                                    occupancy_sha256='0' * 64 if mode == 'SAVED_MAP_AMCL' else '')
                checks.append(check('field.robot.' + robot.id, PASSED,
                                    f'{robot.id}: 로봇 launch 파라미터를 만들 수 있음 (지도 방식 {mode})'))
            except FieldLaunchError as exc:
                checks.append(check('field.robot.' + robot.id, FAILED, str(exc)))
            if mode == 'FROZEN_MAP_AMCL':
                try:
                    start = frozen_localization_values(deployment, field=field, robot_id=robot.id)['initial_pose_xy_yaw']
                    checks.append(check('field.initial_pose.' + robot.id, PASSED, f'{robot.id}: AMCL 출발값 {start}'))
                except FieldLaunchError as exc:
                    checks.append(check('field.initial_pose.' + robot.id, FAILED, str(exc)))
            r = field.robot
            clocks.setdefault(r['task.live.clock_backend'], (r['task.live.clock_max_age_ns'],
                                                             r['task.live.clock_drift_ppm'], 'field robot'))
    if 'camera' in kinds:
        webcam_checks, clock_check = _webcam(webcam_params, clock)
        checks += webcam_checks
        if clock_check is not None and clock_check['id'][len('clock.'):] not in clocks:
            checks.append(clock_check)
    for backend, (max_age, drift, source) in clocks.items():
        checks.append(_clock(clock, backend, max_age, drift, source))
    if 'slam' in kinds:
        checks.append(_map_files(map_yaml))
    return checks


def format_report(checks):
    return '\n'.join(f'{c["status"]:<7} {c["id"]} — {c["detail"]}' for c in checks)


def main(argv=None):
    parser = argparse.ArgumentParser(description='PC 역할별 실행 전 점검 (읽기만, 아무것도 바꾸지 않음)')
    parser.add_argument('--role', required=True, help='프로파일 host id (operations, coordination, compute_a, ...)')
    parser.add_argument('--profile', required=True, type=Path, help='배포 프로파일 v2.1 JSON')
    parser.add_argument('--field', type=Path, help='현장 실행 파일 (config/field/*.field.json)')
    parser.add_argument('--webcam-params', type=Path, help='webcam_node 파라미터 YAML')
    parser.add_argument('--map', type=Path, help='위치 추정 지도 YAML')
    parser.add_argument('--json', type=Path, help='결과 JSON 을 이 경로에도 쓴다')
    args = parser.parse_args(argv)
    checks = run_role(args.role, env=dict(os.environ), profile_path=args.profile, field_path=args.field,
                      webcam_params=args.webcam_params, map_yaml=args.map)
    print(format_report(checks))
    if args.json:
        args.json.write_text(json.dumps({'role': args.role, 'checks': checks}, ensure_ascii=False, indent=2) + '\n')
    code = summarize(checks)
    print(f'exit {code}: ' + {0: '모두 PASSED', 2: 'FAILED 있음', 3: 'NOT_RUN 있음'}[code], file=sys.stderr)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
