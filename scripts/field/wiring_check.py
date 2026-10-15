#!/usr/bin/env python3
"""Field wiring check: every expected connection has its writer and its readers in the live graph (read-only).

Why: "로컬 환경 테스트가 어렵다, Discovery Server 등 배선 문제" (owner, 2026-10-14). preflight_role.py checks one
PC, preflight.py duplicate cmd_vel/map publishers, link_check.py message gaps. This tool checks the whole
graph at once: for every connection the field launches make, is the publisher (service/action server)
there exactly once, is every expected reader there, do the endpoints agree on type and QoS, and is any
node name used twice (e.g. two /robot6/slam_toolbox answering one lifecycle request, mu-06 2026-10-14
[추정]).

Expected connections come from the code, not from docs/design/sdd-node-connections-v0.1.csv (old names):
the profile + the field run file through rescue_bringup.field_launch (robot_launch_values,
frozen_localization_values, overhead_passage_launch_values, passage_points_launch_values),
rescue_bringup.operations_launch, the rescue_coordination topic constants, and the node names of the
rescue_bringup launch files (NODES below; tests/test_field_wiring_check.py keeps them equal to the launch
files).

It only reads the graph (rclpy graph API: node names, endpoint info, services and actions by node). It
makes no subscription or publisher of its own (the /parameter_events publisher rclpy creates is
destroyed), calls no service or action and never moves a robot. ROS_SUPER_CLIENT=True is set for this
process before rclpy starts so a Discovery Server shows every endpoint (docs/design/network-discovery.md 3절).

Verdict per connection: FAILED when the writer is missing or there are two or more (also a duplicate
node), an expected reader is missing, the endpoint types differ or a publisher/subscriber pair has
incompatible QoS (reliability, durability); PASSED otherwise; NOT_RUN when the expectation or the graph
is unknown (never PASSED). Exit code as scripts/field/preflight.py: 0 = all PASSED, 2 = a FAILED,
3 = nothing failed but a NOT_RUN. Table on stderr, JSON on stdout.

    source /opt/ros/jazzy/setup.bash && source <install>/setup.bash
    export ROS_DISCOVERY_SERVER=...   # as the run (preflight_role.py PASSED first)
    python3 scripts/field/wiring_check.py --profile <profile.json> --field config/field/search_area.field.json \\
        [--robot robot_b] [--slam-phase FROZEN] [--spin-time 5] [--json <out.json>]
(not -I: ROS needs PYTHONPATH)
"""
import argparse
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import sys
import time

PASSED, FAILED, NOT_RUN = 'PASSED', 'FAILED', 'NOT_RUN'
KINDS = ('topic', 'service', 'action')
SLAM_PHASES = ('SEARCHING', 'FROZEN')   # TB-6 map: slam_toolbox while searching, relay + AMCL after the freeze
ENV_KEYS = ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
            'RMW_IMPLEMENTATION')
# Node names (name=..., without namespace) the launches give. Ours: rescue_bringup/launch/*.launch.py.
# Nav2/SLAM/AMCL: the parameter sections of turtlebot4_navigation config (nav2.yaml, slam.yaml, localization.yaml).
NODES = {
    'adapter': 'rescue_command_adapter',          # robot.launch.py, <ns>
    'watchdog': 'rescue_command_watchdog',        # robot.launch.py, <ns>
    'passage_points': 'passage_points',           # passage_points.launch.py, composable node, <ns>
    'overhead': 'overhead_passage',               # overhead_passage.launch.py, <ns>
    'relay': 'rescue_frozen_map_relay',           # frozen_localization.launch.py, <ns>
    'amcl': 'amcl',                               # frozen_localization.launch.py, <ns>
    'map_freeze': 'rescue_map_freeze',            # map_freeze.launch.py
    'db_writer': 'rescue_db_writer',              # operations.launch.py
    'report_node': 'rescue_report_node',          # operations.launch.py
    'webcam': 'webcam_node',                      # operations.launch.py
    'target_locator': 'target_locator',           # amr_perception.launch.py
    'exit_locator': 'exit_locator',               # amr_perception.launch.py
    'collision_monitor': 'collision_monitor',     # turtlebot4 nav2.yaml (robot.launch.py remaps its cmd_vel)
    'docking_server': 'docking_server',           # turtlebot4 nav2.yaml (robot.launch.py remaps its cmd_vel)
    'bt_navigator': 'bt_navigator',               # turtlebot4 nav2.yaml (navigate_to_pose server)
    'slam_toolbox': 'slam_toolbox',               # turtlebot4 slam.yaml (tb6_slam.launch.py)
}
COLLISION_STATE = 'collision_monitor_state'     # nav2.yaml state_topic; rescue_robot task_site_runtime subscribes it
ROBOT_VIEW = 'robot_view_relay'


class WiringError(Exception):
    """An expectation this tool cannot build (profile or field file it cannot read)."""


@dataclass(frozen=True)
class Link:
    id: str
    kind: str                # topic | service | action
    name: str
    writers: tuple           # full node names of the publisher / server, each exactly once
    readers: tuple           # full node names that must subscribe / be a client (more are allowed)
    external_writer: bool    # writer outside this repository (robot Pi, Create3): exactly one, node not judged
    external_reader: bool    # reader outside this repository: at least one
    source: str


def node(name, namespace=''):
    return namespace.rstrip('/') + '/' + name


def link(lid, kind, name, writers=(), readers=(), *, source, external_writer=False, external_reader=False):
    if kind not in KINDS or not name.startswith('/'):
        raise WiringError(f'{lid}: bad expectation {kind} {name!r}')
    if not writers and not external_writer:
        raise WiringError(f'{lid}: no writer expected for {name}')   # zero writers would always pass
    return Link(lid, kind, name, tuple(writers), tuple(readers), external_writer, external_reader, source)


# ---- expectations ------------------------------------------------------------

NAMES_ONLY_SHA = '0' * 64   # SAVED_MAP_AMCL needs a map hash to build values; only topic names are read here


def expected_links(deployment, field, *, robots=None, slam_phase='SEARCHING', now_ns=None):
    """Every connection the field launches make, from the code. ``robots`` limits the robot links."""
    from rescue_bringup import field_launch as fl
    from rescue_bringup.operations_launch import EXIT_LOCATION_TOPIC
    from rescue_contracts.overhead import overhead_passages_topic
    from rescue_coordination import ros_node as co

    if slam_phase not in SLAM_PHASES:
        raise WiringError(f'slam_phase must be one of {SLAM_PHASES}')
    ids = [r.id for r in deployment.robots]
    picked = ids if not robots else list(robots)
    unknown = sorted(set(picked) - set(ids))
    if unknown:
        raise WiringError(f'--robot {unknown} not in the profile ({ids})')
    if deployment.mission is None:
        raise WiringError('profile v2 with mission roles is required')
    role = {r: deployment.mission.role_of(r) for r in ids}
    scout = [r for r in ids if role[r] == 'SCOUT']
    now_ns = time.time_ns() if now_ns is None else now_ns
    coord = node(fl.COORDINATOR_NODE)
    freeze, db, report = node(NODES['map_freeze']), node(NODES['db_writer']), node(NODES['report_node'])
    values, modes = {}, {}
    for r in ids:
        if r not in field.robots:
            raise WiringError(f'field run file has no robots.{r}')
        modes[r] = field.robots[r]['task.saved_map.mode']
        try:
            values[r] = fl.robot_launch_values(
                deployment, field=field, robot_id=r, now_ns=now_ns,
                occupancy_sha256=NAMES_ONLY_SHA if modes[r] == 'SAVED_MAP_AMCL' else '')
        except fl.FieldLaunchError as exc:
            raise WiringError(f'{r}: {exc}') from exc
    adapter = {r: node(NODES['adapter'], values[r]['namespace']) for r in ids}
    relayed = [r for r in ids if modes[r] == 'FROZEN_MAP_AMCL'
               or (modes[r] == 'SLAM_THEN_FROZEN' and slam_phase == 'FROZEN')]
    frozen_readers = [r for r in ids if modes[r] != 'SAVED_MAP_AMCL']
    shown = [r for r in ids if r in picked]
    links = []
    for r in shown:
        v, ns = values[r], values[r]['namespace']
        t, w = v['task_input'], v['watchdog']
        here = lambda key: node(NODES[key], ns)  # noqa: E731
        src = f'field_launch.robot_launch_values({r})'
        links += [
            link(f'{r}.cmd_vel', 'topic', w['cmd_vel_out_topic'], [here('watchdog')], external_reader=True,
                 source=src + ' watchdog.cmd_vel_out_topic; reader Create3'),
            link(f'{r}.cmd_vel_watchdog_in', 'topic', w['cmd_vel_in_topic'],
                 [here('collision_monitor'), here('docking_server')], [here('watchdog')],
                 source=src + ' nav2_remaps'),
            link(f'{r}.executor_heartbeat', 'topic', t['executor_heartbeat_topic'], [adapter[r]], [here('watchdog')],
                 source=src),
            link(f'{r}.watchdog_heartbeat', 'topic', t['watchdog_heartbeat_topic'], [here('watchdog')], [adapter[r]],
                 source=src),
            link(f'{r}.odom', 'topic', t['odom_topic'], (), [adapter[r]], external_writer=True, source=src),
            link(f'{r}.scan', 'topic', t['scan_topic'], (), [adapter[r]], external_writer=True, source=src),
            link(f'{r}.dock_status', 'topic', t['dock_status_topic'], (), [adapter[r]], external_writer=True,
                 source=src),
        ]
        p = v['passage']
        for camera in t['camera_topics']:
            readers = [adapter[r]] + ([here('passage_points')] if camera == p['readiness_topic'] else [])
            links.append(link(f'{r}.camera:{camera}', 'topic', camera, (), readers, external_writer=True,
                              source=src + ' camera_topics (passage depth = readiness camera)'))
        if t['wheel_status_topic']:
            links.append(link(f'{r}.wheel_status', 'topic', t['wheel_status_topic'], (), [adapter[r]],
                              external_writer=True, source=src))
        links += [
            link(f'{r}.depth_camera_info', 'topic', p['camera_info_topic'], (), [here('passage_points')],
                 external_writer=True, source='field_launch.passage_points_launch_values'),
            link(f'{r}.passage_points', 'topic', p['points_topic'], [here('passage_points')],
                 [here('collision_monitor'), here('overhead')], source='field_launch._passage points_topic'),
            link(f'{r}.collision_monitor_state', 'topic', node(COLLISION_STATE, ns), [here('collision_monitor')],
                 [adapter[r]], source='task_site_runtime attach_passage (relative name)'),
        ]
        base = co.ROBOT_PREFIX + r
        links += [
            link(f'{r}.status', 'topic', base + '/status', [adapter[r]], [coord, db],
                 source='rescue_coordination ROBOT_PREFIX; operations_launch status_topics'),
            link(f'{r}.task_admission', 'topic', base + '/task_admission', [adapter[r]], [coord],
                 source='rescue_coordination ROBOT_PREFIX'),
            link(f'{r}.command_lease', 'topic', base + '/command_lease', [coord], [adapter[r]],
                 source='rescue_coordination ROBOT_PREFIX'),
            link(f'{r}.pose', 'topic', base + '/pose', [adapter[r]], [coord, db],
                 source='rescue_robot live_pose; operations_launch pose_topics'),
            link(f'{r}.map_state', 'topic', base + '/map_state', [adapter[r]],
                 [coord] + ([db] + [node(NODES['overhead'], values[o]['namespace']) for o in shown]
                            if r in scout else [])
                 + ([here('relay')] if r in relayed else []),
                 source='rescue_robot live_pose; db_writer/overhead read the SCOUT map_state'),
            link(f'{r}.overhead_passages', 'topic', overhead_passages_topic(r), [here('overhead')], [db],
                 source='rescue_contracts.overhead_passages_topic; operations_launch'),
            link(f'{r}.execute_task', 'action', base + '/execute_task', [adapter[r]], [coord],
                 source='rescue_coordination ROBOT_PREFIX + /execute_task'),
            link(f'{r}.navigate_to_pose', 'action', t['nav_action'], [here('bt_navigator')], [adapter[r]],
                 source=src + ' nav_action'),
            # the coordinator names this robot reads, from the field file (merged with the shared rows below)
            link(f'{r}.task_inputs', 'service', t['task.live.coordinator_service'], [coord], [adapter[r]],
                 source=src + ' task.live.coordinator_service'),
            link(f'{r}.system_state', 'topic', t['task.coordinator_state_topic'], [coord], [adapter[r]],
                 source=src + ' task.coordinator_state_topic'),
            link(f'{r}.targets', 'topic', co.TARGETS_TOPIC, [coord], [adapter[r]],
                 source="live_parameters coordinator_targets_topic '/rescue/targets'"),
            link(f'{r}.task_arm', 'service', node('task_arm', adapter[r]), [adapter[r]],
                 source='task_site_runtime ~/task_arm'),
            link(f'{r}.task_stop', 'service', node('task_stop', adapter[r]), [adapter[r]],
                 source='task_site_runtime ~/task_stop'),
        ]
        map_topic, src_map = t['task.saved_map.map_topic'], src + ' task.saved_map.*'
        if modes[r] == 'SLAM_THEN_FROZEN' and slam_phase == 'SEARCHING':
            links += [link(f'{r}.map', 'topic', map_topic, [here('slam_toolbox')], [adapter[r], freeze], source=src_map),
                      link(f'{r}.slam_pose', 'topic', t['task.saved_map.slam_localization_topic'],
                           [here('slam_toolbox')], [adapter[r], freeze], source=src_map)]
        elif r in relayed:
            links += [link(f'{r}.map', 'topic', map_topic, [here('relay')], [adapter[r], here('amcl')],
                           source='field_launch.frozen_localization_values map_topic'),
                      link(f'{r}.amcl_pose', 'topic', t['task.saved_map.localization_topic'], [here('amcl')],
                           [adapter[r]], source=src_map)]
            if modes[r] == 'FROZEN_MAP_AMCL':
                links.append(link(f'{r}.initialpose', 'topic', t['task.saved_map.initial_pose_topic'], [adapter[r]],
                                  [here('amcl')], source=src_map + ' (re-alignment on a newer frozen map)'))
    frozen = [adapter[r] for r in frozen_readers if r in picked] + [node(NODES['relay'], values[r]['namespace'])
                                                                    for r in relayed if r in picked]
    links += [
        link('system_state', 'topic', co.SYSTEM_STATE_TOPIC, [coord], [db], source='SYSTEM_STATE_TOPIC'),
        link('targets', 'topic', co.TARGETS_TOPIC, [coord], [db], source='TARGETS_TOPIC'),
        link('site_events', 'topic', co.SITE_EVENT_TOPIC, [node(NODES['webcam'])], [coord, db],
             source='SITE_EVENT_TOPIC; operations_launch SITE_EVENTS_TOPIC'),
        link('reports', 'topic', co.REPORT_TOPIC, [coord], [report], source='REPORT_TOPIC'),
        link('report_receipts', 'topic', co.REPORT_RECEIPT_TOPIC, [report], [coord], source='REPORT_RECEIPT_TOPIC'),
        link('target_locations', 'topic', co.TARGET_LOCATION_TOPIC, [node(NODES['target_locator'])], [coord],
             source='TARGET_LOCATION_TOPIC (amr_perception.launch.py)'),
        link('exit_location', 'topic', EXIT_LOCATION_TOPIC, [node(NODES['exit_locator'])], [coord, db],
             source='operations_launch EXIT_LOCATION_TOPIC (amr_perception.launch.py)'),
        link('operator_command', 'service', co.OPERATOR_COMMAND_SERVICE, [coord], [db],
             source='OPERATOR_COMMAND_SERVICE'),
    ]
    # FRONTIER: the coordinator reads the SCOUT live map under a fixed name (rescue_coordination SCOUT_MAP_TOPIC);
    # a profile namespace other than that name leaves it without a publisher, which this row shows
    if field.coordinator.get('search_mode') == 'FRONTIER':
        for r in (r for r in scout if r in picked):
            own = next(lk for lk in links if lk.id == f'{r}.map')
            links.append(link(f'{r}.coordinator_map', 'topic', co.SCOUT_MAP_TOPIC, own.writers, [coord],
                              source='rescue_coordination SCOUT_MAP_TOPIC (search_mode FRONTIER)'))
    if field.coordinator.get('map_mode') == 'SLAM_FREEZE':
        links += [
            link('map_freeze', 'service', co.MAP_FREEZE_SERVICE, [freeze], [coord], source='MAP_FREEZE_SERVICE'),
            link('frozen_map', 'topic', fl.FROZEN_TOPIC, [freeze], [coord] + frozen, source='FROZEN_TOPIC'),
            link('frozen_revision', 'topic', fl.FROZEN_REVISION_TOPIC, [freeze], [coord] + frozen,
                 source='FROZEN_REVISION_TOPIC'),
        ]
    return merge(links)


def merge(links):
    """One row per (kind, name): readers joined. Two different writer expectations for one name are refused."""
    rows = {}
    for lk in links:
        key = (lk.kind, lk.name)
        if key not in rows:
            rows[key] = lk
            continue
        old = rows[key]
        if (old.writers, old.external_writer) != (lk.writers, lk.external_writer):
            raise WiringError(f'{lk.name}: two writer expectations ({old.id}, {lk.id})')
        # a shared row keeps its id; rows of several robots become one robot-free id (robot_a.x + robot_b.x -> x)
        shared = [i for i in (old.id, lk.id) if '.' not in i]
        if shared:
            lid = shared[0]
        elif old.id.split('.', 1)[0] == lk.id.split('.', 1)[0]:
            lid = old.id   # one robot's row read by another node too (robot_a.map + robot_a.coordinator_map)
        else:
            lid = lk.id.split('.', 1)[1]
        rows[key] = Link(lid, lk.kind, lk.name, lk.writers, tuple(dict.fromkeys(old.readers + lk.readers)),
                         lk.external_writer, old.external_reader or lk.external_reader,
                         '; '.join(dict.fromkeys(old.source.split('; ') + lk.source.split('; '))))
    return list(rows.values())


# ---- judging -----------------------------------------------------------------

def qos_problem(pub, sub):
    """Why a publisher's offered QoS cannot reach a subscription's request, '' if it can, None if unknown."""
    reliability = {'RELIABLE': 1, 'BEST_EFFORT': 0}
    durability = {'TRANSIENT_LOCAL': 1, 'VOLATILE': 0}
    try:
        if reliability[pub['reliability']] < reliability[sub['reliability']]:
            return f'reliability {pub["reliability"]} -> {sub["reliability"]}'
        if durability[pub['durability']] < durability[sub['durability']]:
            return f'durability {pub["durability"]} -> {sub["durability"]}'
    except (KeyError, TypeError):
        return None
    return ''


def _missing_and_extra(want, seen):
    missing = [n for n in want if n not in seen]
    counted = {n: seen.count(n) for n in seen}
    duplicate = sorted(n for n, k in counted.items() if k > 1)
    extra = sorted(n for n in counted if n not in want)
    return missing, duplicate, extra


def judge(lk, graph):
    """One connection against the graph snapshot (see read_graph for its shape)."""
    row = dict(asdict(lk), status=NOT_RUN, problems=[], seen_writers=None, seen_readers=None, qos=[])
    if graph is None:
        row['problems'] = ['그래프를 읽지 못함']
        return row
    if lk.kind == 'topic':
        entry = graph['topics'].get(lk.name, {'publishers': [], 'subscribers': []})
        pubs, subs = entry['publishers'], entry['subscribers']
        writers, readers = [e['node'] for e in pubs], [e['node'] for e in subs]
    else:
        entry = graph[lk.kind + 's'].get(lk.name, {'servers': [], 'clients': []})
        writers, readers, pubs, subs = list(entry['servers']), list(entry['clients']), [], []
    row['seen_writers'], row['seen_readers'] = writers, readers
    problems, unknown = [], []
    what = '발행자' if lk.kind == 'topic' else '서버'
    if lk.external_writer:
        if len(writers) != 1:
            problems.append(f'{what} {len(writers)}개 (1개여야 함)')
    else:
        missing, duplicate, extra = _missing_and_extra(lk.writers, writers)
        if missing:
            problems.append(f'{what} 없음: {", ".join(missing)}')
        if duplicate:
            problems.append(f'{what} 중복: {", ".join(duplicate)}')
        if extra:
            problems.append(f'다른 {what}: {", ".join(extra)}')
    who = '구독' if lk.kind == 'topic' else '클라이언트'
    absent = [n for n in lk.readers if n not in readers]
    if absent:
        problems.append(f'{who} 없음: {", ".join(absent)}')
    if lk.external_reader and not readers:
        problems.append(f'{who} 0개')
    types = sorted({e['type'] for e in pubs + subs if e.get('type')})
    if len(types) > 1:
        problems.append('타입 불일치: ' + ', '.join(types))
    for p in pubs:
        for s in subs:
            why = qos_problem(p['qos'], s['qos'])
            if why is None:
                unknown.append(f'{p["node"]} -> {s["node"]}')
            elif why:
                problems.append(f'QoS 불일치 {p["node"]} -> {s["node"]}: {why}')
                row['qos'].append(dict(publisher=p['node'], subscriber=s['node'], problem=why))
    row['problems'] = problems + [f'QoS 판정 불가: {u}' for u in unknown]
    row['status'] = FAILED if problems else (NOT_RUN if unknown else PASSED)
    return row


def duplicate_nodes(graph):
    if graph is None:
        return dict(id='graph.duplicate_nodes', status=NOT_RUN, problems=['그래프를 읽지 못함'], names=[])
    counted = {}
    for name in graph['nodes']:
        counted[name] = counted.get(name, 0) + 1
    names = sorted(n for n, k in counted.items() if k > 1)
    return dict(id='graph.duplicate_nodes', status=FAILED if names else PASSED,
                problems=[f'같은 이름 노드 {counted[n]}개: {n}' for n in names], names=names)


def summarize(statuses):
    """Exit code: 2 if any FAILED, else 3 if any NOT_RUN or none, else 0 (scripts/field/preflight.py)."""
    if FAILED in statuses:
        return 2
    if not statuses or any(s != PASSED for s in statuses):
        return 3
    return 0


def check(links, graph):
    rows = [judge(lk, graph) for lk in links]
    nodes = duplicate_nodes(graph)
    return rows, nodes, summarize([r['status'] for r in rows] + [nodes['status']])


def table(rows, nodes):
    lines = [f'{"status":8} {"kind":7} {"id":40} name / problems']
    for r in rows:
        lines.append(f'{r["status"]:8} {r["kind"]:7} {r["id"]:40} {r["name"]}')
        lines += [f'{"":58}- {p}' for p in r['problems']]
    lines.append(f'{nodes["status"]:8} {"node":7} {nodes["id"]:40}')
    lines += [f'{"":58}- {p}' for p in nodes['problems']]
    return '\n'.join(lines)


# ---- graph -------------------------------------------------------------------

def _qos(profile):
    return {'reliability': profile.reliability.name, 'durability': profile.durability.name}


def read_graph(links, spin_time):
    """Graph snapshot for the given connections. Needs a sourced ROS 2; sets ROS_SUPER_CLIENT=True first.

    {'nodes': [full names, repeated when duplicated], 'topics': {name: {'publishers': [...], 'subscribers':
    [...]}} with entries {'node', 'type', 'qos': {'reliability', 'durability'}}, 'services'/'actions':
    {name: {'servers': [nodes], 'clients': [nodes]}}}
    """
    os.environ['ROS_SUPER_CLIENT'] = 'True'
    import rclpy
    from rclpy.action.graph import (get_action_client_names_and_types_by_node,
                                    get_action_server_names_and_types_by_node)
    from rclpy.parameter import Parameter

    rclpy.init()
    me = f'wiring_check_{os.getpid()}'
    probe = rclpy.create_node(me, enable_rosout=False, start_parameter_services=False,
                              parameter_overrides=[Parameter('start_type_description_service', value=False)])
    probe.destroy_publisher(probe._parameter_event_publisher)
    try:
        end = time.monotonic() + spin_time
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=0.1)
        pairs = [(n, ns) for n, ns in probe.get_node_names_and_namespaces() if n != me]
        graph = {'nodes': [node(n, ns) for n, ns in pairs], 'topics': {}, 'services': {}, 'actions': {}}
        for name in sorted({lk.name for lk in links if lk.kind == 'topic'}):
            graph['topics'][name] = {
                key: [{'node': node(e.node_name, e.node_namespace), 'type': e.topic_type, 'qos': _qos(e.qos_profile)}
                      for e in get(name)]
                for key, get in (('publishers', probe.get_publishers_info_by_topic),
                                 ('subscribers', probe.get_subscriptions_info_by_topic))}
        wanted = {kind: {lk.name for lk in links if lk.kind == kind} for kind in ('service', 'action')}
        for kind in ('services', 'actions'):
            graph[kind] = {name: {'servers': [], 'clients': []} for name in wanted[kind[:-1]]}
        for n, ns in sorted(set(pairs)):
            full = node(n, ns)
            for key, names in (
                    ('servers', probe.get_service_names_and_types_by_node(n, ns)),
                    ('clients', probe.get_client_names_and_types_by_node(n, ns))):
                for name, _ in names:
                    if name in graph['services']:
                        graph['services'][name][key].append(full)
            for key, names in (
                    ('servers', get_action_server_names_and_types_by_node(probe, n, ns)),
                    ('clients', get_action_client_names_and_types_by_node(probe, n, ns))):
                for name, _ in names:
                    if name in graph['actions']:
                        graph['actions'][name][key].append(full)
        return graph
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()


# ---- main --------------------------------------------------------------------

def main(argv=None, *, graph_reader=read_graph):
    ap = argparse.ArgumentParser(description='현장 배선 점검 (그래프 읽기만). 연결마다 발행자·구독자·QoS')
    ap.add_argument('--profile', required=True, type=Path, help='배치 프로파일 v2 JSON')
    ap.add_argument('--field', required=True, type=Path, help='현장 실행 파일 (config/field/search_area.field.json)')
    ap.add_argument('--robot', action='append', default=[], help='이 로봇의 연결만 (여러 번). 공통 연결은 늘 본다')
    ap.add_argument('--slam-phase', choices=SLAM_PHASES, default='SEARCHING',
                    help='TB-6 지도 단계: SEARCHING(slam_toolbox) 또는 FROZEN(고정 뒤 relay + AMCL)')
    ap.add_argument('--spin-time', type=float, default=5.0, help='그래프 발견 대기 (s)')
    ap.add_argument('--json', type=Path, help='JSON 결과를 이 경로에도 쓴다')
    args = ap.parse_args(argv)
    if not args.spin_time > 0:
        ap.error('--spin-time must be positive')
    from rescue_bringup.field_launch import FieldLaunchError, load_field
    from rescue_bringup.profile import ProfileError, load_deployment
    try:
        links = expected_links(load_deployment(args.profile), load_field(args.field), robots=args.robot,
                               slam_phase=args.slam_phase)
    except (WiringError, FieldLaunchError, ProfileError, OSError) as exc:
        print(f'wiring_check: 기대 연결표를 만들지 못함: {exc}', file=sys.stderr)
        return 3
    graph, error = None, None
    try:
        graph = graph_reader(links, args.spin_time)
    except ImportError as exc:
        error = f'ROS 를 가져오지 못함 (NOT_RUN): {exc}'
    except Exception as exc:   # rclpy.init, RMW or Discovery Server failure: no graph, so nothing is judged
        error = f'그래프를 읽지 못함 (NOT_RUN): {type(exc).__name__}: {exc}'
    rows, nodes, code = check(links, graph)
    result = dict(tool='scripts/field/wiring_check.py', profile=str(args.profile), field=str(args.field),
                  robots=args.robot or None, slam_phase=args.slam_phase, spin_time_s=args.spin_time,
                  environment={k: os.environ.get(k) for k in ENV_KEYS}, graph_error=error,
                  measured_at_ns=time.time_ns(), connections=rows, duplicate_nodes=nodes, exit_code=code)
    text = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.json:
        args.json.write_text(text)
    if error:
        print(error, file=sys.stderr)
    print(table(rows, nodes), file=sys.stderr)
    print(f'wiring_check exit={code} ({ {0: "PASSED", 2: "FAILED", 3: "NOT_RUN 있음"}[code] })', file=sys.stderr)
    sys.stdout.write(text)
    return code


if __name__ == '__main__':
    sys.exit(main())
