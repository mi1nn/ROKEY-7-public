#!/usr/bin/env python3
"""Field link check: how often each robot topic arrives here, and how long it went silent (read-only).

Why: "the AMR keeps stopping". The command watchdog latches a stop when the executor heartbeat is
older than ``heartbeat_max_age_s`` (rescue_robot/watchdog.py, a launch argument), and the
readiness monitor drops READY when odom/scan/TF are older than the profile ``max_age_s``. A ping
between two PCs on the same Wi-Fi lost 2 of 3 and took 594 ms (measured by the owner,
2026-10-13). This tool shows per topic whether the silences are long enough to explain a stop.

It only subscribes. It calls no service or action and never moves the robot. The node is made
without /rosout, parameter services and the type description service, and the /parameter_events
publisher rclpy always creates is destroyed right after. The one thing it cannot avoid: rclpy
publishes one ParameterEvent (use_sim_time declared) while the node is being created. Its footprint
is then one DDS participant joining the graph of the current environment (ROS_DOMAIN_ID, Discovery
Server) with subscriptions only (tests/ros/e3_link_check_probe.py checks the graph).

Topics (``--robot`` = the logical id in the profile):
- executor_heartbeat, watchdog_heartbeat: std_msgs/Header at ``<namespace>/rescue/...`` (the launch
  convention of scripts/field/search-area/nav2_watchdog.launch.py; the profile keeps no heartbeat
  topic). Limit: ``--heartbeat-max-age-s``, the value the watchdog runs with.
- odom, scan, tf: the robot parameters file of the profile; limits = readiness ``max_age_s``. ``tf`` counts
  only messages that carry the profile ``map_frame`` -> ``odom_frame`` transform: /tf also carries the fast
  odom -> base_link, which would hide a map -> odom silence.
- robot_status: rescue_interfaces/RobotStatus at /rescue/robots/<robot>/status; no limit (measured only).
- camera_status:<cam>: rescue_interfaces/CameraStatus of every profile site camera; no limit.
``--topic role=/name`` replaces a topic name (e.g. a field namespace that differs from the profile).

Per topic: messages, mean period, the longest silence (including the start and end of the window,
after ``--warmup-s``), the silences over 0.5 s and over 1.0 s, and the receive time minus the message
stamp. That last value assumes synchronized clocks and is [참고] only. All subscriptions are
BEST_EFFORT so they match both reliable and best-effort publishers; a reliable publisher would
resend to a reliable subscriber, so a loss here is the raw link, not necessarily what the watchdog saw.

Verdict per topic: FAILED when nothing arrived or the longest silence is over its limit, PASSED
otherwise, NOT_RUN when the profile does not configure the topic (no odom topic, no readiness block):
that row stays in the table, nothing is subscribed. Exit code as scripts/field/preflight.py:
0 = all PASSED, 2 = at least one FAILED, 3 = nothing failed but at least one NOT_RUN.

    python3 scripts/field/link_check.py --profile <profile.json> --robot robot_a --duration-s 60 \\
        --heartbeat-max-age-s 0.5 [--topic executor_heartbeat=/robot6/rescue/executor_heartbeat] \\
        [--json <out.json>]
(not -I: ROS needs PYTHONPATH; source ROS and the rescue_interfaces install first)
"""
import argparse
from dataclasses import asdict, dataclass
import json
import os
import sys
import time
from pathlib import Path

PASSED, FAILED, NOT_RUN = 'PASSED', 'FAILED', 'NOT_RUN'
S = 1_000_000_000
THRESHOLDS_NS = (S // 2, S)  # the two silence counts: over 0.5 s, over 1.0 s
LAUNCH_CONVENTION = 'scripts/field/search-area/nav2_watchdog.launch.py:36-37 (namespace + /rescue/...)'
ENV_KEYS = ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
            'RMW_IMPLEMENTATION')


class LinkCheckError(Exception):
    """A request this tool cannot answer (unknown robot, role or profile value)."""


@dataclass(frozen=True)
class TopicSpec:
    role: str
    topic: str
    type: str          # pkg/msg/Name
    limit_s: float | None
    source: str        # where the name and limit come from
    frames: tuple | None = None  # tf only: (parent, child) a message must carry to count


def gap_stats(arrivals_ns, start_ns, end_ns):
    """Arrival times (monotonic ns) in [start, end] -> counts and silences; edges of the window count."""
    arrivals = sorted(arrivals_ns)
    edges = [start_ns, *arrivals, end_ns]
    gaps = [b - a for a, b in zip(edges, edges[1:])]
    if not arrivals:
        gaps = [end_ns - start_ns]
    return dict(
        count=len(arrivals),
        mean_period_s=(arrivals[-1] - arrivals[0]) / (len(arrivals) - 1) / S if len(arrivals) > 1 else None,
        max_gap_s=max(gaps) / S,
        gaps_over_0_5_s=sum(g > THRESHOLDS_NS[0] for g in gaps),
        gaps_over_1_0_s=sum(g > THRESHOLDS_NS[1] for g in gaps))


def stamp_delay(pairs):
    """[(received wall ns, message stamp ns)] -> receive minus stamp; needs synced clocks, so [참고] only."""
    if not pairs:
        return None
    delays = [(received - stamp) / S for received, stamp in pairs]
    return dict(mean_s=sum(delays) / len(delays), max_s=max(delays), min_s=min(delays), basis='참고')


def has_transform(msg, frames):
    """True when ``frames`` is None (count every message) or the TF message carries parent -> child."""
    if frames is None:
        return True
    parent, child = (f.lstrip('/') for f in frames)
    return any(t.header.frame_id.lstrip('/') == parent and t.child_frame_id.lstrip('/') == child
               for t in getattr(msg, 'transforms', ()))


def select_topics(deployment, robot_id, overrides, *, heartbeat_max_age_s):
    """The topics of one robot (and the site cameras) from a loaded profile; ``overrides`` = {role: topic}."""
    robot = next((r for r in deployment.robots if r.id == robot_id), None)
    if robot is None:
        raise LinkCheckError(f'robot {robot_id!r} is not in the profile ({[r.id for r in deployment.robots]})')
    params, ns = robot.parameters, robot.namespace.rstrip('/')
    ready = params.readiness
    hb = f'{LAUNCH_CONVENTION}; limit --heartbeat-max-age-s'
    specs = [TopicSpec('executor_heartbeat', f'{ns}/rescue/executor_heartbeat', 'std_msgs/msg/Header',
                       heartbeat_max_age_s, hb),
             TopicSpec('watchdog_heartbeat', f'{ns}/rescue/watchdog_heartbeat', 'std_msgs/msg/Header',
                       heartbeat_max_age_s, hb),
             TopicSpec('robot_status', f'/rescue/robots/{robot_id}/status', 'rescue_interfaces/msg/RobotStatus',
                       None, 'rescue_interfaces README (/rescue/robots/{r}/status); no limit, measured only')]
    unset = f'not configured in profile {robot.parameters_file}'
    specs.append(TopicSpec('odom', params.odom_topic, 'nav_msgs/msg/Odometry',
                           ready.max_age('odom') if ready else None, f'profile {robot.parameters_file}')
                 if params.odom_topic else TopicSpec('odom', '', 'nav_msgs/msg/Odometry', None, unset))
    if ready is not None:
        specs += [TopicSpec('scan', ready.scan_topic, 'sensor_msgs/msg/LaserScan', ready.max_age('scan'),
                            f'profile {robot.parameters_file}'),
                  TopicSpec('tf', ready.tf_topic, 'tf2_msgs/msg/TFMessage', ready.max_age('map_to_odom_tf'),
                            f'profile {robot.parameters_file} (limit map_to_odom_tf, '
                            f'{ready.map_frame} -> {ready.odom_frame} only)', (ready.map_frame, ready.odom_frame))]
    else:
        specs += [TopicSpec('scan', '', 'sensor_msgs/msg/LaserScan', None, unset),
                  TopicSpec('tf', '', 'tf2_msgs/msg/TFMessage', None, unset)]
    for camera in deployment.site_cameras:
        specs.append(TopicSpec(f'camera_status:{camera.id}', f'/rescue/cameras/{camera.id}/status',
                               'rescue_interfaces/msg/CameraStatus', None,
                               'rescue_interfaces README (/rescue/cameras/{cam}/status); no limit, measured only'))
    by_role = {s.role: s for s in specs}
    unknown = sorted(set(overrides) - set(by_role))
    if unknown:
        raise LinkCheckError(f'--topic role(s) {unknown} unknown; roles are {sorted(by_role)}')
    return [TopicSpec(s.role, overrides[s.role], s.type, s.limit_s, 'command line', s.frames) if s.role in overrides
            else s for s in specs]


def verdict(spec, stats):
    if not spec.topic:
        return NOT_RUN
    if stats['count'] == 0:
        return FAILED
    if spec.limit_s is not None and stats['max_gap_s'] > spec.limit_s:
        return FAILED
    return PASSED


def summarize(statuses):
    """Exit code: 2 if any FAILED, else 3 if any NOT_RUN or none, else 0 (scripts/field/preflight.py)."""
    if FAILED in statuses:
        return 2
    if not statuses or any(s != PASSED for s in statuses):
        return 3
    return 0


def message_stamp_ns(msg):
    """The stamp a message carries, or None (Header, header.stamp, TF's first transform, *_at_ns times)."""
    stamp = getattr(msg, 'header', None)
    stamp = getattr(stamp, 'stamp', None) if stamp is not None else getattr(msg, 'stamp', None)
    if stamp is None and getattr(msg, 'transforms', None):
        stamp = msg.transforms[0].header.stamp
    if stamp is not None:
        ns = stamp.sec * S + stamp.nanosec
        return ns or None
    for value in (getattr(msg, 'observed_at_ns', None), getattr(getattr(msg, 'time', None), 'observed_at_ns', None)):
        if value:
            return int(value)
    return None


def table(rows, window_s):
    lines = [f'window {window_s:.1f} s  (silence = longest gap incl. window edges; delay = receive - stamp [참고])',
             f'{"status":8} {"role":28} {"count":>6} {"period":>8} {"max gap":>8} {"limit":>6} '
             f'{">0.5s":>6} {">1.0s":>6} {"delay":>14}  topic']
    for r in rows:
        st, d = r.get('stats'), r.get('stamp_delay')
        fmt = lambda v, n=3: '-' if v is None else f'{v:.{n}f}'  # noqa: E731
        lines.append(
            f'{r["status"]:8} {r["role"]:28} {fmt(st and st["count"], 0) if st else "-":>6} '
            f'{fmt(st["mean_period_s"]) if st else "-":>8} {fmt(st["max_gap_s"]) if st else "-":>8} '
            f'{fmt(r["limit_s"], 2):>6} {st["gaps_over_0_5_s"] if st else "-":>6} {st["gaps_over_1_0_s"] if st else "-":>6} '
            f'{(fmt(d["mean_s"]) + "/" + fmt(d["max_s"])) if d else "-":>14}  {r["topic"]}')
    return '\n'.join(lines)


def run(specs, duration_s, warmup_s):
    """Subscribe to every spec for warmup + duration; return rows. Needs a sourced ROS 2 environment."""
    import importlib
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node(f'link_check_{os.getpid()}', enable_rosout=False, start_parameter_services=False,
                             parameter_overrides=[Parameter('start_type_description_service', value=False)])
    # rclpy always makes this publisher (it already sent the use_sim_time event); nothing here needs it.
    node.destroy_publisher(node._parameter_event_publisher)
    qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    seen = {s.role: ([], []) for s in specs}   # role -> (monotonic arrivals, (wall, stamp) pairs)
    window = {}
    rows = []
    try:
        for spec in (s for s in specs if s.topic):
            pkg, _, name = spec.type.partition('/msg/')
            msg_type = getattr(importlib.import_module(f'{pkg}.msg'), name)

            def on_msg(msg, role=spec.role, frames=spec.frames):
                mono, wall = time.monotonic_ns(), time.time_ns()
                if 'start' not in window or mono > window['end'] or not has_transform(msg, frames):
                    return  # warm-up (discovery), after the window, or a TF without the checked transform
                arrivals, pairs = seen[role]
                arrivals.append(mono)
                stamp = message_stamp_ns(msg)
                if stamp is not None:
                    pairs.append((wall, stamp))
            node.create_subscription(msg_type, spec.topic, on_msg, qos)
        end_warmup = time.monotonic() + warmup_s
        while time.monotonic() < end_warmup:
            rclpy.spin_once(node, timeout_sec=0.05)
        window['start'] = time.monotonic_ns()
        window['end'] = window['start'] + int(duration_s * S)
        while time.monotonic_ns() < window['end']:
            rclpy.spin_once(node, timeout_sec=0.02)
        for spec in specs:
            if not spec.topic:
                rows.append(dict(asdict(spec), status=NOT_RUN, stats=None, stamp_delay=None, publishers_seen=None))
                continue
            arrivals, pairs = seen[spec.role]
            stats = gap_stats(arrivals, window['start'], window['end'])
            publishers = node.count_publishers(spec.topic)
            rows.append(dict(asdict(spec), status=verdict(spec, stats), stats=stats, stamp_delay=stamp_delay(pairs),
                             publishers_seen=publishers))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--profile', required=True, help='deployment profile v2 JSON')
    parser.add_argument('--robot', required=True, help='logical robot id in the profile (e.g. robot_a)')
    parser.add_argument('--duration-s', type=float, required=True, help='measurement window after warm-up')
    parser.add_argument('--heartbeat-max-age-s', type=float, required=True,
                        help='the command watchdog heartbeat_max_age_s it runs with (launch argument)')
    parser.add_argument('--warmup-s', type=float, default=2.0, help='discovery time not measured (default 2.0)')
    parser.add_argument('--topic', action='append', default=[], metavar='ROLE=/NAME', help='replace a topic name')
    parser.add_argument('--json', help='also write the result as JSON to this file')
    args = parser.parse_args(argv)
    if not args.duration_s > 0 or not args.heartbeat_max_age_s > 0 or args.warmup_s < 0:
        parser.error('--duration-s and --heartbeat-max-age-s must be positive, --warmup-s not negative')
    overrides = {}
    for item in args.topic:
        role, sep, name = item.partition('=')
        if not sep or not name.startswith('/'):
            parser.error(f'--topic {item!r}: use ROLE=/absolute/name')
        overrides[role] = name
    from rescue_bringup.profile import load_deployment
    try:
        specs = select_topics(load_deployment(args.profile), args.robot, overrides,
                              heartbeat_max_age_s=args.heartbeat_max_age_s)
    except LinkCheckError as exc:
        print(f'link_check: {exc}', file=sys.stderr)
        return 2
    rows = run(specs, args.duration_s, args.warmup_s)
    result = dict(tool='link_check', robot=args.robot, profile=str(Path(args.profile)), duration_s=args.duration_s,
                  warmup_s=args.warmup_s, heartbeat_max_age_s=args.heartbeat_max_age_s,
                  environment={k: os.environ.get(k) for k in ENV_KEYS}, measured_at_ns=time.time_ns(), topics=rows)
    code = summarize([r['status'] for r in rows])
    result['exit_code'] = code
    print(table(rows, args.duration_s))
    print(f'exit {code} (0 all PASSED, 2 a FAILED, 3 a NOT_RUN)')
    if args.json:
        Path(args.json).write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    return code


if __name__ == '__main__':
    sys.exit(main())
