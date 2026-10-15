#!/usr/bin/env python3
"""Fake nodes for the parts of the field graph this repository does not start (one-PC wiring rehearsal).

Run by run.sh after rehearsal.launch.py. Every expected connection of scripts/field/wiring_check.py whose
writer or reader is not one of this repository's nodes gets an endpoint on a fake node of the expected
name: robot Pi topics (one fake Pi per robot), the Create3 cmd_vel reader, Nav2 (collision_monitor,
docking_server, bt_navigator), slam_toolbox, AMCL, the depth container (passage_points), the webcam
node and the perception locators. Fakes only make endpoints; they publish nothing and answer nothing, except
that the Nav2 lifecycle manager answers is_active success (a Nav2 that came up; ACTIVE below).

Types: the type a real node uses on that name when one is there, else DEFAULT_TYPES. QoS of a fake writer:
FAKE_WRITER_QOS [추정, the real Pi / Nav2 / SLAM QoS is not measured here]; a fake reader asks the least
(BEST_EFFORT, VOLATILE), so a fake never makes a QoS failure of its own. So the rehearsal judges real
node <-> real node connections fully and real <-> fake ones by name, count and type only.

    python3 fake_graph.py --profile <p> --field <f> --slam-phase SEARCHING|FROZEN [--wait-s 20]
"""
import argparse
from pathlib import Path
import signal
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))   # scripts/field: wiring_check

DEFAULT_TYPES = {
    'odom': 'nav_msgs/msg/Odometry', 'scan': 'sensor_msgs/msg/LaserScan',
    'dock_status': 'irobot_create_msgs/msg/DockStatus', 'camera': 'sensor_msgs/msg/CompressedImage',
    'depth_camera_info': 'sensor_msgs/msg/CameraInfo', 'passage_points': 'sensor_msgs/msg/PointCloud2',
    'cmd_vel_watchdog_in': 'geometry_msgs/msg/TwistStamped', 'cmd_vel': 'geometry_msgs/msg/TwistStamped',
    'collision_monitor_state': 'nav2_msgs/msg/CollisionMonitorState', 'map': 'nav_msgs/msg/OccupancyGrid',
    'slam_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    'amcl_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    'initialpose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    'start_align': 'std_msgs/msg/String',
    'site_events': 'rescue_interfaces/msg/SiteEvent', 'target_locations': 'rescue_interfaces/msg/TargetLocation',
    'exit_location': 'rescue_interfaces/msg/TargetLocation',
    'navigate_to_pose': 'nav2_msgs/action/NavigateToPose',
}
# [추정] not measured: sensor streams best effort, latched maps and AMCL pose transient local
FAKE_WRITER_QOS = {'odom': ('BEST_EFFORT', 'VOLATILE'), 'scan': ('BEST_EFFORT', 'VOLATILE'),
                   'dock_status': ('BEST_EFFORT', 'VOLATILE'), 'camera': ('BEST_EFFORT', 'VOLATILE'),
                   'depth_camera_info': ('BEST_EFFORT', 'VOLATILE'), 'passage_points': ('BEST_EFFORT', 'VOLATILE'),
                   'map': ('RELIABLE', 'TRANSIENT_LOCAL'), 'amcl_pose': ('RELIABLE', 'TRANSIENT_LOCAL'),
                   'start_align': ('RELIABLE', 'TRANSIENT_LOCAL')}   # start_align latches its verdict


def ours(deployment, field, slam_phase):
    """Full names of the nodes rehearsal.launch.py starts (this repository's own field nodes)."""
    import wiring_check as wc
    from rescue_bringup.field_launch import COORDINATOR_NODE
    names = {'/' + COORDINATOR_NODE, '/' + wc.NODES['map_freeze'], '/' + wc.NODES['db_writer'],
             '/' + wc.NODES['report_node']}
    for robot in deployment.robots:
        ns = robot.parameters.namespace
        names |= {wc.node(wc.NODES[k], ns) for k in ('adapter', 'watchdog', 'overhead')}
        mode = field.robots[robot.id]['task.saved_map.mode']
        if mode == 'FROZEN_MAP_AMCL' or (mode == 'SLAM_THEN_FROZEN' and slam_phase == 'FROZEN'):
            names.add(wc.node(wc.NODES['relay'], ns))
    return names


def IDLE(request, response):
    return response


def ACTIVE(request, response):
    """The fake Nav2 lifecycle manager says active, as a Nav2 that came up: the executor's nav2 READY needs it
    (field run1015b), and the rehearsal arms both executors. The only fake answer that is not empty."""
    response.success = True
    return response


def role(link_id):
    tail = link_id.split('.', 1)[-1]
    return 'camera' if tail.startswith('camera:') else tail


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--profile', required=True)
    ap.add_argument('--field', required=True)
    ap.add_argument('--slam-phase', choices=('SEARCHING', 'FROZEN'), required=True)
    ap.add_argument('--wait-s', type=float, default=20.0, help='wait this long for our nodes to appear')
    args = ap.parse_args(argv)

    import rclpy
    from rclpy.action import ActionClient, ActionServer, GoalResponse
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_action, get_message, get_service
    import wiring_check as wc
    from rescue_bringup.field_launch import load_field
    from rescue_bringup.profile import load_deployment

    deployment, field = load_deployment(args.profile), load_field(args.field)
    links = wc.expected_links(deployment, field, slam_phase=args.slam_phase)
    real = ours(deployment, field, args.slam_phase)
    rclpy.init()
    looker = rclpy.create_node('rehearsal_fake_looker', enable_rosout=False, start_parameter_services=False)
    end = time.monotonic() + args.wait_s
    while time.monotonic() < end:
        seen = {wc.node(n, ns) for n, ns in looker.get_node_names_and_namespaces()}
        if real <= seen:
            break
        rclpy.spin_once(looker, timeout_sec=0.2)
    missing = sorted(real - {wc.node(n, ns) for n, ns in looker.get_node_names_and_namespaces()})
    print(f'fake_graph: our nodes not seen after {args.wait_s} s: {missing or "none"}', flush=True)
    time.sleep(2.0)   # let our nodes finish making endpoints

    topic_types = dict(looker.get_topic_names_and_types())
    service_types = dict(looker.get_service_names_and_types())
    nodes, made = {}, []

    def fake(full):
        if full not in nodes:
            namespace, _, name = full.rpartition('/')
            nodes[full] = rclpy.create_node(name, namespace=namespace or '/', enable_rosout=False,
                                            start_parameter_services=False)
        return nodes[full]

    def qos(reliability, durability):
        return QoSProfile(depth=10, reliability=getattr(ReliabilityPolicy, reliability),
                          durability=getattr(DurabilityPolicy, durability))
    for lk in links:
        r = role(lk.id)
        writers = [wc.node('rehearsal_pi', '/' + lk.name.split('/')[1])] if lk.external_writer else \
            [w for w in lk.writers if w not in real]
        readers = [n for n in lk.readers if n not in real]
        if lk.external_reader:
            readers.append(wc.node('rehearsal_create3', '/' + lk.name.split('/')[1]))
        if not writers and not readers:
            continue
        if lk.kind == 'topic':
            name = (topic_types.get(lk.name) or [DEFAULT_TYPES.get(r, 'std_msgs/msg/String')])[0]
            kind = get_message(name)
            for w in writers:
                made.append(fake(w).create_publisher(kind, lk.name, qos(*FAKE_WRITER_QOS.get(r, ('RELIABLE', 'VOLATILE')))))
            for n in readers:
                made.append(fake(n).create_subscription(kind, lk.name, lambda m: None, qos('BEST_EFFORT', 'VOLATILE')))
        elif lk.kind == 'service':
            kind = get_service((service_types.get(lk.name) or ['std_srvs/srv/Trigger'])[0])
            for w in writers:
                made.append(fake(w).create_service(kind, lk.name, ACTIVE if r == 'nav2_lifecycle' else IDLE))
            for n in readers:
                made.append(fake(n).create_client(kind, lk.name))
        else:
            kind = get_action(DEFAULT_TYPES.get(r, 'rescue_interfaces/action/ExecuteTask'))
            for w in writers:   # never accepts: a fake must not look like a robot that moves
                made.append(ActionServer(fake(w), kind, lk.name, lambda goal: kind.Result(),
                                         goal_callback=lambda goal: GoalResponse.REJECT))
            for n in readers:
                made.append(ActionClient(fake(n), kind, lk.name))
        print(f'fake {lk.kind:7} {lk.name:55} writers={writers} readers={readers}', flush=True)
    looker.destroy_node()
    executor = MultiThreadedExecutor()
    for n in nodes.values():
        executor.add_node(n)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    print(f'fake_graph: {len(nodes)} fake nodes up', flush=True)
    try:
        while not stop:
            executor.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for h in made:
            if isinstance(h, (ActionServer, ActionClient)):
                h.destroy()
        for n in nodes.values():
            n.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
