#!/usr/bin/env python3
"""Targets and QR exits as RViz markers, for runs without the web console (웹 없는 트랙, 2026-10-15).

    target_markers.py [--period 1.0]

RViz cannot draw our own messages (TargetSnapshot, TargetLocation), so this node subscribes to them and
publishes a visualization_msgs/MarkerArray on /rescue/noweb/markers:
- /rescue/targets (the coordinator's TargetSnapshot, latched): one sphere per target, coloured by stage
  (CANDIDATE, CONFIRMED, RESCUED, DISMISSED), with a short ID and the stage as text.
- /rescue/target_locations (target_locator, latched): the latest doll location per track that is not yet part
  of a target, as a small CANDIDATE sphere.
- /rescue/exit_location (exit_locator, latched): the latest location per QR exit, as a cube named by its QR.
A location past its own valid_until_ns is drawn faded and its text says STALE and how many seconds past: the
age comes from the message time and this PC's clock (the publisher's clock error is not removed). A message
with no mission frame is not drawn; the reason is logged. Markers use the message's mission frame (map).

It only subscribes and publishes markers: nothing goes to the robots, no images are read. The text is ASCII
because RViz's text font may not have Hangul [추정, not checked].
"""
import argparse

TARGETS_TOPIC = '/rescue/targets'                    # rescue_coordination.ros_node.TARGETS_TOPIC
TARGET_LOCATIONS_TOPIC = '/rescue/target_locations'  # rescue_perception.target_locator
EXIT_LOCATION_TOPIC = '/rescue/exit_location'        # rescue_perception.exit_qr
MARKERS_TOPIC = '/rescue/noweb/markers'
EXIT_CLASS_LABEL = 'exit_qr'                         # rescue_coordination.ros_node.EXIT_CLASS_LABEL

SECOND = 1_000_000_000
COLOURS = {  # rgb 0..1
    'CANDIDATE': (1.0, 0.85, 0.0),   # yellow
    'CONFIRMED': (1.0, 0.35, 0.0),   # orange
    'RESCUED': (0.1, 0.8, 0.2),      # green
    'DISMISSED': (0.5, 0.5, 0.5),    # grey
    'EXIT': (0.1, 0.5, 1.0),         # blue
}
FADED_ALPHA = 0.3
SIZE = {'target': 0.35, 'candidate': 0.2, 'exit': 0.3}   # metres, display only
TEXT_SIZE, TEXT_LIFT = 0.18, 0.4


def stage(estimate):
    """The stage drawn for one TargetEstimate."""
    if estimate.rescue_state == 'RESCUE_CONFIRMED':
        return 'RESCUED'
    if estimate.confirmation_state == 'CONFIRMED':
        return 'CONFIRMED'
    if estimate.confirmation_state == 'DISMISSED':
        return 'DISMISSED'
    return 'CANDIDATE'   # CANDIDATE, NEEDS_REOBSERVATION


def location_key(location):
    observation = location.observation
    return observation.track_id if observation.has_track_id else location.location_id


def keep_latest(store, location):
    """Keep the latest TargetLocation per track (or per location_id without a track), by its own time."""
    key = location_key(location)
    kept = store.get(key)
    if kept is None or location.observation.time.valid_until_ns >= kept.observation.time.valid_until_ns:
        store[key] = location


def _age_note(valid_until_ns, now_ns):
    late = now_ns - valid_until_ns
    return (f' STALE {late // SECOND}s', FADED_ALPHA) if late > 0 else ('', 1.0)


def _pair(specs, ns, kind, frame, point, rgb, alpha, text):
    index = len([s for s in specs if s['ns'] == ns])
    size = SIZE[ns]
    specs.append(dict(ns=ns, id=index, kind=kind, frame=frame, x=point.x, y=point.y, z=point.z + size / 2,
                      scale=(size, size, size), rgba=rgb + (alpha,), text=''))
    specs.append(dict(ns=ns, id=index + 1, kind='TEXT', frame=frame, x=point.x, y=point.y, z=point.z + TEXT_LIFT,
                      scale=(0.0, 0.0, TEXT_SIZE), rgba=(1.0, 1.0, 1.0, max(alpha, 0.6)), text=text))


def marker_specs(targets, candidates, exits, *, now_ns):
    """(marker dicts, skipped reasons) for the latest snapshot targets, candidate and exit locations.

    Each marker dict: ns, id, kind (SPHERE, CUBE, TEXT), frame, x, y, z, scale, rgba, text.
    """
    specs, skipped = [], []
    used = {loc.location_id for estimate in targets for loc in estimate.locations}
    for estimate in targets:
        frame = estimate.locations[-1].frame_ref.mission_frame if estimate.locations else ''
        if not frame:
            skipped.append(f'target {estimate.target_id}: no mission frame')
            continue
        name = stage(estimate)
        note, alpha = _age_note(estimate.valid_until_ns, now_ns)
        _pair(specs, 'target', 'SPHERE', frame, estimate.mission_position, COLOURS[name], alpha,
              f'{estimate.target_id[:8]} {name}{note}')
    for ns, kind, store in (('candidate', 'SPHERE', candidates), ('exit', 'CUBE', exits)):
        for key in sorted(store):
            location = store[key]
            if ns == 'candidate' and location.location_id in used:
                continue
            frame = location.frame_ref.mission_frame
            if not frame:
                skipped.append(f'{ns} {key}: no mission frame')
                continue
            note, alpha = _age_note(location.observation.time.valid_until_ns, now_ns)
            if location.projection_status != 'VALID':
                note = f' {location.projection_status}{note}'
            text = f'EXIT {key[:12]}{note}' if ns == 'exit' else f'{location.location_id[:8]} CANDIDATE{note}'
            _pair(specs, ns, kind, frame, location.mission_position, COLOURS['EXIT' if ns == 'exit' else 'CANDIDATE'],
                  alpha, text)
    return specs, skipped


def to_marker_array(specs, stamp):
    """MarkerArray: DELETEALL first, then every spec (the whole picture is redrawn each time)."""
    from visualization_msgs.msg import Marker, MarkerArray
    clear = Marker(action=Marker.DELETEALL)
    markers = [clear]
    kinds = {'SPHERE': Marker.SPHERE, 'CUBE': Marker.CUBE, 'TEXT': Marker.TEXT_VIEW_FACING}
    for spec in specs:
        marker = Marker(ns=spec['ns'], id=spec['id'], type=kinds[spec['kind']], action=Marker.ADD, text=spec['text'])
        marker.header.frame_id, marker.header.stamp = spec['frame'], stamp
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec['x'], spec['y'], spec['z']
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = spec['scale']
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec['rgba']
        markers.append(marker)
    return MarkerArray(markers=markers)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--period', type=float, default=1.0,
                        help='redraw period in seconds, so STALE shows without new messages (display only)')
    args = parser.parse_args(argv)
    if not args.period > 0:
        parser.error('--period must be positive')

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import TargetLocation, TargetSnapshot
    from visualization_msgs.msg import MarkerArray

    rclpy.init()
    node = rclpy.create_node('noweb_target_markers')
    log = node.get_logger()
    state = {'targets': [], 'candidates': {}, 'exits': {}, 'skipped': set()}
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(MarkerArray, MARKERS_TOPIC, 1)

    def redraw():
        now = node.get_clock().now()
        specs, skipped = marker_specs(state['targets'], state['candidates'], state['exits'], now_ns=now.nanoseconds)
        for reason in set(skipped) - state['skipped']:
            log.warning(f'not drawn: {reason}')
        state['skipped'] = set(skipped)
        publisher.publish(to_marker_array(specs, now.to_msg()))

    def on_targets(msg):
        state['targets'] = list(msg.targets)
        log.info(f'targets {len(msg.targets)}: ' + ', '.join(f'{t.target_id[:8]} {stage(t)}' for t in msg.targets))
        redraw()

    def on_location(store, exit_topic):
        def handle(msg):
            # as the coordinator: exit_qr only on the exit topic, never on the doll topic
            if (msg.observation.class_label == EXIT_CLASS_LABEL) != exit_topic:
                log.warning(f'{msg.location_id}: class {msg.observation.class_label!r} on the wrong topic; not drawn')
                return
            keep_latest(store, msg)
            redraw()
        return handle

    node.create_subscription(TargetSnapshot, TARGETS_TOPIC, on_targets, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TargetLocation, TARGET_LOCATIONS_TOPIC, on_location(state['candidates'], False),
                             latched)
    node.create_subscription(TargetLocation, EXIT_LOCATION_TOPIC, on_location(state['exits'], True), latched)
    node.create_timer(args.period, redraw)
    log.info(f'{TARGETS_TOPIC}, {TARGET_LOCATIONS_TOPIC}, {EXIT_LOCATION_TOPIC} -> {MARKERS_TOPIC} '
             f'every {args.period}s')
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
