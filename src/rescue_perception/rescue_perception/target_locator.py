"""ROS boundary for target_location: TargetObservation -> capture-time TF -> TargetLocation (B2).

Subscribes the confirmed observations amr_target publishes, looks up map_frame <- sensor_frame
at the observation's RGB stamp (never the latest TF), and publishes a VALID TargetLocation to
the coordinator. A TF that does not arrive within tf_wait_s, or any refused input, is logged
and nothing is published. The robot's namespaced TF is given by remapping /tf and /tf_static.
"""
import json
import math
import sys
import time
from uuid import uuid4

from .camera_utils import git_sha
from .target_location import LocationBasis, LocationRefused, fill_location, locate
from .target_utils import SETTINGS as TARGET_SETTINGS, fill_envelope

TARGET_LOCATIONS_TOPIC = '/rescue/target_locations'  # the coordinator's subscription (rescue_coordination)

PARAMS = {  # name -> rcl_interfaces ParameterType name; every one is required, none has a default
    'robot_id': 'STRING', 'observations_topic': 'STRING', 'target_locations_topic': 'STRING',
    # Map basis: the same values as the coordinator's goal_map_session_id/goal_map_revision/
    # goal_transform_revision and mission_frame_id, or it refuses the route (ROUTE_BASIS_MISMATCH).
    'map_frame': 'STRING', 'mission_frame': 'STRING', 'map_session_id': 'STRING',
    'map_revision': 'INTEGER', 'transform_revision': 'INTEGER',
    'alignment_xy_yaw': 'DOUBLE_ARRAY',  # mission <- map, [x, y, yaw]; identity is [0.0, 0.0, 0.0]
    'alignment_position_error_m': 'DOUBLE', 'alignment_rotation_error_rad': 'DOUBLE',
    'tf_wait_s': 'DOUBLE',  # how long to wait for the TF at the capture time to arrive
    # C-03 (OPS-103): observations must carry this measured clock domain, SYNCED with an error bound.
    'clock_domain_id': 'STRING',
    # C-04 (OPS-103): who declared the alignment (profile map_basis + the robot's saved-map identity,
    # #132/#145) and the window it holds for. Outside it nothing is VALID.
    'alignment_authority_id': 'STRING', 'alignment_valid_from_ns': 'INTEGER', 'alignment_valid_until_ns': 'INTEGER',
    'publisher_id': 'STRING', 'envelope_schema_version': 'STRING',
}


def profile_parameters(deployment, map_basis, *, tf_wait_s):
    """Every PARAMS value from the deployment profile (map_basis, the SCOUT robot) and the TF wait.

    The profile has no map -> mission transform, so the identity is used only when the SCOUT's
    map frame is the map_basis frame; otherwise this refuses rather than assume an alignment.
    """
    if type(tf_wait_s) is not float or not math.isfinite(tf_wait_s) or tf_wait_s <= 0:
        raise ValueError('tf_wait_s must be finite and positive')
    scouts = [robot_id for robot_id, role in deployment.mission.roles if role == 'SCOUT']
    if len(scouts) != 1:
        raise ValueError(f'exactly one SCOUT robot required, got {scouts}')
    readiness = {robot.id: robot for robot in deployment.robots}[scouts[0]].parameters.readiness
    if readiness is None:
        raise ValueError(f'profile robot {scouts[0]} has no readiness.map_frame')
    if readiness.map_frame != map_basis.frame_id:
        raise ValueError(f'map frame {readiness.map_frame!r} is not the map_basis frame {map_basis.frame_id!r} '
                         'and the profile has no map -> mission alignment')
    values = dict(robot_id=scouts[0], observations_topic=TARGET_SETTINGS['observations_topic'],
                  target_locations_topic=TARGET_LOCATIONS_TOPIC, map_frame=readiness.map_frame,
                  mission_frame=map_basis.frame_id, map_session_id=map_basis.map_session_id,
                  map_revision=map_basis.map_revision, transform_revision=map_basis.transform_revision,
                  alignment_xy_yaw=[0.0, 0.0, 0.0], alignment_position_error_m=0.0,
                  alignment_rotation_error_rad=0.0, tf_wait_s=tf_wait_s, publisher_id='amr.target_locator',
                  envelope_schema_version=TARGET_SETTINGS['envelope_version'])
    LocationBasis.from_values(values)
    return values


def main(args=None):
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rescue_interfaces.msg import TargetLocation, TargetObservation
    from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

    rclpy.init(args=args)
    node = Node('target_locator')
    log = node.get_logger()
    try:
        values = {}
        for name, kind in PARAMS.items():
            node.declare_parameter(name, Parameter.Type[kind], ParameterDescriptor(description='required; no default'))
            try:
                value = node.get_parameter(name).value
                values[name] = list(value) if kind.endswith('_ARRAY') else value
            except ParameterUninitializedException:
                values[name] = None
        missing = sorted(name for name, value in values.items() if value is None or value == '')
        if missing:
            log.fatal(f'missing parameters: {missing}')
            return 2
        try:
            basis = LocationBasis.from_values(values)
            if not values['tf_wait_s'] > 0:
                raise ValueError('tf_wait_s must be positive')
        except ValueError as exc:
            log.fatal(f'invalid parameters: {exc}')
            return 2
        boot_id = str(uuid4())
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(TargetLocation, values['target_locations_topic'], qos)
        buffer = Buffer()
        node.tf_listener = TransformListener(buffer, node)
        pending = {}  # observation_id -> (TargetObservation, monotonic deadline)
        handled = set()
        state = {'sequence': 0}

        def on_observation(msg):
            if msg.observation_id in handled or msg.observation_id in pending:
                return
            pending[msg.observation_id] = (msg, time.monotonic() + values['tf_wait_s'])
            try_pending()

        def try_pending():
            for observation_id, (msg, deadline) in list(pending.items()):
                try:
                    tf = buffer.lookup_transform(basis.map_frame, msg.sensor_frame,
                                                 Time(nanoseconds=msg.time.observed_at_ns))
                except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                    if time.monotonic() >= deadline:
                        del pending[observation_id]
                        handled.add(observation_id)
                        log.error(f'observation {observation_id}: no TF {basis.map_frame}<-{msg.sensor_frame} at '
                                  f'capture {msg.time.observed_at_ns} within {values["tf_wait_s"]}s; not published: '
                                  f'{type(exc).__name__}: {exc}')
                    continue
                del pending[observation_id]
                handled.add(observation_id)
                try:
                    located = locate(msg, tf, basis, now_ns=time.time_ns())
                except LocationRefused as exc:
                    log.warning(f'observation {observation_id} refused, not published: {exc}')
                    continue
                location = fill_location(TargetLocation(), msg, located, basis)
                state['sequence'] += 1
                fill_envelope(location.envelope, version=values['envelope_schema_version'],
                              publisher_id=values['publisher_id'], boot_id=boot_id, sequence=state['sequence'],
                              now_ns=node.get_clock().now().nanoseconds)
                publisher.publish(location)
                p = location.mission_position
                log.info(f'TargetLocation {location.location_id} VALID observation={observation_id} '
                         f'mission=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) frame={basis.mission_frame}')

        node.create_subscription(TargetObservation, values['observations_topic'], on_observation, qos)
        node.create_timer(0.1, try_pending)
        log.info(f'target_locator git={git_sha()} boot={boot_id} parameters={json.dumps(values, sort_keys=True)}')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
