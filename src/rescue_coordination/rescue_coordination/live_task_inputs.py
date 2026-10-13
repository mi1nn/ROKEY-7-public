"""Online clock/destination reads from the existing coordinator process.

Standard rcl_interfaces/GetParameters transport; no new node or custom IDL.
The `source` STRING parameter is a generated in-memory record, not an input
JSON file. ExecuteTask, CommandLease and SystemState stay unchanged.
"""
from copy import deepcopy
from dataclasses import replace
import json
import math

from rescue_contracts.clock_measurement import ClockSampler


class StatePublication:
    """Publish changed state, or explicitly resend the latest unchanged snapshot.

    Resending updates only the actual publication envelope. changed_at_ns and
    state_sequence retain their original meanings. No mission/lease tick runs.
    """
    def __init__(self, core, message_type, stamp_envelope, publish):
        self.core = core
        self.message_type = message_type
        self.stamp_envelope = stamp_envelope
        self.publish = publish
        self.last_fields = None

    def send(self, now_ns, *, refresh=False):
        fields = self.core.system_state_change(now_ns)
        if fields is None:
            if not refresh or self.last_fields is None:
                return None
            fields = deepcopy(self.last_fields)
        self.publish(self.stamp_envelope(self.message_type(**fields)))
        self.last_fields = deepcopy(fields)
        return fields


class SelectedSearchGoal:
    """Operator goal selection before START_SEARCH, held only in memory.

    The configured first area's identity/map basis is retained. A click supplies
    only a planar goal in the configured mission frame; it cannot assert map
    alignment, readiness, authority or lease validity. No task is sent here.
    """
    def __init__(self, core):
        self.core = core
        policy = core.search.policy
        if not policy.areas or not policy.executable(policy.areas[0]):
            raise ValueError('RViz selection requires a configured executable first-area map basis')
        self.template = policy.areas[0]
        self.selected = False
        # An enabled selector never exposes the example profile coordinates.
        core.search.policy = replace(policy, areas=())

    def select(self, message):
        core, mission = self.core, self.core.mission
        allocator = core.search.allocator
        if (mission.search_authorized or mission.search_started or allocator.pending
                or allocator.active or core.outbox or core.leases):
            raise ValueError('goal selection locked after START_SEARCH or while a task is reserved')
        policy = core.search.policy
        if message.header.frame_id != policy.mission_frame_id:
            raise ValueError('selected goal frame must equal configured mission frame')
        p, q = message.pose.position, message.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            raise ValueError('selected goal contains non-finite coordinates')
        scale = max(abs(q.z), abs(q.w))
        if p.z != 0 or q.x != 0 or q.y != 0 or scale == 0:
            raise ValueError('selected goal must be a planar pose with nonzero orientation')
        z, w = q.z / scale, q.w / scale
        yaw = math.atan2(2 * z * w, w * w - z * z)
        area = replace(self.template, mission_xy=(p.x, p.y), entry_yaw=yaw)
        core.search.policy = replace(policy, areas=(area,))
        self.selected = True
        return area


def attach(node, core, values, boot_id):
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rcl_interfaces.msg import ParameterDescriptor
    from rcl_interfaces.srv import GetParameters
    from rescue_coordination.ros_node import declare_required

    node.declare_parameter('live_inputs.enabled', False, descriptor=ParameterDescriptor(
        type=ParameterType.PARAMETER_BOOL,read_only=True))
    if not node.get_parameter('live_inputs.enabled').value:
        return None
    names = {'live_inputs.clock_domain_id': ParameterType.PARAMETER_STRING,
             'live_inputs.clock_backend': ParameterType.PARAMETER_STRING,
             'live_inputs.clock_max_age_ns': ParameterType.PARAMETER_INTEGER,
             'live_inputs.clock_drift_ppm': ParameterType.PARAMETER_DOUBLE,
             'live_inputs.poll_period_s': ParameterType.PARAMETER_DOUBLE}
    settings, missing = declare_required(node, names)
    if missing:
        raise ValueError('missing live coordinator settings: ' + str(missing))
    period = settings['live_inputs.poll_period_s']
    if not 0 < period * 1e9 < settings['live_inputs.clock_max_age_ns']:
        raise ValueError('clock read period must be positive and shorter than clock age budget')
    sampler = ClockSampler(settings['live_inputs.clock_backend'],
        settings['live_inputs.clock_max_age_ns'], settings['live_inputs.clock_drift_ppm'])
    last_reason = [None]
    node.declare_parameter('live_inputs.search_goal_topic', '', descriptor=ParameterDescriptor(
        type=ParameterType.PARAMETER_STRING, read_only=True))
    goal_topic = node.get_parameter('live_inputs.search_goal_topic').value
    selector = None
    if goal_topic:
        if not isinstance(goal_topic, str) or not goal_topic.startswith('/'):
            sampler.close()
            raise ValueError('explicit absolute search goal selection topic required')
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        try:
            selector = SelectedSearchGoal(core)
            def receive_goal(message):
                try:
                    area = selector.select(message)
                    node.get_logger().info('SEARCH_GOAL_SELECTED area=' + area.site_id
                        + ' frame=' + core.search.policy.mission_frame_id
                        + ' xy_yaw=' + str([*area.mission_xy, area.entry_yaw])
                        + '; awaiting explicit START_SEARCH')
                except (ValueError, TypeError, AttributeError) as exc:
                    node.get_logger().warning('SEARCH_GOAL_REJECTED: ' + str(exc))
            node.create_subscription(PoseStamped, goal_topic, receive_goal,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE))
        except Exception:
            sampler.close()
            raise
        node.get_logger().info('awaiting RViz search goal on ' + goal_topic
            + '; profile example coordinates excluded; selection alone sends no task')

    def poll():
        record = sampler.poll()
        reason = record.get('reason', record['state'])
        if reason != last_reason[0]:
            node.get_logger().info('central live clock: ' + reason)
            last_reason[0] = reason

    def get(request, response):
        policy = core.search.policy
        scout = core.roles['SCOUT']
        destinations = {area.site_id: dict(robot_id=scout, frame_id=policy.mission_frame_id,
            xy_yaw=[*area.mission_xy, area.entry_yaw], map_session_id=area.map_session_id,
            map_revision=area.map_revision, transform_revision=area.transform_revision)
            for area in policy.areas if policy.executable(area)}
        record = dict(format='rescue-live-task-source/1', coordinator_id=values['coordinator_id'],
            publisher_boot_id=boot_id, authority_epoch=values['authority_epoch'],
            schema_version=values['envelope_schema_version'], clock_domain_id=settings['live_inputs.clock_domain_id'],
            clock=sampler.record, lease_duration_s=values['lease_duration_s'],
            lease_renew_period_s=values['lease_renew_period_s'], destinations=destinations)
        response.values = [ParameterValue(type=ParameterType.PARAMETER_STRING,
            string_value=json.dumps(record, sort_keys=True, allow_nan=False)) if name == 'source'
            else ParameterValue(type=ParameterType.PARAMETER_NOT_SET) for name in request.names]
        return response

    service = node.create_service(GetParameters, '~/task_inputs', get)
    timer = node.create_timer(period, poll)
    poll()
    node.get_logger().info('live task source enabled: ~/task_inputs; no input files')
    return sampler, service, timer, selector
