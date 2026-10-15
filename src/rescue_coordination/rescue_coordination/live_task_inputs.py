"""Online clock/destination reads from the existing coordinator process.

Standard rcl_interfaces/GetParameters transport; no new node or custom IDL.
The `source` STRING parameter is a generated in-memory record, not an input
JSON file. ExecuteTask, CommandLease and SystemState stay unchanged.
"""
from copy import deepcopy
from dataclasses import replace
import json
import math
import time

from rescue_contracts.clock_measurement import ClockSampler


class StatePublication:
    """Publish changed state, or resend the latest unchanged snapshot: explicitly (``refresh``) or once
    ``resend_after_ns`` passed since the last publication.

    Resending updates only the actual publication envelope. changed_at_ns and
    state_sequence retain their original meanings. No mission/lease tick runs.
    The periodic resend keeps the envelope fresh for an executor started later
    (it refuses a SystemState older than its lease max age, 현장 2026-10-15).
    """
    def __init__(self, core, message_type, stamp_envelope, publish):
        self.core = core
        self.message_type = message_type
        self.stamp_envelope = stamp_envelope
        self.publish = publish
        self.last_fields = None
        self.last_sent_ns = None
        self.resent = False  # the last publication was an unchanged resend

    def send(self, now_ns, *, refresh=False, resend_after_ns=None):
        fields = self.core.system_state_change(now_ns)
        resent = fields is None
        if resent:
            due = (resend_after_ns is not None and self.last_sent_ns is not None
                   and now_ns - self.last_sent_ns >= resend_after_ns)
            if not (refresh or due) or self.last_fields is None:
                return None
            fields = deepcopy(self.last_fields)
        self.publish(self.stamp_envelope(self.message_type(**fields)))
        self.last_fields = deepcopy(fields)
        self.last_sent_ns, self.resent = now_ns, resent
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
        # OPERATOR: START_SEARCH needs the pick first, so an accepted start locks it. AUTO: the disaster sign
        # authorizes without a pick, so the pick stays open until the first area is reserved (OPS-136).
        started = mission.search_authorized if core.search_start == 'OPERATOR' else mission.search_started
        if (started or mission.search_started or allocator.pending
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


def destinations(core):
    """Destinations record of ~/task_inputs: the executable search areas (SCOUT) and the RETURN_HOME destinations
    the coordinator computes (``home/<robot_id>``: TB-5 exit approach, TB-6 docking approach; SLAM_FREEZE after
    the freeze). Each robot takes its own; the robot accepts only its own 'home/<id>' key besides UUIDs (#232)."""
    policy = core.search.policy
    scout = core.roles['SCOUT']
    out = {area.site_id: dict(robot_id=scout, frame_id=policy.mission_frame_id,
           xy_yaw=[*area.mission_xy, area.entry_yaw], map_session_id=area.map_session_id,
           map_revision=area.map_revision, transform_revision=area.transform_revision)
           for area in policy.areas if policy.executable(area)}
    return out | core.live_destinations()


def goal_pick_topic(core, topic):
    """The RViz search goal topic when this run uses the pick (AREAS: it replaces the configured areas), else None.
    FRONTIER and PATROL make their own areas; the pick stays off even if the field gives a topic (OPS-174)."""
    return topic if topic and core.search_mode == 'AREAS' else None


def clock_log_line(record, now_ns):
    """The start log line for a clock reason; a known next timesyncd poll adds the seconds left (C13, 2026-10-16)."""
    line = 'central live clock: ' + record.get('reason', record['state'])
    if record.get('next_poll_ns') is not None:
        line += f' (next poll in <= {max(0, math.ceil((record["next_poll_ns"] - now_ns) / 10**9))} s)'
    return line


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
    given_topic = node.get_parameter('live_inputs.search_goal_topic').value
    goal_topic = goal_pick_topic(core, given_topic)
    if given_topic and goal_topic is None:
        node.get_logger().info(f'RViz search goal pick off: search_mode={core.search_mode} makes its own areas '
                               f'({given_topic} not subscribed)')
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
            node.get_logger().info(clock_log_line(record, time.time_ns()))
            last_reason[0] = reason

    def get(request, response):
        record = dict(format='rescue-live-task-source/1', coordinator_id=values['coordinator_id'],
            publisher_boot_id=boot_id, authority_epoch=values['authority_epoch'],
            schema_version=values['envelope_schema_version'], clock_domain_id=settings['live_inputs.clock_domain_id'],
            clock=sampler.record, lease_duration_s=values['lease_duration_s'],
            lease_renew_period_s=values['lease_renew_period_s'], destinations=destinations(core))
        response.values = [ParameterValue(type=ParameterType.PARAMETER_STRING,
            string_value=json.dumps(record, sort_keys=True, allow_nan=False)) if name == 'source'
            else ParameterValue(type=ParameterType.PARAMETER_NOT_SET) for name in request.names]
        return response

    service = node.create_service(GetParameters, '~/task_inputs', get)
    timer = node.create_timer(period, poll)
    poll()
    node.get_logger().info('live task source enabled: ~/task_inputs; no input files')
    return sampler, service, timer, selector
