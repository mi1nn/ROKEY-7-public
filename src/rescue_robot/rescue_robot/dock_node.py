"""TB-6 dock after its return (방식 B full scenario, user work order 2026-10-15): one Create3 dock per request.

The coordinator (completion_mode EXIT_WAIT_SCOUT_DOCK) calls ``/rescue/robots/<id>/dock`` (std_srvs/Trigger) once
TB-6 stands at its start pose after RETURN_HOME. This node, the only DOCK sender (AGENTS 6, contracts C-06), accepts
it only when the robot's own RobotStatus is fresh, READY, STOPPED and without a task and a fresh dock_status says
undocked. It then sends the Create3 ``<ns>/dock`` action (irobot_create_msgs/action/Dock, the action
turtlebot4_navigator uses) and latches its state on ``/rescue/robots/<id>/dock_state`` (std_msgs/String JSON):

- ``DOCKED`` only when the action result says ``is_docked`` true AND a dock_status received after that result says
  ``is_docked`` true (the robot's own sensor, not our wish); either missing within ``confirm_timeout_s`` = FAILED.
- ``FAILED`` with a reason: goal refused, no action server, result not docked, dock_status not confirming, or no
  result within ``result_timeout_s`` (the goal is then canceled). Never retried here.
- ``~/cancel`` (std_srvs/Trigger, also called by the full-scenario runner's safe stop) and node shutdown cancel a
  running dock goal.
Nothing else moves the robot. A dock without the hardware (no action server) is FAILED, never pretended.
"""
import json

STATES = ('IDLE', 'DOCKING', 'DOCKED', 'FAILED')


class DockCore:
    """Request gate and verdict of one dock at a time; no ROS. Times are wall-clock ns."""

    def __init__(self, *, status_max_age_ns, dock_status_max_age_ns, action_wait_ns, result_timeout_ns,
                 confirm_timeout_ns):
        for name, value in (('status_max_age_ns', status_max_age_ns), ('dock_status_max_age_ns', dock_status_max_age_ns),
                            ('action_wait_ns', action_wait_ns), ('result_timeout_ns', result_timeout_ns),
                            ('confirm_timeout_ns', confirm_timeout_ns)):
            if type(value) is not int or value <= 0:
                raise ValueError(f'dock {name} must be a positive integer')
        self.status_max_age_ns, self.dock_status_max_age_ns = status_max_age_ns, dock_status_max_age_ns
        self.action_wait_ns, self.result_timeout_ns = action_wait_ns, result_timeout_ns
        self.confirm_timeout_ns = confirm_timeout_ns
        self.state, self.reason, self.request = 'IDLE', '', 0
        self.status = None        # (readiness, motion_state, active_task_id, observed_at_ns)
        self.dock = None          # (is_docked, received_ns)
        self.started_ns = self.result_ns = None
        self.goal_sent = False
        self.result_docked = None

    # inputs
    def on_status(self, readiness, motion_state, active_task_id, observed_at_ns):
        self.status = (readiness, motion_state, active_task_id, int(observed_at_ns))

    def on_dock_status(self, is_docked, received_ns):
        self.dock = (bool(is_docked), int(received_ns))

    def refusal(self, now_ns):
        """Why a dock request is refused now, or None."""
        if self.state == 'DOCKING':
            return 'DOCK_BUSY'
        s = self.status
        if s is None or not 0 <= now_ns - s[3] <= self.status_max_age_ns:
            return 'ROBOT_STATUS_STALE'
        if s[0] != 'READY':
            return f'ROBOT_NOT_READY_{s[0]}'
        if s[1] != 'STOPPED':
            return f'ROBOT_NOT_STOPPED_{s[1]}'
        if s[2]:
            return 'ROBOT_TASK_ACTIVE'
        if self.dock is None or not 0 <= now_ns - self.dock[1] <= self.dock_status_max_age_ns:
            return 'DOCK_STATUS_STALE'
        if self.dock[0]:
            return 'ALREADY_DOCKED'
        return None

    def start(self, now_ns):
        """Accept a request (the caller checked ``refusal``): a new request number, DOCKING."""
        self.request += 1
        self.state, self.reason = 'DOCKING', ''
        self.started_ns, self.result_ns, self.goal_sent, self.result_docked = now_ns, None, False, None
        return self.request

    def on_goal_sent(self):
        self.goal_sent = True

    def on_goal_response(self, accepted):
        if self.state == 'DOCKING' and not accepted:
            self._fail('DOCK_GOAL_REJECTED')

    def on_result(self, is_docked, now_ns, *, succeeded=True):
        if self.state != 'DOCKING':
            return
        if not succeeded:
            self._fail('DOCK_ACTION_ABORTED')
            return
        self.result_docked, self.result_ns = bool(is_docked), now_ns
        if not is_docked:
            self._fail('DOCK_RESULT_NOT_DOCKED')

    def tick(self, now_ns, *, server_ready):
        """'send' (send the goal now), 'cancel' (cancel it: timed out) or None; also settles DOCKED/FAILED."""
        if self.state != 'DOCKING':
            return None
        if not self.goal_sent:
            if server_ready:
                return 'send'
            if now_ns - self.started_ns > self.action_wait_ns:
                self._fail('DOCK_ACTION_UNAVAILABLE')
            return None
        if self.result_ns is None:
            if now_ns - self.started_ns > self.result_timeout_ns:
                self._fail('DOCK_TIMEOUT')
                return 'cancel'
            return None
        # result says docked: wait for the robot's own dock_status after it
        if self.dock is not None and self.dock[1] >= self.result_ns and self.dock[0]:
            self.state, self.reason = 'DOCKED', ''
            return None
        if now_ns - self.result_ns > self.confirm_timeout_ns:
            self._fail('DOCK_STATUS_NOT_CONFIRMED')
        return None

    def cancel(self, reason):
        """Operator / runner cancel: a running dock ends FAILED with ``reason``; True when one was running."""
        if self.state != 'DOCKING':
            return False
        self._fail(reason)
        return True

    def _fail(self, reason):
        self.state, self.reason = 'FAILED', reason

    def record(self, now_ns):
        return dict(state=self.state, reason=self.reason, request=self.request,
                    is_docked=self.state == 'DOCKED' and bool(self.dock and self.dock[0]),
                    dock_status_docked=None if self.dock is None else self.dock[0], at_ns=now_ns)


def main(args=None):
    import time

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rcl_interfaces.msg import ParameterType as T
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from irobot_create_msgs.action import Dock
    from irobot_create_msgs.msg import DockStatus
    from rescue_interfaces.msg import RobotStatus
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rescue_robot.ros_node import declare_required, git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_dock')
    log = node.get_logger()
    S, D = T.PARAMETER_STRING, T.PARAMETER_DOUBLE
    values, missing = declare_required(node, {
        'robot_id': S, 'dock_action': S, 'dock_status_topic': S, 'robot_status_topic': S, 'service': S,
        'state_topic': S, 'status_max_age_s': D, 'dock_status_max_age_s': D, 'action_wait_s': D,
        'result_timeout_s': D, 'confirm_timeout_s': D})
    if missing:
        log.error('dock missing parameters: ' + ', '.join(missing))
        raise SystemExit(2)
    try:
        core = DockCore(**{name[:-2] + '_ns': round(values[name] * 1e9) for name in (
            'status_max_age_s', 'dock_status_max_age_s', 'action_wait_s', 'result_timeout_s', 'confirm_timeout_s')})
    except ValueError as exc:
        log.error(str(exc))
        raise SystemExit(2)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state_pub = node.create_publisher(String, values['state_topic'], latched)
    client = ActionClient(node, Dock, values['dock_action'])
    goal = {'handle': None}
    said = {'key': None}

    def publish():
        record = core.record(time.time_ns())
        key = (record['state'], record['reason'], record['request'], record['is_docked'])
        if key == said['key']:
            return
        said['key'] = key
        state_pub.publish(String(data=json.dumps(record, sort_keys=True)))
        # rclpy refuses two severities on one call site ('Logger severity cannot be changed between calls')
        if record['state'] == 'FAILED':
            log.error(f'DOCK_{record["state"]} {json.dumps(record)}')
        else:
            log.info(f'DOCK_{record["state"]} {json.dumps(record)}')

    def on_status(msg):
        if msg.robot_id == values['robot_id']:
            core.on_status(msg.readiness, msg.motion_state, msg.active_task_id, msg.observed_at_ns)

    def on_dock_status(msg):
        core.on_dock_status(msg.is_docked, time.time_ns())

    def request(_request, response):
        now = time.time_ns()
        reason = core.refusal(now)
        if reason is not None:
            response.success, response.message = False, reason
            log.warning(f'DOCK_REFUSED {reason}')
            return response
        number = core.start(now)
        response.success = True
        response.message = json.dumps(dict(request=number, robot_id=values['robot_id']))
        log.info(f'DOCK_ACCEPTED request={number}')
        publish()
        return response

    def cancel_goal(reason):
        handle = goal['handle']
        if handle is not None:
            handle.cancel_goal_async()
            log.warning(f'DOCK goal cancel sent ({reason})')

    def cancel(_request, response):
        running = core.cancel('DOCK_CANCELED')
        cancel_goal('cancel service')
        response.success, response.message = True, 'dock canceled' if running else 'no dock running'
        publish()
        return response

    def on_result(future, number):
        if number != core.request:
            return
        try:
            wrapped = future.result()
            core.on_result(bool(wrapped.result.is_docked), time.time_ns(), succeeded=wrapped.status == 4)
        except Exception as exc:   # an action error is a failed dock, shown
            log.error(f'DOCK result error: {exc}')
            core.on_result(False, time.time_ns(), succeeded=False)
        goal['handle'] = None
        publish()

    def on_goal_response(future, number):
        if number != core.request:
            return
        handle = future.result()
        core.on_goal_response(bool(handle.accepted))
        if handle.accepted:
            goal['handle'] = handle
            handle.get_result_async().add_done_callback(lambda f: on_result(f, number))
        publish()

    def tick():
        action = core.tick(time.time_ns(), server_ready=client.server_is_ready())
        if action == 'send':
            number = core.request
            core.on_goal_sent()
            log.info(f'DOCK goal -> {values["dock_action"]} request={number}')
            client.send_goal_async(Dock.Goal()).add_done_callback(lambda f: on_goal_response(f, number))
        elif action == 'cancel':
            cancel_goal('timeout')
        publish()

    node.create_subscription(RobotStatus, values['robot_status_topic'], on_status, 10)
    node.create_subscription(DockStatus, values['dock_status_topic'], on_dock_status, qos_profile_sensor_data)
    node.create_service(Trigger, values['service'], request)
    node.create_service(Trigger, '~/cancel', cancel)
    node.create_timer(0.2, tick)
    log.info(f'git_sha={git_sha()} parameters={values}; docks only on request, never moves otherwise')
    publish()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if core.cancel('DOCK_CANCELED_ON_SHUTDOWN'):
            cancel_goal('shutdown')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
