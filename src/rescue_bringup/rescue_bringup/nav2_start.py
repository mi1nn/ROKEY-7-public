"""Start a FROZEN_MAP_AMCL robot's Nav2 once its map frame exists (TB-5; simulation 2026-10-15 C8, field 17:30).

robot.launch.py runs before the first freeze (the executor's MapState tells the frozen map relay which grid
to copy), so TB-5 has no map frame yet. Nav2 started then gives up: the global costmap waits for
map -> base_link at most initial_transform_timeout (Nav2 default 60 s), the lifecycle manager aborts the
bringup and never tries again. So robot.launch.py starts that robot's Nav2 with autostart false, and this
node sends <ns>/lifecycle_manager_navigation/manage_nodes STARTUP once, after map -> base_link is on the
robot's TF (AMCL on the frozen map gives map -> odom).

One STARTUP only. A refused or unanswered STARTUP is logged (NAV2_STARTUP_FAILED / NAV2_STARTUP_NO_ANSWER)
with the command to retry by hand; the executor's readiness keeps saying NAV2_LIFECYCLE ... inactive and
refuses arm (#340). Nothing is retried here and the robot is never moved.
"""
STARTUP = 0   # nav2_msgs/srv/ManageLifecycleNodes.Request.STARTUP (main checks the message constant)
FINAL = ('ACTIVE', 'FAILED', 'NO_ANSWER')


def step(state, *, has_map, service_ready, answer, sent_for_s, answer_wait_s, service, map_frame, base_frame):
    """(next state, (level, line) or None, send STARTUP now). ``answer``: None (none yet), True or False."""
    if state in FINAL:
        return state, None, False
    if state == 'SENT':
        if answer is True:
            return 'ACTIVE', ('info', f'NAV2_STARTUP {service} active'), False
        retry = f'retry by hand: ros2 service call {service} nav2_msgs/srv/ManageLifecycleNodes "{{command: 0}}"'
        if answer is False:
            return 'FAILED', ('error', f'NAV2_STARTUP_FAILED {service} refused STARTUP: Nav2 stays inactive '
                                       f'(NAV2_LIFECYCLE inactive, arm refused); {retry}'), False
        if sent_for_s >= answer_wait_s:
            return 'NO_ANSWER', ('error', f'NAV2_STARTUP_NO_ANSWER {service} no answer within {answer_wait_s} s: '
                                          f'Nav2 may stay inactive (see NAV2_LIFECYCLE); {retry}'), False
        return 'SENT', None, False
    if not has_map:
        if state == 'WAITING_MAP':
            return state, None, False
        return 'WAITING_MAP', ('info', f'NAV2_STARTUP_WAITING {map_frame} -> {base_frame} not on TF yet '
                                       '(first frozen map not localized): Nav2 not started'), False
    if not service_ready:
        if state == 'WAITING_SERVICE':
            return state, None, False
        return 'WAITING_SERVICE', ('warning', f'NAV2_STARTUP_WAITING {service} not available'), False
    return 'SENT', ('info', f'NAV2_STARTUP_SENT {service}: {map_frame} -> {base_frame} is on TF'), True


def emit(log, said):
    """One severity per call site (rclpy raises otherwise)."""
    if said is None:
        return
    level, line = said
    if level == 'info':
        log.info(line)
    elif level == 'warning':
        log.warning(line)
    else:
        log.error(line)


def main(args=None):
    import time

    import rclpy
    from nav2_msgs.srv import ManageLifecycleNodes
    from rclpy.time import Time
    import tf2_ros

    assert ManageLifecycleNodes.Request.STARTUP == STARTUP
    rclpy.init(args=args)
    node = rclpy.create_node('rescue_nav2_start')
    log = node.get_logger()
    defaults = {'manage_service': '', 'map_frame': '', 'base_frame': '', 'poll_period_s': 0.0, 'answer_wait_s': 0.0}
    values = {name: node.declare_parameter(name, default).value for name, default in defaults.items()}
    missing = [name for name, value in values.items() if not value or (isinstance(value, float) and value <= 0)]
    if missing:
        log.fatal('nav2_start needs these parameters (text non-empty, times positive): ' + ', '.join(missing))
        raise SystemExit(2)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)   # /tf and /tf_static, remapped to <ns>/tf by the launch
    client = node.create_client(ManageLifecycleNodes, values['manage_service'])
    run = {'state': 'START', 'future': None, 'sent_at': None}

    def tick():
        future = run['future']
        answer = None
        if future is not None and future.done():
            answer = future.exception() is None and bool(future.result().success)
        state, said, send = step(
            run['state'], has_map=tf_buffer.can_transform(values['map_frame'], values['base_frame'], Time()),
            service_ready=client.service_is_ready(), answer=answer,
            sent_for_s=0.0 if run['sent_at'] is None else time.monotonic() - run['sent_at'],
            answer_wait_s=values['answer_wait_s'], service=values['manage_service'],
            map_frame=values['map_frame'], base_frame=values['base_frame'])
        run['state'] = state
        emit(log, said)
        if send:
            request = ManageLifecycleNodes.Request()
            request.command = STARTUP
            run['future'], run['sent_at'] = client.call_async(request), time.monotonic()

    log.info(f'nav2_start parameters={values}')
    node.create_timer(values['poll_period_s'], tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
