#!/usr/bin/env python3
"""Native ROS software check for installed SEARCH_AREA -> Nav2Port -> lease -> stop.

Run with ROS Jazzy and the current overlay sourced; no rebuild is needed.
All endpoints are in a unique namespace in a process-local LOCALHOST domain.
Authority, clock, map, readiness, Nav2 server and odometry are TEST_ONLY data.
There is no cmd_vel publisher, physical robot, Nav2 planner or hardware test.
"""
import argparse
from copy import deepcopy
import inspect
import json
import math
import os
import sys
import time
from types import SimpleNamespace
from uuid import uuid4


TARGET_DELIVERY = '33b0be0'
LEASE_SECONDS = 2.0
RENEW_SECONDS = 0.4


def make_fixture(now_ns, prefix):
    """Build explicit synthetic records; this helper needs no ROS imports."""
    ids = {name: str(uuid4()) for name in ('robot_boot_id', 'authority_epoch',
        'map_session_id', 'command_id', 'task_id', 'lease_id', 'search_area_ref')}
    ids['robot_id'] = 'TEST_ONLY_robot'
    ids['native_frame'] = prefix.strip('/') + '/map'
    until = now_ns + 120_000_000_000
    config = dict(format='rescue-robot-runtime/1', robot_id=ids['robot_id'],
        mission_frame='TEST_ONLY_mission', schema_version='TEST_ONLY',
        publisher_id='TEST_ONLY_executor', clock_error_bound_ns=100_000_000,
        lease=dict(duration_s=LEASE_SECONDS, max_age_ns=1_000_000_000,
                   max_error_ns=100_000_000, clock_domain_id='TEST_ONLY_clock'),
        mapping_authority='TEST_ONLY_mapping', max_position_error_m=.2,
        max_rotation_error_rad=.1, input_max_age_ns=120_000_000_000,
        poll_period_s=.05, input_file='/TEST_ONLY/injected-no-file-used.json',
        endpoints=dict(action_name=prefix+'/execute_task', lease_topic=prefix+'/lease',
            admission_topic=prefix+'/admission', status_topic=prefix+'/status'))
    data = dict(format='rescue-robot-inputs/1', generated_at_ns=now_ns, valid_until_ns=until,
        clock=dict(domain_id='TEST_ONLY_clock', state='SYNCED', error_bound_ns=100_000_000,
                   epoch_id='TEST_ONLY_clock_session', valid_until_ns=until),
        authority=dict(coordinator_id='TEST_ONLY_central', authority_epoch=ids['authority_epoch'],
            robot_boot_id=ids['robot_boot_id'], publisher_id='TEST_ONLY_central',
            publisher_boot_id='TEST_ONLY_central_boot'),
        maps={ids['robot_id']: dict(map_frame=ids['native_frame'], map_session_id=ids['map_session_id'],
            map_revision=0, map_update_sequence=0, observed_at_ns=now_ns, valid_until_ns=until,
            localization_status='VALID', transform_revision=1, authority_id='TEST_ONLY_mapping',
            clock_domain_id='TEST_ONLY_clock', valid_from_ns=now_ns-1_000_000_000,
            transform_valid_until_ns=until, transform_xy_yaw=[2., 0., 0.],
            position_error_m=.01, rotation_error_rad=.01, evidence_id='TEST_ONLY_calibration')},
        destinations={ids['search_area_ref']: dict(robot_id=ids['robot_id'],
            map_session_id=ids['map_session_id'], map_revision=0, transform_revision=1,
            xy_yaw=[4., 2., math.pi/2], observed_at_ns=now_ns, valid_until_ns=until)})
    return config, data, ids


def isolated_environment(domain_id):
    if not 0 <= domain_id <= 232:
        raise ValueError('--domain-id must be in 0..232')
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'ROS_STATIC_PEERS',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'RMW_FASTRTPS_USE_QOS_FROM_XML', 'ROS_LOCALHOST_ONLY'):
        os.environ.pop(key, None)
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'


def log(stage, message):
    print(f'[{stage}] {message}', flush=True)


def run(domain_id, timeout):
    isolated_environment(domain_id)  # Must precede every rclpy import/init.
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.task import Future
    from rescue_interfaces.action import ExecuteTask
    from rescue_interfaces.msg import CommandLease, RobotStatus, TaskAdmission
    from rescue_robot.executor import Executor, Policy
    from rescue_robot.nav2_adapter import CommandAdapter
    from rescue_robot.ros_node import Nav2Port, bind_odometry, git_sha
    from rescue_robot.task_input_node import SearchInputStore, SearchRuntime

    prefix = '/test_search_nav2_' + uuid4().hex
    config, data, ids = make_fixture(time.time_ns(), prefix)
    state = dict(moving=False, closing=False, odom_received=0, nav_received=[],
                 admissions=[], statuses=[], last_odom_steady=0.)
    executor = None
    robot = peer = runtime = port = client = nav_server = nav_finish = None
    adapter = None
    deadline = time.monotonic() + timeout
    rclpy.init(args=[])
    try:
        robot = Node('search_input_probe', namespace=prefix)
        peer = Node('test_only_peer', namespace=prefix)
        executor = SingleThreadedExecutor()
        executor.add_node(robot)
        executor.add_node(peer)
        nav_finish = Future(executor=executor)
        group = ReentrantCallbackGroup()

        async def navigate(handle):
            state['nav_received'].append(handle.request)
            await nav_finish
            if handle.is_cancel_requested:
                handle.canceled()
            elif state['closing']:
                handle.abort()
            else:
                handle.succeed()
            return NavigateToPose.Result()

        nav_action = prefix + '/navigate_to_pose'
        nav_server = ActionServer(peer, NavigateToPose, nav_action, execute_callback=navigate,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT, callback_group=group)
        port = Nav2Port(robot, nav_action)
        core = Executor(ids['robot_id'], ids['robot_boot_id'], ids['authority_epoch'],
            Policy('TEST_ONLY', 2., 3., .3, .25, 1., .01, .01),
            allowed_robot_ids=(ids['robot_id'], 'TEST_ONLY_other'))
        adapter = CommandAdapter(core, port, guards=lambda _task: False,
                                 clock_error_bound_s=.1)
        port.bind(adapter)
        inputs = SearchInputStore(config, read=lambda: deepcopy(data))
        runtime = SearchRuntime(robot, adapter, config, inputs)

        def arrived(_source, steady):
            state['odom_received'] += 1
            state['last_odom_steady'] = steady
        bind_odometry(robot, adapter, prefix+'/odom', monitor=SimpleNamespace(arrived=arrived))
        odom_pub = peer.create_publisher(Odometry, prefix+'/odom', qos_profile_sensor_data)

        def odometry():
            msg = Odometry()
            msg.header.stamp = peer.get_clock().now().to_msg()
            msg.header.frame_id = prefix.strip('/') + '/odom'
            msg.child_frame_id = prefix.strip('/') + '/base_link'
            msg.pose.pose.orientation.w = 1.
            msg.twist.twist.linear.x = .2 if state['moving'] else 0.
            odom_pub.publish(msg)
        peer.create_timer(.05, odometry)

        def step():
            # Sensor/watchdog flags are deliberately injected TEST_ONLY readiness.
            adapter.observe_status(driver=state['odom_received'] > 0, sensors=True,
                                   slam=True, nav2=port.server_ready(), watchdog=True)
            adapter.on_dock_status(False)
            adapter.step()
        robot.create_timer(.05, step)
        lease_pub = peer.create_publisher(CommandLease, config['endpoints']['lease_topic'], 10)
        peer.create_subscription(TaskAdmission, config['endpoints']['admission_topic'],
                                 state['admissions'].append, 10)
        peer.create_subscription(RobotStatus, config['endpoints']['status_topic'],
                                 state['statuses'].append, 10)
        client = ActionClient(peer, ExecuteTask, config['endpoints']['action_name'], callback_group=group)

        def wait(predicate, stage, stage_timeout=8.):
            end = min(deadline, time.monotonic()+stage_timeout)
            while time.monotonic() < end:
                if predicate():
                    return
                if not rclpy.ok():
                    raise RuntimeError(stage + ': ROS context stopped')
                executor.spin_once(timeout_sec=.02)
            raise TimeoutError(stage + ': timeout; executor_state=' + core.state +
                ', reason=' + str(core.stop_cause) + ', lease_sequence=' + str(core.lease_sequence))

        log('ISOLATED', f'domain={domain_id} LOCALHOST namespace={prefix}')
        log('LOAD', f'target_delivery={TARGET_DELIVERY} runtime={inspect.getfile(SearchRuntime)} '
                    f'git_sha={git_sha()}')
        log('TEST_ONLY', 'native ROS transport; fake Nav2 server/odom/readiness/calibration; no hardware')
        wait(lambda: client.server_is_ready() and port.server_ready() and
             lease_pub.get_subscription_count() > 0 and odom_pub.get_subscription_count() > 0 and
             runtime.server.publisher.get_subscription_count() > 0 and state['statuses'] and
             core.motion == 'STOPPED' and adapter.readiness(time.monotonic()).valid(time.monotonic(), core.policy),
             'discovery and initial stop')
        if not runtime.arm():
            raise RuntimeError('explicit arm rejected after readiness/stop observations')
        log('READY', f'explicit arm accepted; odom_received={state["odom_received"]}')

        now = time.time_ns()
        goal = ExecuteTask.Goal()
        goal.envelope.schema_version = 'TEST_ONLY'
        goal.envelope.publisher_id = 'TEST_ONLY_central'
        goal.envelope.publisher_boot_id = 'TEST_ONLY_central_boot'
        goal.envelope.published_at_ns = now
        for field in ('command_id', 'task_id', 'robot_id', 'authority_epoch', 'lease_id', 'search_area_ref'):
            setattr(goal, field, ids[field])
        goal.expected_robot_boot_id = ids['robot_boot_id']
        goal.coordinator_id = 'TEST_ONLY_central'
        goal.generation, goal.kind = 1, 'SEARCH_AREA'
        goal.issued_at_ns, goal.deadline_ns = now, now+60_000_000_000
        goal.policy_version = 'TEST_ONLY'
        goal.frame_ref.robot_id = ids['robot_id']
        goal.frame_ref.mission_frame = config['mission_frame']
        goal.frame_ref.map_session_id = ids['map_session_id']
        goal.frame_ref.map_revision, goal.frame_ref.transform_revision = 0, 1
        # SEARCH_AREA wire alignment remains empty; trusted live inputs resolve it.
        send = client.send_goal_async(goal)
        wait(send.done, 'ExecuteTask goal response')
        handle = send.result()
        if handle is None or not handle.accepted:
            reasons = [msg.reason_code for msg in state['admissions']]
            raise RuntimeError('SEARCH_AREA rejected: ' + repr(reasons))
        result_future = handle.get_result_async()
        with adapter.lock:
            initial_until = core.lease_until
        log('ADMITTED', f'command={goal.command_id} initial_local_lease={LEASE_SECONDS}s')
        wait(lambda: state['nav_received'] and core.state == 'NAVIGATING', 'NavigateToPose goal')
        if len(state['nav_received']) != 1:
            raise AssertionError('expected exactly one Nav2 goal')
        pose = state['nav_received'][0].pose
        q = pose.pose.orientation
        if (pose.header.frame_id != ids['native_frame'] or
                not math.isclose(pose.pose.position.x, 2., abs_tol=1e-8) or
                not math.isclose(pose.pose.position.y, 2., abs_tol=1e-8) or
                not math.isclose(q.z, math.sin(math.pi/4), abs_tol=1e-8) or
                not math.isclose(q.w, math.cos(math.pi/4), abs_tol=1e-8)):
            raise AssertionError('Nav2 goal differs from transformed [2, 2, pi/2]')
        state['moving'] = True
        log('NAV2_GOAL', f'frame={pose.header.frame_id} endpoint=[2.0, 2.0, pi/2] goals=1')

        sequence = 0
        while time.monotonic() <= initial_until + .25:
            lease = CommandLease()
            lease.envelope.schema_version = 'TEST_ONLY'
            lease.envelope.publisher_id = 'TEST_ONLY_central'
            lease.envelope.publisher_boot_id = 'TEST_ONLY_central_boot'
            lease.envelope.sequence = sequence
            lease.envelope.published_at_ns = time.time_ns()
            for field in ('lease_id', 'robot_id', 'robot_boot_id', 'authority_epoch'):
                setattr(lease, field, ids[field])
            lease.coordinator_id, lease.generation = goal.coordinator_id, 1
            lease.renewal_sequence = sequence
            lease.issued_at_ns = time.time_ns()
            lease.valid_until_ns = lease.issued_at_ns+1_000_000_000
            lease_pub.publish(lease)
            wait(lambda: core.lease_sequence == sequence, f'lease renewal {sequence}', 1.5)
            if result_future.done() or core.state != 'NAVIGATING':
                raise AssertionError('task stopped while renewable lease should keep it active')
            sequence += 1
            next_send = time.monotonic()+RENEW_SECONDS
            wait(lambda: time.monotonic() >= next_send, 'lease interval', RENEW_SECONDS+1.)
        with adapter.lock:
            renewed_until = core.lease_until
        if sequence < 3 or renewed_until <= initial_until or result_future.done():
            raise AssertionError('lease renewal beyond initial expiry was not demonstrated')
        log('LEASE', f'accepted={sequence} interval={RENEW_SECONDS}s '
                     f'active_beyond_initial_expiry={time.monotonic()-initial_until:.3f}s')

        # Keep moving odometry until the actual Nav2 result callback fences execution.
        nav_finish.set_result(None)
        wait(lambda: core.fenced and core.action_status == 'SUCCEEDED', 'Nav2 terminal fence', 2.)
        if result_future.done() or runtime.server.ingress.report(goal.command_id) is not None:
            raise AssertionError('terminal succeeded before fresh stopped odometry')
        state['moving'] = False
        log('STOP', 'Nav2 terminal observed; waiting for fresh zero odometry window')
        wait(result_future.done, 'ExecuteTask stopped result', 4.)
        response = result_future.result()
        result = response.result.result
        expected = ('COMPLETED', 'SEARCH', True, 'STOPPED')
        actual = (result.outcome, result.completion_scope, result.stop_confirmed, result.motion_state)
        if response.status != GoalStatus.STATUS_SUCCEEDED or actual != expected:
            raise AssertionError('unexpected ExecuteTask result: ' + repr((response.status, actual)))
        if (result.command_id, result.task_id, result.robot_id, result.generation) != (
                goal.command_id, goal.task_id, goal.robot_id, goal.generation):
            raise AssertionError('result identity differs from admitted SEARCH_AREA')
        if len(state['nav_received']) != 1:
            raise AssertionError('lease renewals caused duplicate Nav2 dispatch')
        wait(lambda: any(msg.command_id == goal.command_id and msg.accepted
                         for msg in state['admissions']) and
             any(msg.robot_id == goal.robot_id and msg.motion_state == 'STOPPED'
                 and msg.task_state == 'FINISHED' for msg in state['statuses']),
             'admission and stopped RobotStatus', 2.)
        return dict(status='PASSED', layer='E3_NATIVE_ROS_SOFTWARE', task='SEARCH_AREA',
            nav2_goal_count=1, lease_renewals=sequence, result=list(actual),
            fake_nav2_server=True, synthetic_readiness=True, synthetic_odometry=True,
            real_nav2_planner='NOT_RUN', hardware_motion='NOT_RUN',
            startup_parameter_validation='NOT_RUN', namespace=prefix, domain_id=domain_id)
    finally:
        cleanup_errors = []
        def cleanup(label, action):
            try:
                action()
            except Exception as exc:
                cleanup_errors.append(f'{label}: {type(exc).__name__}: {exc}')
        state['closing'] = True
        state['moving'] = False
        if adapter is not None and adapter.executor.active is not None and adapter.executor.state != 'FINISHED':
            cleanup('stop request', lambda: adapter.request_stop('CANCEL_USER'))
        if nav_finish is not None and not nav_finish.done():
            cleanup('fake Nav2 release', lambda: nav_finish.set_result(None))
        if executor is not None:
            def drain():
                cleanup_end = time.monotonic()+1.
                while time.monotonic() < cleanup_end and rclpy.ok():
                    executor.spin_once(timeout_sec=.02)
                    if adapter is None or adapter.executor.active is None or adapter.executor.state == 'FINISHED':
                        break
            cleanup('executor drain', drain)
            cleanup('executor shutdown', lambda: executor.shutdown(timeout_sec=2.))
        if runtime is not None:
            cleanup('runtime', runtime.destroy)
        if client is not None:
            cleanup('ExecuteTask client', client.destroy)
        if port is not None:
            cleanup('Nav2 client', port.client.destroy)
        if nav_server is not None:
            cleanup('fake Nav2 server', nav_server.destroy)
        for node in (peer, robot):
            if node is not None:
                cleanup('node', node.destroy_node)
        cleanup('ROS shutdown', rclpy.try_shutdown)
        if cleanup_errors:
            log('CLEANUP', '; '.join(cleanup_errors))
            if sys.exc_info()[0] is None:
                raise RuntimeError('probe cleanup failed: ' + '; '.join(cleanup_errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=173)
    parser.add_argument('--timeout', type=float, default=45.)
    args = parser.parse_args()
    try:
        if not math.isfinite(args.timeout) or not 10. <= args.timeout <= 120.:
            raise ValueError('--timeout must be finite and in 10..120 seconds')
        result = run(args.domain_id, args.timeout)
    except (Exception, KeyboardInterrupt) as exc:
        print(json.dumps(dict(status='FAILED', error=f'{type(exc).__name__}: {exc}',
                              hardware_motion='NOT_RUN'), ensure_ascii=False), flush=True)
        return 1
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
