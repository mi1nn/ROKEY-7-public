"""Local ROS action/lease/admission smoke probe. Every motion input is FAKE.

No Nav2 client, cmd_vel publisher, TF publisher or device subscription exists.
The only task sent is HOLD. Passing is an E3 transport check, never a robot
stop, watchdog, calibration, target-approval or navigation acceptance test.
"""
import argparse
from copy import deepcopy
import json
import os
import time
from uuid import uuid4

from rescue_robot.executor import ContractError, Executor, Policy
from rescue_robot.nav2_adapter import CommandAdapter
from rescue_robot.task_ingress import TaskIngress
from rescue_robot.task_server import TaskActionServer
from rescue_robot.task_policy import ClockBasis, LeasePolicy


def probe_namespace():
    """Unique local test namespace whose tokens never start with a digit."""
    return '/rokey_local_probe/p_' + uuid4().hex


def check_environment(domain_id):
    if type(domain_id) is not int or not 0 <= domain_id <= 232:
        raise ContractError('probe domain must be explicitly selected in 0..232')
    if os.environ.get('ROS_DOMAIN_ID') != str(domain_id):
        raise ContractError('ROS_DOMAIN_ID must match --domain-id')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise ContractError('probe requires ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise ContractError('unset ROS_DISCOVERY_SERVER for this probe')
    if os.environ.get('ROS_SUPER_CLIENT', '').lower() not in ('', 'false', '0'):
        raise ContractError('unset ROS_SUPER_CLIENT for this probe')


class NoMotionPort:
    def send_goal(self, *args):
        raise AssertionError('HOLD probe must never send a navigation goal')

    def cancel_goal(self, *args):
        raise AssertionError('HOLD probe must never cancel a navigation goal')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', required=True, type=int,
                        help='select an unused local test domain, not a robot domain')
    parser.add_argument('--timeout', type=float, default=15.0, help='probe timeout, test-only seconds')
    options = parser.parse_args(args)
    check_environment(options.domain_id)
    if not 0 < options.timeout < 120:
        parser.error('--timeout must be between 0 and 120 seconds')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rescue_interfaces.action import ExecuteTask
    from rescue_interfaces.msg import CommandLease, TaskAdmission

    rclpy.init(args=[])
    node = Node('task_transport_probe_' + uuid4().hex[:8])
    robot, boot, epoch, coordinator = 'probe_robot', str(uuid4()), str(uuid4()), str(uuid4())
    prefix = probe_namespace()
    # All policy, readiness and odometry values below are synthetic test data.
    policy = Policy('probe-only', 1., 2., .2, .15, 1., .01, .01)
    executor = Executor(robot, boot, epoch, policy, allowed_robot_ids=(robot, 'probe_other'))
    adapter = CommandAdapter(executor, NoMotionPort(), guards=lambda task: task.kind == 'HOLD',
                             clock_error_bound_s=.1)
    leases, admissions = {}, []
    lease_id, command_id, task_id = str(uuid4()), str(uuid4()), str(uuid4())

    def authorize(goal, lease):
        return (goal.kind == 'HOLD' and goal.command_id == command_id and goal.task_id == task_id
                and goal.robot_id == robot and goal.expected_robot_boot_id == boot
                and goal.authority_epoch == epoch and goal.coordinator_id == coordinator
                and (lease is None or (lease.coordinator_id == coordinator and lease.lease_id == lease_id)))

    def accept_lease(lease):
        if (lease.lease_id, lease.robot_id, lease.robot_boot_id, lease.authority_epoch,
                lease.coordinator_id) != (lease_id, robot, boot, epoch, coordinator):
            return False
        leases[lease.lease_id] = lease
        return True

    def encode(report, goal):
        response = ExecuteTask.Result()
        result = response.result
        result.task_id, result.command_id = report.task_id, report.command_id
        result.robot_id, result.generation = report.robot_id, report.generation
        result.outcome, result.terminal_reason = report.result.outcome, report.result.reason
        result.original_cause, result.stop_confirmed = report.result.original_cause, report.result.stop_confirmed
        result.motion_state = 'STOPPED' if result.stop_confirmed else 'STOP_UNCONFIRMED'
        result.completion_scope = 'HOLD'
        result.final_pose.robot_id, result.final_pose.source_boot_id = robot, boot
        result.final_pose.mission_pose_status = 'UNKNOWN'
        result.final_pose.time.stamp_semantics, result.final_pose.time.clock_state = 'UNKNOWN', 'UNKNOWN'
        # This codec is probe-only: no sensor observation/evidence is asserted.
        bridge.stamp_envelope(result.final_pose.envelope)
        return response

    ingress = TaskIngress(adapter, authorize=authorize,
                          resolve_goal=lambda goal: None, clock_error_bound_ns=1_000_000,
                          lease_policy=LeasePolicy(2., 1_000_000_000, 10_000_000, 'probe-only'),
                          clock_basis=lambda: ClockBasis('probe-only', 'SYNCED', 1_000_000,
                                                         'probe-clock', time.time_ns() + 60_000_000_000))
    bridge = TaskActionServer(node, ingress, action_name=prefix + '/execute_task',
                              lease_topic=prefix + '/lease', admission_topic=prefix + '/admission',
                              status_topic=prefix + '/status',
                              poll_period_s=.05, schema_version='probe-only', publisher_id='probe-executor',
                              lookup_lease=lambda goal: leases.get(goal.lease_id),
                              accept_lease=accept_lease, encode_result=encode)
    node.create_subscription(TaskAdmission, prefix + '/admission', admissions.append, 10)
    publisher = node.create_publisher(CommandLease, prefix + '/lease', 10)
    client = ActionClient(node, ExecuteTask, prefix + '/execute_task')

    def simulated_observation():
        adapter.observe_status(driver=True, sensors=True, slam=True, nav2=True,
                               undocked=True, watchdog=True)
        sec, ns = divmod(time.time_ns(), 10**9)
        adapter.on_odom(stamp_sec=sec, stamp_nanosec=ns, linear=(0., 0., 0.), angular=(0., 0., 0.))
        adapter.step()
    node.create_timer(.05, simulated_observation)
    limit = time.monotonic() + options.timeout

    def wait(predicate):
        while not predicate():
            if time.monotonic() >= limit:
                raise RuntimeError('probe timeout; inspect IDL install, RMW and action discovery')
            rclpy.spin_once(node, timeout_sec=.05)

    try:
        wait(lambda: executor.motion == 'STOPPED' and client.server_is_ready()
             and publisher.get_subscription_count() > 0 and bridge.publisher.get_subscription_count() > 0)
        if not executor.arm(epoch, now=adapter.clock(), readiness=adapter.readiness(adapter.clock())):
            raise RuntimeError('synthetic stop/readiness failed to arm probe executor')
        now = time.time_ns()
        lease = CommandLease()
        lease.lease_id, lease.robot_id, lease.robot_boot_id = lease_id, robot, boot
        lease.coordinator_id, lease.authority_epoch = coordinator, epoch
        lease.generation, lease.renewal_sequence = 1, 0
        lease.issued_at_ns, lease.valid_until_ns = now, now + 5_000_000_000
        lease.envelope.schema_version, lease.envelope.publisher_id = 'probe-only', 'probe-coordinator'
        lease.envelope.publisher_boot_id, lease.envelope.published_at_ns = coordinator, now
        publisher.publish(lease)
        wait(lambda: lease_id in leases)
        goal = ExecuteTask.Goal()
        goal.command_id, goal.task_id, goal.robot_id = command_id, task_id, robot
        goal.expected_robot_boot_id, goal.coordinator_id, goal.authority_epoch = boot, coordinator, epoch
        goal.generation, goal.kind, goal.lease_id = 1, 'HOLD', lease_id
        goal.issued_at_ns, goal.deadline_ns, goal.policy_version = now, now + 10_000_000_000, 'probe-only'
        goal.envelope = deepcopy(lease.envelope)
        goal.envelope.sequence, goal.envelope.published_at_ns = 1, time.time_ns()
        future = client.send_goal_async(goal)
        wait(future.done)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('HOLD goal was rejected')
        result_future = handle.get_result_async()
        wait(result_future.done)
        wait(lambda: bool(admissions))
        result = result_future.result()
        if not (result.status == 4 and result.result.result.outcome == 'COMPLETED'
                and result.result.result.stop_confirmed and admissions[-1].accepted):
            raise RuntimeError('action/admission/stop result mismatch')
        print(json.dumps({'status': 'PASSED', 'layer': 'E3-local-ROS', 'task': 'HOLD',
                          'admission_received': True, 'action_result_received': True,
                          'motion_inputs': 'SYNTHETIC', 'robot_validation': 'NOT_RUN',
                          'nav2_client_created': False, 'cmd_vel_publisher_created': False}))
    finally:
        bridge.destroy()
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
