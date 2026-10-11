"""E3 (partial): command watchdog over real rclpy topics, two namespaces.

Not collected by scripts/run_tests.py. Run on an isolated graph, never a robot's domain:

    ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_robot:$PYTHONPATH \
      python3 tests/ros/e3_watchdog_probe.py   # not -I: ROS needs PYTHONPATH

No robot and no real cmd_vel: a fake mover publishes Twist on each robot's
`cmd_vel_nav`, and the probe reads `cmd_vel_out` (a probe-only name). Only
/robot_a gets an executor heartbeat. Expected:
1. robot_a forwards the moving command and its watchdog heartbeat makes the
   adapter's `watchdog` readiness flag True; robot_b sends only zeros, no flag.
2. Executor heartbeat stops -> robot_a sends a zero, latches, flag goes False.
3. Heartbeat resumes -> still latched (no forwarding). `~/reset` -> forwarding again.
Exit code 0 means all checks held.
"""
import os
import sys
import time
from uuid import UUID

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from rescue_robot.executor import Executor, Policy
from rescue_robot.nav2_adapter import CommandAdapter
from rescue_robot.readiness import ReadinessMonitor
from rescue_robot.ros_node import bind_readiness, readiness_policy
from rescue_robot.watchdog import CommandWatchdog, WatchdogPolicy
from rescue_robot.watchdog_node import bind_watchdog

ROBOTS = (('robot_a', '/robot_a'), ('robot_b', '/robot_b'))  # = config/deployment.example.json
POLICY = Policy('e3-probe', 1, 3, .2, .11, .5, .01, .01)    # probe-only thresholds
DOG = WatchdogPolicy('e3-probe', heartbeat_max_age_s=.3, command_max_age_s=.3,
                     max_linear_mps=.3, max_angular_rps=1.)
AGES = {'odom': .5, 'scan': .5, 'camera': 1., 'wheel_status': .5, 'map_to_odom_tf': 1.,
        'watchdog_heartbeat': .3}


class FakePort:
    def send_goal(self, command_id, pose):
        raise AssertionError('probe sends no goal')

    def cancel_goal(self, command_id):
        raise AssertionError('probe cancels nothing')


def main():
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if domain < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        sys.exit('refuse: need ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    rclpy.init()
    spin = SingleThreadedExecutor()
    nodes, dogs, monitors, adapters, outputs = [], {}, {}, {}, {}
    for n, (robot_id, namespace) in enumerate(ROBOTS):
        executor = Executor(robot_id, str(UUID(int=n + 1)), str(UUID(int=n + 11)), POLICY,
                            allowed_robot_ids=[r for r, _ in ROBOTS])
        adapter_node = Node('rescue_command_adapter', namespace=namespace)
        adapter = CommandAdapter(executor, FakePort(), guards=lambda task: False, clock_error_bound_s=.5)
        monitor = ReadinessMonitor(readiness_policy('e3-probe', AGES, (), '', 'watchdog/heartbeat'))
        bind_readiness(adapter_node, monitor, adapter, scan_topic='scan', camera_topics=(),
                       wheel_status_topic='', tf_topic='tf', map_frame='map', odom_frame='odom',
                       watchdog_heartbeat_topic='watchdog/heartbeat')
        dog_node = Node('rescue_command_watchdog', namespace=namespace)
        dog = CommandWatchdog(DOG)
        bind_watchdog(dog_node, dog, cmd_vel_in_topic='cmd_vel_nav', cmd_vel_out_topic='cmd_vel_out',
                      stamped=False, executor_heartbeat_topic='executor/heartbeat',
                      watchdog_heartbeat_topic='watchdog/heartbeat', step_period=.02)
        nodes += [adapter_node, dog_node]
        dogs[robot_id], monitors[robot_id], adapters[robot_id] = dog, monitor, adapter
        outputs[robot_id] = []
    sim = Node('fake_mover')
    nodes.append(sim)
    for node in nodes:
        spin.add_node(node)
    movers = {r: sim.create_publisher(Twist, ns + '/cmd_vel_nav', 10) for r, ns in ROBOTS}
    beat_a = sim.create_publisher(Header, '/robot_a/executor/heartbeat', 10)
    for robot_id, namespace in ROBOTS:
        sim.create_subscription(Twist, namespace + '/cmd_vel_out',
                                lambda msg, r=robot_id: outputs[r].append(msg.linear.x), 10)
    reset_a = sim.create_client(Trigger, '/robot_a/rescue_command_watchdog/reset')

    def run(seconds, beat):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if beat:
                beat_a.publish(Header())
            go = Twist()
            go.linear.x = .2
            for mover in movers.values():
                mover.publish(go)
            stop = time.monotonic() + .03
            while time.monotonic() < stop:
                spin.spin_once(timeout_sec=.01)

    def flag(robot_id):
        return monitors[robot_id].flags(adapters[robot_id].clock()).get('watchdog')

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and (any(p.get_subscription_count() == 0 for p in movers.values())
                                          or beat_a.get_subscription_count() == 0):
        spin.spin_once(timeout_sec=.05)
    checks = {}
    run(1.0, beat=True)
    checks['a_forwards'] = .2 in outputs['robot_a'][-5:]
    checks['b_only_zero'] = bool(outputs['robot_b']) and set(outputs['robot_b']) == {0.0}
    checks['a_flag_true'] = flag('robot_a') is True
    checks['b_flag_false'] = flag('robot_b') is False
    run(.6, beat=False)
    checks['a_latched'] = (dogs['robot_a'].state, dogs['robot_a'].reason) == (
        'STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE')
    checks['a_zero_after_latch'] = outputs['robot_a'][-1] == 0.0
    checks['a_flag_false'] = flag('robot_a') is False
    run(.5, beat=True)
    checks['a_still_latched'] = dogs['robot_a'].state == 'STOP_LATCHED' and outputs['robot_a'][-1] == 0.0
    future = reset_a.call_async(Trigger.Request())
    while not future.done():
        spin.spin_once(timeout_sec=.02)
        beat_a.publish(Header())
    checks['reset_ok'] = future.result().success
    run(.5, beat=True)
    checks['a_forwards_after_reset'] = outputs['robot_a'][-1] == .2
    checks['b_never_armed'] = dogs['robot_b'].state == 'WAITING'
    print({'domain': domain, 'checks': checks, 'events_a': dogs['robot_a'].events})
    ok = all(checks.values())
    for node in nodes:
        node.destroy_node()
    rclpy.try_shutdown()
    print('E3 watchdog probe:', 'PASSED' if ok else 'FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
