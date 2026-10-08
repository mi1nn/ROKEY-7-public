"""E2: readiness flags from arrival freshness; watchdog without a source never readies."""
import unittest
from uuid import UUID

from rescue_robot.executor import ContractError, Executor, Policy
from rescue_robot.nav2_adapter import READINESS_FLAGS, CommandAdapter
from rescue_robot.readiness import MONITORED_FLAGS, ReadinessMonitor, ReadinessPolicy

# Test-only ages and source names; field values are not measured yet.
AGES = {'odom': .5, 'scan': .5, 'camera_front': 1.0, 'map_to_odom_tf': 1.0, 'watchdog_heartbeat': .3}
SOURCES = {'driver': ('odom',), 'sensors': ('scan', 'camera_front'), 'slam': ('map_to_odom_tf',)}


def uid(n):
    return str(UUID(int=n))


class ReadinessMonitorTests(unittest.TestCase):
    def setUp(self):
        self.monitor = ReadinessMonitor(ReadinessPolicy('test-only', dict(AGES), dict(SOURCES)))

    def feed(self, t, *sources):
        for source in sources or AGES:
            self.monitor.arrived(source, t)

    def test_all_sources_fresh_then_one_ages_out(self):
        self.feed(10.0)
        self.assertEqual(self.monitor.flags(10.4), {'driver': True, 'sensors': True, 'slam': True})
        self.assertEqual(self.monitor.flags(10.6), {'driver': False, 'sensors': False, 'slam': True})

    def test_every_source_of_a_flag_is_required(self):
        self.feed(10.0, 'odom', 'scan', 'map_to_odom_tf')  # no camera
        self.assertFalse(self.monitor.flags(10.1)['sensors'])

    def test_out_of_order_or_future_arrival_is_not_fresh(self):
        self.feed(10.0)
        self.monitor.arrived('odom', 9.0)            # older report cannot refresh
        self.assertFalse(self.monitor.flags(10.6)['driver'])
        self.monitor.arrived('scan', 12.0)           # future relative to now
        self.assertFalse(self.monitor.flags(11.0)['sensors'])

    def test_watchdog_without_source_keeps_adapter_not_ready(self):
        self.assertNotIn('watchdog', self.monitor.flags(10.0))
        clock = [10.0]
        executor = Executor('robot_a', uid(1), uid(2), Policy('test-only', 1, 3, .2, .11, .5, .01, .01),
                            allowed_robot_ids=('robot_a', 'robot_b'))
        adapter = CommandAdapter(executor, object(), guards=lambda task: True, clock_error_bound_s=1.0,
                                 clock=lambda: clock[0], wall_clock_ns=lambda: 1700000000_000000000)
        self.feed(10.0)
        adapter.observe_status(nav2=True, undocked=True)
        self.monitor.publish(adapter)
        self.assertFalse(adapter.readiness(10.0).valid(10.0, executor.policy))
        with_watchdog = ReadinessMonitor(ReadinessPolicy(
            'test-only', dict(AGES), dict(SOURCES, watchdog=('watchdog_heartbeat',))))
        for source in AGES:
            with_watchdog.arrived(source, 10.0)
        with_watchdog.publish(adapter)
        self.assertTrue(adapter.readiness(10.0).valid(10.0, executor.policy))

    def test_policy_shape(self):
        for ages, sources in (({'odom': 0}, {'driver': ('odom',)}),
                              ({'odom': .5}, {'driver': ('scan',)}),
                              ({'odom': .5}, {'nav2': ('odom',)}),
                              ({'odom': .5}, {'driver': ['odom']}),
                              ({'odom': float('inf')}, {})):
            with self.assertRaises(ContractError):
                ReadinessPolicy('test-only', ages, sources)
        with self.assertRaises(ContractError):
            self.monitor.arrived('lidar', 1.0)
        self.assertTrue(set(MONITORED_FLAGS) <= set(READINESS_FLAGS))



class ReadinessPolicyBuilderTests(unittest.TestCase):
    def test_ros_node_policy_builder_without_ros(self):
        from rescue_robot.ros_node import readiness_policy
        ages = {'odom': .5, 'scan': .5, 'camera': 1., 'wheel_status': .5, 'map_to_odom_tf': 1.}
        bare = readiness_policy('test-only', ages, (), '')
        self.assertEqual(bare.flag_sources, {'driver': ('odom',), 'sensors': ('scan',),
                                             'slam': ('map_to_odom_tf',)})
        full = readiness_policy('test-only', ages, ('camera/image',), 'wheel_status')
        self.assertEqual(full.flag_sources['driver'], ('odom', 'wheel_status'))
        self.assertEqual(full.flag_sources['sensors'], ('scan', 'camera:camera/image'))
        self.assertNotIn('watchdog', full.flag_sources)


if __name__ == '__main__':
    unittest.main()
