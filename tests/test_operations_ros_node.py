"""DB writer node: the parts that run without ROS (configuration check, import without rclpy)."""
import importlib
import os
import signal
import sys
import unittest

from rescue_operations.schema import OperationsError


def params(**changes):
    values = {'db_path': '/var/lib/rescue/site_state.sqlite3', 'allowed_robot_ids': ['robot_a', 'robot_b'],
              'robot_roles': ['SCOUT', 'SUPPORT'],
              'status_topics': ['/rescue/robots/robot_a/status', '/rescue/robots/robot_b/status'],
              'pose_topics': ['/rescue/robots/robot_a/pose', '/rescue/robots/robot_b/pose'],
              'site_events_topic': '/rescue/site_events', 'system_state_topic': '/rescue/system_state',
              'targets_topic': '/rescue/targets', 'api_host': '127.0.0.1', 'api_port': 8090,
              'operator_command_service': '/rescue/operator_command', 'command_timeout_s': 3.0}
    return values | changes


class NodeConfig(unittest.TestCase):
    def test_module_imports_without_rclpy(self):
        blocked = {name: None for name in ('rclpy', 'rescue_interfaces', 'rescue_interfaces.msg')}
        saved = {name: sys.modules.get(name) for name in blocked}
        sys.modules.update(blocked)  # a None entry makes `import name` fail
        try:
            sys.modules.pop('rescue_operations.ros_node', None)
            importlib.import_module('rescue_operations.ros_node')
        finally:
            for name, module in saved.items():
                if module is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = module

    def test_parallel_lists_become_one_entry_per_robot(self):
        from rescue_operations.ros_node import node_config
        config = node_config(params())
        self.assertEqual({'robot_a': 'SCOUT', 'robot_b': 'SUPPORT'}, config['roles'])
        self.assertEqual([('robot_a', '/rescue/robots/robot_a/status', '/rescue/robots/robot_a/pose'),
                          ('robot_b', '/rescue/robots/robot_b/status', '/rescue/robots/robot_b/pose')],
                         config['robots'])
        self.assertEqual(('/var/lib/rescue/site_state.sqlite3', '/rescue/site_events', '/rescue/system_state',
                          '/rescue/targets'),
                         (config['db_path'], config['site_events_topic'], config['system_state_topic'],
                          config['targets_topic']))

    def test_command_api_settings_are_kept(self):
        """#115: the console posts to this address; the API waits command_timeout_s (< console 5 s)."""
        from rescue_operations.ros_node import node_config
        config = node_config(params())
        self.assertEqual(('127.0.0.1', 8090, '/rescue/operator_command', 3_000_000_000),
                         (config['api_host'], config['api_port'], config['operator_command_service'],
                          config['command_timeout_ns']))

    def test_inconsistent_configuration_is_refused_before_anything_starts(self):
        from rescue_operations.ros_node import node_config
        bad = [dict(robot_roles=['SCOUT']), dict(status_topics=['/a']), dict(pose_topics=[]),
               dict(robot_roles=['SCOUT', 'LEADER']), dict(robot_roles=['SCOUT', 'SCOUT']),
               dict(status_topics=['/same', '/same']), dict(allowed_robot_ids=['robot_a', 'robot_a']),
               dict(site_events_topic=''), dict(db_path=''), dict(system_state_topic=''),
               dict(system_state_topic='/rescue/site_events'), dict(targets_topic=''),
               dict(targets_topic='/rescue/system_state'), dict(api_host=''), dict(api_port=0),
               dict(api_port=70000), dict(operator_command_service=''), dict(command_timeout_s=0.0),
               dict(command_timeout_s=5.0)]
        for change in bad:
            with self.subTest(change=change), self.assertRaises(OperationsError) as caught:
                node_config(params(**change))
            self.assertEqual('INVALID_INPUT', caught.exception.code)

    def test_subscriptions_accept_both_latched_and_plain_publishers(self):
        """Local ROS run with the #90 probe (2026-10-11): a VOLATILE RobotStatus publisher and a
        TRANSIENT_LOCAL subscription are incompatible, so no status arrived. A VOLATILE subscription
        matches either kind; only SystemState keeps TRANSIENT_LOCAL to get the coordinator's latest
        state on a late start (#90 publishes it latched)."""
        from rescue_operations.ros_node import QOS
        durability = {kind: value for kind, (_, value, _) in QOS.items()}
        for kind in ('status', 'pose', 'site_events'):
            with self.subTest(kind=kind):
                self.assertEqual('VOLATILE', durability[kind])
        self.assertEqual('TRANSIENT_LOCAL', durability['system_state'])  # #90 latches it
        self.assertEqual('TRANSIENT_LOCAL', durability['targets'])  # #96 latches TargetSnapshot too


class Teardown(unittest.TestCase):
    """A second SIGINT (launch + terminal Ctrl-C) during cleanup must not abort it (measured 2026-10-13:
    KeyboardInterrupt inside ThreadingHTTPServer.shutdown, traceback, exit -2)."""

    def setUp(self):
        self.addCleanup(signal.signal, signal.SIGINT, signal.getsignal(signal.SIGINT))

    def test_repeated_sigint_does_not_abort_the_remaining_steps(self):
        from rescue_operations.ros_node import teardown
        done = []

        def interrupted():
            os.kill(os.getpid(), signal.SIGINT)  # what the second Ctrl-C does during server.shutdown()
            done.append('server')

        teardown(interrupted, lambda: done.append('ledger'), lambda: done.append('node'))
        self.assertEqual(['server', 'ledger', 'node'], done)

    def test_sigint_stays_ignored_while_the_process_ends(self):
        from rescue_operations.ros_node import teardown
        teardown(lambda: None)
        self.assertIs(signal.SIG_IGN, signal.getsignal(signal.SIGINT))

    def test_report_node_cleans_up_with_the_same_teardown(self):
        # report_node (#137) had the same finally block and died with exit -2 on a second SIGINT (measured 2026-10-13).
        from rescue_operations import report_node, ros_node
        self.assertIs(ros_node.teardown, report_node.teardown)

    def test_a_failing_step_does_not_skip_the_rest_and_is_raised(self):
        from rescue_operations.ros_node import teardown
        done = []

        def broken():
            raise OSError('socket already closed')

        with self.assertRaises(OSError):
            teardown(broken, lambda: done.append('ledger'))
        self.assertEqual(['ledger'], done)


if __name__ == '__main__':
    unittest.main()
