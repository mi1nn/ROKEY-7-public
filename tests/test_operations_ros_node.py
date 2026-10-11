"""DB writer node: the parts that run without ROS (configuration check, import without rclpy)."""
import importlib
import sys
import unittest

from rescue_operations.schema import OperationsError


def params(**changes):
    values = {'db_path': '/var/lib/rescue/site_state.sqlite3', 'allowed_robot_ids': ['robot_a', 'robot_b'],
              'robot_roles': ['SCOUT', 'SUPPORT'],
              'status_topics': ['/rescue/robots/robot_a/status', '/rescue/robots/robot_b/status'],
              'pose_topics': ['/rescue/robots/robot_a/pose', '/rescue/robots/robot_b/pose'],
              'site_events_topic': '/rescue/site_events'}
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
        self.assertEqual(('/var/lib/rescue/site_state.sqlite3', '/rescue/site_events'),
                         (config['db_path'], config['site_events_topic']))

    def test_inconsistent_configuration_is_refused_before_anything_starts(self):
        from rescue_operations.ros_node import node_config
        bad = [dict(robot_roles=['SCOUT']), dict(status_topics=['/a']), dict(pose_topics=[]),
               dict(robot_roles=['SCOUT', 'LEADER']), dict(robot_roles=['SCOUT', 'SCOUT']),
               dict(status_topics=['/same', '/same']), dict(allowed_robot_ids=['robot_a', 'robot_a']),
               dict(site_events_topic=''), dict(db_path='')]
        for change in bad:
            with self.subTest(change=change), self.assertRaises(OperationsError) as caught:
                node_config(params(**change))
            self.assertEqual('INVALID_INPUT', caught.exception.code)


if __name__ == '__main__':
    unittest.main()
