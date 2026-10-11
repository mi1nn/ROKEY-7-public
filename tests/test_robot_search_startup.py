"""Adapter startup validation/cleanup without importing ROS (E2)."""
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from rescue_robot import ros_node


class StartupTests(unittest.TestCase):
    def test_missing_startup_parameters_destroy_node_and_shutdown_context(self):
        calls = []
        node = NS(get_logger=lambda: NS(fatal=lambda message: calls.append('fatal')),
                  destroy_node=lambda: calls.append('destroy'))
        with patch.dict(sys.modules, {
            'rclpy': NS(init=lambda **kwargs: calls.append('init'), try_shutdown=lambda: calls.append('shutdown')),
            'rclpy.node': NS(Node=lambda name: node),
            'rcl_interfaces.msg': NS(ParameterType=NS(PARAMETER_STRING=4, PARAMETER_DOUBLE=3,
                                                      PARAMETER_STRING_ARRAY=9))}), \
                patch.object(ros_node, 'declare_required', return_value=({}, ['robot_id'])):
            with self.assertRaises(SystemExit) as failure:
                ros_node.main()
        self.assertEqual(failure.exception.code, 2)
        self.assertEqual(calls, ['init', 'fatal', 'destroy', 'shutdown'])

    def test_required_parameters_have_explicit_type_and_readonly_descriptor(self):
        declared = []
        node = NS(declare_parameter=lambda *args, **kwargs: declared.append((args, kwargs)),
                  get_parameter=lambda name: NS(value='configured'))
        with patch.dict(sys.modules, {
            'rclpy.exceptions': NS(ParameterUninitializedException=RuntimeError),
            'rclpy.parameter': NS(Parameter=NS(Type=lambda kind: ('type', kind))),
            'rcl_interfaces.msg': NS(ParameterDescriptor=lambda **kwargs: NS(**kwargs))}):
            values, missing = ros_node.declare_required(node, {'robot_id': 4})
        self.assertEqual((values, missing), ({'robot_id': 'configured'}, []))
        args, kwargs = declared[0]
        self.assertEqual(args, ('robot_id', ('type', 4)))
        self.assertIn('descriptor', kwargs)
        self.assertEqual(kwargs['descriptor'].type, 4)
        self.assertTrue(kwargs['descriptor'].read_only)


if __name__ == '__main__':
    unittest.main()
