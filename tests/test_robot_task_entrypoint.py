"""Installed entrypoint validation without importing ROS at module import time."""
import sys
import ast
from pathlib import Path
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from rescue_robot.executor import ContractError
from rescue_robot.task_input_node import main, attach_configured_runtime, create_runtime


class EntryPointTests(unittest.TestCase):
    def test_source_alias_check_has_no_build_tool_dependency(self):
        setup = Path(__file__).resolve().parents[1] / 'src/rescue_robot/setup.py'
        if not setup.is_file():
            self.skipTest('source-only check; installed aliases are checked through distribution metadata')
        with patch.dict(sys.modules, {'setuptools': None}):
            self.test_existing_launch_alias_and_new_entrypoint_share_one_runtime()

    def test_existing_launch_alias_and_new_entrypoint_share_one_runtime(self):
        setup = Path(__file__).resolve().parents[1] / 'src/rescue_robot/setup.py'
        if setup.is_file():
            # The source suite has no build-tool dependency (notably on Python
            # 3.12). Check the declared aliases with stdlib only; the wheel
            # suite below checks the real installed distribution metadata.
            tree = ast.parse(setup.read_text(encoding='utf-8'))
            calls = [node for node in tree.body if isinstance(node, ast.Expr)
                     and isinstance(node.value, ast.Call)
                     and isinstance(node.value.func, ast.Name)
                     and node.value.func.id == 'setup']
            self.assertEqual(len(calls), 1)
            values = [keyword.value for keyword in calls[0].value.keywords
                      if keyword.arg == 'entry_points']
            self.assertEqual(len(values), 1)
            entries = dict(item.split(' = ', 1) for item in
                           ast.literal_eval(values[0])['console_scripts'])
        else:
            from importlib.metadata import distribution
            entries = {e.name: e.value for e in distribution('rescue_robot').entry_points}
        self.assertEqual(entries['robot_task_input'], 'rescue_robot.task_input_node:main')
        self.assertEqual(entries['task_input'], entries['robot_task_input'])

    def test_config_path_must_be_absolute(self):
        with patch.dict(sys.modules, {
            'rcl_interfaces.msg': SimpleNamespace(ParameterType=SimpleNamespace(PARAMETER_STRING=4)),
            'rescue_robot.ros_node': SimpleNamespace(declare_required=lambda node, names: (
                {'task.runtime_config': 'relative/runtime.json'}, []))}):
            with self.assertRaises(ContractError): create_runtime(object(), object())

    def test_missing_runtime_config_rejected_before_reading_input(self):
        with patch.dict(sys.modules, {
            'rcl_interfaces.msg': SimpleNamespace(ParameterType=SimpleNamespace(PARAMETER_STRING=4)),
            'rescue_robot.ros_node': SimpleNamespace(declare_required=lambda node, names: (
                {'task.runtime_config': None}, ['task.runtime_config']))}):
            with self.assertRaises(ContractError): create_runtime(object(), object())

    def test_invalid_runtime_return_is_rejected(self):
        with patch('rescue_robot.task_input_node.create_runtime', return_value=None):
            with self.assertRaises(ContractError): attach_configured_runtime(object(), object())

    def test_entrypoint_shares_adapter_instead_of_creating_second_node(self):
        calls=[]
        def adapter_main(args=None, *, task_runtime_factory=None):
            calls.append((args,task_runtime_factory));return 'shared'
        with patch.dict(sys.modules,{'rescue_robot.ros_node':SimpleNamespace(main=adapter_main)}):
            self.assertEqual(main(['--ros-args']),'shared')
        self.assertEqual(len(calls),1)
        self.assertTrue(callable(calls[0][1]))

    def test_older_adapter_without_runtime_hook_is_rejected(self):
        with patch.dict(sys.modules,{'rescue_robot.ros_node':SimpleNamespace(main=lambda args=None:None)}):
            with self.assertRaises(SystemExit):main()

    def test_builtin_runtime_attaches_without_dynamic_factory(self):
        runtime = SimpleNamespace(destroy=lambda: None)
        node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda message: None))
        with patch('rescue_robot.task_input_node.create_runtime', return_value=runtime):
            self.assertIs(attach_configured_runtime(node, object()), runtime)

    def test_config_parameter_uses_existing_jazzy_typed_helper(self):
        calls = []
        def declare(node, names):
            calls.append(names)
            return {}, ['task.runtime_config']
        with patch.dict(sys.modules, {
            'rcl_interfaces.msg': SimpleNamespace(ParameterType=SimpleNamespace(PARAMETER_STRING=4)),
            'rescue_robot.ros_node': SimpleNamespace(declare_required=declare)}):
            with self.assertRaises(ContractError): create_runtime(object(), object())
        self.assertEqual(calls, [{'task.runtime_config': 4}])


if __name__ == '__main__':
    unittest.main()
