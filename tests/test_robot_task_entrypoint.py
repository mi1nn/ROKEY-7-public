"""Installed entrypoint validation without importing ROS at module import time."""
import sys
import runpy
from pathlib import Path
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from rescue_robot.executor import ContractError
from rescue_robot.task_input_node import resolve_factory, main, attach_configured_runtime


class EntryPointTests(unittest.TestCase):
    def test_existing_launch_alias_and_new_entrypoint_share_one_runtime(self):
        setup = Path(__file__).resolve().parents[1] / 'src/rescue_robot/setup.py'
        if setup.is_file():
            metadata = {}
            with patch('setuptools.setup', side_effect=lambda **kwargs: metadata.update(kwargs)):
                runpy.run_path(str(setup))
            entries = dict(item.split(' = ', 1) for item in metadata['entry_points']['console_scripts'])
        else:
            from importlib.metadata import distribution
            entries = {e.name: e.value for e in distribution('rescue_robot').entry_points}
        self.assertEqual(entries['robot_task_input'], 'rescue_robot.task_input_node:main')
        self.assertEqual(entries['task_input'], entries['robot_task_input'])

    def test_requires_explicit_installed_factory_reference(self):
        for value in (None,'','file.py','module:factory:extra','../module:factory','module:a.b'):
            with self.assertRaises(ContractError):resolve_factory(value)

    def test_factory_must_be_callable(self):
        with patch('rescue_robot.task_input_node.importlib.import_module',return_value=SimpleNamespace(factory=1)):
            with self.assertRaises(ContractError):resolve_factory('module:factory')

    def test_missing_factory_is_visible(self):
        with patch('rescue_robot.task_input_node.importlib.import_module',return_value=SimpleNamespace()):
            with self.assertRaises(AttributeError):resolve_factory('module:factory')

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

    def test_required_factory_uses_existing_jazzy_typed_parameter_helper(self):
        runtime = SimpleNamespace(destroy=lambda: None)
        calls = []
        def declare(node, names):
            calls.append(names)
            return {'task.runtime_factory': 'module:factory'}, []
        node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda message: None))
        with patch.dict(sys.modules, {
            'rcl_interfaces.msg': SimpleNamespace(ParameterType=SimpleNamespace(PARAMETER_STRING=4)),
            'rescue_robot.ros_node': SimpleNamespace(declare_required=declare)}):
            with patch('rescue_robot.task_input_node.resolve_factory', return_value=lambda node, adapter: runtime):
                self.assertIs(attach_configured_runtime(node, object()), runtime)
        self.assertEqual(calls, [{'task.runtime_factory': 4}])

    def test_unset_factory_rejected_before_loading_site_code(self):
        with patch.dict(sys.modules, {
            'rcl_interfaces.msg': SimpleNamespace(ParameterType=SimpleNamespace(PARAMETER_STRING=4)),
            'rescue_robot.ros_node': SimpleNamespace(declare_required=lambda node, names: (
                {'task.runtime_factory': None}, ['task.runtime_factory']))}):
            with self.assertRaises(ContractError): attach_configured_runtime(object(), object())


if __name__ == '__main__':
    unittest.main()
