"""Installed task_input entrypoint sharing the existing command adapter process.

Use the supplied rescue_robot.task_site_runtime:create_runtime factory and
task.runtime_config (task_prepare generates the overlay file). It attaches
TaskActionServer to explicit trusted input snapshots. No defaults arm or
dispatch a robot; a watchdog is not required to create the ROS endpoints.
"""
import importlib
import inspect

from rescue_robot.executor import ContractError


def resolve_factory(reference):
    if not isinstance(reference, str) or reference.count(':') != 1:
        raise ContractError('task.runtime_factory must be installed.module:callable')
    module, name = reference.split(':')
    if not all(part.isidentifier() for part in module.split('.')) or not name.isidentifier():
        raise ContractError('invalid runtime factory reference')
    factory = getattr(importlib.import_module(module), name)
    if not callable(factory):
        raise ContractError('runtime factory must be callable')
    return factory


def attach_configured_runtime(node, adapter):
    from rcl_interfaces.msg import ParameterType
    from rescue_robot.ros_node import declare_required
    values, missing = declare_required(node, {'task.runtime_factory': ParameterType.PARAMETER_STRING})
    if missing:
        raise ContractError('task.runtime_factory is required; supply installed.module:callable')
    reference = values['task.runtime_factory']
    runtime = resolve_factory(reference)(node, adapter)
    if runtime is None or not callable(getattr(runtime, 'destroy', None)):
        raise ContractError('runtime factory must return a destroyable task runtime')
    node.get_logger().info(f'task runtime attached: {reference}')
    return runtime


def main(args=None):
    from rescue_robot.ros_node import main as adapter_main
    # Existing field work adds this hook; refuse an older installed adapter.
    if 'task_runtime_factory' not in inspect.signature(adapter_main).parameters:
        raise SystemExit('task_input requires the field adapter task_runtime_factory hook; rebuild rescue_robot')
    return adapter_main(args=args, task_runtime_factory=attach_configured_runtime)
