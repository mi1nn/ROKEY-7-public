"""Operations node shutdown race (official rehearsal 2026-10-13 on main 2098ca2; coordinator #154, perception #222).

db_writer ended 1 of 3 rehearsal runs on SIGTERM with RCLError("the given context is not valid") and exit 1.
rclpy shuts the context down on SIGTERM; a spin caught between callbacks raises that instead of
ExternalShutdownException. Only that case is a normal stop: the same error while the context is valid, or any
other error, is raised. Both operations nodes (db_writer, report_node) spin through it.
"""
import ast
import importlib.util
from pathlib import Path
import unittest

from rescue_operations.shutdown import spin_until_shutdown

NODES = ('ros_node', 'report_node')


def node_source(name):
    """The module source as installed (wheel or checkout), found without importing it (no ROS here)."""
    return Path(importlib.util.find_spec(f'rescue_operations.{name}').origin).read_text()


class RCLError(RuntimeError):
    """Stands in for rclpy._rclpy_pybind11.RCLError (no ROS here)."""


def raising(exc):
    def spin():
        raise exc
    return spin


class ShutdownRaceTests(unittest.TestCase):
    def test_rcl_error_after_the_context_closed_is_a_normal_stop(self):
        self.assertIsNone(spin_until_shutdown(raising(RCLError('context is not valid')), lambda: False, RCLError))

    def test_rcl_error_while_the_context_is_valid_is_raised(self):
        with self.assertRaises(RCLError):
            spin_until_shutdown(raising(RCLError('wait set failed')), lambda: True, RCLError)

    def test_other_errors_are_raised_even_after_the_context_closed(self):
        with self.assertRaises(ValueError):
            spin_until_shutdown(raising(ValueError('bug')), lambda: False, RCLError)

    def test_both_nodes_spin_through_the_shutdown_guard(self):
        for name in NODES:
            tree = ast.parse(node_source(name))
            bare = [n.lineno for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Attribute) and n.func.attr == 'spin'
                    and not any(isinstance(p, ast.Lambda) and n in ast.walk(p) for p in ast.walk(tree))]
            guarded = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                       and getattr(n.func, 'id', getattr(n.func, 'attr', None)) == 'spin_until_shutdown']
            with self.subTest(node=name):
                self.assertEqual([], bare, 'spin outside spin_until_shutdown')
                self.assertTrue(guarded)


if __name__ == '__main__':
    unittest.main()
