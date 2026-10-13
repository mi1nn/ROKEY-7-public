"""Perception node shutdown race (OPS-129, 2026-10-13; the coordinator had the same one, #154).

Rehearsal #220: target_locator ended 1 of 3 runs on SIGTERM with RCLError("the given context is
not valid") and exit 1. rclpy shuts the context down on SIGTERM; a spin caught between callbacks
raises that instead of ExternalShutdownException. Only that case is a normal stop: the same error
while the context is valid, or any other error, is raised. Every perception node spins through it.
"""
import ast
from pathlib import Path
import unittest

from rescue_perception.shutdown import spin_until_shutdown

PACKAGE = Path(__file__).resolve().parents[1] / 'src/rescue_perception/rescue_perception'
NODES = ('target_locator.py', 'amr_target.py', 'amr_camera.py', 'mjpeg_relay.py')


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

    def test_a_normal_return_is_kept(self):
        calls = []
        spin_until_shutdown(lambda: calls.append('spun'), lambda: True, RCLError)
        self.assertEqual(['spun'], calls)

    def test_every_node_spins_through_the_shutdown_guard(self):
        for name in NODES:
            tree = ast.parse((PACKAGE / name).read_text())
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
