"""Coordinator shutdown race (measured 2026-10-13, A·B rehearsal: 2 of 3 runs ended with exit 1).

On SIGTERM rclpy shuts the context down; a ``rclpy.spin`` caught between callbacks then raises
RCLError("the given context is not valid") instead of ExternalShutdownException. Only that case
is a normal stop: the same error while the context is still valid, or any other error, is raised.
"""
import unittest

from rescue_coordination.ros_node import spin_until_shutdown


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


if __name__ == '__main__':
    unittest.main()
