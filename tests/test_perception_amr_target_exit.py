"""amr_target.main closes its clock sampler on exit (#175 review, 2026-10-13).

The exit path looked for an attribute name from before a rename, so the measurement thread pool was
never shut down. Needs the ROS Python modules amr_target imports; skipped without them.
"""
import sys
import types
import unittest


class AmrTargetExitTests(unittest.TestCase):
    def test_clock_sampler_is_closed_on_exit(self):
        sys.modules.setdefault('ultralytics', types.SimpleNamespace(YOLO=object))   # no model is loaded here
        try:
            import rescue_perception.amr_target as amr
        except ImportError as exc:
            self.skipTest(f'ROS modules not importable (NOT_RUN): {exc}')
        closed = []

        class Node:
            clock_sampler = types.SimpleNamespace(close=lambda: closed.append(True))

            def destroy_node(self):
                pass

        def interrupted(node):
            raise KeyboardInterrupt
        fake_rclpy = types.SimpleNamespace(init=lambda args=None: None, spin=interrupted, ok=lambda: False,
                                           shutdown=lambda: None)
        class Executor:   # main spins a multi-threaded executor (recheck window in its own group)
            def __init__(self, num_threads=None):
                pass

            def add_node(self, node):
                self.node = node

            def spin(self):
                interrupted(self.node)
        original = amr.rclpy, amr.create_node, amr.MultiThreadedExecutor
        amr.rclpy, amr.create_node, amr.MultiThreadedExecutor = fake_rclpy, Node, Executor
        try:
            self.assertEqual(0, amr.main())
        finally:
            amr.rclpy, amr.create_node, amr.MultiThreadedExecutor = original
        self.assertEqual([True], closed)


if __name__ == '__main__':
    unittest.main()
