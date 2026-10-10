"""Login throttle: repeated wrong passwords for one name from one client are refused for a while."""
import sys
import threading
import unittest
from unittest import mock

from rescue_console import throttle as throttle_module
from rescue_console.throttle import (LOGIN_LOCK_S, LOGIN_MAX_FAILURES, LOGIN_MAX_KEYS, LOGIN_WINDOW_S,
                                     LoginThrottle)


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class LoginThrottleTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.throttle = LoginThrottle(clock=self.clock)

    def fail(self, times, name='admin', client='10.0.0.5'):
        for _ in range(times):
            self.throttle.failed(name, client)

    def test_locks_after_the_limit_and_opens_again_after_the_lock(self):
        self.fail(LOGIN_MAX_FAILURES - 1)
        self.assertEqual(self.throttle.locked_for('admin', '10.0.0.5'), 0)
        self.fail(1)
        self.assertEqual(self.throttle.locked_for('admin', '10.0.0.5'), LOGIN_LOCK_S)
        self.clock.now += LOGIN_LOCK_S - 1
        self.assertEqual(self.throttle.locked_for('admin', '10.0.0.5'), 1)
        self.clock.now += 1
        self.assertEqual(self.throttle.locked_for('admin', '10.0.0.5'), 0)

    def test_name_is_case_insensitive_and_clients_are_separate(self):
        self.fail(LOGIN_MAX_FAILURES, name='Admin ')
        self.assertGreater(self.throttle.locked_for('admin', '10.0.0.5'), 0)
        self.assertEqual(self.throttle.locked_for('admin', '10.0.0.6'), 0)
        self.assertEqual(self.throttle.locked_for('jane', '10.0.0.5'), 0)

    def test_success_clears_the_count(self):
        self.fail(LOGIN_MAX_FAILURES - 1)
        self.throttle.succeeded('admin', '10.0.0.5')
        self.fail(LOGIN_MAX_FAILURES - 1)
        self.assertEqual(self.throttle.locked_for('admin', '10.0.0.5'), 0)

    def test_old_failures_fall_out_of_the_window(self):
        self.fail(LOGIN_MAX_FAILURES - 1)
        self.clock.now += LOGIN_WINDOW_S + 1
        self.fail(1)
        self.assertEqual(self.throttle.locked_for('admin', '10.0.0.5'), 0)

    def test_memory_is_bounded_under_a_flood_of_names(self):
        for i in range(LOGIN_MAX_KEYS + 50):
            self.throttle.failed(f'user{i}', '10.0.0.5')
        self.assertLessEqual(len(self.throttle), LOGIN_MAX_KEYS)



class LoginThrottleThreadTests(unittest.TestCase):
    """The dev server is threaded (Flask app.run default): requests call the throttle concurrently (OPS-51-01)."""

    def setUp(self):
        interval = sys.getswitchinterval()
        sys.setswitchinterval(1e-6)          # switch threads as often as possible to expose races
        self.addCleanup(sys.setswitchinterval, interval)

    @staticmethod
    def together(count, work):
        start, errors = threading.Barrier(count), []

        def run(index):
            start.wait()
            try:
                work(index)
            except Exception as exc:          # collected, so a race fails the test instead of a thread
                errors.append(exc)
        threads = [threading.Thread(target=run, args=(i,)) for i in range(count)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        return errors

    def test_concurrent_failures_of_one_pair_still_lock_it(self):
        for _ in range(200):
            throttle = LoginThrottle(clock=lambda: 1000.0)
            errors = self.together(LOGIN_MAX_FAILURES, lambda i: throttle.failed('admin', '10.0.0.5'))
            self.assertEqual(errors, [])
            self.assertEqual(throttle.locked_for('admin', '10.0.0.5'), LOGIN_LOCK_S)

    def test_concurrent_flood_evicts_without_errors(self):
        throttle = LoginThrottle(clock=lambda: 1000.0)
        with mock.patch.object(throttle_module, 'LOGIN_MAX_KEYS', 50):
            errors = self.together(8, lambda k: [throttle.failed(f'user{k}-{i}', 'c') for i in range(2000)])
            self.assertEqual(errors, [])
            self.assertLessEqual(len(throttle), 50)


if __name__ == '__main__':
    unittest.main()
