"""Login throttle: repeated wrong passwords for one name from one client are refused for a while."""
import unittest

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


if __name__ == '__main__':
    unittest.main()
