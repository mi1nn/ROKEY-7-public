"""OPS-04/05: unusable odometry invalidates stop evidence without a made-up sample time."""
import unittest
from uuid import UUID

from rescue_robot.executor import ContractError, Executor, Policy
from rescue_robot.nav2_adapter import CommandAdapter


def uid(n):
    return str(UUID(int=n))


class Port:
    def send_goal(self, command_id, pose):
        raise AssertionError('no goal expected')

    def cancel_goal(self, command_id):
        raise AssertionError('no cancel expected')


class StopEvidenceInvalidationTests(unittest.TestCase):
    def setUp(self):
        policy = Policy('test-only', 1, 3, .2, .11, .5, .01, .01)
        self.e = Executor('robot_a', uid(1), uid(2), policy, allowed_robot_ids=('robot_a', 'robot_b'))
        for t in (0, .1, .2):
            self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
        self.assertEqual(self.e.motion, 'STOPPED')

    def test_executor_invalidation_keeps_last_sample_and_clock_rules(self):
        self.e.invalidate_stop_evidence(now=.3)
        self.assertEqual((self.e.motion, self.e.stop_start, self.e.last_sample), ('STOP_UNCONFIRMED', None, .2))
        with self.assertRaises(ContractError):
            self.e.invalidate_stop_evidence(now=.1)  # reversed monotonic clock still latches
        self.assertEqual(self.e.latch, 'FAULT_LATCHED')

    def test_adapter_never_passes_a_made_up_sample_time(self):
        calls = []
        original = self.e.observe_motion
        self.e.observe_motion = lambda **kw: calls.append(kw) or original(**kw)
        now = [.3]
        adapter = CommandAdapter(self.e, Port(), guards=lambda task: True, clock_error_bound_s=.5,
                                 clock=lambda: now[0], wall_clock_ns=lambda: 1700000000_000000000)
        adapter.on_odom(stamp_sec=0, stamp_nanosec=0, linear=(0, 0, 0), angular=(0, 0, 0))
        adapter.on_odom(stamp_sec=1690000000, stamp_nanosec=0, linear=(0, 0, 0), angular=(0, 0, 0))
        self.assertEqual(calls, [])
        self.assertEqual(self.e.motion, 'STOP_UNCONFIRMED')
        self.assertEqual(sum(e[0] == 'ODOM_UNUSABLE' for e in adapter.events), 2)


if __name__ == '__main__':
    unittest.main()
