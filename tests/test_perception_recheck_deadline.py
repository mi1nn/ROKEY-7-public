"""ObserveTarget deadline against this host's clock with its error bound (C-03, 2026-10-13).

The goal deadline comes from the executor's clock; the observation server may run on another PC. The
server treats the deadline as reached once now + its measured error bound passes it, and refuses a
window that cannot end before the deadline. Unmeasured clock: no bound is claimed (bound 0 is not
assumed) -> the deadline is compared as is and the result carries clock UNKNOWN (C-03 refuses it later).
"""
import unittest

from rescue_perception.recheck_server import deadline_margin_ns, goal_refusal

NOW = 1_760_000_000_000_000_000
RECORD = dict(state='SYNCED', measured_at_ns=NOW - 10_000_000_000, sampled_at_ns=NOW - 1_000_000_000,
              valid_until_ns=NOW + 3_600_000_000_000, error_bound_ns=300_000_000, drift_ppm=100.0)


def goal(**changes):
    values = dict(robot_id='robot_a', native_frame='map', point=(1.0, 2.0, 0.3), window_ns=1_500_000_000,
                  deadline_ns=NOW + 10_000_000_000)
    values.update(changes)
    return values


class DeadlineTests(unittest.TestCase):
    def test_margin_is_the_measured_bound(self):
        self.assertEqual(300_100_000, deadline_margin_ns(RECORD, NOW))   # + 1 s of 100 ppm drift
        self.assertEqual(0, deadline_margin_ns(dict(state='UNKNOWN'), NOW))

    def refusal(self, margin=300_000_000, **changes):
        g = goal(**changes)
        return goal_refusal(robot_id=g['robot_id'], native_frame=g['native_frame'], point=g['point'],
                            window_ns=g['window_ns'], deadline_ns=g['deadline_ns'], now_ns=NOW, margin_ns=margin,
                            expected_robot='robot_a', map_frame='map', max_window_ns=10_000_000_000)

    def test_window_that_cannot_end_before_the_deadline_is_refused(self):
        self.assertIsNone(self.refusal())
        self.assertEqual('DEADLINE_TOO_CLOSE', self.refusal(deadline_ns=NOW + 1_600_000_000))   # 1.5 s + 0.3 s > 1.6 s
        self.assertIsNone(self.refusal(deadline_ns=NOW + 1_600_000_000, margin=0))
        self.assertEqual('DEADLINE_PASSED', self.refusal(deadline_ns=NOW + 200_000_000))

    def test_other_refusals_unchanged(self):
        self.assertEqual('ROBOT_MISMATCH', self.refusal(robot_id='robot_b'))
        self.assertEqual('FRAME_MISMATCH', self.refusal(native_frame='odom'))
        self.assertEqual('INVALID_POINT', self.refusal(point=(float('nan'), 0.0, 0.0)))
        self.assertEqual('INVALID_WINDOW', self.refusal(window_ns=0))


if __name__ == '__main__':
    unittest.main()
