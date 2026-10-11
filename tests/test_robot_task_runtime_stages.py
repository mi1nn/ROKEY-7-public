"""Exercise the #85 requested order through the installed runtime, no robot."""
import unittest
from rescue_robot.task_runtime_demo import run_e2


class OrderedRuntimeTests(unittest.TestCase):
    def test_common_stage_accepts_search_reobserve_hold_and_renews_active_search(self):
        report = run_e2(stage='common')
        self.assertEqual([s['kind'] for s in report['scenarios']],
                         ['SEARCH_AREA', 'REOBSERVE_TARGET', 'HOLD'])
        renewal = report['renewal']
        self.assertEqual(renewal['status'], 'PASSED')
        self.assertGreater(renewal['active_for_s'], renewal['initial_duration_s'])
        self.assertGreater(renewal['last_sequence'], 0)
        self.assertTrue(renewal['duplicate_rejected'])
        self.assertFalse(report['watchdog_process_created'])

    def test_common_cancel_waits_for_stop_and_no_odom_is_not_success(self):
        report = run_e2(stage='common')
        self.assertEqual(report['cancel'], 'CANCELED_STOP_CONFIRMED')
        self.assertEqual(report['no_stop_evidence'], 'STOP_UNCONFIRMED')
        self.assertEqual(report['late_renewal'], 'NO_REVIVAL')

    def test_fallback_stage_only_runs_segment_and_preserves_scope(self):
        report = run_e2(stage='fallback')
        self.assertEqual([s['kind'] for s in report['scenarios']], ['MOVE_SEGMENT'])
        self.assertEqual(report['scenarios'][0]['scope'], 'SEGMENT')
        self.assertEqual(report['lease_expiry'], 'EXPIRED_STOP_CONFIRMED')

    def test_unknown_stage_rejected_before_runtime_creation(self):
        with self.assertRaises(ValueError):
            run_e2(stage='follow')


if __name__ == '__main__':
    unittest.main()
