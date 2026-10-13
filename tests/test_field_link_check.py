"""scripts/field/link_check.py without ROS: gap statistics, topic choice from the profile, verdicts (E2).

The ROS part (subscribe only, nothing published) is exercised by tests/ros/e3_link_check_probe.py.
"""
import importlib.util
from pathlib import Path
import unittest

REPO = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('link_check', REPO / 'scripts' / 'field' / 'link_check.py')
link = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(link)

S = 1_000_000_000


class GapStatsTests(unittest.TestCase):
    def test_steady_stream(self):
        arrivals = [i * S // 10 for i in range(1, 100)]  # 10 Hz from 0.1 s to 9.9 s in a 0..10 s window
        stats = link.gap_stats(arrivals, 0, 10 * S)
        self.assertEqual((99, 0.1, 0, 0), (stats['count'], round(stats['mean_period_s'], 6),
                                           stats['gaps_over_0_5_s'], stats['gaps_over_1_0_s']))
        self.assertAlmostEqual(0.1, stats['max_gap_s'])

    def test_pauses_are_counted_by_threshold(self):
        arrivals = [0, S // 10, S // 10 + 7 * S // 10, S // 10 + 7 * S // 10 + 12 * S // 10]  # gaps 0.1, 0.7, 1.2
        stats = link.gap_stats(arrivals, 0, arrivals[-1])
        self.assertEqual((4, 1, 1), (stats['count'], stats['gaps_over_0_5_s'], stats['gaps_over_1_0_s']))
        self.assertAlmostEqual(1.2, stats['max_gap_s'])
        self.assertAlmostEqual(2.0 / 3, stats['mean_period_s'])

    def test_silence_at_the_window_edges_counts(self):
        stats = link.gap_stats([3 * S], 0, 5 * S)  # 3 s before the only message, 2 s after it
        self.assertEqual((1, None, 3.0, 2, 2), (stats['count'], stats['mean_period_s'], stats['max_gap_s'],
                                               stats['gaps_over_0_5_s'], stats['gaps_over_1_0_s']))

    def test_nothing_received_is_one_gap_of_the_whole_window(self):
        stats = link.gap_stats([], 0, 4 * S)
        self.assertEqual((0, None, 4.0, 1, 1), (stats['count'], stats['mean_period_s'], stats['max_gap_s'],
                                               stats['gaps_over_0_5_s'], stats['gaps_over_1_0_s']))

    def test_exactly_on_a_threshold_is_not_over_it(self):
        stats = link.gap_stats([0, S // 2, S // 2 + S], 0, S // 2 + S)
        self.assertEqual((0, 0), (stats['gaps_over_0_5_s'] - 1, stats['gaps_over_1_0_s']))  # only 1.0 > 0.5

    def test_stamp_delay_is_reference_only(self):
        delay = link.stamp_delay([(10 * S, 10 * S - S // 10), (11 * S, 11 * S - S // 5)])
        self.assertAlmostEqual(0.15, delay['mean_s'])
        self.assertAlmostEqual(0.2, delay['max_s'])
        self.assertEqual('참고', delay['basis'])
        self.assertIsNone(link.stamp_delay([]))


class TopicChoiceTests(unittest.TestCase):
    def setUp(self):
        from rescue_bringup.profile import load_deployment
        self.deployment = load_deployment(str(REPO / 'config' / 'deployment.v2.example.json'))

    def topics(self, robot='robot_a', overrides=()):
        return {t.role: t for t in link.select_topics(self.deployment, robot, dict(overrides))}

    def test_robot_topics_come_from_the_profile(self):
        t = self.topics()
        self.assertEqual(('/robot_a/odom', 'nav_msgs/msg/Odometry', 0.5), (t['odom'].topic, t['odom'].type, t['odom'].limit_s))
        self.assertEqual(('/robot_a/scan', 0.5), (t['scan'].topic, t['scan'].limit_s))
        self.assertEqual(('/robot_a/tf', 1.0), (t['tf'].topic, t['tf'].limit_s))
        self.assertEqual(('/rescue/robots/robot_a/status', None), (t['robot_status'].topic, t['robot_status'].limit_s))
        self.assertEqual('/rescue/cameras/cam_fixed/status', t['camera_status:cam_fixed'].topic)

    def test_heartbeats_follow_the_launch_convention_with_the_watchdog_limit(self):
        t = self.topics()
        self.assertEqual(('/robot_a/rescue/executor_heartbeat', 'std_msgs/msg/Header', 0.5),
                         (t['executor_heartbeat'].topic, t['executor_heartbeat'].type, t['executor_heartbeat'].limit_s))
        self.assertEqual('/robot_a/rescue/watchdog_heartbeat', t['watchdog_heartbeat'].topic)
        self.assertIn('nav2_watchdog.launch.py', t['executor_heartbeat'].source)

    def test_overrides_replace_a_topic_name_and_say_so(self):
        t = self.topics(overrides={'executor_heartbeat': '/robot6/rescue/executor_heartbeat'})
        self.assertEqual(('/robot6/rescue/executor_heartbeat', 'command line'),
                         (t['executor_heartbeat'].topic, t['executor_heartbeat'].source))

    def test_unknown_robot_or_role_is_refused(self):
        with self.assertRaises(link.LinkCheckError):
            self.topics('robot_x')
        with self.assertRaises(link.LinkCheckError):
            self.topics(overrides={'nope': '/x'})


class VerdictTests(unittest.TestCase):
    def spec(self, limit):
        return link.TopicSpec('odom', '/robot_a/odom', 'nav_msgs/msg/Odometry', limit, 'profile')

    def test_verdicts(self):
        ok = dict(count=10, max_gap_s=0.4)
        self.assertEqual('PASSED', link.verdict(self.spec(0.5), ok))
        self.assertEqual('FAILED', link.verdict(self.spec(0.5), dict(count=10, max_gap_s=0.6)))
        self.assertEqual('FAILED', link.verdict(self.spec(None), dict(count=0, max_gap_s=5.0)))
        self.assertEqual('PASSED', link.verdict(self.spec(None), dict(count=3, max_gap_s=3.0)))  # no limit: measured only

    def test_exit_code_follows_preflight(self):
        self.assertEqual(0, link.summarize(['PASSED', 'PASSED']))
        self.assertEqual(2, link.summarize(['PASSED', 'FAILED', 'NOT_RUN']))
        self.assertEqual(3, link.summarize(['PASSED', 'NOT_RUN']))
        self.assertEqual(3, link.summarize([]))


if __name__ == '__main__':
    unittest.main()
