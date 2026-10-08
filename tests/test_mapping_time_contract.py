"""PR6 regressions: common absolute-time boundaries preserve module errors."""
import unittest
from rescue_contracts import MAX_TIMESTAMP_NS
from rescue_mapping import MappingError, MappingRegistry
from test_mapping import observation, transform, alignment, uid


class MappingTimeContractTests(unittest.TestCase):
    def assertInvalid(self, operation):
        with self.assertRaises(MappingError) as caught:
            operation()
        self.assertEqual('INVALID_INPUT', caught.exception.code)
        self.assertEqual('rescue_mapping.INVALID_INPUT', caught.exception.detail_code)

    def test_observation_timestamps_are_positive_int64_not_unbounded_ints(self):
        self.assertInvalid(lambda: observation(observed_at_ns=MAX_TIMESTAMP_NS + 1,
                                               valid_until_ns=MAX_TIMESTAMP_NS + 2))
        self.assertInvalid(lambda: observation(valid_until_ns=MAX_TIMESTAMP_NS + 1))
        self.assertEqual(MAX_TIMESTAMP_NS, observation(valid_until_ns=MAX_TIMESTAMP_NS).valid_until_ns)

    def test_tf_query_and_sample_timestamps_share_common_range(self):
        self.assertInvalid(lambda: transform(query_time_ns=MAX_TIMESTAMP_NS + 1,
                                              sample_times_ns=(MAX_TIMESTAMP_NS + 1,)))
        self.assertInvalid(lambda: transform(mode='INTERPOLATED_VALIDATED',
                                              sample_times_ns=(1, MAX_TIMESTAMP_NS + 1)))
        self.assertEqual(MAX_TIMESTAMP_NS, transform(query_time_ns=MAX_TIMESTAMP_NS,
                          sample_times_ns=(MAX_TIMESTAMP_NS,)).query_time_ns)

    def test_alignment_timestamps_are_int64_while_revisions_remain_uint64(self):
        self.assertInvalid(lambda: alignment(valid_from_ns=MAX_TIMESTAMP_NS + 1,
                                              valid_until_ns=MAX_TIMESTAMP_NS + 2))
        self.assertInvalid(lambda: alignment(valid_until_ns=MAX_TIMESTAMP_NS + 1))
        self.assertEqual(2**64-1, alignment(transform_revision=2**64-1).transform_revision)

    def test_invalid_now_does_not_poison_registry_clock_or_existing_goal(self):
        registry = MappingRegistry(clock_domain_id='test-clock', alignment_authority='mapping',
                                   max_position_error_m=.2, max_rotation_error_rad=.1)
        for robot in ('a', 'b'):
            registry.start_session(robot, 'map', uid(robot+'-session'))
            registry.set_alignment(alignment(robot))
        original = observation()
        registry.project(original, transform(), now_ns=100)
        goal = registry.capture_goal_snapshot((original.observation_id,), 'b', (1, 2, 3), now_ns=100)
        rejected = registry.validate_goal_snapshot(goal, now_ns=MAX_TIMESTAMP_NS + 1)
        self.assertFalse(rejected.coordinates_valid)
        self.assertIn('INVALID_INPUT', rejected.reasons)
        self.assertTrue(registry.validate_goal_snapshot(goal, now_ns=101).coordinates_valid)
