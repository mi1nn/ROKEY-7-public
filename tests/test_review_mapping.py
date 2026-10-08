"""PR6 review regressions for projection meaning and public errors."""
import unittest
from rescue_mapping import MappingError, MappingRegistry, RigidTransform
from test_mapping import alignment, observation, transform, uid


class MappingReviewTests(unittest.TestCase):
    def setUp(self):
        self.registry = MappingRegistry(clock_domain_id='test-clock', alignment_authority='mapping',
                                        max_position_error_m=.2, max_rotation_error_rad=.1)
        for robot in ('a', 'b'):
            self.registry.start_session(robot, 'map', uid(robot + '-session'))
            self.registry.set_alignment(alignment(robot))
        self.original = observation()
        self.projected = self.registry.project(self.original, transform(), now_ns=100)
        self.goal = self.registry.capture_goal_snapshot((self.original.observation_id,), 'b', (3, 2, 3), now_ns=100)

    def test_alignment_only_is_stale_and_can_reproject_original(self):
        self.registry.set_alignment(alignment(transform_revision=2, transform=RigidTransform.planar(1, 0, 0)))
        stale = self.registry.projection(self.original.observation_id, now_ns=101)
        self.assertEqual('STALE_DERIVATION', stale.status)
        self.assertEqual('ALIGNMENT_CHANGED', stale.reason)
        self.assertFalse(self.registry.validate_goal_snapshot(self.goal, now_ns=101).coordinates_valid)
        current = self.registry.project(self.original, transform(), now_ns=101)
        self.assertEqual('VALID', current.status)
        self.assertIs(self.original, current.original)
        self.assertEqual((4., 2., 3.), current.mission_point_xyz)

    def test_stale_derivation_escalates_to_reobservation_after_map_correction(self):
        self.registry.set_alignment(alignment(transform_revision=2))
        self.registry.advance_map_revision('a', uid('a-session'), 1)
        self.assertEqual('NEEDS_REOBSERVATION', self.registry.projection(self.original.observation_id, now_ns=101).status)

    def test_stale_derivation_escalates_after_alignment_invalidation(self):
        self.registry.set_alignment(alignment(transform_revision=2))
        self.registry.invalidate_alignment('a')
        self.assertEqual('NEEDS_REOBSERVATION', self.registry.projection(self.original.observation_id, now_ns=101).status)

    def test_stale_derivation_escalates_after_new_session(self):
        self.registry.set_alignment(alignment(transform_revision=2))
        self.registry.start_session('a', 'map', uid('new-a-session'))
        self.assertEqual('NEEDS_REOBSERVATION', self.registry.projection(self.original.observation_id, now_ns=101).status)

    def test_invalid_alignment_requires_reobservation(self):
        self.registry.set_alignment(alignment(transform_revision=2, status='INVALID'))
        self.assertEqual('NEEDS_REOBSERVATION', self.registry.projection(self.original.observation_id, now_ns=101).status)

    def test_version_mismatch_uses_same_projection_status_vocabulary(self):
        # Simulate an alignment snapshot installed without the notification path.
        self.registry._alignments['a'] = alignment(transform_revision=2)
        self.assertEqual('STALE_DERIVATION', self.registry.projection(self.original.observation_id, now_ns=101).status)
        self.registry._alignments['a'] = alignment(map_revision=1, transform_revision=3)
        self.assertEqual('NEEDS_REOBSERVATION', self.registry.projection(self.original.observation_id, now_ns=101).status)

    def test_public_mapping_error_has_canonical_and_namespaced_detail(self):
        error = MappingError('OBSERVATION_EXPIRED', 'fixture')
        self.assertEqual('STALE_OBSERVATION', error.code)
        self.assertEqual('rescue_mapping.OBSERVATION_EXPIRED', error.detail_code)
        self.assertEqual('rescue_mapping', error.component)
