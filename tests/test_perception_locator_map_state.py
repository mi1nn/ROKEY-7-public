"""target_locator takes the map session/revision from TB-6 MapState, not from a launch value (OPS-115, 2026-10-13).

TB-6 alone runs SLAM; the map is fixed when the target is confirmed and both robots then localize
on it, so the revision changes during a run. A location carries the latest VALID MapState revision,
and only when the observation was captured (minus its clock bound) after that revision's MapState was
first VALID; otherwise (or with no VALID MapState) nothing is published. An observation from just
before the fix never gets the new revision.
"""
from types import SimpleNamespace as NS
import unittest

from rescue_perception.target_location import LocationRefused, MapRevisionTracker

S1, S2 = '22222222-2222-4222-8222-222222222222', '33333333-3333-4333-8333-333333333333'
T0 = 1_760_000_000_000_000_000
SECOND = 1_000_000_000


def state(revision, at, quality='VALID', session=S1, robot='robot_a', frame='map'):
    return NS(robot_id=robot, map_session_id=session, map_revision=revision, generated_at_ns=at,
              localization_quality=quality, current_native_map_frame=frame)


def tracker():
    return MapRevisionTracker(robot_id='robot_a', map_frame='map')


class MapRevisionTests(unittest.TestCase):
    def assertRefused(self, reason, t, observed, bound=5_000_000):
        with self.assertRaises(LocationRefused) as caught:
            t.basis_for(observed, bound)
        self.assertEqual(reason, caught.exception.reason)

    def test_no_valid_map_state_publishes_nothing(self):
        t = tracker()
        self.assertRefused('NO_VALID_MAP_STATE', t, T0)
        t.update(state(0, T0, quality='UNKNOWN'))
        self.assertRefused('NO_VALID_MAP_STATE', t, T0 + SECOND)

    def test_observation_after_the_revision_gets_it(self):
        t = tracker()
        t.update(state(0, T0))
        self.assertEqual((S1, 0), t.basis_for(T0 + SECOND, 5_000_000))

    def test_observation_from_before_the_fix_never_gets_the_new_revision(self):
        t = tracker()
        t.update(state(0, T0))
        t.update(state(1, T0 + 10 * SECOND, session=S2))   # map fixed: new session/revision
        self.assertRefused('CAPTURED_BEFORE_MAP_REVISION', t, T0 + 9 * SECOND)
        self.assertRefused('CAPTURED_BEFORE_MAP_REVISION', t, T0 + 10 * SECOND + 4_000_000)   # within the bound
        self.assertEqual((S2, 1), t.basis_for(T0 + 11 * SECOND, 5_000_000))

    def test_first_valid_time_of_a_revision_does_not_move(self):
        t = tracker()
        t.update(state(0, T0))
        t.update(state(0, T0 + 5 * SECOND))          # the same revision published again later
        self.assertEqual((S1, 0), t.basis_for(T0 + 2 * SECOND, 5_000_000))

    def test_lost_localization_withdraws_the_basis(self):
        t = tracker()
        t.update(state(0, T0))
        t.update(state(0, T0 + SECOND, quality='UNKNOWN'))
        self.assertRefused('NO_VALID_MAP_STATE', t, T0 + 2 * SECOND)

    def test_regained_localization_counts_from_the_regain(self):
        # #207 review REV-207-01: a capture while localization was lost must not get the revision just
        # because the same revision comes back VALID before the observation is processed.
        t = tracker()
        t.update(state(0, T0))
        t.update(state(0, T0 + 5 * SECOND, quality='UNKNOWN'))   # lost
        t.update(state(0, T0 + 8 * SECOND))                      # regained, same revision
        self.assertRefused('CAPTURED_BEFORE_MAP_REVISION', t, T0 + 6 * SECOND)   # captured while lost
        self.assertEqual((S1, 0), t.basis_for(T0 + 9 * SECOND, 5_000_000))

    def test_returning_to_an_earlier_revision_counts_from_the_return(self):
        t = tracker()
        t.update(state(0, T0))
        t.update(state(1, T0 + 5 * SECOND))
        t.update(state(0, T0 + 8 * SECOND))
        self.assertRefused('CAPTURED_BEFORE_MAP_REVISION', t, T0 + 6 * SECOND)
        self.assertEqual((S1, 0), t.basis_for(T0 + 9 * SECOND, 5_000_000))

    def test_other_robot_or_frame_is_refused(self):
        t = tracker()
        with self.assertRaises(ValueError):
            t.update(state(0, T0, robot='robot_b'))
        t.update(state(0, T0, frame='odom'))
        self.assertRefused('MAP_FRAME_MISMATCH', t, T0 + SECOND)


if __name__ == '__main__':
    unittest.main()
