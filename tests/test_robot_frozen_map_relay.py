"""frozen_map_relay copies the frozen grid of this robot's own revision only (OPS-154/159), no ROS.

The map is frozen twice (R0+1, R0+2). The relay keeps every frozen grid it received with its
revision (pair on /rescue/map/frozen_revision, same hash) and copies to <ns>/map the one whose
revision this robot's executor publishes in its MapState (/rescue/robots/<id>/map_state). So
TB-6, still on SLAM at R0 after the first freeze, gets nothing (its SLAM map stays on
/robot6/map); TB-5 stays on R0+1 until its executor takes R0+2 (stopped, no task). A different
grid for a revision it already has blocks copying (a new run is needed).
"""
import unittest

from rescue_robot.frozen_map_relay import RelayCore

from test_robot_live_map_twice import G1, G2, R0, R1, R2, revision_message


def sha(message):
    from rescue_robot.live_sources import Grid
    return Grid(message).sha256


class Relay(unittest.TestCase):
    def setUp(self):
        self.core = RelayCore()

    def give(self, message, revision):
        self.core.on_grid(sha(message), message)
        self.core.on_revision(revision_message(message, revision).data)

    def test_nothing_before_its_executor_names_a_frozen_revision(self):
        self.give(G1, R1)
        self.assertIsNone(self.core.wanted())
        self.core.on_map_state(R0)               # TB-6 searching: its SLAM map stays on /robot6/map
        self.assertIsNone(self.core.wanted())

    def test_follows_its_own_revision(self):
        self.give(G1, R1)
        self.core.on_map_state(R1)
        self.assertIs(G1, self.core.wanted())
        self.give(G2, R2)                        # R2 frozen while TB-5 still works on R1
        self.assertIs(G1, self.core.wanted())
        self.core.on_map_state(R2)               # its executor took R2
        self.assertIs(G2, self.core.wanted())

    def test_grid_and_pair_in_either_order(self):
        self.core.on_map_state(R1)
        self.core.on_revision(revision_message(G1, R1).data)
        self.assertIsNone(self.core.wanted())
        self.core.on_grid(sha(G1), G1)
        self.assertIs(G1, self.core.wanted())

    def test_another_grid_for_a_revision_it_has_blocks(self):
        self.give(G1, R1)
        self.core.on_map_state(R1)
        self.give(G2, R1)
        self.assertIsNone(self.core.wanted())
        self.assertIn('new run', self.core.blocked)

    def test_a_bad_pair_is_refused(self):
        self.core.on_grid(sha(G1), G1)
        with self.assertRaises(ValueError):
            self.core.on_revision('{"format": "rescue-map-frozen/2"}')
        self.core.on_map_state(R1)
        self.assertIsNone(self.core.wanted())


if __name__ == '__main__':
    unittest.main()
