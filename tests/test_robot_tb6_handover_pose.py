"""TB-6 keeps its SLAM pose from the final freeze until its frozen map is on its map topic (no ROS).

Simulation runs 4 and 5 (2026-10-16, #396): TB-6's RobotPose stopped for 73-76 s during runbook 7.5. Run 4:
- 01:36:56 MAP_REVISION_ADOPTED robot_a RNone -> R2: from then on the executor ignored TB-6's SLAM pose (phase
  FROZEN), although slam_toolbox kept running;
- 01:37:25 tb6_slam stopped, 01:37:58 frozen_localization started, 01:38:05 relay R2, 01:38:08.88 VALID (AMCL).
So for the first 29 s nothing but the executor's own phase switch hid a live, valid pose. Meanwhile the
coordinator holds every new TB-5 command (TRAFFIC_SCOUT_POSE_UNKNOWN in _traffic_support).

Now a SLAM_THEN_FROZEN robot that took its final revision keeps using the SLAM TF pose and the SLAM map as
"loaded" until its map topic carries the frozen grid (the relay copies it only after SLAM stopped). The basis
is already the frozen revision: the frozen grid is the SLAM map of that moment, in the same frame, and the robot
was stopped with no task when it took it. When the frozen grid arrives the SLAM pose stops counting and the
pose that came from it is dropped (only AMCL after that counts). A SLAM pose that stops coming gets old and the
coordinator's age limit makes it unknown, as before; nothing is held up longer than the pose is fresh.

The tests marked expectedFailure fail on main.
"""
import unittest

from test_robot_live_map_twice import G1, G2, R1, R2, Twice
from test_robot_live_map_lifecycle import grid_message


class Handover(unittest.TestCase):
    # the map-twice fixture's helpers, without running its tests again here
    make, freeze, pose, state = Twice.make, Twice.freeze, Twice.pose, Twice.state

    def searching_tb6(self):
        s = self.make('SLAM_THEN_FROZEN')
        s.receive_map('robot_a', grid_message({9: 100}))   # SLAM map
        self.pose(source='slam')
        self.assertEqual(('VALID', 0), self.state())
        return s

    @unittest.expectedFailure
    def test_slam_pose_still_counts_after_the_final_freeze_until_the_frozen_map_arrives(self):
        s = self.searching_tb6()
        self.freeze(G1, R1)
        self.freeze(G2, R2)                      # TB-6 takes R2 at once (stopped, no task)
        self.pose(source='slam')                 # slam_toolbox still running (runbook 7.5 not done yet)
        self.assertEqual(('VALID', R2), self.state())
        s.receive_map('robot_a', grid_message({9: 100, 10: 100}))   # SLAM keeps publishing its map
        self.pose(source='slam')
        self.assertEqual(('VALID', R2), self.state())

    def test_after_the_frozen_map_arrives_only_amcl_counts(self):
        s = self.searching_tb6()
        self.freeze(G1, R1)
        self.freeze(G2, R2)
        self.pose(source='slam')
        s.receive_map('robot_a', G2)             # SLAM stopped, the relay copied the frozen grid
        self.assertEqual(('UNKNOWN', R2), self.state())   # the SLAM pose is dropped
        self.pose(source='slam')
        self.assertEqual(('UNKNOWN', R2), self.state())
        self.pose()                              # AMCL
        self.assertEqual(('VALID', R2), self.state())

    def test_the_first_freeze_changes_nothing_for_tb6(self):
        self.searching_tb6()
        self.freeze(G1, R1)
        self.pose(source='slam')
        self.assertEqual(('VALID', 0), self.state())


if __name__ == '__main__':
    unittest.main()
