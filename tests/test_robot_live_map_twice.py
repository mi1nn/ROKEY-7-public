"""Both robots on a map frozen twice per run (OPS-154/159), no ROS.

1st freeze R0+1 (doll found): TB-5 goes on it; TB-6 keeps SLAM (revision R0, SLAM pose, R0 goals).
2nd freeze R0+2 (search done): TB-6 SLAM -> AMCL on it; TB-5, stopped at the doll, re-aligns on it.
A frozen grid has a revision only through its pair on /rescue/map/frozen_revision (same hash).
A robot takes a higher revision only with no task and odometry saying STOPPED; otherwise it
says why (MAP_REVISION_DEFERRED) and takes it as soon as it is stopped. While a revision is
pending a new goal is refused (FRAME_REVISION_MISMATCH 'map revision pending', D2); the task in
progress runs to its end. TB-5 re-aligning starts AMCL from its own last valid pose (same SLAM
frame) on <ns>/initialpose once its map topic carries the new grid; AMCL poses from before that
do not count. The same revision with another hash blocks the map; a lower revision is ignored.
"""
from types import SimpleNamespace as NS
import unittest

from rescue_contracts.occupancy import frozen_revision_text
from rescue_robot.executor import ContractError
from rescue_robot.live_parameters import frozen_revision
from rescue_robot.live_sources import Grid, SavedMapSources

from test_robot_live_map_lifecycle import COV, R0, SESSION, Runtime, grid_message, pose_message, settings

R1, R2 = R0 + 1, R0 + 2
G1, G2 = grid_message({5: 100}), grid_message({5: 100, 6: 100})


def revision_message(message, revision, session=SESSION):
    return NS(data=frozen_revision_text(map_session_id=session, map_revision=revision,
                                        occupancy_sha256=Grid(message).sha256))


class Twice(unittest.TestCase):
    def make(self, mode, robot='robot_a'):
        self.r = Runtime()
        self.robot = robot
        self.s = SavedMapSources(self.r, settings(mode, robot))
        self.reinitialized = []
        self.s.reinitialize = lambda robot, message: self.reinitialized.append((robot, message))
        return self.s

    def freeze(self, message, revision):
        self.s.receive_frozen(message)
        self.s.receive_frozen_revision(revision_message(message, revision))

    def pose(self, source='localization', x=0.1):
        self.r.now += 1_000_000
        message = pose_message(self.r.now - 500_000)
        message.pose.pose.position.x = x
        self.s.receive_pose(self.robot, message, source=source)

    def state(self):
        record = self.s.records()[self.robot]
        return record['localization_status'], record['map_revision']

    def logged(self, text):
        return any(text in line for line in self.r.errors + self.r.infos)

    def tb5_on_r1(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        self.freeze(G1, R1)
        s.receive_map('robot_b', G1)
        self.pose()
        self.assertEqual(('VALID', R1), self.state())
        return s

    # --- TB-6 ----------------------------------------------------------------------------------

    def test_tb6_keeps_slam_and_r0_after_the_first_freeze(self):
        s = self.make('SLAM_THEN_FROZEN')
        s.receive_map('robot_a', grid_message())
        self.pose(source='slam')
        self.freeze(G1, R1)
        s.receive_map('robot_a', grid_message({9: 100}))   # SLAM keeps publishing its growing map
        self.pose(source='slam')
        self.assertEqual(('VALID', R0), self.state())
        self.assertEqual('slam', s.basis('robot_a')['occupancy_key'])
        self.assertFalse(s.pending('robot_a'))

    def test_tb6_changes_to_amcl_on_the_second_freeze(self):
        s = self.make('SLAM_THEN_FROZEN')
        self.freeze(G1, R1)
        self.freeze(G2, R2)
        self.assertEqual(('UNKNOWN', R2), self.state())
        self.assertEqual(Grid(G2).sha256, s.basis('robot_a')['occupancy_key'])
        s.receive_map('robot_a', G2)
        self.pose(source='slam')                 # SLAM pose no longer counts
        self.assertEqual(('UNKNOWN', R2), self.state())
        self.pose()
        self.assertEqual(('VALID', R2), self.state())
        self.assertEqual([], self.reinitialized)   # TB-6 AMCL starts from the frozen pose (launch)

    def test_tb6_with_a_task_waits_for_it_to_end(self):
        s = self.make('SLAM_THEN_FROZEN')
        self.r.adapter.executor.active = object()
        self.freeze(G2, R2)
        self.assertEqual(R0, self.state()[1])
        self.assertTrue(self.logged('MAP_REVISION_DEFERRED'))
        self.r.adapter.executor.active = None
        s.tick()
        self.assertEqual(R2, self.state()[1])

    # --- TB-5 ----------------------------------------------------------------------------------

    def test_tb5_re_aligns_on_r2_from_its_own_pose_when_stopped(self):
        s = self.tb5_on_r1()
        self.pose(x=2.5)                          # at the doll
        self.freeze(G2, R2)
        self.assertEqual(('UNKNOWN', R2), self.state())
        self.assertNotIn('robot_b', s.maps)
        self.assertEqual([], self.reinitialized)  # not before its map topic carries R2
        s.receive_map('robot_b', G2)
        self.assertEqual(1, len(self.reinitialized))
        robot, start = self.reinitialized[0]
        self.assertEqual(('robot_b', 2.5, COV), (robot, start.pose.pose.position.x, list(start.pose.covariance)))
        s.receive_map('robot_b', G2)              # the same map again: one re-initialization only
        self.assertEqual(1, len(self.reinitialized))
        self.pose()
        self.assertEqual(('VALID', R2), self.state())

    def test_amcl_poses_from_before_the_re_initialization_do_not_count(self):
        s = self.tb5_on_r1()
        self.freeze(G2, R2)
        old = pose_message(self.r.now)            # stamped before the map switch
        s.receive_map('robot_b', G2)
        s.receive_pose('robot_b', old)
        self.assertEqual(('UNKNOWN', R2), self.state())

    def test_tb5_moving_keeps_r1_says_why_and_takes_r2_when_stopped(self):
        s = self.tb5_on_r1()
        self.r.adapter.executor.motion = 'MOVING'
        self.freeze(G2, R2)
        self.assertEqual(('VALID', R1), self.state())
        self.assertTrue(self.logged('MAP_REVISION_DEFERRED'))
        self.assertTrue(self.logged('not stopped'))
        self.assertTrue(s.pending('robot_b'))
        self.r.adapter.executor.motion = 'STOPPED'
        s.tick()
        self.assertEqual(('UNKNOWN', R2), self.state())
        self.assertFalse(s.pending('robot_b'))

    def test_map_state_says_the_new_revision_when_a_deferred_one_is_taken(self):
        """The relay copies the grid of the revision in this MapState: taking it on tick must publish it."""
        s = self.tb5_on_r1()
        said = []
        s.publisher = NS(robot='robot_b', on_pose=lambda *a: None,
                         publish_map_state=lambda **_: said.append(s.basis('robot_b')['map_revision']))
        self.r.adapter.executor.motion = 'MOVING'
        self.freeze(G2, R2)
        self.r.adapter.executor.motion = 'STOPPED'
        s.tick()
        self.assertEqual(R2, said[-1])

    def test_a_new_goal_is_refused_while_a_revision_is_pending(self):
        s = self.tb5_on_r1()
        s.check_new_goal('robot_b')               # nothing pending
        self.r.adapter.executor.active = object()
        self.freeze(G2, R2)
        s.check_new_goal('robot_b')               # the task in progress goes on (guard), not refused
        self.r.adapter.executor.active = None
        self.r.adapter.executor.motion = 'MOVING'
        s.tick()
        with self.assertRaises(ContractError) as caught:
            s.check_new_goal('robot_b')
        self.assertEqual('FRAME_REVISION_MISMATCH', caught.exception.code)
        self.assertIn('map revision pending', str(caught.exception))

    def test_tb5_first_map_waits_for_its_pair(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        s.receive_frozen(G1)
        s.receive_frozen_revision(revision_message(G2, R1))   # names another grid
        self.assertEqual('unfrozen', s.basis('robot_b')['occupancy_key'])
        s.receive_frozen_revision(revision_message(G1, R1))
        self.assertEqual((R1, Grid(G1).sha256), (s.basis('robot_b')['map_revision'], s.basis('robot_b')['occupancy_key']))

    def test_revision_first_then_grid_pairs_too(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        s.receive_frozen_revision(revision_message(G1, R1))
        self.assertEqual('unfrozen', s.basis('robot_b')['occupancy_key'])
        s.receive_frozen(G1)
        self.assertEqual(R1, s.basis('robot_b')['map_revision'])

    def test_tb5_started_after_both_freezes_goes_straight_to_r2(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        self.freeze(G2, R2)
        s.receive_map('robot_b', G2)
        self.pose()
        self.assertEqual(('VALID', R2), self.state())
        self.assertEqual([], self.reinitialized)   # first map: the launch start, not a re-alignment

    def test_a_lower_revision_arriving_late_is_ignored(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        self.freeze(G2, R2)
        self.freeze(G1, R1)
        self.assertEqual((R2, Grid(G2).sha256), (s.basis('robot_b')['map_revision'], s.basis('robot_b')['occupancy_key']))

    def test_same_revision_with_another_hash_blocks_the_map(self):
        s = self.tb5_on_r1()
        self.freeze(G2, R1)
        self.assertFalse(s.loaded('robot_b'))
        s.receive_map('robot_b', G1)
        self.assertFalse(s.loaded('robot_b'))
        self.assertTrue(self.logged('new run required'))

    def test_revisions_outside_this_run_are_refused(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        for message in (revision_message(G1, R0 + 3), revision_message(G1, R1, '33333333-3333-4333-8333-333333333333'),
                        NS(data='{"format": "rescue-map-frozen/2"}'), NS(data='R1')):
            with self.subTest(text=message.data):
                s.receive_frozen(G1)
                s.receive_frozen_revision(message)
                self.assertEqual('unfrozen', s.basis('robot_b')['occupancy_key'])

    def test_runtime_refuses_a_new_goal_while_pending_but_not_hold(self):
        from rescue_robot.task_site_runtime import SiteRuntime
        s = self.tb5_on_r1()
        self.r.adapter.executor.motion = 'MOVING'
        self.freeze(G2, R2)
        runtime = object.__new__(SiteRuntime)
        runtime.saved_maps, runtime.adapter = s, NS(robot_id='robot_b')
        self.assertIsNone(runtime.resolve_goal(NS(kind='HOLD', robot_id='robot_b')))
        with self.assertRaisesRegex(ContractError, 'map revision pending'):
            runtime.resolve_goal(NS(kind='RETURN_HOME', robot_id='robot_b', home_ref='home/robot_b'))

    # --- homes and settings --------------------------------------------------------------------

    def test_tb6_home_is_on_r2_and_tb5_has_no_launch_home(self):
        self.assertEqual(R2, frozen_revision(settings('SLAM_THEN_FROZEN')['robot_a']))
        self.assertEqual(R0, frozen_revision(settings('SAVED_MAP_AMCL')['robot_a']))
        with self.assertRaises(ContractError):   # its revision moves R1 -> R2: the coordinator sends home/robot_b
            frozen_revision(settings('FROZEN_MAP_AMCL', 'robot_b')['robot_b'])

    def test_settings_need_the_revision_topic_and_tb5_its_initialpose_topic(self):
        for mode, robot, change in (('SLAM_THEN_FROZEN', 'robot_a', dict(frozen_revision_topic='rescue/map/frozen_revision')),
                                    ('FROZEN_MAP_AMCL', 'robot_b', dict(initial_pose_topic=None)),
                                    ('FROZEN_MAP_AMCL', 'robot_b', dict(initial_pose_topic='initialpose'))):
            with self.subTest(mode=mode, change=change), self.assertRaises(ContractError):
                SavedMapSources(Runtime(), settings(mode, robot, **change))


if __name__ == '__main__':
    unittest.main()
