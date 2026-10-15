"""Map lifecycle on the robot: TB-6 SLAM while searching, then frozen maps (no ROS).

User decision 2026-10-13 evening (scope Q-29 ⑦, OPS-113/114/117): no prior map. TB-6
(robot_a) runs SLAM while searching; its map frame is the common frame. The TB-6 map is frozen
twice (OPS-154, test_robot_live_map_twice): R0 + 1 for TB-5, R0 + 2 when TB-6 also changes to
AMCL. MapState basis: the run's session with revision R0 while searching, the frozen revision
after. A robot loads the frozen map only when its own map topic carries exactly the frozen grid
(same hash), and never goes back to SLAM. TB-5 (robot_b) has no map before the first freeze.
Nothing is made VALID that was not measured; a different grid for the same revision blocks the
map (a new run is needed).
"""
from copy import deepcopy
import threading
from types import SimpleNamespace as NS
import unittest

from rescue_contracts.occupancy import frozen_revision_text
from rescue_robot.executor import ContractError
from rescue_robot.live_parameters import frozen_revision, home_destinations
from rescue_robot.live_sources import Grid, SavedMapSources

SESSION = '22222222-2222-4222-8222-222222222222'
R0 = 0
COV = [0.0] * 36
COV[0] = COV[7] = 0.01
COV[35] = 0.01


def grid_message(cells=None, *, frame='map', width=20, height=10):
    data = [0] * (width * height)
    for index, value in (cells or {}).items():
        data[index] = value
    return NS(header=NS(frame_id=frame), info=NS(width=width, height=height, resolution=0.05,
              origin=NS(position=NS(x=-0.5, y=-0.25, z=0.), orientation=NS(x=0., y=0., z=0., w=1.))), data=data)


def pose_message(stamp_ns, *, frame='map', covariance=COV):
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec)),
              pose=NS(pose=NS(position=NS(x=0.1, y=0.0, z=0.0), orientation=NS(x=0., y=0., z=0., w=1.)),
                      covariance=list(covariance)))


def settings(mode, robot='robot_a', **changes):
    ns = '/robot6' if robot == 'robot_a' else '/robot5'
    c = dict(mode=mode, map_topic=ns + '/map', localization_topic=ns + '/amcl_pose', map_frame='map',
             occupancy_sha256='', map_session_id=SESSION, map_revision=R0, transform_revision=1,
             transform_xy_yaw=[0., 0., 0.], valid_from_ns=1, transform_valid_until_ns=10 ** 18,
             position_error_m=0.05, rotation_error_rad=0.05, max_position_variance_m2=0.25,
             max_yaw_variance_rad2=0.25, authority_id='mapping', clock_domain_id='rokey7-utc-field',
             frozen_topic='/rescue/map/frozen', frozen_revision_topic='/rescue/map/frozen_revision')
    if mode == 'SLAM_THEN_FROZEN':
        c['slam_localization_topic'] = ns + '/pose'
        # SLAM-phase pose = TF map -> base_link, covariance from the last SLAM pose (OPS-177)
        c.update(slam_tf_topic=ns + '/tf', slam_odom_frame='odom', slam_base_frame='base_link',
                 slam_covariance_max_age_ns=30_000_000_000)
    if mode == 'FROZEN_MAP_AMCL':
        c['initial_pose_topic'] = ns + '/initialpose'
        c['start_align_topic'] = ns + '/rescue_start_align'   # VALID only after ALIGNED (#317)
    c.update(changes)
    return {robot: c}


class Runtime:
    def __init__(self):
        self.now = 5_000_000_000
        self.errors, self.infos = [], []
        self.adapter = NS(lock=threading.RLock(), wall_clock_ns=lambda: self.now,
                          executor=NS(active=None, motion='STOPPED'))   # no task, odometry says stopped
        self.inputs = NS(bound=1_000_000, max_age=2_000_000_000)
        logger = NS(error=self.errors.append, warning=self.errors.append, info=self.infos.append)
        self.node = NS(get_logger=lambda: logger)


class Lifecycle(unittest.TestCase):
    def make(self, mode, robot='robot_a', aligned=True, **changes):
        """aligned: a FROZEN_MAP_AMCL robot has its start alignment verdict ALIGNED already (the map lifecycle
        is tested here; the gate itself in test_robot_start_align_gate)."""
        self.r = Runtime()
        self.robot = robot
        self.s = SavedMapSources(self.r, settings(mode, robot, **changes))
        if aligned and mode == 'FROZEN_MAP_AMCL':
            self.s.receive_start_align(robot, NS(data='{"status": "ALIGNED", "reason": ""}'))
        return self.s

    def pose(self, source='localization', **kw):
        self.r.now += 1_000_000
        self.s.receive_pose(self.robot, pose_message(self.r.now - 500_000, **kw), source=source)

    def record(self):
        return self.s.records()[self.robot]

    def freeze(self, message, revision=R0 + 1):
        """map_freeze: the grid and its revision pair (/rescue/map/frozen_revision)."""
        self.s.receive_frozen(message)
        self.s.receive_frozen_revision(NS(data=frozen_revision_text(
            map_session_id=SESSION, map_revision=revision, occupancy_sha256=Grid(message).sha256)))

    # --- TB-6 searching on its own SLAM map ---------------------------------------------------

    def test_searching_tb6_uses_the_live_slam_map_and_slam_pose_at_revision_r0(self):
        s = self.make('SLAM_THEN_FROZEN')
        s.receive_map('robot_a', grid_message())
        self.pose(source='slam')
        self.assertEqual(('VALID', SESSION, R0), (self.record()['localization_status'],
                                                  self.record()['map_session_id'], self.record()['map_revision']))
        self.assertTrue(s.loaded('robot_a'))
        self.assertEqual('slam', s.basis('robot_a')['occupancy_key'])
        # the SLAM map keeps changing while searching; that is not a fault
        s.receive_map('robot_a', grid_message({3: 100}))
        self.assertEqual('VALID', self.record()['localization_status'])

    def test_searching_map_is_not_a_target_footprint_map(self):
        """Automatic target moves check footprints in ``maps``: never the changing SLAM map."""
        s = self.make('SLAM_THEN_FROZEN')
        s.receive_map('robot_a', grid_message())
        self.assertEqual({}, s.maps)

    def test_amcl_pose_is_ignored_while_searching(self):
        s = self.make('SLAM_THEN_FROZEN')
        s.receive_map('robot_a', grid_message())
        self.pose(source='localization')
        self.assertEqual('UNKNOWN', self.record()['localization_status'])

    def test_slam_map_frame_must_be_the_map_frame(self):
        s = self.make('SLAM_THEN_FROZEN')
        s.receive_map('robot_a', grid_message(frame='odom'))
        self.pose(source='slam')
        self.assertFalse(s.loaded('robot_a'))
        self.assertEqual('UNKNOWN', self.record()['localization_status'])

    # --- freeze ---------------------------------------------------------------------------------

    def test_final_frozen_map_moves_tb6_to_r0_plus_2_once_its_map_topic_carries_that_grid(self):
        s = self.make('SLAM_THEN_FROZEN')
        frozen = grid_message({5: 100})
        s.receive_map('robot_a', grid_message())
        self.pose(source='slam')
        self.freeze(frozen, R0 + 2)
        # SLAM is still publishing the old map: the SLAM pose and map still count until the frozen grid
        # arrives (handover, simulation 2026-10-16 runs 4-5; test_robot_tb6_handover_pose)
        self.pose(source='slam')
        self.assertEqual(('VALID', R0 + 2), (self.record()['localization_status'], self.record()['map_revision']))
        self.assertTrue(s.loaded('robot_a'))
        self.assertNotIn('robot_a', s.maps)
        s.receive_map('robot_a', deepcopy(frozen))   # the frozen map republished on /robot6/map
        self.assertEqual(Grid(frozen).sha256, s.maps['robot_a'].sha256)
        self.pose(source='slam')                      # SLAM pose no longer counts
        self.assertEqual('UNKNOWN', self.record()['localization_status'])
        self.pose(source='localization')              # AMCL on the frozen map
        self.assertEqual(('VALID', SESSION, R0 + 2), (self.record()['localization_status'],
                                                      self.record()['map_session_id'], self.record()['map_revision']))
        self.assertEqual(Grid(frozen).sha256, s.basis('robot_a')['occupancy_key'])

    def test_map_arriving_before_the_frozen_record_is_checked_when_it_arrives(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        frozen = grid_message({5: 100})
        s.receive_map('robot_b', deepcopy(frozen))
        self.assertFalse(s.loaded('robot_b'))
        self.freeze(frozen)
        self.assertTrue(s.loaded('robot_b'))

    def test_tb5_has_no_map_and_no_valid_localization_before_the_freeze(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        s.receive_map('robot_b', grid_message())
        self.pose()
        self.assertEqual(('UNKNOWN', R0), (self.record()['localization_status'], self.record()['map_revision']))
        self.assertEqual('unfrozen', s.basis('robot_b')['occupancy_key'])
        self.freeze(grid_message())
        self.pose()
        self.assertEqual(('VALID', R0 + 1), (self.record()['localization_status'], self.record()['map_revision']))

    def test_tb5_amcl_with_zero_spread_rounding_below_zero_is_valid(self):
        # [E3 2026-10-15, #317] Nav2 Jazzy AMCL started with set_initial_pose (zero spread) publishes the yaw
        # variance as -5.7e-14: rounding, not a broken covariance
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        s.receive_map('robot_b', grid_message())
        self.freeze(grid_message())
        rounding = [0.0] * 36
        rounding[35] = -5.728750807065726e-14
        self.pose(covariance=rounding)
        self.assertEqual('VALID', self.record()['localization_status'])

    def test_a_negative_variance_beyond_rounding_is_still_refused(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        s.receive_map('robot_b', grid_message())
        self.freeze(grid_message())
        for index in (0, 7, 35):
            broken = list(COV)
            broken[index] = -1e-6
            with self.subTest(index=index):
                self.pose(covariance=broken)
                self.assertEqual('UNKNOWN', self.record()['localization_status'])

    def test_map_topic_with_another_grid_than_the_frozen_one_is_not_loaded(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        self.freeze(grid_message({5: 100}))
        s.receive_map('robot_b', grid_message({6: 100}))
        self.pose()
        self.assertEqual('UNKNOWN', self.record()['localization_status'])
        self.assertTrue(any('frozen' in e for e in self.r.errors), self.r.errors)

    def test_unconverged_amcl_is_not_valid(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        self.freeze(grid_message())
        s.receive_map('robot_b', grid_message())
        wide = list(COV)
        wide[0] = wide[7] = 1.0
        self.pose(covariance=wide)
        self.assertEqual('UNKNOWN', self.record()['localization_status'])

    def test_same_frozen_map_again_is_harmless(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        self.freeze(grid_message())
        s.receive_map('robot_b', grid_message())
        self.freeze(grid_message())          # a restarted freeze node republishes the saved grid
        self.assertTrue(s.loaded('robot_b'))

    def test_a_different_frozen_map_for_the_same_revision_blocks_the_map(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        self.freeze(grid_message())
        s.receive_map('robot_b', grid_message())
        self.freeze(grid_message({5: 100}))
        self.assertFalse(s.loaded('robot_b'))
        s.receive_map('robot_b', grid_message({5: 100}))
        s.receive_map('robot_b', grid_message())
        self.assertFalse(s.loaded('robot_b'))

    def test_frozen_map_in_another_frame_is_refused(self):
        s = self.make('FROZEN_MAP_AMCL', robot='robot_b')
        self.freeze(grid_message(frame='odom'))
        self.assertEqual('unfrozen', s.basis('robot_b')['occupancy_key'])

    def test_tb6_never_goes_back_to_slam(self):
        s = self.make('SLAM_THEN_FROZEN')
        self.freeze(grid_message(), R0 + 2)
        s.receive_map('robot_a', grid_message())           # the frozen grid on its map topic: SLAM is over
        s.receive_map('robot_a', grid_message({7: 100}))   # a late SLAM map
        self.pose(source='slam')
        self.assertEqual(('UNKNOWN', R0 + 2), (self.record()['localization_status'], self.record()['map_revision']))

    # --- saved maps (tests without robots) are unchanged ----------------------------------------

    def test_saved_map_mode_ignores_frozen_maps(self):
        g = grid_message()
        s = self.make('SAVED_MAP_AMCL', occupancy_sha256=Grid(g).sha256)
        self.freeze(grid_message({5: 100}))
        s.receive_map('robot_a', g)
        self.pose()
        self.assertEqual(('VALID', R0), (self.record()['localization_status'], self.record()['map_revision']))
        self.assertEqual(Grid(g).sha256, s.basis('robot_a')['occupancy_key'])

    def test_homes_are_on_the_frozen_revision(self):
        """RETURN_HOME (TB-6 docking approach) is used after the last freeze: R0 + 2 (OPS-154)."""
        for mode, expected in (('SLAM_THEN_FROZEN', R0 + 2), ('SAVED_MAP_AMCL', R0)):
            with self.subTest(mode=mode):
                m = settings(mode)['robot_a']
                home = home_destinations('robot_a', [1., 2., 0.], frame_id='map', map_session_id=SESSION,
                                         map_revision=frozen_revision(m), transform_revision=1)
                self.assertEqual(expected, home['home/robot_a']['map_revision'])

    # --- settings -------------------------------------------------------------------------------

    def test_settings_are_checked(self):
        bad = (('SLAM_THEN_FROZEN', dict(slam_localization_topic='pose')),
               ('SLAM_THEN_FROZEN', dict(occupancy_sha256='ab' * 32)),   # the freeze decides the hash
               ('FROZEN_MAP_AMCL', dict(occupancy_sha256='ab' * 32)),
               ('FROZEN_MAP_AMCL', dict(frozen_topic='rescue/map/frozen')),
               ('SAVED_MAP_AMCL', dict(occupancy_sha256='')),
               ('SLAM_ONLY', {}))
        for mode, change in bad:
            with self.subTest(mode=mode, change=change), self.assertRaises(ContractError):
                SavedMapSources(Runtime(), settings(mode, **change))
        with self.assertRaises(ContractError):
            SavedMapSources(Runtime(), {'robot_a': {k: v for k, v in settings('SLAM_THEN_FROZEN')['robot_a'].items()
                                                    if k != 'slam_localization_topic'}})


if __name__ == '__main__':
    unittest.main()
