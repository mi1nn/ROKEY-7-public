"""TB-5 localization is VALID only after its start alignment says ALIGNED (재범 decision 2026-10-15, method B, (나)).

AMCL starts TB-5 at the measured start with zero spread [Nav2 jazzy amcl_node.cpp:283-295], so its small
covariance says nothing about the measured start being right. rescue_robot.start_align (#317) latches its
verdict on <ns>/rescue_start_align (JSON); the executor counts TB-5's AMCL pose as VALID only while the last
verdict there is ALIGNED. The shared covariance thresholds stay as they are (TB-6 SLAM covariance not measured).
TB-6 (SLAM_THEN_FROZEN) has no start alignment and is not gated.
"""
import json
from types import SimpleNamespace as NS
import unittest

from rescue_robot.executor import ContractError
from rescue_robot.live_sources import SavedMapSources

from test_robot_live_map_lifecycle import Runtime, grid_message, pose_message, settings
from test_robot_live_map_lifecycle import Lifecycle


def verdict(status, reason=''):
    return NS(data=json.dumps({'status': status, 'reason': reason}))


class Gate(unittest.TestCase):
    def tb5(self):
        case = Lifecycle()
        case.make('FROZEN_MAP_AMCL', robot='robot_b', aligned=False)
        case.s.receive_map('robot_b', grid_message())
        case.freeze(grid_message())
        return case

    def test_no_verdict_yet_is_not_valid(self):
        case = self.tb5()
        case.pose()
        self.assertEqual('UNKNOWN', case.record()['localization_status'])

    def test_aligned_makes_the_same_pose_valid(self):
        case = self.tb5()
        case.pose()
        case.s.receive_start_align('robot_b', verdict('ALIGNED'))
        self.assertEqual('VALID', case.record()['localization_status'])
        self.assertTrue(any('START_ALIGN robot_b ALIGNED' in line for line in case.r.infos))

    def test_waiting_or_failed_is_not_valid(self):
        for status, reason in (('WAITING', 'START_ALIGN_DOCKED'), ('ALIGNING', ''),
                               ('FAILED', 'START_ALIGN_JUMP_TOO_LARGE')):
            case = self.tb5()
            case.s.receive_start_align('robot_b', verdict('ALIGNED'))
            case.s.receive_start_align('robot_b', verdict(status, reason))   # e.g. ~/restart
            case.pose()
            with self.subTest(status=status):
                self.assertEqual('UNKNOWN', case.record()['localization_status'])
                self.assertTrue(any(f'START_ALIGN robot_b {status} {reason}'.strip() in line
                                    for line in case.r.errors + case.r.infos))

    def test_an_unreadable_verdict_is_not_aligned(self):
        for text in ('not json', '{"reason": ""}', '[]', '{"status": 1}'):
            case = self.tb5()
            case.s.receive_start_align('robot_b', verdict('ALIGNED'))
            case.s.receive_start_align('robot_b', NS(data=text))
            case.pose()
            with self.subTest(text=text):
                self.assertEqual('UNKNOWN', case.record()['localization_status'])

    def test_map_state_is_republished_when_the_verdict_changes(self):
        case = self.tb5()
        published = []
        case.s.publisher = NS(robot='robot_b', publish_map_state=lambda **kw: published.append(kw),
                              on_pose=lambda *a: None)
        case.pose()
        case.s.receive_start_align('robot_b', verdict('ALIGNED'))
        self.assertEqual({'localization_valid': True}, published[-1])

    def test_tb6_is_not_gated(self):
        case = Lifecycle()
        s = case.make('SLAM_THEN_FROZEN')
        s.receive_map('robot_a', grid_message())
        case.pose(source='slam')
        self.assertEqual('VALID', case.record()['localization_status'])
        with self.assertRaises(ContractError):
            s.receive_start_align('robot_a', verdict('ALIGNED'))   # no start alignment for TB-6

    def test_frozen_map_robot_needs_its_start_align_topic(self):
        for value in (None, 'rescue_start_align'):
            with self.subTest(value=value), self.assertRaises(ContractError):
                SavedMapSources(Runtime(), settings('FROZEN_MAP_AMCL', 'robot_b', start_align_topic=value))


if __name__ == '__main__':
    unittest.main()
