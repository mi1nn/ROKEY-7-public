"""Rescue judgement from one TB-6 recheck (contracts C-11 rows 7-9 and 11; E2).

``verdict.judge`` returns ('RESCUE_CONFIRMED', 'CONFIRMED') only when the recheck is the linked SCOUT task of
the target, its frame_ref is the coordinator's map basis with alignment VALID, the candidate and the recheck
both declare a time basis on the same clock domain with an error sum within the policy limit, the sensor
window started after the candidate by more than that sum, and camera ABSENT, LiDAR CLEAR, coverage VALID.
Anything else is ('RECHECK_NEEDED', <first reason>). An operator recheck (row 11) needs this coordinator's own
clock basis, which is not measured here: it is CANDIDATE_TIME_BASIS_UNUSABLE.
"""
from types import SimpleNamespace as NS
import unittest

from rescue_coordination import verdict

MAP = '22222222-2222-4222-8222-222222222222'
BASIS = (MAP, 0, 1)
DOMAIN = 'site-clock'
T0 = 1_800_000_000_000_000_000
MS = 1_000_000
TASK, TARGET, SCOUT = 'task-1', 'target-1', 'robot_a'


def time_(**changes):
    values = dict(observed_at_ns=T0 + 2000 * MS, stamp_semantics='CAPTURE', clock_domain_id=DOMAIN,
                  clock_state='SYNCED', has_clock_error_bound=True, clock_error_bound_ns=5 * MS)
    return NS(**(values | changes))


def obs(**changes):
    frame = NS(map_session_id=MAP, map_revision=0, transform_revision=1, alignment_status='VALID',
               mission_frame='map')
    values = dict(task_id=TASK, target_id=TARGET, robot_id=SCOUT, time=time_(), frame_ref=frame,
                  camera_state='ABSENT', lidar_state='CLEAR', coverage_state='VALID',
                  sensor_window_start_ns=T0 + 1000 * MS)
    return NS(**(values | changes))


def camera(**changes):
    values = dict(source='CAMERA', at_ns=T0, clock_domain_id=DOMAIN, clock_error_bound_ns=5 * MS, declared=True)
    return verdict.Reference(**(values | changes))


def judge(observation=None, reference=None, **changes):
    values = dict(expected_task_id=TASK, target_id=TARGET, scout_id=SCOUT, basis=BASIS, mission_frame='map',
                  max_clock_error_ns=20 * MS)
    return verdict.judge(observation or obs(), reference or camera(), **(values | changes))


class JudgeTests(unittest.TestCase):
    def test_everything_holds_is_rescue_confirmed(self):
        self.assertEqual(('RESCUE_CONFIRMED', 'CONFIRMED'), judge())

    def needed(self, reason, **kwargs):
        self.assertEqual(('RECHECK_NEEDED', reason), judge(**kwargs))

    def test_not_the_linked_scout_task(self):
        for o in (obs(task_id='other'), obs(target_id='other'), obs(robot_id='robot_b')):
            with self.subTest(o=o):
                self.needed('CANDIDATE_NOT_LINKED', observation=o)

    def test_frame_not_on_the_basis_or_not_aligned(self):
        for frame in (dict(map_revision=1), dict(transform_revision=2), dict(alignment_status='DEGRADED'),
                      dict(mission_frame='other')):
            with self.subTest(frame=frame):
                f = obs().frame_ref
                self.needed('FRAME_REVISION_MISMATCH', observation=obs(frame_ref=NS(**(vars(f) | frame))))

    def test_time_basis_not_comparable(self):
        cases = (dict(observation=obs(time=time_(clock_state='UNKNOWN'))),
                 dict(observation=obs(time=time_(has_clock_error_bound=False))),
                 dict(observation=obs(time=time_(stamp_semantics='UNKNOWN'))),
                 dict(reference=camera(declared=False)),
                 dict(reference=camera(clock_error_bound_ns=None)),
                 dict(reference=camera(clock_domain_id='other')),
                 dict(reference=camera(clock_error_bound_ns=16 * MS)))  # 5 + 16 > 20 ms limit
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                self.needed('CANDIDATE_TIME_BASIS_UNUSABLE', **kwargs)

    def test_operator_recheck_has_no_measured_coordinator_clock(self):
        self.needed('CANDIDATE_TIME_BASIS_UNUSABLE',
                    reference=verdict.Reference('OPERATOR', T0, None, None, False))

    def test_window_must_start_after_the_candidate_by_more_than_the_error_sum(self):
        self.needed('ORDER_VIOLATION', observation=obs(sensor_window_start_ns=T0 + 10 * MS))  # == 5 + 5 ms
        self.assertEqual('RESCUE_CONFIRMED', judge(observation=obs(sensor_window_start_ns=T0 + 10 * MS + 1))[0])

    def test_sensing_must_be_absent_clear_valid(self):
        for change, reason in ((dict(camera_state='PRESENT'), 'CAMERA_NOT_ABSENT'),
                               (dict(camera_state='UNKNOWN'), 'CAMERA_NOT_ABSENT'),
                               (dict(lidar_state='BLOCKED'), 'LIDAR_NOT_CLEAR'),
                               (dict(lidar_state='UNKNOWN'), 'LIDAR_NOT_CLEAR'),
                               (dict(coverage_state='INVALID'), 'COVERAGE_NOT_VALID'),
                               (dict(coverage_state='UNKNOWN'), 'COVERAGE_NOT_VALID')):
            with self.subTest(change=change):
                self.needed(reason, observation=obs(**change))

    def test_reasons_are_a_closed_set(self):
        self.assertEqual({'CANDIDATE_NOT_LINKED', 'FRAME_REVISION_MISMATCH', 'CANDIDATE_TIME_BASIS_UNUSABLE',
                          'ORDER_VIOLATION', 'CAMERA_NOT_ABSENT', 'LIDAR_NOT_CLEAR', 'COVERAGE_NOT_VALID'},
                         set(verdict.RECHECK_NEEDED_REASONS))


if __name__ == '__main__':
    unittest.main()
