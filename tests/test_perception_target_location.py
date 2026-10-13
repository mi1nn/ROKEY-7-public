"""Confirmed TargetObservation + capture-time TF map<-camera -> TargetLocation (B2, 2026-10-13).

Synthetic TF and observations only (E2). The coordinator accepts a location when it is
VALID and its frame_ref basis equals goal_map_session_id/goal_map_revision/goal_transform_revision
(rescue_coordination/ros_node.py on_target_location, _route_frame).

OPS-103 (2026-10-13, CDX-142-01·02): the producer itself refuses an observation whose time basis is
not usable (C-03: not SYNCED, no error bound, UNKNOWN stamp semantics, expired, future stamp, other
clock domain), a location outside the declared alignment authority window (C-04), and a covariance
that is not symmetric, finite, non-negative on the diagonal and positive semi-definite (C-02).
"""
import math
import unittest
from types import SimpleNamespace as NS
from uuid import NAMESPACE_URL, uuid5

from rescue_perception.target_location import LocationBasis, LocationRefused, fill_location, locate

OBSERVED = 1_760_000_000_123_456_789
NOW = OBSERVED + 500_000_000
HOUR = 3_600_000_000_000
SESSION = '22222222-2222-4222-8222-222222222222'  # #132 field values (session, revision 0, transform 1)
OBSERVATION_ID = '0d9c5a62-5d0f-5a35-9d6e-5c2b0b0d4a11'
DOMAIN = 'rokey7-utc-field'
AUTHORITY = 'saved-map-identity-test'


def basis(**changes):
    values = dict(robot_id='robot_a', map_frame='map', mission_frame='map', map_session_id=SESSION,
                  map_revision=0, transform_revision=1, alignment_xy_yaw=[0.0, 0.0, 0.0],
                  alignment_position_error_m=0.0, alignment_rotation_error_rad=0.0, clock_domain_id=DOMAIN,
                  alignment_authority_id=AUTHORITY, alignment_valid_from_ns=OBSERVED - HOUR,
                  alignment_valid_until_ns=OBSERVED + HOUR)
    values.update(changes)
    return LocationBasis.from_values(values)


def observation(point=(1.0, 0.0, 0.0), status='VALID', robot='robot_a', frame='oakd_rgb_camera_optical_frame',
                covariance=(0.01, 0, 0, 0, 0.04, 0, 0, 0, 0.09), **time):
    clock = dict(observed_at_ns=OBSERVED, valid_until_ns=OBSERVED + 2_000_000_000, stamp_semantics='CAPTURE',
                 clock_domain_id=DOMAIN, clock_state='SYNCED', has_clock_error_bound=True,
                 clock_error_bound_ns=5_000_000)
    clock.update(time)
    return NS(observation_id=OBSERVATION_ID, source_robot=robot, sensor_frame=frame, time=NS(**clock),
              localization_status=status, sensor_position=NS(x=point[0], y=point[1], z=point[2]),
              position_covariance=list(covariance))


def transform(stamp=OBSERVED, parent='map', child='oakd_rgb_camera_optical_frame', yaw=math.pi / 2,
              xyz=(1.0, 2.0, 0.3)):
    return NS(header=NS(frame_id=parent, stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9)), child_frame_id=child,
              transform=NS(translation=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
                           rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


def run(obs=None, tf=None, b=None, now=NOW):
    return locate(obs or observation(), tf or transform(), b or basis(), now_ns=now)


class LocateTests(unittest.TestCase):
    def test_capture_time_tf_moves_the_camera_point_into_the_map(self):
        for got, want in zip(run().position, (1.0, 3.0, 0.3)):
            self.assertAlmostEqual(want, got)

    def test_alignment_map_to_mission_is_applied_after_tf(self):
        for got, want in zip(run(b=basis(alignment_xy_yaw=[10.0, 0.0, 0.0])).position, (11.0, 3.0, 0.3)):
            self.assertAlmostEqual(want, got)

    def test_covariance_is_rotated_into_the_map(self):
        for got, want in zip(run().covariance, (0.04, 0, 0, 0, 0.01, 0, 0, 0, 0.09)):
            self.assertAlmostEqual(want, got)

    def test_declared_alignment_error_widens_the_covariance(self):
        c = run(b=basis(alignment_position_error_m=0.1)).covariance
        self.assertAlmostEqual(0.04 + 0.01, c[0])
        self.assertAlmostEqual(0.01 + 0.01, c[4])

    def assertRefused(self, reason, **kw):
        with self.assertRaises(LocationRefused) as caught:
            run(**kw)
        self.assertEqual(reason, caught.exception.reason)

    def test_refused_without_valid_capture_time_inputs(self):
        cases = {
            'NO_SENSOR_POSITION': dict(obs=observation(status='UNKNOWN')),
            'NONFINITE_POSITION': dict(obs=observation(point=(math.nan, 0.0, 1.0))),
            'SOURCE_ROBOT_MISMATCH': dict(obs=observation(robot='robot_b')),
            'TF_FRAME_MISMATCH': dict(tf=transform(parent='odom')),
            'TF_CHILD_MISMATCH': dict(tf=transform(child='base_link')),
            'TF_STAMP_MISMATCH': dict(tf=transform(stamp=OBSERVED - 50_000_000)),
        }
        for reason, kw in cases.items():
            with self.subTest(reason):
                self.assertRefused(reason, **kw)

    def test_unusable_time_basis_is_refused_by_the_producer(self):   # C-03, OPS-103 decision 1
        cases = {
            'CLOCK_NOT_SYNCED': dict(obs=observation(clock_state='UNKNOWN')),
            'NO_CLOCK_ERROR_BOUND': dict(obs=observation(has_clock_error_bound=False)),
            'STAMP_SEMANTICS_UNKNOWN': dict(obs=observation(stamp_semantics='UNKNOWN')),
            'CLOCK_DOMAIN_MISMATCH': dict(obs=observation(clock_domain_id='other-clock')),
            'OBSERVATION_EXPIRED': dict(now=OBSERVED + 2_000_000_001),
            'FUTURE_STAMP': dict(now=OBSERVED - 5_000_001),
        }
        for reason, kw in cases.items():
            with self.subTest(reason):
                self.assertRefused(reason, **kw)
        run(now=OBSERVED - 5_000_000)   # within the error bound: not future

    def test_location_outside_the_alignment_authority_window_is_refused(self):   # C-04, decision 2
        self.assertRefused('ALIGNMENT_NOT_VALID_AT_CAPTURE', b=basis(alignment_valid_from_ns=OBSERVED + 1))
        self.assertRefused('ALIGNMENT_EXPIRED', b=basis(alignment_valid_until_ns=NOW))

    def test_bad_covariance_is_refused(self):   # C-02, CDX-142-02
        cases = {'negative': (-1, 0, 0, 0, -1, 0, 0, 0, -1), 'asymmetric': (1, 0.5, 0, 0, 1, 0, 0, 0, 1),
                 'not PSD': (1, 2, 0, 2, 1, 0, 0, 0, 1), 'not finite': (math.nan, 0, 0, 0, 1, 0, 0, 0, 1),
                 'short': (1, 0, 0)}
        for name, covariance in cases.items():
            with self.subTest(name):
                self.assertRefused('COVARIANCE_INVALID', obs=observation(covariance=covariance))
        run(obs=observation(covariance=(0.0,) * 9))   # zero is PSD: allowed

    def test_basis_values_are_checked_at_start(self):
        for bad in (dict(map_session_id='not-a-uuid'), dict(map_revision=-1), dict(transform_revision=1.5),
                    dict(alignment_xy_yaw=[0.0, 0.0]), dict(alignment_position_error_m=-0.1),
                    dict(alignment_rotation_error_rad=math.inf), dict(map_frame=''), dict(robot_id=''),
                    dict(clock_domain_id=''), dict(alignment_authority_id=' '),
                    dict(alignment_valid_until_ns=OBSERVED - HOUR), dict(alignment_valid_from_ns=1.5)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                basis(**bad)


class FillTests(unittest.TestCase):
    def test_message_is_valid_with_the_coordinator_basis(self):
        obs = observation()
        message = NS(frame_ref=NS(), mission_position=NS())
        fill_location(message, obs, run(obs=obs), basis())
        f = message.frame_ref
        self.assertEqual('VALID', message.projection_status)
        self.assertEqual(('robot_a', 'map', 'map', SESSION, 0, 1, 'VALID'),
                         (f.robot_id, f.native_frame, f.mission_frame, f.map_session_id, f.map_revision,
                          f.transform_revision, f.alignment_status))
        self.assertIs(obs, message.observation)
        self.assertAlmostEqual(3.0, message.mission_position.y)
        self.assertEqual(9, len(message.covariance))

    def test_location_id_is_stable_per_observation_and_alignment(self):
        obs = observation()
        ids = []
        for b in (basis(), basis(), basis(transform_revision=2), basis(alignment_xy_yaw=[0.5, 0.0, 0.0]),
                  basis(alignment_authority_id='other-authority')):
            message = NS(frame_ref=NS(), mission_position=NS())
            ids.append(fill_location(message, obs, run(obs=obs, b=b), b).location_id)
        self.assertEqual(ids[0], ids[1])
        self.assertEqual(len(set(ids)), 4)   # transform revision, alignment value and authority all change it
        self.assertEqual(str(uuid5(NAMESPACE_URL, f'ROKEY-7/location/{OBSERVATION_ID}/{SESSION}/0/1/'
                                                  f'{AUTHORITY}/0,0,0')), ids[0])


if __name__ == '__main__':
    unittest.main()
