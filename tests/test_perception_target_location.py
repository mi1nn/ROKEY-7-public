"""Confirmed TargetObservation + capture-time TF map<-camera -> TargetLocation (B2, 2026-10-13).

Synthetic TF and observations only (E2). The coordinator accepts a location when it is
VALID and its frame_ref basis equals goal_map_session_id/goal_map_revision/goal_transform_revision
(rescue_coordination/ros_node.py on_target_location, _route_frame).
"""
import math
import unittest
from types import SimpleNamespace as NS
from uuid import NAMESPACE_URL, uuid5

from rescue_perception.target_location import LocationBasis, LocationRefused, fill_location, locate

OBSERVED = 1_760_000_000_123_456_789
SESSION = '22222222-2222-4222-8222-222222222222'  # #132 field values (session, revision 0, transform 1)
OBSERVATION_ID = '0d9c5a62-5d0f-5a35-9d6e-5c2b0b0d4a11'


def basis(**changes):
    values = dict(robot_id='robot_a', map_frame='map', mission_frame='map', map_session_id=SESSION,
                  map_revision=0, transform_revision=1, alignment_xy_yaw=[0.0, 0.0, 0.0],
                  alignment_position_error_m=0.0, alignment_rotation_error_rad=0.0)
    values.update(changes)
    return LocationBasis.from_values(values)


def observation(point=(1.0, 0.0, 0.0), status='VALID', robot='robot_a', frame='oakd_rgb_camera_optical_frame',
                covariance=(0.01, 0, 0, 0, 0.04, 0, 0, 0, 0.09)):
    return NS(observation_id=OBSERVATION_ID, source_robot=robot, sensor_frame=frame,
              time=NS(observed_at_ns=OBSERVED), localization_status=status,
              sensor_position=NS(x=point[0], y=point[1], z=point[2]), position_covariance=list(covariance))


def transform(stamp=OBSERVED, parent='map', child='oakd_rgb_camera_optical_frame', yaw=math.pi / 2,
              xyz=(1.0, 2.0, 0.3)):
    return NS(header=NS(frame_id=parent, stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9)), child_frame_id=child,
              transform=NS(translation=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
                           rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


class LocateTests(unittest.TestCase):
    def test_capture_time_tf_moves_the_camera_point_into_the_map(self):
        result = locate(observation(), transform(), basis())
        for got, want in zip(result.position, (1.0, 3.0, 0.3)):
            self.assertAlmostEqual(want, got)

    def test_alignment_map_to_mission_is_applied_after_tf(self):
        result = locate(observation(), transform(), basis(alignment_xy_yaw=[10.0, 0.0, 0.0]))
        for got, want in zip(result.position, (11.0, 3.0, 0.3)):
            self.assertAlmostEqual(want, got)

    def test_covariance_is_rotated_into_the_map(self):
        c = locate(observation(), transform(), basis()).covariance
        for got, want in zip(c, (0.04, 0, 0, 0, 0.01, 0, 0, 0, 0.09)):
            self.assertAlmostEqual(want, got)

    def test_declared_alignment_error_widens_the_covariance(self):
        c = locate(observation(), transform(), basis(alignment_position_error_m=0.1)).covariance
        self.assertAlmostEqual(0.04 + 0.01, c[0])
        self.assertAlmostEqual(0.01 + 0.01, c[4])

    def test_refused_without_valid_capture_time_inputs(self):
        cases = {
            'NO_SENSOR_POSITION': (observation(status='UNKNOWN'), transform()),
            'NONFINITE_POSITION': (observation(point=(math.nan, 0.0, 1.0)), transform()),
            'SOURCE_ROBOT_MISMATCH': (observation(robot='robot_b'), transform()),
            'TF_FRAME_MISMATCH': (observation(), transform(parent='odom')),
            'TF_CHILD_MISMATCH': (observation(), transform(child='base_link')),
            'TF_STAMP_MISMATCH': (observation(), transform(stamp=OBSERVED - 50_000_000)),
        }
        for reason, (obs, tf) in cases.items():
            with self.subTest(reason), self.assertRaises(LocationRefused) as caught:
                locate(obs, tf, basis())
            self.assertEqual(reason, caught.exception.reason)

    def test_basis_values_are_checked_at_start(self):
        for bad in (dict(map_session_id='not-a-uuid'), dict(map_revision=-1), dict(transform_revision=1.5),
                    dict(alignment_xy_yaw=[0.0, 0.0]), dict(alignment_position_error_m=-0.1),
                    dict(alignment_rotation_error_rad=math.inf), dict(map_frame=''), dict(robot_id='')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                basis(**bad)


class FillTests(unittest.TestCase):
    def test_message_is_valid_with_the_coordinator_basis(self):
        obs = observation()
        message = NS(frame_ref=NS(), mission_position=NS())
        fill_location(message, obs, locate(obs, transform(), basis()), basis())
        f = message.frame_ref
        self.assertEqual('VALID', message.projection_status)
        self.assertEqual(('robot_a', 'map', 'map', SESSION, 0, 1, 'VALID'),
                         (f.robot_id, f.native_frame, f.mission_frame, f.map_session_id, f.map_revision,
                          f.transform_revision, f.alignment_status))
        self.assertIs(obs, message.observation)
        self.assertAlmostEqual(3.0, message.mission_position.y)
        self.assertEqual(9, len(message.covariance))

    def test_location_id_is_stable_per_observation_and_basis(self):
        obs = observation()
        ids = []
        for b in (basis(), basis(), basis(transform_revision=2)):
            message = NS(frame_ref=NS(), mission_position=NS())
            ids.append(fill_location(message, obs, locate(obs, transform(), b), b).location_id)
        self.assertEqual(ids[0], ids[1])
        self.assertNotEqual(ids[0], ids[2])
        self.assertEqual(str(uuid5(NAMESPACE_URL, f'ROKEY-7/location/{OBSERVATION_ID}/{SESSION}/0/1')), ids[0])


if __name__ == '__main__':
    unittest.main()
