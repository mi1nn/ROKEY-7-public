"""RobotPose / MapState from the robot's own measured inputs (no ROS).

The coordinator places the SUPPORT robot from /rescue/robots/<id>/pose (VALID mission pose on
the map basis) and compares homes with /rescue/robots/<id>/map_state; db_writer shows the pose
on the console map. Nothing publishes them today. task_input already validates AMCL poses and
the hashed saved map (live_sources); these builders turn those into the two messages. A pose
is VALID only with a valid AMCL pose, the loaded map, a current alignment and a SYNCED clock;
otherwise UNKNOWN with no mission numbers (nothing is made up).
"""
import math
import unittest

from rescue_mapping import RigidTransform
from rescue_robot.live_pose import map_state_record, mission_from_native, pose_record

COV = [0.0] * 36
COV[0], COV[7], COV[35] = 0.04, 0.01, 0.02
BASIS = dict(mission_frame='map', native_frame='map', map_session_id='22222222-2222-4222-8222-222222222222',
             map_revision=0, transform_revision=1)
SYNCED = dict(clock_state='SYNCED', error_bound_ns=5_000_000)


def record(**changes):
    values = dict(robot_id='robot_b', boot_id='boot', observed_at_ns=1_000, pose_valid_ns=2_000_000_000,
                  native=(1.0, 2.0, 0.5, COV), amcl_valid=True, map_loaded=True,
                  transform=RigidTransform.identity(), basis=BASIS, clock=SYNCED, clock_domain_id='rokey7-utc-field',
                  sensor_clock=dict(domain_id='rokey7-utc-field', error_bound_ns=7_000_000))
    values.update(changes)
    return pose_record(**values)


class MissionPose(unittest.TestCase):
    def test_identity_alignment_keeps_the_pose(self):
        self.assertEqual((1.0, 2.0, 0.5, COV), mission_from_native(RigidTransform.identity(), 1.0, 2.0, 0.5, COV))

    def test_planar_alignment_moves_turns_and_rotates_covariance(self):
        x, y, yaw, cov = mission_from_native(RigidTransform.planar(2.0, 0.0, math.pi / 2), 1.0, 0.0, 0.0, COV)
        self.assertAlmostEqual(2.0, x)
        self.assertAlmostEqual(1.0, y)
        self.assertAlmostEqual(math.pi / 2, yaw)
        self.assertAlmostEqual(0.01, cov[0])   # x and y variances swap under a quarter turn
        self.assertAlmostEqual(0.04, cov[7])
        self.assertAlmostEqual(0.02, cov[35])

    def test_alignment_rotates_position_yaw_cross_covariance(self):
        covariance = list(COV)
        covariance[5] = covariance[30] = 0.005  # x-yaw correlation
        covariance[11] = covariance[31] = 0.003  # y-yaw correlation
        *_, rotated = mission_from_native(RigidTransform.planar(0., 0., math.pi / 2),
                                           1., 0., 0., covariance)
        self.assertAlmostEqual(-0.003, rotated[5])
        self.assertAlmostEqual(0.005, rotated[11])
        self.assertAlmostEqual(rotated[5], rotated[30])
        self.assertAlmostEqual(rotated[11], rotated[31])


class PoseRecord(unittest.TestCase):
    def test_valid_pose_on_the_map_basis(self):
        r = record()
        self.assertEqual('VALID', r['mission_pose_status'])
        self.assertEqual((1.0, 2.0, 0.5), r['mission_pose'][:3])
        self.assertEqual(dict(robot_id='robot_b', native_frame='map', mission_frame='map',
                              map_session_id=BASIS['map_session_id'], map_revision=0, transform_revision=1,
                              alignment_status='VALID'), r['frame_ref'])
        self.assertEqual(dict(observed_at_ns=1_000, valid_until_ns=1_000 + 2_000_000_000,
                              stamp_semantics='CAPTURE', clock_domain_id='rokey7-utc-field', clock_state='SYNCED',
                              has_clock_error_bound=True, clock_error_bound_ns=5_000_000 + 7_000_000), r['time'])
        self.assertEqual(('robot_b', 'boot'), (r['robot_id'], r['source_boot_id']))

    def test_any_missing_basis_makes_the_mission_pose_unknown_without_numbers(self):
        for change in (dict(amcl_valid=False), dict(map_loaded=False), dict(transform=None),
                       dict(clock=dict(clock_state='UNKNOWN', error_bound_ns=None))):
            with self.subTest(change=change):
                r = record(**change)
                self.assertEqual('UNKNOWN', r['mission_pose_status'])
                self.assertIsNone(r['mission_pose'])
                self.assertEqual((1.0, 2.0, 0.5), r['native_pose'][:3])  # the measured AMCL pose stays

    def test_the_robot_pi_clock_is_part_of_the_bound(self):
        """AMCL stamps follow the scan, stamped on the robot Pi: its measured bound adds to this PC's."""
        self.assertEqual(5_000_000 + 7_000_000, record()['time']['clock_error_bound_ns'])
        for sensor in (dict(domain_id='other-domain', error_bound_ns=7_000_000),   # Pi not in this clock domain
                       dict(domain_id='rokey7-utc-field', error_bound_ns=-1)):      # Pi bound not measured
            with self.subTest(sensor=sensor):
                r = record(sensor_clock=sensor)
                self.assertEqual(('UNKNOWN', 'UNKNOWN', False), (r['mission_pose_status'], r['time']['clock_state'],
                                                                 r['time']['has_clock_error_bound']))

    def test_unknown_clock_has_no_error_bound(self):
        r = record(clock=dict(clock_state='UNKNOWN', error_bound_ns=None))
        self.assertEqual(('UNKNOWN', False, 0), (r['time']['clock_state'], r['time']['has_clock_error_bound'],
                                                 r['time']['clock_error_bound_ns']))

    def test_unusable_alignment_is_not_valid_in_the_frame_ref(self):
        self.assertEqual('INVALID', record(transform=None)['frame_ref']['alignment_status'])

    def test_covariance_as_ros_gives_it(self):
        """rclpy hands covariance over as a numpy float64 array (found on a ROS run, 2026-10-13)."""
        try:
            import numpy
        except ImportError:
            self.skipTest('numpy not installed')
        r = record(native=(1.0, 2.0, 0.5, numpy.array(COV, dtype=numpy.float64)))
        self.assertEqual('VALID', r['mission_pose_status'])

    def test_bad_inputs_are_refused(self):
        for change in (dict(observed_at_ns=0), dict(pose_valid_ns=0), dict(native=(float('nan'), 0.0, 0.0, COV)),
                       dict(native=(True, 0.0, 0.0, COV)),
                       dict(sensor_clock=dict(domain_id='', error_bound_ns=0)),
                       dict(sensor_clock=dict(domain_id='d', error_bound_ns=-2)),
                       dict(native=(0.0, 0.0, 0.0, COV[:35]))):
            with self.subTest(change=change), self.assertRaises(ValueError):
                record(**change)


class MapStateRecord(unittest.TestCase):
    SHA = 'ab' * 32

    def state(self, **changes):
        values = dict(robot_id='robot_b', occupancy_sha256=self.SHA, basis=BASIS, map_loaded=True,
                      localization_valid=True, generated_at_ns=5)
        values.update(changes)
        return map_state_record(**values)

    def test_declared_basis_with_measured_quality(self):
        s = self.state()
        self.assertEqual(('robot_b', BASIS['map_session_id'], 0, 0, 5, 'VALID', 'UNKNOWN', 'map'),
                         (s['robot_id'], s['map_session_id'], s['map_revision'], s['map_update_sequence'],
                          s['generated_at_ns'], s['localization_quality'], s['correction_detection_status'],
                          s['current_native_map_frame']))

    def test_snapshot_id_is_a_stable_uuid_of_the_map_content_and_basis(self):
        import uuid
        a, b = self.state()['snapshot_id'], self.state(generated_at_ns=99)['snapshot_id']
        self.assertEqual(a, b)
        self.assertEqual(5, uuid.UUID(a).version)
        self.assertNotEqual(a, self.state(occupancy_sha256='cd' * 32)['snapshot_id'])
        self.assertNotEqual(a, self.state(basis=dict(BASIS, map_revision=1))['snapshot_id'])

    def test_quality_is_unknown_without_a_loaded_map_or_valid_localization(self):
        for change in (dict(map_loaded=False), dict(localization_valid=False)):
            with self.subTest(change=change):
                self.assertEqual('UNKNOWN', self.state(**change)['localization_quality'])


if __name__ == '__main__':
    unittest.main()
