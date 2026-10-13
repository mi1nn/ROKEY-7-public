import unittest
from types import SimpleNamespace
from rescue_perception.target_utils import target_diagnostic, localization_blockers, SETTINGS

class TargetDiagnosticsTests(unittest.TestCase):
    def test_high_confidence_does_not_hide_localization_failure(self):
        candidate = SimpleNamespace(status='UNCONFIRMED', reasons=('CLOCK_UNCERTAIN',),
            observation=SimpleNamespace(detection=SimpleNamespace(confidence=0.97)))
        self.assertEqual(target_diagnostic(candidate),
                         'detected confidence=0.970 localization=CLOCK_UNCERTAIN status=UNCONFIRMED')

    def test_confirmed_has_located_status(self):
        candidate = SimpleNamespace(status='CONFIRMED', reasons=(),
            observation=SimpleNamespace(detection=SimpleNamespace(confidence=0.95)))
        self.assertIn('localization=LOCATED', target_diagnostic(candidate))

    def test_reports_all_configuration_blockers(self):
        v = dict(SETTINGS, rgb_stamp_semantics='UNKNOWN', depth_stamp_semantics='UNKNOWN', rgb_rectified=False, rgb_depth_alignment_verified=False)
        self.assertEqual(localization_blockers(v), ('CLOCK_UNCERTAIN', 'ALIGNMENT_UNVERIFIED'))
        v.update(rgb_stamp_semantics='CAPTURE', depth_stamp_semantics='CAPTURE',
                 rgb_rectified=True, rgb_depth_alignment_verified=True)
        self.assertEqual(localization_blockers(v), ())
        v['depth_stamp_semantics'] = 'DRIVER_RECEIVE'
        self.assertEqual(localization_blockers(v), ('CLOCK_UNCERTAIN',))

class ReusedCalibrationTests(unittest.TestCase):
    def test_robot5_reuses_exact_robot6_coefficients_with_correct_identity(self):
        from rescue_perception.target_utils import load_depth_calibration
        a = load_depth_calibration('package:robot_depth.json', physical_robot_id='robot6')
        b = load_depth_calibration('package:robot_depth.json', physical_robot_id='robot5')
        self.assertEqual((a.scale, a.offset_m), (b.scale, b.offset_m))
        self.assertEqual(a.calibration_id, b.calibration_id)
        self.assertEqual(a.sha256, b.sha256)
        self.assertNotEqual(a.physical_robot_id, b.physical_robot_id)
        with self.assertRaisesRegex(ValueError, 'robot mismatch'):
            load_depth_calibration('package:robot_depth.json', physical_robot_id='robot7')
    def test_unrectified_input_can_use_main_point_correction(self):
        self.assertEqual(localization_blockers(dict(rgb_stamp_semantics='CAPTURE', depth_stamp_semantics='CAPTURE', rgb_rectified=False, rgb_depth_alignment_verified=True)), ())
