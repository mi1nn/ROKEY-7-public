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
        v = dict(SETTINGS)
        self.assertEqual(localization_blockers(v), ('CLOCK_UNCERTAIN', 'ALIGNMENT_UNVERIFIED'))
        v.update(rgb_stamp_semantics='CAPTURE', depth_stamp_semantics='CAPTURE',
                 rgb_rectified=True, rgb_depth_alignment_verified=True)
        self.assertEqual(localization_blockers(v), ())
        v['depth_stamp_semantics'] = 'DRIVER_RECEIVE'
        self.assertEqual(localization_blockers(v), ('CLOCK_UNCERTAIN',))
