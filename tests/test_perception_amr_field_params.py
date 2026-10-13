"""AMR camera topic/frame names are field parameters without defaults (B1, 2026-10-13).

Three places named the robot6 OAK-D topic differently (code default ``camera/...``, the
readiness config ``oakd/rgb/preview/...``, the #132 field script ``/robot6/oakd/rgb/...``).
The code must not carry a guess: no default, typed, and a missing value stops the node.
"""
import unittest

from rescue_perception.camera_utils import FIELD_PARAMS as CAMERA_FIELD_PARAMS
from rescue_perception.camera_utils import SETTINGS as CAMERA_SETTINGS
from rescue_perception.camera_utils import field_values
from rescue_perception.target_utils import FIELD_PARAMS as TARGET_FIELD_PARAMS
from rescue_perception.target_utils import SETTINGS as TARGET_SETTINGS

TARGET_NAMES = {'image_topic', 'depth_topic', 'camera_info_topic', 'rgb_optical_frame', 'depth_optical_frame',
                # Localization evidence (2026-10-13): with the old code values UNKNOWN/False every detection
                # was refused (core.locate_detection CLOCK_UNCERTAIN / ALIGNMENT_UNVERIFIED), so nothing was sent.
                'rgb_stamp_semantics', 'depth_stamp_semantics', 'rgb_depth_alignment_verified', 'rgb_rectified',
                'calibration_id', 'alignment_evidence_id',
                'sync_queue_size', 'sync_slop_ns', 'camera_info_max_age_ns',
                # Transport of each input (TB-6 Depth measured as raw 16UC1, code value was compressedDepth).
                'image_type', 'depth_transport'}
BOOL_NAMES = {'rgb_depth_alignment_verified', 'rgb_rectified'}
INTEGER_NAMES = {'sync_queue_size', 'sync_slop_ns', 'camera_info_max_age_ns'}


def supplied(**override):
    values = {'image_topic': '/robot6/oakd/rgb/image_raw/compressed',
              'depth_topic': '/robot6/oakd/stereo/image_raw/compressedDepth',
              'camera_info_topic': '/robot6/oakd/rgb/camera_info',
              'rgb_optical_frame': 'oakd_rgb_camera_optical_frame',
              'depth_optical_frame': 'oakd_rgb_camera_optical_frame',
              'rgb_stamp_semantics': 'DRIVER_RECEIVE', 'depth_stamp_semantics': 'DRIVER_RECEIVE',
              'rgb_depth_alignment_verified': False, 'rgb_rectified': False,
              'calibration_id': 'robot6-oakd-test', 'alignment_evidence_id': 'robot6-oakd-test-evidence',
              'sync_queue_size': 40, 'sync_slop_ns': 20_000_000, 'camera_info_max_age_ns': 1_500_000_000,
              'image_type': 'compressed', 'depth_transport': 'raw'}
    values.update(override)
    return values


class AmrFieldParamsTests(unittest.TestCase):
    def test_no_topic_or_frame_default_in_code(self):
        self.assertEqual({'image_topic', 'image_type'}, set(CAMERA_FIELD_PARAMS))
        self.assertEqual(TARGET_NAMES, set(TARGET_FIELD_PARAMS))
        self.assertEqual({name: 'BOOL' if name in BOOL_NAMES else 'INTEGER' if name in INTEGER_NAMES else 'STRING'
                          for name in TARGET_NAMES},
                         dict(TARGET_FIELD_PARAMS))
        for name in TARGET_NAMES:
            self.assertNotIn(name, CAMERA_SETTINGS)
            self.assertNotIn(name, TARGET_SETTINGS)

    def test_supplied_values_are_merged(self):
        values = field_values(TARGET_SETTINGS, supplied(), TARGET_FIELD_PARAMS)
        self.assertEqual('/robot6/oakd/rgb/image_raw/compressed', values['image_topic'])
        self.assertEqual('oakd_rgb_camera_optical_frame', values['rgb_optical_frame'])
        self.assertEqual(TARGET_SETTINGS['model_label'], values['model_label'])

    def test_flags_must_be_real_bools(self):
        for bad in ('true', 1, None):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                field_values(TARGET_SETTINGS, supplied(rgb_rectified=bad), TARGET_FIELD_PARAMS)
        self.assertIs(True, field_values(TARGET_SETTINGS, supplied(rgb_rectified=True), TARGET_FIELD_PARAMS)['rgb_rectified'])

    def test_stamp_semantics_vocabulary(self):
        with self.assertRaises(ValueError):
            field_values(TARGET_SETTINGS, supplied(rgb_stamp_semantics='CAMERA'), TARGET_FIELD_PARAMS)

    def test_missing_empty_or_wrong_type_is_refused(self):
        for bad in (None, '', '  ', 3):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                field_values(TARGET_SETTINGS, supplied(depth_topic=bad), TARGET_FIELD_PARAMS)
        partial = supplied()
        del partial['camera_info_topic']
        with self.assertRaisesRegex(ValueError, 'camera_info_topic'):
            field_values(TARGET_SETTINGS, partial, TARGET_FIELD_PARAMS)


class AmrPerceptionLaunchTests(unittest.TestCase):
    def test_launch_requires_every_field_parameter(self):
        try:
            from launch import LaunchDescription  # noqa: F401  (ROS only; a bare `launch` may be src/rescue_bringup/launch)
        except ImportError:
            self.skipTest('ROS launch not importable (NOT_RUN without a sourced ROS)')
        import runpy
        from pathlib import Path
        path = Path(__file__).resolve().parents[1] / 'src/rescue_bringup/launch/amr_perception.launch.py'
        description = runpy.run_path(str(path))['generate_launch_description']()
        arguments = {a.name: a for a in description.get_launch_arguments()}
        from rescue_perception.target_utils import CLOCK_PARAMS
        from rescue_perception.recheck_feed import RECHECK_PARAMS
        self.assertTrue(TARGET_NAMES | set(CLOCK_PARAMS) | set(RECHECK_PARAMS) | {'tf_topic', 'tf_static_topic'}
                        <= set(arguments))
        self.assertTrue(all(a.default_value is None for a in arguments.values()))


if __name__ == '__main__':
    unittest.main()
