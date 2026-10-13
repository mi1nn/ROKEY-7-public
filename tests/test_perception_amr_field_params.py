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

TARGET_NAMES = {'image_topic', 'depth_topic', 'camera_info_topic', 'rgb_optical_frame', 'depth_optical_frame'}


def supplied(**override):
    values = {'image_topic': '/robot6/oakd/rgb/image_raw/compressed',
              'depth_topic': '/robot6/oakd/stereo/image_raw/compressedDepth',
              'camera_info_topic': '/robot6/oakd/rgb/camera_info',
              'rgb_optical_frame': 'oakd_rgb_camera_optical_frame',
              'depth_optical_frame': 'oakd_rgb_camera_optical_frame'}
    values.update(override)
    return values


class AmrFieldParamsTests(unittest.TestCase):
    def test_no_topic_or_frame_default_in_code(self):
        self.assertEqual({'image_topic'}, set(CAMERA_FIELD_PARAMS))
        self.assertEqual(TARGET_NAMES, set(TARGET_FIELD_PARAMS))
        self.assertEqual({'STRING'}, set(TARGET_FIELD_PARAMS.values()))
        for name in TARGET_NAMES:
            self.assertNotIn(name, CAMERA_SETTINGS)
            self.assertNotIn(name, TARGET_SETTINGS)

    def test_supplied_values_are_merged(self):
        values = field_values(TARGET_SETTINGS, supplied(), TARGET_FIELD_PARAMS)
        self.assertEqual('/robot6/oakd/rgb/image_raw/compressed', values['image_topic'])
        self.assertEqual('oakd_rgb_camera_optical_frame', values['rgb_optical_frame'])
        self.assertEqual(TARGET_SETTINGS['model_label'], values['model_label'])

    def test_missing_empty_or_wrong_type_is_refused(self):
        for bad in (None, '', '  ', 3):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                field_values(TARGET_SETTINGS, supplied(depth_topic=bad), TARGET_FIELD_PARAMS)
        partial = supplied()
        del partial['camera_info_topic']
        with self.assertRaisesRegex(ValueError, 'camera_info_topic'):
            field_values(TARGET_SETTINGS, partial, TARGET_FIELD_PARAMS)


if __name__ == '__main__':
    unittest.main()
