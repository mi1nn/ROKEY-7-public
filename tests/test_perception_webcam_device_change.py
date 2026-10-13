"""webcam_node 의 실행 중 장치 바꾸기(OPS-183): device_path 만 바꿀 수 있고, 없는 장치는 거절한다."""
from pathlib import Path
import tempfile
import unittest

from rescue_perception.webcam_node import device_change_refusal


class DeviceChangeRefusal(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        self.camera = self.dir / 'usb-Logitech_C920-video-index0'
        self.camera.touch()

    def test_an_existing_device_path_is_taken(self):
        self.assertIsNone(device_change_refusal('device_path', str(self.camera)))

    def test_a_missing_device_is_refused(self):
        self.assertIn('no such camera device', device_change_refusal('device_path', str(self.dir / 'gone')))

    def test_other_parameters_stay_fixed(self):
        self.assertIn('fixed at start', device_change_refusal('width', 1280))
        self.assertIn('fixed at start', device_change_refusal('quake_shakes_min', 2))

    def test_empty_or_non_text_is_refused(self):
        for bad in ('', 3, None):
            with self.subTest(bad=bad):
                self.assertIsNotNone(device_change_refusal('device_path', bad))


if __name__ == '__main__':
    unittest.main()
