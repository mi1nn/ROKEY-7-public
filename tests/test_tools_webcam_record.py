"""tools/webcam_record.py 의 순수 함수 시험. ffmpeg·카메라를 실행하지 않는다(출력 문자열만 넣는다)."""
import importlib.util
from pathlib import Path
import tempfile
import json
import unittest

_PATH = Path(__file__).resolve().parents[1] / "tools" / "webcam_record.py"
_SPEC = importlib.util.spec_from_file_location("webcam_record", _PATH)
wr = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(wr)

DEVICES = """[AVFoundation indev @ 0x1] AVFoundation video devices:
[AVFoundation indev @ 0x1] [0] FaceTime HD 카메라
[AVFoundation indev @ 0x1] [1] USB Camera C920
[AVFoundation indev @ 0x1] [2] Capture screen 0
[AVFoundation indev @ 0x1] AVFoundation audio devices:
[AVFoundation indev @ 0x1] [0] MacBook Pro 마이크
"""
MODES = """[in#0 @ 0x1] Selected video size (1x1) is not supported by the device.
[in#0 @ 0x1] Supported modes:
[in#0 @ 0x1]   1920x1080@[15.000000 30.000000]fps
[in#0 @ 0x1]   1280x720@[15.000000 30.000000]fps
[in#0 @ 0x1]   640x480@[15.000000 30.000000]fps
"""


def probe(w, h, rate):
    return {"streams": [{"codec_type": "video", "width": w, "height": h, "avg_frame_rate": rate}]}


class WebcamRecordTests(unittest.TestCase):
    def test_devices_are_video_only(self):
        self.assertEqual(wr.parse_devices(DEVICES),
                         [(0, "FaceTime HD 카메라"), (1, "USB Camera C920"), (2, "Capture screen 0")])

    def test_modes_are_parsed(self):
        self.assertEqual(wr.parse_modes(MODES)[1], (1280, 720, 15.0, 30.0))
        self.assertEqual(len(wr.parse_modes(MODES)), 3)

    def test_device_by_index_or_name(self):
        devices = wr.parse_devices(DEVICES)
        self.assertEqual(wr.resolve_device(devices, "1"), (1, "USB Camera C920"))
        self.assertEqual(wr.resolve_device(devices, "c920"), (1, "USB Camera C920"))

    def test_ambiguous_or_missing_device_fails(self):
        devices = wr.parse_devices(DEVICES)
        for selector in ("9", "logitech", "a"):  # "a" 는 여러 장치에 걸린다
            with self.assertRaises(SystemExit, msg=selector):
                wr.resolve_device(devices, selector)

    def test_unsupported_mode_fails_instead_of_falling_back(self):
        modes = wr.parse_modes(MODES)
        wr.check_mode(modes, (1280, 720), 30)
        for size, fps in (((1280, 720), 60), ((3840, 2160), 30), ((1280, 720), 10)):
            with self.assertRaises(SystemExit):
                wr.check_mode(modes, size, fps)

    def test_device_rate_with_float_noise_is_accepted_and_used_exactly(self):
        # 2026-10-11 USB 웹캠: 1280x720@[30.000030 30.000030] 를 30 으로 요청하면 거절됐다.
        # ffmpeg 에는 장치가 보고한 값 그대로를 넘긴다(반올림한 30 은 장치가 거절).
        modes = wr.parse_modes("[in#0 @ 0x1]   1280x720@[30.000030 30.000030]fps\n"
                               "[in#0 @ 0x1]   1280x720@[10.000000 10.000000]fps\n")
        self.assertEqual(wr.check_mode(modes, (1280, 720), 30), "30.000030")
        self.assertEqual(wr.check_mode(modes, (1280, 720), 10), "10.000000")
        with self.assertRaises(SystemExit):
            wr.check_mode(modes, (1280, 720), 20)

    def test_actual_resolution_and_fps_are_checked(self):
        self.assertEqual(wr.check_actual(probe(1280, 720, "30/1"), (1280, 720), 30), [])
        self.assertEqual(wr.check_actual(probe(1280, 720, "30000/1001"), (1280, 720), 30), [])
        self.assertEqual(len(wr.check_actual(probe(640, 480, "30/1"), (1280, 720), 30)), 1)
        self.assertEqual(len(wr.check_actual(probe(1280, 720, "15/1"), (1280, 720), 30)), 1)
        self.assertEqual(wr.check_actual({"streams": []}, (1280, 720), 30), ["영상 스트림 없음"])

    def test_size_argument(self):
        self.assertEqual(wr.parse_size("1280x720"), (1280, 720))
        for bad in ("1280*720", "720p", "x720"):
            with self.assertRaises(Exception):
                wr.parse_size(bad)

    def test_checklist_has_train_and_held_out_clips(self):
        splits = {split for split, _, _, _ in wr.CLIPS.values()}
        self.assertEqual(splits, {"train", "val"})
        self.assertTrue(all(type(s) is int and s > 0 for _, s, _, _ in wr.CLIPS.values()))

    def test_only_checked_clips_count_as_done(self):
        with tempfile.TemporaryDirectory() as tmp:
            day = Path(tmp) / "20261011"
            day.mkdir()
            (day / "C06-1.json").write_text(json.dumps({"clip": "C06", "ok": True}))
            (day / "C06-2.json").write_text(json.dumps({"clip": "C06", "ok": True}))
            (day / "C07-1.json").write_text(json.dumps({"clip": "C07", "ok": False}))
            (day / "broken.json").write_text("{")
            self.assertEqual(wr.done_clips(Path(tmp)), {"C06": 2})


if __name__ == "__main__":
    unittest.main()
