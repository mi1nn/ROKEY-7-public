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

    def test_one_take_ends_with_a_held_out_validation_part(self):
        splits = [split for split, _, _, _ in wr.SCENES]
        self.assertEqual(set(splits), {"train", "val"})
        # 검증 구간은 마지막에 몰려 있어야 한 번 자르기로 나뉜다
        self.assertEqual(splits, sorted(splits, key=lambda x: x == "val"))
        self.assertEqual(wr.VAL_START_S, sum(s for split, s, _, _ in wr.SCENES if split == "train"))
        self.assertGreater(wr.CLIPS["S01"][1], wr.VAL_START_S)
        self.assertTrue(all(type(s) is int and s > 0 for _, s, _, _ in wr.SCENES))

    def test_session_start_pause_resume(self):
        s = wr.Session(10)
        self.assertEqual(s.state, "WAIT")
        with self.assertRaises(RuntimeError):
            s.wrote_frame()  # 시작 전에는 쓰지 않는다
        s.toggle()
        for _ in range(20):
            s.wrote_frame()
        s.toggle()
        self.assertEqual((s.state, s.video_s), ("PAUSE", 2.0))
        with self.assertRaises(RuntimeError):
            s.wrote_frame()  # 일시정지 중에도 쓰지 않는다 -> 영상 시간이 멈춘다
        s.toggle()
        s.wrote_frame()
        self.assertEqual((s.state, s.frames), ("REC", 21))

    def test_scene_markers_give_validation_start(self):
        scenes = [("train", 5, "", "a"), ("train", 5, "", "b"), ("val", 5, "", "c")]
        s = wr.Session(10, scenes)
        s.move(1)  # 시작 전에 넘긴 장면은 시각을 남기지 않는다
        s.move(-1)
        s.toggle()
        for step in (1, 1):
            for _ in range(30):
                s.wrote_frame()
            s.move(step)
        s.move(1)  # 마지막 장면에서 더 넘겨도 그대로
        self.assertEqual([m["video_s"] for m in s.markers], [0.0, 3.0, 6.0])
        self.assertEqual(s.val_start_s, 6.0)
        for _ in range(15):
            s.wrote_frame()
        self.assertEqual(s.scene_s, 1.5)

    def test_markers_follow_the_real_frame_rate(self):
        # 2026-10-11 실제 녹화: 30 fps 요청, 카메라가 16.1 fps 만 줌 -> 영상이 1.9배 빨리 재생됐다.
        # 실제 fps 로 저장하면 장면 시각도 그 fps 로 다시 계산돼야 한다.
        scenes = [("train", 5, "", "a"), ("val", 5, "", "b")]
        s = wr.Session(30, scenes)
        s.toggle()
        for _ in range(160):
            s.wrote_frame()
        s.move(1)
        self.assertEqual(s.markers[-1]["frame"], 160)
        s.fps = 16.0  # 저장할 때 실제 fps 로 바꾼다
        self.assertEqual(s.val_start_s, 10.0)
        self.assertEqual(s.video_s, 10.0)

    def test_no_validation_marker_means_unknown(self):
        s = wr.Session(30)
        s.toggle()
        self.assertIsNone(s.val_start_s)

    def test_mmss(self):
        self.assertEqual([wr.mmss(x) for x in (0, 59.9, 61, 370)], ["00:00", "00:59", "01:01", "06:10"])

    def test_eval_clips_cover_each_condition_in_valid_and_test(self):
        for prefix, split in (("V", "val"), ("T", "test")):
            for key in wr.EVAL_CONDITIONS:
                self.assertEqual(wr.CLIPS[prefix + key][0], split)
        self.assertEqual(wr.CLIPS["S02"][0], "train")
        self.assertTrue(all(sc[0] == "train" for sc in wr.scenes_for("S02")))
        self.assertEqual(wr.scenes_for("S01"), wr.SCENES)
        self.assertEqual(len(wr.scenes_for("TBG")), 1)

    def test_condition_from_roboflow_file_name(self):
        self.assertEqual(wr.condition_of("VDARK-142233_mp4-0003_jpg.rf.ab12.jpg"), "저조도")
        self.assertEqual(wr.condition_of("TBG-150102_mp4-0010_jpg.rf.cd34.jpg"), "배경")
        for name in ("S01-131956_mp4-0001_jpg.rf.x.jpg", "VXYZ-1_mp4-1.jpg", "NORM-1.jpg"):
            self.assertIsNone(wr.condition_of(name), name)

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
