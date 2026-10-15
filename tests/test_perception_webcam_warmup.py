"""webcam_node 의 YOLO 준비(2026-10-15): 지진 감지 순간 첫 YOLO 호출이 1.5 ~ 3 초 걸려 웹캠 영상이 멈췄다
(측정: 10:57 녹화 1.77 초, 11:06 녹화 3.0 초 — 중계기가 stale 2 초를 넘겨 스트림을 닫음). 기동할 때 한 번 미리 돌린다."""
import unittest

from rescue_perception.webcam_node import warm_up


class FakeModel:
    def __init__(self, fail=False):
        self.calls, self.fail = [], fail

    def predict(self, frame, **kwargs):
        self.calls.append((frame, kwargs))
        if self.fail:
            raise RuntimeError('CUDA error')
        return [object()]


class WarmUp(unittest.TestCase):
    def test_one_predict_with_the_detection_settings(self):
        model, frame = FakeModel(), object()
        elapsed = warm_up(model, frame, imgsz=640, conf=0.1)
        self.assertEqual([(frame, {'imgsz': 640, 'conf': 0.1, 'verbose': False})], model.calls)
        self.assertGreaterEqual(elapsed, 0.0)

    def test_a_failure_is_raised_not_hidden(self):
        with self.assertRaises(RuntimeError):
            warm_up(FakeModel(fail=True), object(), imgsz=640, conf=0.1)


if __name__ == '__main__':
    unittest.main()
