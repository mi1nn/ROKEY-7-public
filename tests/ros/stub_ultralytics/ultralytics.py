"""Test stand-in for the ultralytics package used only by tests/ros/e3_amr_target_probe.py.

No model runs: a pure red square in the frame is reported as one 'survivor' box (score 0.95). It lets
the real amr_target process frames on a PC without the model runtime. Never on a robot.
"""
import numpy as np


class _List(list):
    def tolist(self):
        return list(self)


class _Boxes:
    def __init__(self, boxes):
        self.xyxy = _List([list(b) for b in boxes])
        self.conf = _List([0.95] * len(boxes))
        self.cls = _List([0.0] * len(boxes))
        self.xyxyn = _List([])

    def __len__(self):
        return len(self.xyxy)


class _Result:
    def __init__(self, frame, boxes):
        self.frame, self.boxes = frame, _Boxes(boxes)

    def plot(self, labels=True, conf=True):
        return self.frame.copy()


class YOLO:
    def __init__(self, path):
        self.names = {0: 'survivor'}

    def predict(self, frame, **kwargs):
        red = (frame[:, :, 2] > 200) & (frame[:, :, 1] < 60) & (frame[:, :, 0] < 60)
        ys, xs = np.nonzero(red)
        boxes = [] if len(xs) == 0 else [(float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1))]
        return [_Result(frame, boxes)]
