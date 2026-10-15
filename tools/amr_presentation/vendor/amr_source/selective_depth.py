"""Bounded, undecoded Depth cache for one detection-triggered RGB request.

No ROS, model inference or decompression occurs here. Original integer stamps
select a pair; steady time bounds waiting and memory retention.
"""
from collections import deque
import math

class SelectiveDepth:
    def __init__(self, *, capacity, slop_ns, wait_s, confidence, label):
        if (type(capacity) is not int or capacity <= 0 or type(slop_ns) is not int
                or slop_ns < 0 or not math.isfinite(wait_s) or wait_s <= 0
                or not math.isfinite(confidence) or not 0 <= confidence <= 1):
            raise ValueError('invalid selective depth policy')
        self.capacity, self.slop_ns, self.wait_s = capacity, slop_ns, wait_s
        self.confidence, self.label = confidence, label
        self._depth = deque(maxlen=capacity)
        self._pending = None
        self._last_rgb_stamp = None
        self.sent = False
        self.expired_requests = 0

    @property
    def buffered_count(self):
        return len(self._depth)

    def expire(self, now):
        self._depth = deque((r for r in self._depth if now - r[2] < self.wait_s),
                            maxlen=self.capacity)
        if self._pending is not None and now - self._pending[3] >= self.wait_s:
            self._pending = None
            self.expired_requests += 1

    def offer_rgb(self, stamp, width, height, outputs, payload, now, *, force=False):
        self.expire(now)
        if self.sent or (self._last_rgb_stamp is not None and stamp <= self._last_rgb_stamp):
            return False
        self._last_rgb_stamp = stamp
        if self._pending is not None:
            return False
        eligible = []
        for output in outputs:
            x0, y0, x1, y1 = output.bbox
            if (output.class_label == self.label and math.isfinite(output.confidence)
                    and output.confidence >= self.confidence
                    and all(math.isfinite(x) for x in output.bbox)
                    and 0 < x0 < x1 < width and 0 < y0 < y1 < height):
                eligible.append(output)
        if not eligible and not force:
            return False
        best = max(eligible, key=lambda item: item.confidence) if eligible else None
        self._pending = (stamp, payload, best, now)
        return True

    def offer_depth(self, stamp, message, now):
        self.expire(now)
        if not self.sent:
            self._depth.append((stamp, message, now))

    def take(self, now):
        self.expire(now)
        if self.sent or self._pending is None:
            return None
        stamp, payload, selected, _ = self._pending
        matches = [r for r in self._depth if abs(r[0] - stamp) <= self.slop_ns]
        if not matches:
            return None
        matched = min(matches, key=lambda row: abs(row[0] - stamp))
        self._pending = None
        # A failed localization retries on a NEW RGB frame, never reprocesses this one.
        return payload, selected, matched[1], abs(matched[0] - stamp)

    def mark_sent(self):
        self.sent = True
        self._pending = None
        self._depth.clear()
