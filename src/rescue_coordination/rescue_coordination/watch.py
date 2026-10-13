"""Fixed-webcam watch book: the coordinator side of SetCameraWatch (IF-078, contracts C-11 rows 1-4).

- ``watch_id`` is issued here for one operator SET_CAMERA_WATCH, recorded when the request is
  built and dropped when the camera refuses it (row 2). (camera, target, ROI) is fixed per
  watch_id; another ROI is another watch_id.
- One active watch per camera; a new one replaces it. ``revision`` is the camera's slot revision
  as the camera last reported it; every request names it (the camera refuses otherwise).
- A candidate is accepted only for the active watch with the same camera and target and a
  declared time basis (row 4). Any candidate on the active watch ends it, accepted or not; a
  refused one asks the operator to arm again. Event-id replay is handled by the mission first;
  another event_id for the same watch_id is a conflict (row 3).
- No reply before the request deadline: the request is dropped and the operator arms again.
  Nothing is retried silently except a release, which stands while its cause stands.

Pure Python; the node moves the fields to the service and the replies back.
"""
from dataclasses import dataclass
import math

from .planning import CoordinationError, uid

ROI_SIZE = 4


def roi_value(values):
    """roi_xyxy_normalized -> 4-tuple, or None for the IDL's all-zero "not given". Else INVALID_INPUT.

    Same rule as the camera (rescue_perception.target_watch.WatchSpec): each in [0, 1], x0 < x1, y0 < y1.
    """
    values = tuple(values)
    # isinstance, not type(): a ROS float64[4] arrives as numpy.float64 (a float subclass), measured E3 2026-10-13.
    if len(values) != ROI_SIZE or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                                          and math.isfinite(v) for v in values):
        raise CoordinationError('INVALID_INPUT', 'roi_xyxy_normalized must be 4 finite numbers')
    if values == (0.0,) * ROI_SIZE:
        return None
    x0, y0, x1, y1 = values = tuple(float(v) for v in values)
    if not (all(0.0 <= v <= 1.0 for v in values) and x0 < x1 and y0 < y1):
        raise CoordinationError('INVALID_INPUT', 'roi must lie in [0, 1] with x0 < x1 and y0 < y1')
    return values


@dataclass(frozen=True)
class Watch:
    watch_id: str
    camera_id: str
    target_id: str
    roi: tuple
    command_id: str  # the operator's SET_CAMERA_WATCH = association_evidence_id


@dataclass(frozen=True)
class AcceptedCandidate:
    event_id: str
    watch_id: str
    target_id: str
    observed_at_ns: int


class CameraWatchBook:
    """The coordinator's record of the one TARGET_WATCH camera's slot."""

    def __init__(self, camera_id, *, roi, request_timeout_ns, new_id):
        if not isinstance(camera_id, str) or not camera_id:
            raise CoordinationError('INVALID_POLICY', 'watch camera_id')
        if type(request_timeout_ns) is not int or request_timeout_ns <= 0:
            raise CoordinationError('INVALID_POLICY', 'watch request timeout must be a positive int')
        self.camera_id = camera_id
        self.roi = None if roi is None else roi_value(roi)  # deployment ROI; None = only the request's
        if roi is not None and self.roi is None:
            raise CoordinationError('INVALID_POLICY', 'an all-zero watch ROI is no ROI; leave it empty')
        self.request_timeout_ns = request_timeout_ns
        self.new_id = new_id
        self.revision = 0
        self.active = None       # Watch the camera accepted
        self.pending = None      # dict(fields, watch, enable) waiting for the camera's reply
        self.bindings = {}       # watch_id -> Watch, sent and not refused
        self.candidates = {}     # watch_id -> the one candidate event_id seen for it
        self.accepted = {}       # target_id -> latest AcceptedCandidate (row 5 links rechecks to it later)
        self.ended = False       # the active watch got its candidate and must be released
        self.rearm = None        # 'WATCH_REARM_NEEDED' until the next accepted arm
        self.last_detail = None  # camera reason / candidate reason behind the latest change, for the log
        self.outbox = []

    def arm(self, command_id, target_id, roi, now_ns):
        """Queue an enable request for a new watch_id. The caller checked mode, target and ``pending``."""
        watch = Watch(self.new_id(), self.camera_id, uid(target_id), roi, command_id)
        self.bindings[watch.watch_id] = watch
        self._send(watch, True, now_ns)
        return watch

    def release_due(self, should_end, now_ns):
        """Queue a disable request for the active watch when ``should_end`` and nothing is in flight."""
        if self.active is not None and self.pending is None and (should_end or self.ended):
            self._send(self.active, False, now_ns)
            return True
        return False

    def expire(self, now_ns):
        """Drop a request whose deadline passed without a reply (the camera may be down)."""
        if self.pending is not None and now_ns > self.pending['fields']['deadline_ns']:
            if self.pending['enable']:
                self.rearm, self.last_detail = 'WATCH_REARM_NEEDED', 'NO_REPLY'
            self.pending = None  # a release is queued again by release_due while its cause stands

    def on_reply(self, *, watch_id, accepted, watch_revision, reason_code):
        """The camera's answer to the pending request. False when it answers nothing pending."""
        pending = self.pending
        if pending is None or watch_id != pending['watch'].watch_id or type(watch_revision) is not int:
            return False
        self.pending, self.revision, self.last_detail = None, watch_revision, reason_code
        watch = pending['watch']
        if pending['enable']:
            if accepted:
                self.active, self.ended, self.rearm = watch, False, None
            else:
                self.bindings.pop(watch.watch_id, None)  # row 2: refused -> not recorded
                self.rearm = 'WATCH_REARM_NEEDED'
        elif accepted or reason_code != 'REVISION_MISMATCH':
            # Released, or the camera does not hold this watch (WATCH_MISMATCH, restart): nothing to release.
            if self.active is watch:
                self.active, self.ended = None, False
        return True

    def on_candidate(self, event):
        """C-11 rows 3-4 for a TARGET_DISAPPEARED_CANDIDATE in DISASTER mode. Returns the closed reason."""
        seen = self.candidates.get(event.watch_id)
        if seen is not None and seen != event.event_id:
            raise CoordinationError('ID_CONTENT_CONFLICT', 'another candidate event_id for the same watch_id')
        self.candidates[event.watch_id] = event.event_id
        watch = self.bindings.get(event.watch_id)
        if watch is None or watch is not self.active or (watch.camera_id, watch.target_id) != (
                event.camera_id, event.target_id):
            reason = 'CANDIDATE_NOT_LINKED'
        elif not event.time_basis_declared:
            reason = 'CANDIDATE_TIME_BASIS_UNUSABLE'
        else:
            reason = 'CANDIDATE_ACCEPTED'
            self.accepted[watch.target_id] = AcceptedCandidate(event.event_id, watch.watch_id, watch.target_id,
                                                              event.observed_at_ns)
        if watch is not None and watch is self.active:
            self.ended = True  # row 2: a watch ends with its one candidate, accepted or refused
        if reason != 'CANDIDATE_ACCEPTED':
            self.rearm = 'WATCH_REARM_NEEDED'
        self.last_detail = reason
        return reason

    def drain(self):
        sent, self.outbox = self.outbox, []
        return sent

    def _send(self, watch, enable, now_ns):
        fields = dict(command_id=self.new_id(), camera_id=watch.camera_id, watch_id=watch.watch_id,
                      target_id=watch.target_id, expected_watch_revision=self.revision,
                      roi_xyxy_normalized=watch.roi, association_method='OPERATOR_ROI',
                      association_evidence_id=watch.command_id, enable=enable, issued_at_ns=now_ns,
                      deadline_ns=now_ns + self.request_timeout_ns)
        self.pending = dict(fields=fields, watch=watch, enable=enable)
        self.outbox.append(fields)
