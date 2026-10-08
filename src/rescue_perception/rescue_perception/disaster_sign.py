"""Fixed camera A: edge-based shake/occlusion sign -> SiteEvent(DISASTER_SIGN) (E2 only).

No model, ROS, storage or mode change. Two candidate indicators from TR-06
("영상 엣지 기반 지표"), both computed on a strided grayscale grid:

- shake: fraction of edge cells that changed between consecutive frames
  (|E_t xor E_t-1| / |E_t or E_t-1|), high when the whole view moves;
- occlusion: edge density (edge cells / cells), low when the lens is covered.

Every threshold, the AND/OR rule, the hold time and the release condition come
from DisasterSignPolicy (Q-11, no defaults). The event is evidence; coordination
decides the mode (contracts C-01).
"""
from dataclasses import dataclass
import math
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import RescueError, SiteEvent, camera_id, duration_ns

from .adapter import SourceBinding, rgb_frame
from .core import PerceptionError

EVENT_NAMESPACE = uuid5(NAMESPACE_URL, "rescue_perception/disaster_sign/v1")
# Luma weights (ITU-R BT.601) for 8-bit colour images; channel order per encoding.
_LUMA = {"rgb8": (0.299, 0.587, 0.114), "bgr8": (0.114, 0.587, 0.299)}
_CHANNELS = {"rgb8": 3, "bgr8": 3, "mono8": 1}
RULES = ("ANY", "ALL")


def _require(condition, code, detail):
    if not condition:
        raise PerceptionError(code, detail)


def _contract(check, value, label):
    try:
        return check(value)
    except RescueError as exc:
        raise PerceptionError("INVALID_INPUT", f"{label}: {exc.detail}") from exc


def _finite(value, name, low, high):
    _require(type(value) in (int, float) and math.isfinite(value) and low <= value <= high,
             "INVALID_INPUT", name)


@dataclass(frozen=True)
class DisasterSignPolicy:
    """Q-11 values, supplied by the deployment; nothing here is a default.

    ``use_shake``/``use_occlusion`` choose indicators; ``rule`` combines the chosen
    ones (ANY = OR, ALL = AND). The combined condition must hold continuously for
    ``hold_ns`` to emit one event, and be false continuously for ``release_ns``
    before another event is possible. A gap over ``max_gap_ns`` between frames
    breaks both runs (missing frames are not evidence either way).
    """
    sample_stride: int
    edge_gradient_min: float
    use_shake: bool
    shake_change_min: float
    use_occlusion: bool
    occlusion_density_max: float
    rule: str
    hold_ns: int
    release_ns: int
    max_gap_ns: int

    def __post_init__(self):
        _require(type(self.sample_stride) is int and self.sample_stride >= 1, "INVALID_INPUT", "sample_stride")
        _finite(self.edge_gradient_min, "edge_gradient_min", 0, 510)
        _require(self.edge_gradient_min > 0, "INVALID_INPUT", "edge_gradient_min must be positive")
        _require(type(self.use_shake) is bool and type(self.use_occlusion) is bool, "INVALID_INPUT", "indicator flags")
        _require(self.use_shake or self.use_occlusion, "INVALID_INPUT", "no indicator selected")
        _finite(self.shake_change_min, "shake_change_min", 0, 1)
        _finite(self.occlusion_density_max, "occlusion_density_max", 0, 1)
        _require(self.rule in RULES, "INVALID_INPUT", "rule")
        for name in ("hold_ns", "release_ns", "max_gap_ns"):
            _contract(duration_ns, getattr(self, name), name)
        _require(self.max_gap_ns > 0, "INVALID_INPUT", "max_gap_ns must be positive")


@dataclass(frozen=True)
class SignFrame:
    """Per-frame indicator values. ``shake`` is None for the first frame of a run."""
    status: str
    shake: float | None
    edge_density: float | None
    condition: bool
    event: SiteEvent | None


def _gray_grid(image, stride):
    _require(image.encoding in _CHANNELS, "INVALID_INPUT", f"unsupported encoding {image.encoding!r}")
    channels = _CHANNELS[image.encoding]
    data = image.data.tobytes() if hasattr(image.data, "tobytes") else bytes(image.data)
    _require(image.step >= image.width * channels, "INVALID_INPUT", "step shorter than row")
    weights = _LUMA.get(image.encoding)
    grid = []
    for y in range(0, image.height, stride):
        base, row = y * image.step, []
        for x in range(0, image.width, stride):
            i = base + x * channels
            row.append(data[i] if weights is None else
                       weights[0] * data[i] + weights[1] * data[i + 1] + weights[2] * data[i + 2])
        grid.append(row)
    return grid


def _edges(grid, gradient_min):
    """Forward-difference |gx| + |gy| on the sampled grid; last row/column excluded."""
    rows, cols = len(grid) - 1, len(grid[0]) - 1
    _require(rows >= 1 and cols >= 1, "INVALID_INPUT", "image too small for sample_stride")
    return frozenset((y, x) for y in range(rows) for x in range(cols)
                     if abs(grid[y][x + 1] - grid[y][x]) + abs(grid[y + 1][x] - grid[y][x]) >= gradient_min), \
        rows * cols


class DisasterSignDetector:
    """One fixed camera, one process; state only moves forward in sensor time.

    Emits at most one DISASTER_SIGN per episode (hold satisfied), re-armed only
    after the release condition. Duplicate/older frames change nothing.
    """

    def __init__(self, binding, policy):
        _require(isinstance(binding, SourceBinding) and binding.source_robot is None, "INVALID_INPUT",
                 "fixed camera binding must have source_robot=None")
        _require(isinstance(policy, DisasterSignPolicy), "INVALID_INPUT", "policy")
        self.camera_id = _contract(camera_id, binding.source_camera, "camera_id")
        self.binding, self.policy = binding, policy
        self._last_ns = None
        self._previous_edges = None
        self._true_since = None
        self._false_since = None
        self._armed = True

    def _condition(self, shake, density):
        p, checks = self.policy, []
        if p.use_shake:
            checks.append(shake is not None and shake >= p.shake_change_min)
        if p.use_occlusion:
            checks.append(density <= p.occlusion_density_max)
        return any(checks) if p.rule == "ANY" else all(checks)

    def update(self, rgb):
        frame, evidence_sha256 = rgb_frame(rgb, self.binding)
        stamp = frame.observed_at_ns
        if self._last_ns is not None and stamp <= self._last_ns:
            return SignFrame("DUPLICATE" if stamp == self._last_ns else "REORDERED", None, None, False, None)
        if self._last_ns is not None and stamp - self._last_ns > self.policy.max_gap_ns:
            self._previous_edges, self._true_since, self._false_since = None, None, None
        self._last_ns = stamp
        edges, cells = _edges(_gray_grid(rgb, self.policy.sample_stride), self.policy.edge_gradient_min)
        density = len(edges) / cells
        shake = None
        if self._previous_edges is not None:
            union = edges | self._previous_edges
            shake = len(edges ^ self._previous_edges) / len(union) if union else 0.0
        self._previous_edges = edges
        condition = self._condition(shake, density)
        event = None
        if condition:
            self._false_since = None
            self._true_since = stamp if self._true_since is None else self._true_since
            if self._armed and stamp - self._true_since >= self.policy.hold_ns:
                self._armed = False
                event = self._event(frame, evidence_sha256, shake, density)
        else:
            self._true_since = None
            self._false_since = stamp if self._false_since is None else self._false_since
            if not self._armed and stamp - self._false_since >= self.policy.release_ns:
                self._armed = True
        return SignFrame("PROCESSED", shake, density, condition, event)

    def _event(self, frame, evidence_sha256, shake, density):
        p, stamp = self.policy, frame.observed_at_ns
        detail = (f"rule={p.rule} shake={'n/a' if shake is None else f'{shake:.4f}'}"
                  f"{f'>={p.shake_change_min}' if p.use_shake else '(unused)'} "
                  f"edge_density={density:.4f}{f'<={p.occlusion_density_max}' if p.use_occlusion else '(unused)'} "
                  f"onset_ns={self._true_since} hold_ns={p.hold_ns} stride={p.sample_stride}")
        # Rule-based sign, not a calibrated probability: 1.0 means "policy satisfied".
        return SiteEvent(str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/disaster")), self.camera_id,
                         "DISASTER_SIGN", stamp, str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/frame")),
                         evidence_sha256, 1.0, frame.clock_domain_id, frame.stamp_semantics,
                         self.binding.clock_state, self.binding.clock_error_bound_ns, detail)
