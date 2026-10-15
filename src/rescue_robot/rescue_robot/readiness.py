"""Readiness flags from message arrival freshness. Imports no ROS package.

Each source (an input topic such as odom, scan, a camera, map->odom TF, the
robot-side watchdog heartbeat) records its last local monotonic arrival. A flag
is True only while every source mapped to it arrived within that source's
max age. A flag with no configured source is never produced, so the adapter's
Readiness stays not-ready (watchdog has no source until a robot-side command
watchdog with a heartbeat exists).

Freshness here is link liveness only. It does not judge sensor header age
(perception owns that) and never treats TF changes as a SLAM correction.
All ages come from a ReadinessPolicy; no field value is defined here [미확인].
"""
from dataclasses import dataclass
import math
import threading

from rescue_robot.executor import ContractError

# nav2 (action server ready + Nav2 lifecycle manager is_active, ros_node.Nav2Port) and undocked (dock_status)
# already have producers.
MONITORED_FLAGS = ('driver', 'sensors', 'slam', 'watchdog')


@dataclass(frozen=True)
class ReadinessPolicy:
    """version, {source: max_age_s}, {flag: (source, ...)}; all caller-supplied."""
    version: str
    max_age_s: dict
    flag_sources: dict

    def __post_init__(self):
        if not isinstance(self.version, str) or not self.version:
            raise ContractError('readiness policy version')
        if not isinstance(self.max_age_s, dict) or not isinstance(self.flag_sources, dict):
            raise ContractError('readiness policy tables')
        for source, age in self.max_age_s.items():
            if not isinstance(source, str) or not source:
                raise ContractError('readiness source name')
            if type(age) not in (int, float) or not 0 < age < math.inf:
                raise ContractError('max_age_s for ' + source)
        for flag, sources in self.flag_sources.items():
            if flag not in MONITORED_FLAGS:
                raise ContractError('flag without a monitor: ' + str(flag))
            if not isinstance(sources, tuple) or not sources:
                raise ContractError('flag sources must be a nonempty tuple')
            for source in sources:
                if source not in self.max_age_s:
                    raise ContractError('source without max age: ' + str(source))


class ReadinessMonitor:
    def __init__(self, policy):
        if not isinstance(policy, ReadinessPolicy):
            raise ContractError('readiness policy type')
        self.policy = policy
        self.lock = threading.Lock()
        self.last_arrival = {}

    def arrived(self, source, received_steady):
        with self.lock:
            if source not in self.policy.max_age_s:
                raise ContractError('unknown readiness source ' + str(source))
            if type(received_steady) not in (int, float) or not math.isfinite(received_steady):
                raise ContractError('received_steady')
            previous = self.last_arrival.get(source)
            # An out-of-order report cannot make a source look fresher.
            if previous is None or received_steady > previous:
                self.last_arrival[source] = float(received_steady)

    def flags(self, now):
        """{flag: bool} for configured flags only; missing or old sources -> False."""
        with self.lock:
            result = {}
            for flag, sources in self.policy.flag_sources.items():
                result[flag] = all(
                    source in self.last_arrival
                    and 0 <= now - self.last_arrival[source] <= self.policy.max_age_s[source]
                    for source in sources)
            return result

    def not_fresh(self, now):
        """'source(age s)' or 'source(never)' for every configured source that is missing or too old."""
        with self.lock:
            out = []
            for source in sorted({s for sources in self.policy.flag_sources.values() for s in sources}):
                last = self.last_arrival.get(source)
                if last is None:
                    out.append(f'{source}(never)')
                elif not 0 <= now - last <= self.policy.max_age_s[source]:
                    out.append(f'{source}({now - last:.1f} s)')
            return out

    def publish(self, adapter):
        """Push current flags into a CommandAdapter (stamped by its own clock)."""
        flags = self.flags(adapter.clock())
        if flags:
            adapter.observe_status(**flags)
        return flags
