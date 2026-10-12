"""#89 AST88-02 reception checks; local stop timers remain steady-clock timers.

No production defaults. ClockBasis must come from an authenticated, current
measurement of BOTH hosts; message fields alone do not establish a clock basis.
"""
from dataclasses import dataclass

from rescue_robot.executor import ContractError, number, wall_time


@dataclass(frozen=True)
class LeasePolicy:
    duration_s: float
    max_age_ns: int
    max_error_ns: int
    clock_domain_id: str

    def __post_init__(self):
        number(self.duration_s, 'lease_duration', positive=True)
        for name in ('max_age_ns', 'max_error_ns'):
            value = getattr(self, name)
            if type(value) is not int or not 0 < value < 2**63:
                raise ContractError('explicit positive lease policy required: ' + name)
        if not isinstance(self.clock_domain_id, str) or not self.clock_domain_id:
            raise ContractError('declared clock domain required', 'CLOCK_UNCERTAIN')


@dataclass(frozen=True)
class ClockBasis:
    domain_id: str
    state: str
    error_bound_ns: int | None  # sum of measured sender/receiver bounds
    epoch_id: str              # changes after a clock step/restart
    valid_until_ns: int         # validity of the measurement, not the lease


def validate_clock_basis(basis, *, policy, now_ns):
    wall_time(now_ns)
    if not isinstance(policy, LeasePolicy):
        raise ContractError('explicit LeasePolicy required')
    if (not isinstance(basis, ClockBasis) or basis.domain_id != policy.clock_domain_id
            or basis.state != 'SYNCED' or type(basis.error_bound_ns) is not int
            or not 0 <= basis.error_bound_ns <= policy.max_error_ns
            or not isinstance(basis.epoch_id, str) or not basis.epoch_id):
        raise ContractError('verified common SYNCED clock basis required', 'CLOCK_UNCERTAIN')
    if wall_time(basis.valid_until_ns) <= now_ns + basis.error_bound_ns:
        raise ContractError('clock measurement expired', 'CLOCK_UNCERTAIN')
    return basis.error_bound_ns


def validate_lease_reception(lease, *, policy, basis, now_ns):
    """Validate absolute message reception; never derive a steady deadline here."""
    error = validate_clock_basis(basis, policy=policy, now_ns=now_ns)
    issued, until = wall_time(lease.issued_at_ns), wall_time(lease.valid_until_ns)
    if issued <= 0 or until <= issued:
        raise ContractError('lease reception interval')
    if issued > now_ns + error:
        raise ContractError('lease issued beyond clock uncertainty', 'CLOCK_UNCERTAIN')
    if now_ns - issued + error > policy.max_age_ns:
        raise ContractError('lease message too old', 'DEADLINE_EXPIRED')
    if now_ns + error >= until:
        raise ContractError('lease reception expired', 'DEADLINE_EXPIRED')
    return True
