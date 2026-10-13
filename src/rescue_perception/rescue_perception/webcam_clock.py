"""Clock fields of webcam messages from an OS clock measurement (A2, 2026-10-13). No ROS.

The measurement is rescue_contracts.clock_measurement (ClockSampler: timesyncd or chrony,
read-only). A message is SYNCED only while the latest record is SYNCED and fresh at the
message's observed time; its bound then grows with the agreed drift since the sample.
Otherwise UNKNOWN with no bound. Nothing here writes SYNCED from configuration.
"""
import math

BACKENDS = ('timesyncd', 'chrony', 'NONE')  # NONE: no measurement on this PC -> always UNKNOWN


def clock_settings(v):
    backend, max_age, drift = v['clock_backend'], v['clock_max_age_ns'], v['clock_drift_ppm']
    if backend not in BACKENDS:
        raise ValueError(f'clock_backend must be one of {BACKENDS}')
    if type(max_age) is not int or max_age <= 0:
        raise ValueError('clock_max_age_ns must be a positive integer')
    if type(drift) is not float or not math.isfinite(drift) or drift <= 0:
        raise ValueError('clock_drift_ppm must be finite and positive')
    return backend, max_age, drift


def observation_clock(record, observed_ns):
    """(clock_state, error_bound_ns or None) for a message observed at ``observed_ns``."""
    unknown = ('UNKNOWN', None)
    if record.get('state') != 'SYNCED':
        return unknown
    bound, measured, sampled = record.get('error_bound_ns'), record.get('measured_at_ns'), record.get('sampled_at_ns')
    until, drift = record.get('valid_until_ns'), record.get('drift_ppm')
    if (any(type(x) is not int for x in (bound, measured, sampled, until)) or bound < 0
            or type(drift) is not float or not math.isfinite(drift) or drift <= 0):
        return unknown
    if not measured <= observed_ns < until:
        return unknown
    return 'SYNCED', bound + math.ceil(max(0, observed_ns - sampled) * drift / 1e6)
