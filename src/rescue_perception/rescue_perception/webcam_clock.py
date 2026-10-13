"""웹캠 메시지의 시계 칸을 OS 시계 측정값으로 채운다(A2, 2026-10-13). ROS 없음.

측정은 rescue_contracts.clock_measurement(ClockSampler: timesyncd 또는 chrony, 읽기만 함)가 한다.
최신 측정 기록이 SYNCED 이고 메시지의 관측 시각에 아직 유효할 때만 그 메시지는 SYNCED 다.
이때 오차 상한은 측정 뒤 흐른 시간만큼 합의한 drift 로 늘린다. 아니면 오차 상한 없이 UNKNOWN 이다.
설정값만으로 SYNCED 를 쓰는 곳은 없다.
"""
import math

# 시계 측정 방식. timesyncd·chrony 는 OS 시계 서비스 상태를 읽기만 한다.
BACKENDS = ('timesyncd', 'chrony', 'NONE')  # NONE: 이 PC 에서 측정하지 않음 → 항상 UNKNOWN


def clock_settings(v):
    """YAML 시계 설정 검사. 틀리면 ValueError(노드 기동 거부)."""
    backend, max_age, drift = v['clock_backend'], v['clock_max_age_ns'], v['clock_drift_ppm']
    if backend not in BACKENDS:
        raise ValueError(f'clock_backend must be one of {BACKENDS}')
    if type(max_age) is not int or max_age <= 0:
        raise ValueError('clock_max_age_ns must be a positive integer')
    if type(drift) is not float or not math.isfinite(drift) or drift <= 0:
        raise ValueError('clock_drift_ppm must be finite and positive')
    return backend, max_age, drift


def observation_clock(record, observed_ns):
    """``observed_ns`` 에 찍은 메시지의 (clock_state, 오차 상한 ns 또는 None)."""
    unknown = ('UNKNOWN', None)
    if record.get('state') != 'SYNCED':
        return unknown
    bound, measured, sampled = record.get('error_bound_ns'), record.get('measured_at_ns'), record.get('sampled_at_ns')
    until, drift = record.get('valid_until_ns'), record.get('drift_ppm')
    if (any(type(x) is not int for x in (bound, measured, sampled, until)) or bound < 0
            or type(drift) is not float or not math.isfinite(drift) or drift <= 0):
        return unknown
    # 측정 시각 이후이고 유효 기간 안에 찍은 프레임만 SYNCED.
    if not measured <= observed_ns < until:
        return unknown
    # 오차 상한 = 측정 당시 상한 + (측정 뒤 흐른 시간 × drift ppm / 1e6). 측정이 오래될수록 넓어진다.
    return 'SYNCED', bound + math.ceil(max(0, observed_ns - sampled) * drift / 1e6)
