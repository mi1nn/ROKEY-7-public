"""TB-6 observation clock is measured, never a code value (OPS-103 decision 1, 2026-10-13).

amr_target had clock_state='UNKNOWN' and clock_error_bound_ns=-1 in code (target_utils SETTINGS), so
every TargetObservation was refused downstream (C-03). The clock is now measured on the host that runs
amr_target the same way as the webcam (#146: rescue_contracts ClockSampler + webcam_clock): SYNCED
with a bound only while the latest measurement is fresh at the frame's capture time, else UNKNOWN.

The OAK-D stamp comes from the camera driver host (TB-6 Pi), not from this PC (OPS-103 decision 1
addendum, 2026-10-13): its error bound and clock domain are field values; the bound adds to the PC's,
and a different domain is never SYNCED.
"""
import unittest

from rescue_perception import SourceBinding
from rescue_perception.target_utils import CLOCK_PARAMS, SETTINGS, SensorHostClock, clock_binding, clock_values

MEASURED = 1_760_000_000_000_000_000
SAMPLED = MEASURED + 5_000_000_000
BINDING = SourceBinding('robot_a', 'amr_rgbd', '00000000-0000-4000-8000-000000000100', 'rokey7-utc-field',
                        'CAPTURE', 'CAPTURE', 'cal', 'UNKNOWN', None)


PI = SensorHostClock(error_bound_ns=2_000_000, clock_domain_id='rokey7-utc-field')


def record(**changes):
    values = dict(state='SYNCED', measured_at_ns=MEASURED, sampled_at_ns=SAMPLED,
                  valid_until_ns=MEASURED + 3_600_000_000_000, error_bound_ns=4_000_000, drift_ppm=100.0)
    values.update(changes)
    return values


class AmrClockTests(unittest.TestCase):
    def test_no_clock_value_in_code(self):
        for name in ('clock_state', 'clock_error_bound_ns', 'clock_domain_id'):
            self.assertNotIn(name, SETTINGS)
        self.assertEqual({'clock_domain_id': 'STRING', 'clock_backend': 'STRING', 'clock_max_age_ns': 'INTEGER',
                          'clock_drift_ppm': 'DOUBLE', 'clock_poll_period_s': 'DOUBLE',
                          'sensor_host_clock_error_bound_ns': 'INTEGER', 'sensor_host_clock_domain_id': 'STRING'},
                         CLOCK_PARAMS)

    def test_frame_clock_follows_the_measurement(self):
        synced = clock_binding(BINDING, record(), SAMPLED + 10_000_000_000, PI)
        self.assertEqual(('SYNCED', 5_000_000 + 2_000_000), (synced.clock_state, synced.clock_error_bound_ns))
        self.assertEqual(BINDING.calibration_id, synced.calibration_id)
        for stale in (record(state='UNKNOWN', reason='waiting'), record(valid_until_ns=SAMPLED)):
            unknown = clock_binding(BINDING, stale, SAMPLED + 1, PI)
            self.assertEqual(('UNKNOWN', None), (unknown.clock_state, unknown.clock_error_bound_ns))

    def test_sensor_host_in_another_clock_domain_is_never_synced(self):
        other = SensorHostClock(error_bound_ns=0, clock_domain_id='tb6-pi-unsynced')
        unknown = clock_binding(BINDING, record(), SAMPLED + 1, other)
        self.assertEqual(('UNKNOWN', None), (unknown.clock_state, unknown.clock_error_bound_ns))

    def test_clock_values_are_checked(self):
        good = dict(clock_domain_id='rokey7-utc-field', clock_backend='timesyncd', clock_max_age_ns=3_600_000_000_000,
                    clock_drift_ppm=100.0, clock_poll_period_s=5.0, sensor_host_clock_error_bound_ns=2_000_000,
                    sensor_host_clock_domain_id='rokey7-utc-field')
        self.assertEqual('timesyncd', clock_values(good)['clock_backend'])
        for bad in (dict(clock_domain_id=''), dict(clock_backend='ntp'), dict(clock_max_age_ns=0),
                    dict(clock_drift_ppm=0.0), dict(clock_poll_period_s=0.0), dict(clock_domain_id=None),
                    dict(sensor_host_clock_error_bound_ns=-1), dict(sensor_host_clock_error_bound_ns=None),
                    dict(sensor_host_clock_domain_id=' ')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                clock_values(good | bad)


if __name__ == '__main__':
    unittest.main()
