"""Webcam clock fields come from an OS clock measurement, never from a fixed parameter (A2, 2026-10-13).

Before this, clock_state/clock_error_bound_ns were parameters: the example says UNKNOWN, so a real
quake was refused by the coordinator (TIME_BASIS_UNUSABLE), and writing SYNCED there would have
declared a clock nobody measured. Now each message takes the latest measurement record
(rescue_contracts.clock_measurement.ClockSampler); without a fresh SYNCED record it is UNKNOWN.
"""
import math
import unittest

from rescue_perception.webcam_clock import clock_settings, observation_clock

MEASURED = 1_760_000_000_000_000_000
SAMPLED = MEASURED + 5_000_000_000
MAX_AGE = 3_600_000_000_000


def record(**changes):
    values = dict(state='SYNCED', measured_at_ns=MEASURED, sampled_at_ns=SAMPLED,
                  valid_until_ns=MEASURED + MAX_AGE, error_bound_ns=4_000_000, drift_ppm=100.0)
    values.update(changes)
    return values


class ObservationClockTests(unittest.TestCase):
    def test_fresh_synced_measurement_gives_synced_with_bound(self):
        self.assertEqual(('SYNCED', 4_000_000), observation_clock(record(), SAMPLED))

    def test_bound_grows_with_drift_after_the_sample(self):
        later = SAMPLED + 10_000_000_000  # 10 s at 100 ppm -> +1 ms
        self.assertEqual(('SYNCED', 5_000_000), observation_clock(record(), later))

    def test_unknown_without_a_usable_measurement(self):
        cases = {
            'waiting': dict(state='UNKNOWN', reason='waiting for clock measurement'),
            'expired': record(valid_until_ns=SAMPLED),
            'before measurement': record(measured_at_ns=SAMPLED + 1),
            'no bound': record(error_bound_ns=None),
            'negative bound': record(error_bound_ns=-1),
            'bad drift': record(drift_ppm=math.nan),
        }
        for name, rec in cases.items():
            with self.subTest(name):
                self.assertEqual(('UNKNOWN', None), observation_clock(rec, SAMPLED))


class ClockSettingsTests(unittest.TestCase):
    def test_backend_and_budgets_are_explicit(self):
        self.assertEqual(('timesyncd', MAX_AGE, 100.0),
                         clock_settings(dict(clock_backend='timesyncd', clock_max_age_ns=MAX_AGE,
                                             clock_drift_ppm=100.0)))
        self.assertEqual('NONE', clock_settings(dict(clock_backend='NONE', clock_max_age_ns=MAX_AGE,
                                                     clock_drift_ppm=100.0))[0])
        for bad in (dict(clock_backend='ntp'), dict(clock_max_age_ns=0), dict(clock_drift_ppm=0.0),
                    dict(clock_drift_ppm=math.inf)):
            values = dict(clock_backend='timesyncd', clock_max_age_ns=MAX_AGE, clock_drift_ppm=100.0) | bad
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                clock_settings(values)

    def test_node_has_no_fixed_clock_state_parameter(self):
        from rescue_perception.webcam_node import PARAMS
        self.assertNotIn('clock_state', PARAMS)
        self.assertNotIn('clock_error_bound_ns', PARAMS)
        self.assertEqual(('STRING', 'INTEGER', 'DOUBLE'),
                         (PARAMS['clock_backend'], PARAMS['clock_max_age_ns'], PARAMS['clock_drift_ppm']))


if __name__ == '__main__':
    unittest.main()
