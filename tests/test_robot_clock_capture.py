"""chrony estimates are measured inputs; no adjustment commands or defaults."""
import unittest
from rescue_robot.clock_capture import parse_tracking, combine_measurements

TRACKING='''Reference ID    : CB00710F (ntp.example)
Stratum         : 3
System time     : 0.000006523 seconds slow of NTP time
Root delay      : 0.013639022 seconds
Root dispersion : 0.001100737 seconds
Skew            : 0.129 ppm
Leap status     : Normal
'''

class ClockCaptureTests(unittest.TestCase):
    def test_bound_contains_remaining_correction_root_delay_and_dispersion(self):
        p=parse_tracking(TRACKING)
        self.assertEqual(p['state'],'SYNCED')
        self.assertEqual(p['error_bound_ns'],7_926_771)

    def test_unsynced_local_mode_and_missing_or_nan_values_never_become_synced(self):
        for value in (TRACKING.replace('Normal','Not synchronised'),
                      TRACKING.replace('CB00710F','7F7F0101'),
                      TRACKING.replace('0.013639022','nan'), ''):
            self.assertEqual(parse_tracking(value)['state'],'UNKNOWN')

    def test_two_host_bounds_are_added_and_epoch_is_bound_to_both_boots(self):
        a=dict(parse_tracking(TRACKING),host_id='sender',host_boot_id='a',
               clock_domain_id='ntp-team',measured_at_ns=1000)
        b=dict(a,host_id='receiver',host_boot_id='b')
        basis=combine_measurements(a,b,domain_id='ntp-team',max_age_ns=100,max_error_ns=20_000_000,now_ns=1050)
        self.assertEqual(basis.error_bound_ns,15_853_542)
        self.assertEqual(basis.valid_until_ns,1100)
        with self.assertRaises(Exception): combine_measurements(a,b,domain_id='other',max_age_ns=100,max_error_ns=20_000_000,now_ns=1050)
        with self.assertRaises(Exception): combine_measurements(a,b,domain_id='ntp-team',max_age_ns=100,max_error_ns=10,now_ns=1050)
        with self.assertRaises(Exception): combine_measurements(a,b,domain_id='ntp-team',max_age_ns=100,max_error_ns=20_000_000,now_ns=1100)
