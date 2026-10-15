"""timesyncd replies: which are used and why one is not (9회차 2026-10-16, C13).

The coordinator start stopped twice on ``central live clock: no valid synchronized NTP reply``: timesyncd had
flagged its last reply (03:21:15, delay 317 ms, offset +70 ms) as a spike. That reply is used now with its bound
(SpikeAcceptedTests, C13 A); a reply not used says why with NO_NTP_REPLY or NOT_SYNCHRONIZED.
"""
import unittest

from rescue_contracts import clock_measurement as cm

T1 = 1_792_088_475_000_000  # µs, 2026-10-16 03:21:15 KST (18:21:15 UTC the day before), the 9회차 last reply
NOW = (T1 + 60_000_000) * 1000  # a minute later, ns


def message(*, spike=False, count=9):
    """NTPMessage fields (systemd v255 order) for offset +70 ms and delay 317 ms."""
    t2 = T1 + 228_500  # offset + delay / 2
    t3 = t2
    t4 = t3 + 88_500   # delay / 2 - offset
    return [0, 4, 4, 2, -20, 10_000, 20_000, [1, 2, 3, 4], T1, t2, t3, t4, spike, count, 5_000]


def parse(msg, synchronized=True, **changes):
    return cm.parse_timesync(msg, synchronized=synchronized, now_ns=NOW, max_age_ns=3_600_000_000_000,
                             drift_ppm=100.0, **changes)


class ReasonTests(unittest.TestCase):
    def reason(self, msg, **kwargs):
        with self.assertRaises(ValueError) as caught:
            parse(msg, **kwargs)
        return caught.exception

    def test_not_synchronized_is_its_own_reason(self):
        self.assertTrue(str(self.reason(message(), synchronized=False)).startswith('NOT_SYNCHRONIZED'))

    def test_no_reply_yet(self):
        empty = [0, 0, 0, 0, 0, 0, 0, [], 0, 0, 0, 0, False, 0, 0]
        self.assertTrue(str(self.reason(empty)).startswith('NO_NTP_REPLY'))

    def test_a_good_reply_is_still_synced(self):
        self.assertEqual('SYNCED', parse(message())['state'])


class SpikeAcceptedTests(unittest.TestCase):
    """C13 A (재범 결정 대기): a reply timesyncd flagged as a spike is still a valid NTP measurement; timesyncd
    only did not correct the clock with it (systemd v255 timesyncd-manager.c 571). The bound already holds the
    whole offset and half the round trip (NTP: the true offset is within offset +- delay / 2), so the uncorrected
    clock is covered. 9회차 values: about 0.26 s, inside the field 2 s (task.lease.max_error_ns, clock_error_bound_s).
    """

    def test_a_spike_reply_is_synced_with_a_bound_over_offset_and_half_the_delay(self):
        record = parse(message(spike=True))
        self.assertEqual(('SYNCED', True), (record['state'], record['ntp_spike']))
        self.assertGreaterEqual(record['error_bound_ns'], 70_000_000 + 317_000_000 // 2)
        self.assertLess(record['error_bound_ns'], 2_000_000_000)

    def test_a_wide_spike_reply_gets_a_bound_over_the_field_2_s_limit(self):
        # 2026-10-15 field: delay 7.5 s, offset -3.6 s; a bound over 2 s is past the field limits (as before)
        t2 = T1 + 150_000
        wide = [0, 4, 4, 2, -20, 10_000, 20_000, [1, 2, 3, 4], T1, t2, t2, T1 + 7_500_000, True, 9, 5_000]
        self.assertGreater(parse(wide)['error_bound_ns'], 2_000_000_000)


class LogLineTests(unittest.TestCase):
    def test_the_start_log_says_a_spike_reply_is_used_with_its_bound(self):
        from rescue_coordination import live_task_inputs
        record = parse(message(spike=True))
        line = live_task_inputs.clock_log_line(record)
        self.assertTrue(line.startswith('central live clock: SYNCED (timesyncd spike reply used, bound '), line)
        self.assertEqual('central live clock: SYNCED', live_task_inputs.clock_log_line(parse(message())))
        self.assertEqual('central live clock: NO_NTP_REPLY: x',
                         live_task_inputs.clock_log_line(dict(state='UNKNOWN', reason='NO_NTP_REPLY: x')))


if __name__ == '__main__':
    unittest.main()
