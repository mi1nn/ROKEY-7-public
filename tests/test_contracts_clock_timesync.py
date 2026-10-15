"""A timesyncd reply that is not used says why (9회차 2026-10-16, C13 B).

The coordinator start stopped twice on ``central live clock: no valid synchronized NTP reply``: timesyncd had
flagged its last reply (03:21:15, delay 317 ms, offset +70 ms) as a spike and the next poll was minutes away. That
one message stood for every cause, so nobody could tell a spike from no synchronisation. Now the reason starts with
NO_NTP_REPLY, NOT_SYNCHRONIZED or NTP_SPIKE; a spike names the ignored reply and the next poll (the reply time plus
timesyncd's PollIntervalUSec, which it already halved for the spike, systemd v255 timesyncd-manager.c 388), and
the coordinator start log shows the seconds left. Which replies are used is unchanged (A is the owner's decision).
"""
from concurrent.futures import wait
import json
import types
import unittest

from rescue_contracts import clock_measurement as cm
from rescue_coordination import live_task_inputs

T1 = 1_792_088_475_000_000  # µs, 2026-10-16 03:21:15 KST (18:21:15 UTC the day before), the 9회차 last reply
NOW = (T1 + 60_000_000) * 1000  # a minute later, ns
POLL_US = 512_000_000


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

    def test_a_spike_names_the_reply_and_the_next_poll(self):
        error = self.reason(message(spike=True), poll_interval_us=POLL_US)
        self.assertEqual('NTP_SPIKE: last reply 18:21:15 UTC ignored by timesyncd (delay 317 ms, offset +70 ms), '
                         'next poll by 18:29:47 UTC', str(error))
        self.assertEqual((T1 + 88_500 + 228_500 + POLL_US) * 1000, error.next_poll_ns)

    def test_a_spike_without_the_poll_interval_still_says_spike(self):
        error = self.reason(message(spike=True))
        self.assertTrue(str(error).startswith('NTP_SPIKE: last reply 18:21:15 UTC ignored by timesyncd'))
        self.assertIsNone(error.next_poll_ns)

    def test_not_synchronized_is_its_own_reason(self):
        self.assertTrue(str(self.reason(message(), synchronized=False)).startswith('NOT_SYNCHRONIZED'))

    def test_no_reply_yet(self):
        empty = [0, 0, 0, 0, 0, 0, 0, [], 0, 0, 0, 0, False, 0, 0]
        self.assertTrue(str(self.reason(empty)).startswith('NO_NTP_REPLY'))

    def test_a_good_reply_is_still_synced(self):
        self.assertEqual('SYNCED', parse(message())['state'])


class CollectTests(unittest.TestCase):
    def test_collect_reads_the_poll_interval_for_a_spike(self):
        calls = []

        def run(args, **_):
            calls.append(args)
            out = {'is-active': 'active', 'NTPSynchronized': 'yes'}
            if args[0] == 'busctl':
                out = (json.dumps({'type': '(uuuuittayttttbtt)', 'data': message(spike=True)})
                       if args[-1] == 'NTPMessage' else json.dumps({'type': 't', 'data': POLL_US}))
            elif 'is-active' in args:
                out = 'active'
            elif 'timedatectl' in args[0]:
                out = 'yes'
            else:
                out = 'invocation-1'
            return types.SimpleNamespace(returncode=0, stdout=out, stderr='')

        with self.assertRaises(ValueError) as caught:
            cm.collect('timesyncd', max_age_ns=3_600_000_000_000, drift_ppm=100.0, run=run, now_ns=lambda: NOW)
        self.assertIn('next poll by 18:29:47 UTC', str(caught.exception))
        self.assertIn('PollIntervalUSec', [a[-1] for a in calls if a[0] == 'busctl'])


class SamplerTests(unittest.TestCase):
    def test_the_record_keeps_the_next_poll(self):
        def collector(*_, **__):
            raise cm.ClockUnavailable('NTP_SPIKE: x', next_poll_ns=NOW + 5)

        sampler = cm.ClockSampler('timesyncd', 3_600_000_000_000, 100.0, collector=collector)
        try:
            sampler.poll()
            wait([sampler.pending])
            record = sampler.poll()
        finally:
            sampler.close()
        self.assertEqual(('UNKNOWN', 'NTP_SPIKE: x', NOW + 5), (record['state'], record['reason'],
                                                                record['next_poll_ns']))


class LogLineTests(unittest.TestCase):
    def test_the_start_log_shows_the_seconds_to_the_next_poll(self):
        record = dict(state='UNKNOWN', reason='NTP_SPIKE: last reply …', next_poll_ns=NOW + 511_200_000_000)
        self.assertEqual('central live clock: NTP_SPIKE: last reply … (next poll in <= 512 s)',
                         live_task_inputs.clock_log_line(record, NOW))
        self.assertEqual('central live clock: SYNCED', live_task_inputs.clock_log_line(dict(state='SYNCED'), NOW))


if __name__ == '__main__':
    unittest.main()
