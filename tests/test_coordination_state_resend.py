"""SystemState is resent unchanged while nothing changes, so an executor started after the coordinator registers
without ``~/refresh_state`` (현장 2026-10-15, #337 기록 5절) (E2).

The executor refuses a SystemState whose envelope ``published_at_ns`` is older than its lease max age
(rescue_robot task_site_runtime ``_receive_coordinator``: CLOCK_UNCERTAIN "future or stale"). The coordinator sent
SystemState only on a change, so a late executor got the latched old one and never registered. Now the latest
snapshot goes again every ``lease_renew_period_s`` (the CommandLease cadence, which already has to stay inside that
max age) with a new envelope only: ``changed_at_ns`` and ``state_sequence`` keep their meaning. Only while the
coordinator clock is usable, as for ``~/refresh_state``.
"""
from types import SimpleNamespace as NS
import itertools
import unittest

from rescue_coordination.live_task_inputs import StatePublication
from test_coordination_ros_core import make_core
from test_coordination_search import NOW

PERIOD = 500_000_000      # lease_renew_period_s 0.5 (config/field)
MAX_AGE = 2_000_000_000   # an executor lease max age, larger than the period


class Channel:
    def __init__(self, core):
        self.sent = []
        self.clock = [NOW]
        sequence = itertools.count(1)

        def stamp(msg):
            msg.envelope = NS(sequence=next(sequence), published_at_ns=self.clock[0])
            return msg

        self.publication = StatePublication(core, lambda **fields: NS(**fields), stamp, self.sent.append)

    def send(self, now, **kwargs):
        self.clock[0] = now
        return self.publication.send(now, **kwargs)


def executor_accepts(message, now):
    """The executor's freshness rule for a coordinator SystemState (clock bound 0 here)."""
    stamp = message.envelope.published_at_ns
    return 0 < stamp <= now and now - stamp <= MAX_AGE


class StateResendTests(unittest.TestCase):
    def setUp(self):
        self.core = make_core()
        self.channel = Channel(self.core)

    def test_unchanged_state_goes_again_after_the_period_with_a_new_envelope_only(self):
        first = self.channel.send(NOW, resend_after_ns=PERIOD)
        self.assertIsNotNone(first)
        self.assertIsNone(self.channel.send(NOW + PERIOD - 1, resend_after_ns=PERIOD))
        again = self.channel.send(NOW + PERIOD, resend_after_ns=PERIOD)
        self.assertEqual(first, again)  # changed_at_ns and state_sequence unchanged
        self.assertEqual([NOW, NOW + PERIOD], [m.envelope.published_at_ns for m in self.channel.sent])
        self.assertTrue(self.channel.publication.resent)

    def test_a_late_executor_registers_without_refresh_state(self):
        self.channel.send(NOW, resend_after_ns=PERIOD)  # the coordinator's only change, long before
        late = NOW + 60_000_000_000                       # the executor starts a minute later
        latched = self.channel.sent[-1]
        self.assertFalse(executor_accepts(latched, late))  # the latched copy is stale (the field symptom)
        for now in range(NOW + 1, late + PERIOD + 1, PERIOD // 5):
            self.channel.send(now, resend_after_ns=PERIOD)
        self.assertTrue(executor_accepts(self.channel.sent[-1], late + PERIOD))

    def test_without_a_period_only_changes_go_out(self):
        self.channel.send(NOW)
        self.assertIsNone(self.channel.send(NOW + 10 * PERIOD))
        self.assertEqual(1, len(self.channel.sent))

    def test_a_change_resets_the_period(self):
        self.channel.send(NOW, resend_after_ns=PERIOD)
        self.core.mission.mode = 'DISASTER'
        self.assertIsNotNone(self.channel.send(NOW + PERIOD - 1, resend_after_ns=PERIOD))
        self.assertFalse(self.channel.publication.resent)
        self.assertIsNone(self.channel.send(NOW + PERIOD, resend_after_ns=PERIOD))

    def test_the_explicit_refresh_still_works(self):
        self.channel.send(NOW)
        self.assertIsNotNone(self.channel.send(NOW + 1, refresh=True))
        self.assertEqual(2, len(self.channel.sent))


if __name__ == '__main__':
    unittest.main()
