"""The 'target not sent' log names why a detection is not confirmed (field diagnosis, 2026-10-13).

On the robot the only sign that TB-6 sees the doll but sends no position is this log line. It said
just UNCONFIRMED, hiding the reason the pipeline already had (CLOCK_UNCERTAIN, ALIGNMENT_UNVERIFIED,
DEPTH_INSUFFICIENT, LOW_CONFIDENCE ...), so time, alignment, pairing and depth could not be told apart.
"""
from types import SimpleNamespace as NS
import unittest

from rescue_perception.target_utils import OneShotTarget


class NotSentReasonTests(unittest.TestCase):
    def test_unconfirmed_names_the_pipeline_reasons(self):
        candidate = NS(status='UNCONFIRMED', reasons=('LOW_CONFIDENCE', 'ALIGNMENT_UNVERIFIED'), observation=None)
        self.assertEqual('UNCONFIRMED(LOW_CONFIDENCE,ALIGNMENT_UNVERIFIED)', OneShotTarget().reason(candidate))

    def test_unconfirmed_without_reasons_stays_plain(self):
        self.assertEqual('UNCONFIRMED', OneShotTarget().reason(NS(status='UNCONFIRMED', reasons=(), observation=None)))


if __name__ == '__main__':
    unittest.main()
