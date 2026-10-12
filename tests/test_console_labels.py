"""Korean display labels: every contract value has one, and an unknown code is shown as itself."""
import unittest

from rescue_contracts import RESCUE_STATES, ROBOT_ROLES, SITE_EVENT_KINDS, SYSTEM_MODES
from rescue_coordination.ros_node import REASON_CODES
from rescue_operations.site_state import MISSION_MODES, MISSION_PHASES, SEGMENT_PHASES, SUPPORT_STATES
from rescue_console.labels import LABELS, label

# Report vocabularies are inline tuples in rescue_operations.schema (no exported constant).
APPROACH_RESULTS = {'APPROACH_COMPLETE', 'UNREACHABLE', 'FAILED', 'CANCELED', 'NOT_ATTEMPTED'}
CONFIRMATION_RESULTS = {'CONFIRMED', 'UNCONFIRMED'}
RECEIPT_STATES = {'RECEIVED', 'PENDING_EVIDENCE', 'PERSISTED'}


class LabelTests(unittest.TestCase):
    def test_every_upstream_value_has_a_korean_word(self):
        # A value added upstream fails here until it gets a label (it would still show as its code).
        for kind, values in (('role', set(ROBOT_ROLES) | {'UNKNOWN'}), ('rescue', RESCUE_STATES),
                             ('support', SUPPORT_STATES), ('event', set(SITE_EVENT_KINDS) | {'REPORT'}),
                             ('mode', set(SYSTEM_MODES) | MISSION_MODES), ('approach', APPROACH_RESULTS),
                             ('confirmation', CONFIRMATION_RESULTS), ('receipt', RECEIPT_STATES),
                             ('phase', MISSION_PHASES), ('segment', SEGMENT_PHASES),
                             # what the coordinator answers an operator command with (#98: NAV_NOT_READY)
                             ('reply', set(REASON_CODES.values()) | {'NAV_NOT_READY'})):
            with self.subTest(kind=kind):
                self.assertLessEqual(set(values), set(LABELS[kind]))

    def test_words_are_nonempty_and_differ_from_the_code(self):
        for kind, table in LABELS.items():
            for code, word in table.items():
                with self.subTest(kind=kind, code=code):
                    self.assertTrue(word.strip())
                    self.assertNotEqual(word, code)

    def test_unknown_code_is_shown_unchanged_and_none_stays_none(self):
        self.assertEqual(label('motion', 'DOCKING_NEW_STATE'), 'DOCKING_NEW_STATE')
        self.assertEqual(label('readiness', 'READY'), '준비됨')
        self.assertIsNone(label('rescue', None))
        with self.assertRaises(KeyError):
            label('no_such_kind', 'READY')


if __name__ == '__main__':
    unittest.main()
