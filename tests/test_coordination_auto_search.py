"""TB-6 search starts by itself on the disaster sign (owner decision 2026-10-13 evening; 작전 OPS-136·139; E2).

``search_start=AUTO``: DISASTER counts as the operator's START_SEARCH (#85 changed). The gates stay: a SCOUT that
is not READY, has a stale status or is stopping gets no SEARCH_AREA, and the start waits (SEARCH_QUEUED) until it
is. The operator's START_SEARCH is refused (NOT_AUTHORIZED, AUTO_SEARCH_ENABLED), as DISPATCH_SUPPORT is with
``support_dispatch=AUTO``. ``OPERATOR`` keeps the button (tests/test_coordination_ros_core.py ScoutDispatchTests).
"""
import math
import unittest

from rescue_contracts import RescueError
from rescue_coordination import ros_node
from test_coordination_ros_core import MAX_AGE, SCOUT, make_core, request, site_msg, status_msg
from test_coordination_search import NOW, uid


def searches(core, now=NOW):
    return [f for f in core.tick(now) if f['kind'] == 'SEARCH_AREA']


class AutoSearchStartTests(unittest.TestCase):
    def setUp(self):
        self.core = make_core(search_start='AUTO')

    def test_nothing_before_the_disaster_sign(self):
        self.core.on_status(status_msg())
        self.assertEqual([], searches(self.core))

    def test_disaster_sign_alone_sends_the_first_area_to_a_ready_scout(self):
        self.core.on_status(status_msg())
        self.core.on_site_event(site_msg())
        (fields,) = searches(self.core)
        self.assertEqual(SCOUT, fields['robot_id'])
        self.assertEqual([], searches(self.core))  # one area at a time

    def test_a_scout_not_ready_waits_with_search_queued_then_goes(self):
        self.core.on_site_event(site_msg())
        for status in (None, dict(readiness='NOT_READY'), dict(observed_at_ns=NOW - MAX_AGE - 1)):
            with self.subTest(status=status):
                if status is not None:
                    self.core.on_status(status_msg(**status))
                self.assertEqual([], searches(self.core))
                self.core.system_state_change(NOW)  # None when unchanged since the last subtest
                self.assertEqual('SEARCH_QUEUED', self.core.last_state['reason'])
        self.core.on_status(status_msg())
        self.assertEqual(1, len(searches(self.core)))

    def test_a_stopping_scout_gets_no_search(self):
        self.core.on_site_event(site_msg())
        self.core.on_status(status_msg(motion_state='STOPPING'))
        self.assertEqual([], searches(self.core))
        self.core.on_status(status_msg())
        self.assertEqual(1, len(searches(self.core)))

    def test_the_operator_button_is_refused(self):
        self.core.on_status(status_msg())
        self.core.on_site_event(site_msg())
        reply = self.core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        self.assertEqual((False, 'REJECTED', 'NOT_AUTHORIZED', 'AUTO_SEARCH_ENABLED'),
                         (reply['accepted'], reply['request_state'], reply['reason_code'], self.core.last_reason))

    def test_operator_mode_still_needs_the_button(self):
        core = make_core(search_start='OPERATOR')
        core.on_status(status_msg())
        core.on_site_event(site_msg())
        self.assertEqual([], searches(core))


class RvizSelectionTests(unittest.TestCase):
    """The field picks the search goal in RViz (live_inputs.search_goal_topic, #132). With AUTO there is no button:
    the pick may come before or after the disaster sign, until the first area is reserved; until then the
    reason says the goal is missing."""

    def setUp(self):
        from rescue_coordination.live_task_inputs import SelectedSearchGoal
        self.core = make_core(search_start='AUTO')
        self.selector = SelectedSearchGoal(self.core)

    def pick(self, x=2.0, y=1.0):
        from types import SimpleNamespace as NS
        return self.selector.select(NS(header=NS(frame_id='mission'), pose=NS(
            position=NS(x=x, y=y, z=0.), orientation=NS(x=0., y=0., z=0., w=1.))))

    def test_no_goal_picked_holds_with_a_reason(self):
        self.core.on_status(status_msg())
        self.core.on_site_event(site_msg())
        self.assertEqual([], searches(self.core))
        self.assertEqual('SEARCH_GOAL_NOT_SELECTED', self.core.system_state_change(NOW)['reason'])

    def test_a_goal_picked_after_the_disaster_sign_starts_the_search(self):
        self.core.on_status(status_msg())
        self.core.on_site_event(site_msg())
        searches(self.core)
        self.pick()
        (fields,) = searches(self.core)
        self.assertEqual(SCOUT, fields['robot_id'])
        with self.assertRaises(ValueError):  # locked once the area is reserved
            self.pick(3.0, 1.0)


class ParameterTests(unittest.TestCase):
    def test_typed_and_required(self):
        self.assertEqual('STRING', ros_node.PARAMETERS['search_start'])

    def test_unknown_mode_is_refused(self):
        for bad in ('auto', '', None, math.nan):
            with self.subTest(bad=bad), self.assertRaises(RescueError):
                make_core(search_start=bad)


if __name__ == '__main__':
    unittest.main()
