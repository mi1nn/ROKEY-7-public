"""The operator declares the disaster from the console (owner decision 2026-10-15; 작전 design) (E2).

DECLARE_DISASTER takes the same NORMAL -> DISASTER path as a DISASTER_SIGN: the command_id is the trigger and the
acceptance time its time; the source (the operator) shows in SystemState and the node log. No SiteEvent is made, so
the event list and the KPI-04 display delay keep only what a camera saw. Not a console button of COMMAND_FIELDS.
"""
import unittest

from rescue_coordination import mission, ros_node
from test_coordination_mission import MissionTestCase, sign
from test_coordination_ros_core import SCOUT, make_core, request, site_msg, status_msg
from test_coordination_search import NOW, uid


class DeclareDisasterGateTests(MissionTestCase):
    def test_fields_and_no_console_button(self):
        self.assertEqual((), mission.ALL_COMMAND_FIELDS['DECLARE_DISASTER'])
        self.assertNotIn('DECLARE_DISASTER', mission.COMMAND_FIELDS)
        self.assertIn('ALREADY_DISASTER', mission.COMMAND_REJECT_REASONS)

    def test_normal_enters_disaster_with_the_command_as_trigger(self):
        command = self.command('DECLARE_DISASTER')
        result = self.m.handle_command(command, now_ns=NOW + 5)
        self.assertEqual((True, 'ACCEPTED'), (result.accepted, result.reason))
        view = self.m.view()
        self.assertEqual(('DISASTER', 'DISASTER', command.command_id, NOW + 5, 'OPERATOR'),
                         (view.mode, view.phase, view.trigger_event_id, view.trigger_observed_at_ns,
                          view.trigger_source))
        self.assertEqual({}, self.m.events)  # no made-up SiteEvent

    def test_already_disaster_is_refused(self):
        self.disaster()
        self.assertEqual('ALREADY_DISASTER', self.reason('DECLARE_DISASTER'))
        self.assertEqual(('SITE_EVENT', uid(301)), (self.m.view().trigger_source, self.m.view().trigger_event_id))

    def test_a_second_declaration_is_refused(self):
        self.assertEqual('ACCEPTED', self.reason('DECLARE_DISASTER'))
        self.assertEqual('ALREADY_DISASTER', self.reason('DECLARE_DISASTER'))

    def test_a_replayed_command_id_returns_its_first_answer(self):
        command = self.command('DECLARE_DISASTER')
        first = self.m.handle_command(command, now_ns=NOW + 1)
        self.assertEqual(first, self.m.handle_command(command, now_ns=NOW + 9))
        self.assertEqual(NOW + 1, self.m.view().trigger_observed_at_ns)

    def test_a_sign_after_the_button_is_recorded_and_the_mode_stays(self):
        command = self.command('DECLARE_DISASTER')
        self.m.handle_command(command, now_ns=NOW + 1)
        self.assertEqual('ALREADY_DISASTER', self.m.record_site_event(sign()).reason)
        self.assertIn(uid(301), self.m.events)
        self.assertEqual(('DISASTER', command.command_id, 'OPERATOR'),
                         (self.m.mode, self.m.view().trigger_event_id, self.m.view().trigger_source))

    def test_other_commands_still_need_disaster(self):
        self.assertEqual('MODE_NOT_DISASTER', self.reason('RETURN_HOME'))
        self.assertEqual('MODE_NOT_DISASTER', self.reason('START_SEARCH'))


class DeclareDisasterNodeTests(unittest.TestCase):
    def setUp(self):
        self.core = make_core(search_start='AUTO')

    def declare(self, n=0, **changes):
        return self.core.on_command(request('DECLARE_DISASTER', command_id=uid(780 + n), **changes), now_ns=NOW)

    def test_the_button_starts_the_search_like_the_sign(self):
        self.core.on_status(status_msg())
        reply = self.declare()
        self.assertEqual((True, 'ACCEPTED', ''), (reply['accepted'], reply['request_state'], reply['reason_code']))
        (fields,) = [f for f in self.core.tick(NOW) if f['kind'] == 'SEARCH_AREA']
        self.assertEqual(SCOUT, fields['robot_id'])
        self.assertIn(f'DISASTER_DECLARED by operator admin command {uid(780)}', self.core.notes)

    def test_system_state_names_the_operator_as_the_source(self):
        self.declare()
        state = self.core.system_state_change(NOW)
        self.assertEqual(('DISASTER', True, uid(780), NOW, []),
                         (state['mode'], state['has_trigger'], state['trigger_event_id'],
                          state['trigger_observed_at_ns'], state['cause_event_ids']))
        self.assertEqual('DISASTER_DECLARED_BY_OPERATOR', state['reason'])

    def test_after_the_sign_it_is_task_busy(self):
        self.core.on_site_event(site_msg())
        reply = self.declare()
        self.assertEqual((False, 'REJECTED', 'TASK_BUSY', 'ALREADY_DISASTER'),
                         (reply['accepted'], reply['request_state'], reply['reason_code'], self.core.last_reason))

    def test_a_replay_is_duplicate_with_the_first_answer(self):
        self.declare()
        reply = self.declare()
        self.assertEqual((True, 'DUPLICATE', ''), (reply['accepted'], reply['request_state'], reply['reason_code']))

    def test_a_robot_cannot_be_named(self):
        reply = self.declare(robot_id=SCOUT)
        self.assertEqual((False, 'INVALID_INPUT'), (reply['accepted'], reply['reason_code']))
        self.assertEqual('NORMAL', self.core.mission.mode)

    def test_the_operator_start_search_mode_still_needs_the_button(self):
        core = make_core(search_start='OPERATOR')
        core.on_status(status_msg())
        core.on_command(request('DECLARE_DISASTER', command_id=uid(781)), now_ns=NOW)
        self.assertEqual([], [f for f in core.tick(NOW) if f['kind'] == 'SEARCH_AREA'])
        self.assertEqual('DISASTER', core.mission.mode)

    def test_reason_code_mapping(self):
        self.assertEqual('TASK_BUSY', ros_node.REASON_CODES['ALREADY_DISASTER'])


if __name__ == '__main__':
    unittest.main()
