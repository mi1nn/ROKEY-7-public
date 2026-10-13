"""The node's timer tick sends a TargetSnapshot before the goals that name its revision (E2; #179, 2026-10-13).

The robot looks up the REOBSERVE_TARGET's target revision in the latched /rescue/targets snapshot. When a new
location raised the revision since the last tick, the tick used to send the goal first and the snapshot after
it, so the robot could see the goal before that revision existed. ``node_tick`` publishes changed targets
before the goals and again after them (approach states the goals changed); unchanged targets are not resent.
"""
import unittest

from rescue_coordination import ros_node
from test_coordination_ros_core import SCOUT, location, make_core, request, site_msg, status_msg
from test_coordination_search import NOW, uid


class Recorder:
    def __init__(self):
        self.events = []

    def task(self, fields):
        self.events.append(('task', fields['kind'], fields['target_revision']))

    def targets(self, snapshot):
        self.events.append(('targets', tuple(t['estimate_revision'] for t in snapshot['targets'])))

    def watch(self, fields):
        self.events.append(('watch',))

    def report(self, report):
        self.events.append(('report', report['target_revision']))

    def lease(self, lease):
        self.events.append(('lease',))

    def block(self, reason):
        pass

    def state(self, now_ns):
        self.events.append(('state',))


class NodeTickOrderTests(unittest.TestCase):
    def test_the_snapshot_with_a_new_revision_goes_before_the_goal_naming_it(self):
        core = make_core()
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        target = core.on_target_location(location(1, (4., 4.)), now_ns=NOW)
        ros_node.node_tick(core, NOW, Recorder())  # revision 0 published
        core.on_target_location(location(2, (4.3, 4.)), now_ns=NOW)  # revision 1, not published yet
        core.on_command(request('REQUEST_RECHECK', command_id=uid(702), target_id=target), now_ns=NOW)
        out = Recorder()
        ros_node.node_tick(core, NOW, out)
        kinds = [e[0] for e in out.events]
        self.assertLess(kinds.index('targets'), kinds.index('task'))
        self.assertEqual(('targets', (1,)), out.events[kinds.index('targets')])
        self.assertEqual(('task', 'REOBSERVE_TARGET', 1), out.events[kinds.index('task')])

    def test_unchanged_targets_are_not_sent_twice(self):
        core = make_core()
        core.on_site_event(site_msg())
        core.on_target_location(location(1), now_ns=NOW)
        out = Recorder()
        ros_node.node_tick(core, NOW, out)
        self.assertEqual(1, [e[0] for e in out.events].count('targets'))
        out = Recorder()
        ros_node.node_tick(core, NOW + 1, out)
        self.assertEqual(0, [e[0] for e in out.events].count('targets'))


if __name__ == '__main__':
    unittest.main()
