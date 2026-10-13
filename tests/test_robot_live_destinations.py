"""Coordinator destinations vs. the launch home on a live robot (OPS-144), no ROS.

Bug in main (from #192 on #132's LiveClock): the robot launch puts this robot's profile home in
configured_destinations['home/<robot_id>'], and every coordinator record (~/task_inputs)
replaced configured_destinations wholesale. The coordinator sends only search-area
destinations today, so after the first record the home was gone and RETURN_HOME was refused.
Now the launch home stays until the coordinator sends one for this robot, which then wins
(the QR exit for TB-5, OPS-139).

Decision a (OPS-144): destination keys are UUIDs, except 'home/<this robot_id>' (RETURN_HOME
home_ref); its point must be this robot's. Another robot's home is skipped like its other
destinations; a home key whose point names a different robot is refused.
"""
from copy import deepcopy
import threading
from types import MethodType, SimpleNamespace as NS
import unittest

from rescue_robot.executor import ContractError
from rescue_robot.live_clock import LiveClock
from rescue_robot.live_parameters import home_destinations

SESSION = '22222222-2222-4222-8222-222222222222'
AREA = '33333333-3333-4333-8333-333333333333'


def point(robot, xy_yaw, revision=1):
    return dict(robot_id=robot, frame_id='map', xy_yaw=list(xy_yaw), map_session_id=SESSION, map_revision=revision,
                transform_revision=1)


class Destinations(unittest.TestCase):
    def setUp(self):
        launch_home = home_destinations('robot_a', [1.0, 2.0, 0.5], frame_id='map', map_session_id=SESSION,
                                        map_revision=1, transform_revision=1)
        authority = dict(coordinator_id='c', authority_epoch='e')
        self.runtime = NS(
            adapter=NS(lock=threading.RLock(), robot_id='robot_a'),
            config=dict(coordinator_authority=authority, configured_destinations=deepcopy(launch_home)),
            inputs=NS(config=dict(schema_version='s', configured_destinations=deepcopy(launch_home)),
                      lease_policy=NS(clock_domain_id='d', duration_s=4.0)),
            _retired_coordinators=set(), _coordinator=None, _stop_input_session=lambda: None)
        self.clock = object.__new__(LiveClock)
        self.clock.runtime, self.clock.peer = self.runtime, None
        self.clock.validate_measurement = MethodType(lambda self, m: None, self.clock)

    def receive(self, destinations):
        self.clock.receive(dict(format='rescue-live-task-source/1', coordinator_id='c', authority_epoch='e',
                                schema_version='s', clock_domain_id='d', lease_duration_s=4.0,
                                lease_renew_period_s=1.0, publisher_boot_id='44444444-4444-4444-8444-444444444444',
                                clock={}, destinations=destinations))
        return self.runtime.inputs.config['configured_destinations']

    def test_launch_home_survives_a_coordinator_record_without_homes(self):
        got = self.receive({AREA: point('robot_a', [4.0, 0.0, 0.0])})
        self.assertEqual([1.0, 2.0, 0.5], got['home/robot_a']['xy_yaw'])
        self.assertIn(AREA, got)

    def test_coordinator_home_for_this_robot_wins(self):
        got = self.receive({'home/robot_a': point('robot_a', [7.0, 8.0, 0.0])})
        self.assertEqual([7.0, 8.0, 0.0], got['home/robot_a']['xy_yaw'])
        got = self.receive({})   # a later record without it: back to the launch home, nothing kept from before
        self.assertEqual([1.0, 2.0, 0.5], got['home/robot_a']['xy_yaw'])

    def test_another_robots_home_is_skipped(self):
        got = self.receive({'home/robot_b': point('robot_b', [9.0, 9.0, 0.0]), AREA: point('robot_a', [4.0, 0.0, 0.0])})
        self.assertNotIn('home/robot_b', got)
        self.assertIn(AREA, got)

    def test_home_key_with_another_robots_point_or_other_text_keys_are_refused(self):
        for bad in ({'home/robot_a': point('robot_b', [1.0, 1.0, 0.0])}, {'exit': point('robot_a', [1.0, 1.0, 0.0])},
                    {'home/': point('robot_a', [1.0, 1.0, 0.0])}):
            with self.subTest(bad=list(bad)), self.assertRaises(ContractError):
                self.receive(bad)

    def test_robot_without_a_launch_home_gets_only_what_the_coordinator_sends(self):
        self.runtime.config['configured_destinations'] = {}
        self.assertNotIn('home/robot_a', self.receive({AREA: point('robot_a', [4.0, 0.0, 0.0])}))
        self.assertEqual([5.0, 6.0, 0.0],
                         self.receive({'home/robot_a': point('robot_a', [5.0, 6.0, 0.0])})['home/robot_a']['xy_yaw'])


if __name__ == '__main__':
    unittest.main()
