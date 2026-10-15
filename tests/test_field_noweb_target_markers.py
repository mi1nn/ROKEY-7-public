"""scripts/field/noweb/target_markers.py: targets and QR exits drawn in RViz (웹 없는 트랙, 2026-10-15).

RViz cannot draw our own messages, so the tool turns them into plain marker descriptions (no ROS here):
the coordinator's targets (/rescue/targets) by stage, the locator's doll locations (/rescue/target_locations)
not yet in a target as candidates, the QR exits (/rescue/exit_location) as cubes. A location past its own
valid_until_ns is drawn faded and marked STALE: the age comes from the message, nothing is made up. A message
without a mission frame is not drawn and the reason is kept.
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / 'scripts' / 'field' / 'noweb' / 'target_markers.py'
if not (REPO / 'scripts' / 'field' / 'noweb' / 'operator_command.py').is_file():
    # as tests/test_field_noweb_status_watch.py: repo scripts, not part of the installed-wheel checkout
    raise unittest.SkipTest('scripts/field/noweb/ absent (installed-wheel checkout)')
SPEC = importlib.util.spec_from_file_location('noweb_target_markers', SCRIPT)
tm = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tm)

SECOND = 1_000_000_000
NOW = 1_000 * SECOND


def location(location_id, x, y, *, label='survivor', track='', valid_until_ns=NOW + SECOND, frame='map'):
    return NS(location_id=location_id, mission_position=NS(x=x, y=y, z=0.0), projection_status='VALID',
              frame_ref=NS(mission_frame=frame),
              observation=NS(class_label=label, has_track_id=bool(track), track_id=track,
                             time=NS(observed_at_ns=valid_until_ns - 2 * SECOND, valid_until_ns=valid_until_ns)))


def estimate(target_id, x, y, *, locations, confirmation='CONFIRMED', rescue='NOT_CHECKED',
             valid_until_ns=NOW + SECOND):
    return NS(target_id=target_id, mission_position=NS(x=x, y=y, z=0.0), locations=locations,
              confirmation_state=confirmation, rescue_state=rescue, valid_until_ns=valid_until_ns)


def by_ns(specs, ns):
    return [s for s in specs if s['ns'] == ns]


class StageTests(unittest.TestCase):
    def test_stage_of_a_target(self):
        self.assertEqual('RESCUED', tm.stage(estimate('t', 0, 0, locations=[], rescue='RESCUE_CONFIRMED')))
        self.assertEqual('CONFIRMED', tm.stage(estimate('t', 0, 0, locations=[])))
        self.assertEqual('CANDIDATE', tm.stage(estimate('t', 0, 0, locations=[], confirmation='CANDIDATE')))
        self.assertEqual('CANDIDATE', tm.stage(estimate('t', 0, 0, locations=[],
                                                        confirmation='NEEDS_REOBSERVATION')))
        self.assertEqual('DISMISSED', tm.stage(estimate('t', 0, 0, locations=[], confirmation='DISMISSED')))


class LatestTests(unittest.TestCase):
    def test_keeps_the_latest_location_per_track_or_location_id(self):
        store = {}
        tm.keep_latest(store, location('a', 1, 1, track='7', valid_until_ns=NOW))
        tm.keep_latest(store, location('b', 2, 2, track='7', valid_until_ns=NOW + SECOND))
        tm.keep_latest(store, location('c', 3, 3, track='7', valid_until_ns=NOW - SECOND))   # older: ignored
        tm.keep_latest(store, location('d', 4, 4))                                           # no track: own id
        self.assertEqual({'7': 'b', 'd': 'd'}, {k: v.location_id for k, v in store.items()})


class MarkerTests(unittest.TestCase):
    def test_target_stage_colour_and_label(self):
        seen = location('loc-1', 1.0, 2.0)
        specs, skipped = tm.marker_specs([estimate('0123456789ab', 1.0, 2.0, locations=[seen])], {}, {}, now_ns=NOW)
        self.assertEqual([], skipped)
        shape, text = by_ns(specs, 'target')
        self.assertEqual(('SPHERE', 'map', 1.0, 2.0), (shape['kind'], shape['frame'], shape['x'], shape['y']))
        self.assertEqual(tm.COLOURS['CONFIRMED'], shape['rgba'][:3])
        self.assertEqual(('TEXT', '01234567 CONFIRMED'), (text['kind'], text['text']))

    def test_rescued_target_has_its_own_colour(self):
        specs, _ = tm.marker_specs([estimate('t1', 0, 0, locations=[location('l', 0, 0)],
                                             rescue='RESCUE_CONFIRMED')], {}, {}, now_ns=NOW)
        self.assertEqual(tm.COLOURS['RESCUED'], by_ns(specs, 'target')[0]['rgba'][:3])
        self.assertNotEqual(tm.COLOURS['RESCUED'], tm.COLOURS['CONFIRMED'])

    def test_located_doll_not_yet_a_target_is_a_candidate(self):
        used, new = location('used', 1, 1, track='1'), location('new', 5, 5, track='2')
        specs, _ = tm.marker_specs([estimate('t1', 1, 1, locations=[used])], {'1': used, '2': new}, {}, now_ns=NOW)
        candidates = by_ns(specs, 'candidate')
        self.assertEqual([(5, 5, 'SPHERE'), (5, 5, 'TEXT')], [(s['x'], s['y'], s['kind']) for s in candidates])
        self.assertEqual('new CANDIDATE', candidates[1]['text'])

    def test_exit_is_a_cube_named_by_its_qr(self):
        door = location('x1', 3, -1, label='exit_qr', track='EXIT-A')
        specs, _ = tm.marker_specs([], {}, {'EXIT-A': door}, now_ns=NOW)
        shape, text = by_ns(specs, 'exit')
        self.assertEqual(('CUBE', 3, -1), (shape['kind'], shape['x'], shape['y']))
        self.assertEqual('EXIT EXIT-A', text['text'])

    def test_past_its_own_validity_is_faded_and_marked(self):
        old = location('old', 1, 1, valid_until_ns=NOW - 3 * SECOND)
        specs, _ = tm.marker_specs([], {'old': old}, {}, now_ns=NOW)
        shape, text = by_ns(specs, 'candidate')
        self.assertLess(shape['rgba'][3], 1.0)
        self.assertEqual('old CANDIDATE STALE 3s', text['text'])
        fresh, _ = tm.marker_specs([], {'old': location('old', 1, 1)}, {}, now_ns=NOW)
        self.assertEqual(1.0, by_ns(fresh, 'candidate')[0]['rgba'][3])

    def test_target_age_uses_the_snapshot_validity(self):
        specs, _ = tm.marker_specs([estimate('t1', 0, 0, locations=[location('l', 0, 0)],
                                             valid_until_ns=NOW - 10 * SECOND)], {}, {}, now_ns=NOW)
        self.assertEqual('t1 CONFIRMED STALE 10s', by_ns(specs, 'target')[1]['text'])

    def test_no_mission_frame_is_not_drawn(self):
        lost, alone = location('lost', 1, 1, frame=''), location('alone', 2, 2, frame='')
        specs, skipped = tm.marker_specs([estimate('t1', 0, 0, locations=[lost])], {'alone': alone}, {}, now_ns=NOW)
        self.assertEqual([], [s for s in specs if s['ns'] in ('target', 'candidate')])
        self.assertEqual(2, len(skipped))
        self.assertIn('no mission frame', skipped[0])

    def test_projection_that_is_not_valid_is_shown(self):
        odd = location('odd', 1, 1)
        odd.projection_status = 'NEEDS_REOBSERVATION'
        specs, _ = tm.marker_specs([], {'odd': odd}, {}, now_ns=NOW)
        self.assertEqual('odd CANDIDATE NEEDS_REOBSERVATION', by_ns(specs, 'candidate')[1]['text'])

    def test_ids_are_unique_per_namespace(self):
        locs = {str(i): location(f'l{i}', i, 0, track=str(i)) for i in range(3)}
        specs, _ = tm.marker_specs([], locs, {}, now_ns=NOW)
        self.assertEqual(6, len({(s['ns'], s['id']) for s in specs}))


class TopicTests(unittest.TestCase):
    def test_topics_are_the_ones_the_code_uses(self):
        from rescue_coordination import ros_node as coordinator
        from rescue_perception import exit_qr, target_locator
        self.assertEqual(coordinator.TARGETS_TOPIC, tm.TARGETS_TOPIC)
        self.assertEqual(target_locator.TARGET_LOCATIONS_TOPIC, tm.TARGET_LOCATIONS_TOPIC)
        self.assertEqual(exit_qr.EXIT_LOCATION_TOPIC, tm.EXIT_LOCATION_TOPIC)
        self.assertEqual('/rescue/noweb/markers', tm.MARKERS_TOPIC)
        self.assertEqual(coordinator.EXIT_CLASS_LABEL, tm.EXIT_CLASS_LABEL)


if __name__ == '__main__':
    unittest.main()
