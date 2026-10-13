"""Overhead passages (tunnels) on the map: one JSON text per robot, read by one function (OPS-185/186).

Each robot's overhead_passage node publishes its whole list on /rescue/robots/<id>/overhead_passages
(std_msgs/String, latched). db_writer, the web map and the coordinator read it only through
parse_overhead_passages; anything else is refused, never guessed.
"""
import json
import math
import unittest

from rescue_contracts.overhead import (OVERHEAD_PASSAGES_FORMAT, overhead_passages_text, overhead_passages_topic,
                                       parse_overhead_passages)

SESSION = '22222222-2222-4222-8222-222222222222'


def item(**changes):
    value = dict(id='33333333-3333-4333-8333-333333333333', state='PASSABLE', lowest_m=0.47, x=1.5, y=-0.25,
                 map_session_id=SESSION, map_revision=1, frames=3, first_seen_ns=1_000, last_seen_ns=2_000, reason='')
    value.update(changes)
    return value


def text(**changes):
    value = dict(robot_id='robot_a', map_frame='map', generated_at_ns=3_000, clearance_m=0.46, floor_noise_m=0.05,
                 lidar_plane_m=0.19, items=[item()])
    value.update(changes)
    return overhead_passages_text(**value)


class OverheadPassagesContract(unittest.TestCase):
    def test_round_trip(self):
        parsed = parse_overhead_passages(text())
        self.assertEqual(OVERHEAD_PASSAGES_FORMAT, 'rescue-overhead-passages/1')
        self.assertEqual(('robot_a', 'map', 0.46), (parsed['robot_id'], parsed['map_frame'], parsed['clearance_m']))
        self.assertEqual([item()], parsed['items'])
        self.assertEqual(text(), text())   # canonical: same list, same text

    def test_states_carry_their_height(self):
        blocked = item(id='44444444-4444-4444-8444-444444444444', state='BLOCKED', lowest_m=0.40)
        unknown = item(id='55555555-5555-4555-8555-555555555555', state='UNKNOWN', lowest_m=None, reason='NO_FLOOR_SEEN')
        parsed = parse_overhead_passages(text(items=[item(), blocked, unknown]))
        self.assertEqual(['PASSABLE', 'BLOCKED', 'UNKNOWN'], [i['state'] for i in parsed['items']])
        self.assertIsNone(parsed['items'][2]['lowest_m'])
        self.assertEqual([], parse_overhead_passages(text(items=[]))['items'])

    def test_bad_items_are_refused_on_write_and_read(self):
        bad = {
            'state': item(state='OPEN'),
            'passable below clearance': item(lowest_m=0.40),
            'blocked at clearance': item(state='BLOCKED', lowest_m=0.46),
            'unknown with a height': item(state='UNKNOWN', lowest_m=0.47),
            'known without a height': item(lowest_m=None),
            'not finite': item(x=math.nan),
            'session not a UUID': item(map_session_id='e3'),
            'negative revision': item(map_revision=-1),
            'bool revision': item(map_revision=True),
            'no frames': item(frames=0),
            'last before first': item(last_seen_ns=999),
            'missing key': {k: v for k, v in item().items() if k != 'reason'},
            'extra key': item(colour='green'),
        }
        for name, value in bad.items():
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    text(items=[value])
                raw = json.loads(text())
                raw['items'] = [value]
                with self.assertRaises(ValueError):
                    parse_overhead_passages(json.dumps(raw))

    def test_bad_lists_are_refused(self):
        for changes in (dict(items=[item(), item()]),             # same id twice
                        dict(clearance_m=0.05), dict(floor_noise_m=-0.01), dict(robot_id=''),
                        dict(lidar_plane_m=math.inf), dict(generated_at_ns=1.5)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                text(**changes)

    def test_other_texts_are_refused(self):
        raw = json.loads(text())
        for value in ('', 'null', '[]', json.dumps(dict(raw, format='rescue-overhead-passages/2')),
                      json.dumps({k: v for k, v in raw.items() if k != 'lidar_plane_m'}), json.dumps(dict(raw, x=1)),
                      None, b'{}'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_overhead_passages(value)

    def test_topic_per_robot(self):
        self.assertEqual('/rescue/robots/robot_b/overhead_passages', overhead_passages_topic('robot_b'))
        with self.assertRaises(ValueError):
            overhead_passages_topic('robot b')


if __name__ == '__main__':
    unittest.main()
