"""Which revision a frozen map is: /rescue/map/frozen_revision (OPS-154/159), no ROS.

The map is frozen twice per run (R0+1 when the doll is found, R0+2 when the search is done) on
the same /rescue/map/frozen (OccupancyGrid, latched depth 1). A grid has no revision field, so
map_freeze also publishes a std_msgs/String JSON on /rescue/map/frozen_revision. Robot and
coordinator read it with the one function in rescue_contracts.occupancy: another format, a
missing or extra field or a bad value is refused, and a grid has that revision only when its
hash is the one named there.
"""
import json
import unittest

from rescue_contracts.occupancy import (FROZEN_REVISION_FORMAT, frozen_revision_text, paired_revision,
                                        parse_frozen_revision)

SESSION = '22222222-2222-4222-8222-222222222222'
SHA = 'a' * 64


class FrozenRevision(unittest.TestCase):
    def text(self, **changes):
        fields = dict(format=FROZEN_REVISION_FORMAT, map_session_id=SESSION, map_revision=2, occupancy_sha256=SHA)
        fields.update(changes)
        return json.dumps({k: v for k, v in fields.items() if v is not None})

    def test_round_trip(self):
        self.assertEqual('rescue-map-frozen/1', FROZEN_REVISION_FORMAT)
        text = frozen_revision_text(map_session_id=SESSION, map_revision=2, occupancy_sha256=SHA)
        self.assertEqual(dict(map_session_id=SESSION, map_revision=2, occupancy_sha256=SHA),
                         parse_frozen_revision(text))
        self.assertEqual(parse_frozen_revision(self.text()), parse_frozen_revision(text))

    def test_other_formats_and_bad_values_are_refused(self):
        bad = {
            'not json': 'R2',
            'not an object': json.dumps([SESSION, 2, SHA]),
            'other format': self.text(format='rescue-map-frozen/2'),
            'no format': self.text(format=None),
            'no sha': self.text(occupancy_sha256=None),
            'extra field': json.dumps(dict(json.loads(self.text()), stamp=1)),
            'session not a uuid': self.text(map_session_id='run-1'),
            'revision zero': self.text(map_revision=0),
            'revision text': self.text(map_revision='2'),
            'revision bool': self.text(map_revision=True),
            'revision float': self.text(map_revision=2.0),
            'short sha': self.text(occupancy_sha256='a' * 63),
            'upper-case sha': self.text(occupancy_sha256='A' * 64),
        }
        for name, text in bad.items():
            with self.subTest(name), self.assertRaises(ValueError):
                parse_frozen_revision(text)
        with self.assertRaises(ValueError):
            parse_frozen_revision(b'{}')
        with self.assertRaises(ValueError):
            frozen_revision_text(map_session_id=SESSION, map_revision=0, occupancy_sha256=SHA)

    def test_a_grid_has_the_revision_only_with_the_named_hash(self):
        info = parse_frozen_revision(self.text())
        self.assertEqual(2, paired_revision(info, SHA))
        self.assertIsNone(paired_revision(info, 'b' * 64))   # not paired (yet): no revision, never a guess


if __name__ == '__main__':
    unittest.main()
