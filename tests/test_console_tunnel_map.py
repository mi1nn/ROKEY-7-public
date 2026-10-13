"""지도에 높이 장애물(터널) 판정(OPS-189·196, 재범): 초록 「통과 가능 ↑h」 / 빨강 「통과 불가 ↑h」 / 회색 「판정 불가」.

현장 상태 DB overhead_passages(#262, read_overhead_passages)를 그린다. 지금 지도(current_map 의 session·revision)와
같은 항목은 진하게, 다른 revision 은 흐리게. current_map 을 모르면 전부 진하게 그리고 그 사실을 적는다.
"""
import unittest

from rescue_console.render import render_fragment
from rescue_operations.map_snapshot import write_snapshot
from test_console_live_map import LiveMapFixture, grid

SESSION, OLD_SESSION = '7d1c4f6a-0000-4000-8000-000000000001', '7d1c4f6a-0000-4000-8000-000000000002'


def item(ident, state, lowest, x, y, *, session=SESSION, revision=2):
    return {'id': f'00000000-0000-4000-8000-00000000000{ident}', 'state': state, 'lowest_m': lowest, 'x': x, 'y': y,
            'map_session_id': session, 'map_revision': revision, 'frames': 12, 'first_seen_ns': 1, 'last_seen_ns': 2,
            'reason': 'NO_FLOOR_SEEN' if state == 'UNKNOWN' else ''}


class TunnelMapTests(LiveMapFixture):
    def passages(self, items, current=(SESSION, 2)):
        self.status['overhead_passages'] = {'schema_version': '1.4', 'robots': {'robot_a': {
            'robot_id': 'robot_a', 'generated_at_ns': 5, 'received_at_ns': 6, 'map_frame': 'map', 'clearance_m': 0.46,
            'floor_noise_m': 0.02, 'lidar_plane_m': 0.18, 'items': items}},
            'current_map': None if current is None else {'robot_id': 'robot_a', 'map_session_id': current[0],
                                                         'map_revision': current[1], 'generated_at_ns': 5,
                                                         'received_at_ns': 6}}

    def drawn(self):
        write_snapshot(self.maps, **grid())
        self.received(0)
        view = self.view(self.live())
        return view, render_fragment(view)

    def test_states_words_and_colors(self):
        self.passages([item(1, 'PASSABLE', 0.47, 0.0, -1.0), item(2, 'BLOCKED', 0.40, 0.2, -1.2),
                       item(3, 'UNKNOWN', None, 0.4, -1.4)])
        view, html = self.drawn()
        self.assertEqual(3, len(view['map']['passages']))
        self.assertIn('class="passage passable current"', html)
        self.assertIn('통과 가능 ↑0.47 m', html)
        self.assertIn('class="passage blocked current"', html)
        self.assertIn('통과 불가 ↑0.40 m', html)
        self.assertIn('class="passage unknown current"', html)
        self.assertIn('판정 불가', html)

    def test_an_older_revision_is_faint(self):
        self.passages([item(1, 'PASSABLE', 0.47, 0.0, -1.0, revision=1),
                       item(2, 'BLOCKED', 0.40, 0.2, -1.2, session=OLD_SESSION)])
        html = self.drawn()[1]
        self.assertIn('class="passage passable old"', html)
        self.assertIn('class="passage blocked old"', html)

    def test_unknown_current_map_draws_all_and_says_so(self):
        self.passages([item(1, 'PASSABLE', 0.47, 0.0, -1.0)], current=None)
        html = self.drawn()[1]
        self.assertIn('class="passage passable current"', html)
        self.assertIn('현재 지도 revision 모름', html)

    def test_off_map_passage_is_listed(self):
        self.passages([item(1, 'BLOCKED', 0.40, 50.0, 0.0)])
        view, html = self.drawn()
        self.assertEqual([], view['map']['passages'])
        self.assertIn('통과 불가 ↑0.40 m: 지도 밖', html)

    def test_no_table_shows_nothing(self):
        view, html = self.drawn()
        self.assertEqual([], view['passages'])
        self.assertNotIn('class="passage', html)


if __name__ == '__main__':
    unittest.main()
