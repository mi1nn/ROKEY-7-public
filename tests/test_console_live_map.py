"""--live-map: the SLAM map file is re-read when it changes; a missing, stale or unreadable map is said, never hidden."""
from datetime import datetime
import json
import os
import unittest

from rescue_console.app import current_view
from rescue_console.mapimage import LiveMap
from rescue_console.render import render_fragment
from rescue_operations.map_snapshot import ALIVE_FILE, LIVE_YAML, touch_alive, write_snapshot
from test_console_view import LedgerFixture

SECOND = 1_000_000_000


def grid(fill=0, size=40):
    # 40 x 40 cells of 0.05 m from (-1, -2): the fixture robots at (0.5, -1.0) land on it
    return dict(width=size, height=size, data=[fill] * (size * size), resolution=0.05, origin_x=-1.0, origin_y=-2.0,
                orientation=(0.0, 0.0, 0.0, 1.0))


class LiveMapFixture(LedgerFixture):
    def setUp(self):
        super().setUp()
        self.maps = self.dir / 'live'
        self.maps.mkdir()
        self.yaml = self.maps / LIVE_YAML
        self.now = [0]

    def live(self, stale_s=10.0):
        return LiveMap(self.yaml, stale_s=stale_s, clock=lambda: self.now[0])

    def received(self, at_ns):
        os.utime(self.yaml, ns=(at_ns, at_ns))

    def view(self, live, status=True):
        status_path = None
        if status:
            status_path = self.dir / 'status.json'
            status_path.write_text(json.dumps(self.status), encoding='utf-8')
        return current_view(self.db, status_path, clock=lambda: self.now[0], map_source=live)


class LiveMapTests(LiveMapFixture):
    def test_no_map_yet_is_said_not_an_error(self):
        image, state = self.live().current()
        self.assertIsNone(image)
        self.assertEqual('MISSING', state['state'])
        view = self.view(self.live())
        self.assertIn('지도 없음', ' '.join(view['warnings']))
        self.assertIn('실시간 지도 없음', render_fragment(view))

    def test_a_new_map_is_read_and_shows_its_version(self):
        live = self.live()
        sha = write_snapshot(self.maps, **grid())
        self.received(100 * SECOND)
        self.now[0] = 103 * SECOND
        image, state = live.current()
        self.assertEqual(sha, image.meta['pgm_sha256'])
        self.assertEqual(('OK', 100 * SECOND, 3.0), (state['state'], state['received_at_ns'], state['age_s']))
        html = render_fragment(self.view(live))
        self.assertIn(sha[:12], html)          # hash on screen
        self.assertIn('40×40', html)           # size on screen
        self.assertIn('3초 전', html)          # received age on screen

        second = write_snapshot(self.maps, **grid(fill=100))
        self.received(104 * SECOND)
        self.now[0] = 105 * SECOND
        self.assertEqual(second, live.current()[0].meta['pgm_sha256'])

    def test_a_stale_map_is_flagged(self):
        live = self.live(stale_s=10.0)
        write_snapshot(self.maps, **grid())
        self.received(100 * SECOND)
        self.now[0] = 111 * SECOND
        image, state = live.current()
        self.assertIsNotNone(image)            # still drawn, but said to be old
        self.assertEqual('STALE', state['state'])
        view = self.view(live)
        self.assertIn('지도 낡음', ' '.join(view['warnings']))
        self.assertIn('class="map stale"', render_fragment(view))

    def test_an_unreadable_new_map_keeps_the_old_one_and_says_so(self):
        live = self.live()
        write_snapshot(self.maps, **grid())
        self.received(100 * SECOND)
        self.now[0] = 101 * SECOND
        good = live.current()[0]
        self.yaml.write_text('image: absent.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n', encoding='utf-8')
        self.received(102 * SECOND)
        image, state = live.current()
        self.assertIs(good, image)
        self.assertEqual('ERROR', state['state'])
        self.assertIn('지도 읽기 실패', ' '.join(self.view(live)['warnings']))


class FrozenMapTests(LiveMapFixture):
    """After the map is frozen TB-6 republishes it once on the same topic (OPS-117/118): the hash stops
    changing and no new message comes. A live publisher keeps it current; only a gone publisher is stale."""

    def alive(self, at_ns):
        touch_alive(self.maps)
        os.utime(self.maps / ALIVE_FILE, ns=(at_ns, at_ns))

    def test_an_unchanged_map_with_a_live_publisher_is_not_stale(self):
        live = self.live(stale_s=10.0)
        write_snapshot(self.maps, **grid())
        self.received(100 * SECOND)
        self.alive(195 * SECOND)
        self.now[0] = 200 * SECOND
        image, state = live.current()
        self.assertEqual('OK', state['state'])
        self.assertEqual((100 * SECOND, 100.0), (state['received_at_ns'], state['age_s']))   # still when it came

    def test_a_gone_publisher_is_stale(self):
        live = self.live(stale_s=10.0)
        write_snapshot(self.maps, **grid())
        self.received(100 * SECOND)
        self.alive(150 * SECOND)
        self.now[0] = 161 * SECOND
        self.assertEqual('STALE', live.current()[1]['state'])
        self.assertIn('발행', live.current()[1]['message'])

    def test_touch_alive_creates_then_refreshes(self):
        touch_alive(self.maps)
        first = (self.maps / ALIVE_FILE).stat().st_mtime_ns
        os.utime(self.maps / ALIVE_FILE, ns=(1, 1))
        touch_alive(self.maps)
        self.assertGreater((self.maps / ALIVE_FILE).stat().st_mtime_ns, 1)
        self.assertGreater(first, 1)


class UnplacedRobotTests(LiveMapFixture):
    def test_a_robot_without_a_pose_is_listed_not_placed(self):
        """TB-5 has no pose before the map is fixed (owner 2026-10-13): say so, do not invent one."""
        self.status['robot_status']['robots'][1]['pose'] = None
        write_snapshot(self.maps, **grid())
        self.received(0)
        view = self.view(self.live())
        self.assertEqual(['robot_a'], [r['id'] for r in view['map']['robots']])
        self.assertEqual(['robot_b'], [r['id'] for r in view['map']['no_pose']])
        self.assertIn('robot_b: 위치 미확정', render_fragment(view))


class ExitMapTests(LiveMapFixture):
    """The newest exit_qr observation (#237, OPS-150) on the map: 「출구(QR) 관측 HH:MM:SS」, said to be an
    observation, not the exit the coordinator registered. Nothing is shown or warned without the table."""

    def exits(self, x=0.5, y=-1.5, frame='map'):
        self.status['exit_locations'] = {'schema_version': '1.3', 'exits': [
            {'location_id': 'e0e0e0e0-0000-5000-8000-000000000001', 'observed_at_ns': 100 * SECOND,
             'received_at_ns': 101 * SECOND, 'map_session_id': 'e0e0e0e0-0000-5000-8000-000000000002',
             'map_revision': 1, 'transform_revision': 0, 'mission_frame': frame, 'x': x, 'y': y,
             'class_label': 'exit_qr'}]}

    def test_the_exit_is_drawn_with_its_observation_time(self):
        self.exits()
        write_snapshot(self.maps, **grid())
        self.received(0)
        view = self.view(self.live())
        self.assertEqual(1, len(view['map']['exits']))
        html = render_fragment(view)
        stamp = datetime.fromtimestamp(100).strftime('%H:%M:%S')
        self.assertIn(f'출구(QR) 관측 {stamp}', html)
        self.assertIn('class="exit"', html)
        self.assertIn('map_revision 1', html)          # in the hover title only

    def test_an_exit_off_the_map_is_listed_not_placed(self):
        self.exits(x=50.0)
        write_snapshot(self.maps, **grid())
        self.received(0)
        view = self.view(self.live())
        self.assertEqual([], view['map']['exits'])
        self.assertIn('출구(QR): 지도 밖', render_fragment(view))

    def test_no_exit_table_shows_nothing(self):
        write_snapshot(self.maps, **grid())
        self.received(0)
        view = self.view(self.live())
        self.assertIsNone(view['exit'])
        self.assertEqual([], view['map']['exits'])
        self.assertNotIn('출구(QR)', render_fragment(view))
        self.assertFalse([w for w in view['warnings'] if 'exit' in w])


if __name__ == '__main__':
    unittest.main()
