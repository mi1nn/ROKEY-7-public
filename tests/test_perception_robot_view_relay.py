"""로봇 카메라 칸(OPS-191, 재범 결정): 고른 로봇 하나의 판정 그린 영상만 중계한다(기본 TB-6, 누구나 바꿈).

고르지 않은 로봇은 구독하지 않는다(Wi-Fi 에 영상 한 줄). 바꾸면 이전 로봇의 마지막 화면을 버린다(다른 로봇 화면으로
보이지 않게). 바꾸기는 HTTP 스레드가 요청하고 노드 타이머가 구독을 바꾼다.
"""
import json
import threading
import unittest
import urllib.request

from rescue_perception.mjpeg_relay import FrameHub, make_server
from rescue_perception.robot_view_relay import Selector, parse_sources

SOURCES = ['robot_a=/robot6/rescue/passage_view/compressed', 'robot_b=/robot5/rescue/passage_view/compressed']


class ParseSources(unittest.TestCase):
    def test_pairs_in_order(self):
        self.assertEqual({'robot_a': '/robot6/rescue/passage_view/compressed',
                          'robot_b': '/robot5/rescue/passage_view/compressed'}, parse_sources(SOURCES))

    def test_bad_items_are_refused(self):
        for bad in (['robot_a'], ['robot_a=robot6/x'], SOURCES + ['robot_a=/x/y'], []):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                parse_sources(bad)


class SelectorTests(unittest.TestCase):
    def setUp(self):
        self.selector = Selector(parse_sources(SOURCES), 'robot_a')

    def test_default_is_pending_once_then_applied(self):
        self.assertEqual(('robot_a', '/robot6/rescue/passage_view/compressed'), self.selector.take_pending())
        self.assertIsNone(self.selector.take_pending())

    def test_select_known_robot(self):
        self.selector.take_pending()
        self.assertEqual((True, ''), self.selector.select('robot_b'))
        self.assertEqual('robot_b', self.selector.state()['robot'])
        self.assertEqual(('robot_b', '/robot5/rescue/passage_view/compressed'), self.selector.take_pending())

    def test_unknown_robot_is_refused_and_nothing_changes(self):
        self.selector.take_pending()
        ok, reason = self.selector.select('robot_x')
        self.assertFalse(ok)
        self.assertIn('robot_x', reason)
        self.assertEqual('robot_a', self.selector.state()['robot'])
        self.assertIsNone(self.selector.take_pending())

    def test_default_must_be_a_source(self):
        with self.assertRaises(ValueError):
            Selector(parse_sources(SOURCES), 'robot_x')


class HubReset(unittest.TestCase):
    def test_reset_drops_the_last_frame(self):
        hub = FrameHub(5.0)
        hub.put(b'\xff\xd8old')
        self.assertIsNotNone(hub.fresh())
        hub.reset()
        self.assertIsNone(hub.fresh())


class SelectRoute(unittest.TestCase):
    def setUp(self):
        self.selector = Selector(parse_sources(SOURCES), 'robot_a')
        self.server = make_server(FrameHub(1.0), '127.0.0.1', 0, '/robot_view.mjpg', select=self.selector)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.base = f'http://127.0.0.1:{self.server.server_address[1]}'

    def call(self, body=None):
        req = urllib.request.Request(self.base + '/select', method='GET' if body is None else 'POST',
                                     data=None if body is None else json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=2) as res:
                return res.status, json.loads(res.read())
        except urllib.error.HTTPError as exc:
            with exc:
                return exc.code, json.loads(exc.read())

    def test_get_and_post(self):
        self.assertEqual((200, {'robot': 'robot_a', 'robots': ['robot_a', 'robot_b']}), self.call())
        self.assertEqual(200, self.call({'robot': 'robot_b'})[0])
        self.assertEqual('robot_b', self.call()[1]['robot'])
        self.assertEqual(400, self.call({'robot': 'robot_x'})[0])

    def test_without_selector_select_is_404(self):
        server = make_server(FrameHub(1.0), '127.0.0.1', 0, '/cam.mjpg')
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        with self.assertRaises(urllib.error.HTTPError) as caught:
            urllib.request.urlopen(f'http://127.0.0.1:{server.server_address[1]}/select', timeout=2)
        self.assertEqual(404, caught.exception.code)


if __name__ == '__main__':
    unittest.main()
