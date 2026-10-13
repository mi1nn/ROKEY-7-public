"""로봇 카메라 칸(OPS-191): 웹캠 옆에 고른 로봇 하나의 판정 영상, TB-6/TB-5 선택(기본 TB-6, 누구나 — 보기 설정).

영상과 선택은 operations PC 의 robot_view_relay 가 맡는다(--robot-camera URL). 끊기면 「카메라 영상 낡음」이고
마지막 화면을 감춘다(웹캠 칸과 같은 감시 스크립트).
"""
import json
import unittest

from rescue_console.app import parse_robot_camera
from rescue_console.view import ConsoleError
from test_console_view import write_site_state
from test_console_web import ADMIN, CUSTOMER, HAS_FLASK, SECRET, AccountsFixture

URL = 'http://ops.invalid:18083/robot_view.mjpg'
SELECT = 'http://ops.invalid:18083/select'


class ParseRobotCamera(unittest.TestCase):
    def test_http_url_only(self):
        self.assertEqual(URL, parse_robot_camera(URL))
        for bad in ('robot_a=' + URL, 'file:///x', 'ops.invalid:18083/robot_view.mjpg'):
            with self.subTest(bad=bad), self.assertRaises(ConsoleError):
                parse_robot_camera(bad)


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
class RobotCameraRouteTests(AccountsFixture):
    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        write_site_state(self.site_db, self.status)
        self.calls = []
        self.state = (200, json.dumps({'robot': 'robot_a', 'robots': ['robot_a', 'robot_b']}).encode())
        self.reply = (200, json.dumps({'robot': 'robot_b'}).encode())

    def transport(self, url, payload, timeout):
        self.calls.append((url, payload))
        return self.state if payload is None else self.reply

    def client(self, account=ADMIN):
        from rescue_console.app import create_app
        app = create_app(self.db, accounts_path=self.accounts, secret_key=SECRET, site_state_path=self.site_db,
                         command_transport=self.transport, robot_camera=URL, camera_stale_s=2.0,
                         robot_names={'robot_a': 'TB-6', 'robot_b': 'TB-5'})
        client = app.test_client()
        self.login(client, *account)
        return client

    def test_panel_shows_the_selected_robot_stream_and_choices(self):
        page = self.client().get('/').get_data(as_text=True)
        self.assertIn(f'src="{URL}"', page)
        self.assertIn('data-lost-text="카메라 영상 낡음"', page)
        self.assertIn('<option value="robot_a" selected>TB-6</option>', page)
        self.assertIn('<option value="robot_b">TB-5</option>', page)
        self.assertEqual((SELECT, None), self.calls[0])

    def test_anyone_can_switch_without_a_confirm(self):
        client = self.client(CUSTOMER)
        csrf = self.csrf(client, '/')
        self.calls.clear()
        response = client.post('/robot-camera', data={'csrf': csrf, 'robot': 'robot_b'})
        self.assertEqual(302, response.status_code)
        self.assertEqual([(SELECT, {'robot': 'robot_b'})], [c for c in self.calls if c[1] is not None])
        self.assertNotIn('confirm(', client.get('/').get_data(as_text=True).split('id="robot-camera"')[1][:2000])

    def test_csrf_is_checked(self):
        response = self.client().post('/robot-camera', data={'csrf': 'nope', 'robot': 'robot_b'})
        self.assertEqual(400, response.status_code)
        self.assertEqual([], [c for c in self.calls if c[1] is not None])

    def test_relay_down_is_said(self):
        from rescue_console.commands import CommandSendError

        def down(url, payload, timeout):
            raise CommandSendError('UNREACHABLE', 'connection refused', delivery='NOT_SENT')
        self.transport = down
        self.assertIn('로봇 카메라 선택 상태를 못 받음', self.client().get('/').get_data(as_text=True))


if __name__ == '__main__':
    unittest.main()
