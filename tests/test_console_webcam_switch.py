"""관제 웹 웹캠 칸에서 카메라 고르기(OPS-183): 관리자만, operations 로 한 번 보내고, 답을 그대로 보인다.

목록은 operations(GET /api/webcam)에서 온다. 재난 모드에서는 바꾸기 전에 확인을 한 번 더 받는다(지진 판정 다시 시작·출구 감시 풀림).
"""
import json
import unittest

from rescue_operations.site_state import SiteStateLedger
from rescue_console.commands import WEBCAM_PATH
from test_console_commands import OPS, mission_state
from test_console_view import ROBOTS, write_site_state
from test_console_web import ADMIN, CUSTOMER, HAS_FLASK, SECRET, AccountsFixture

HP, USB = 'usb-HP_Wide_Vision-video-index0', 'usb-Logitech_C920-video-index0'
CAMERAS = [('cam_fixed', 'http://cam.invalid:18082/cam_fixed.mjpg')]


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
class WebcamSwitchRouteTests(AccountsFixture):
    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        write_site_state(self.site_db, self.status)
        self.calls = []
        self.state = (200, json.dumps({'cameras': [HP, USB], 'current': HP}).encode())
        self.reply = (200, json.dumps({'switched': True, 'current': USB}).encode())

    def transport(self, url, payload, timeout):
        self.calls.append((url, payload))
        return self.state if payload is None else self.reply

    def client(self, account=ADMIN, mode=None):
        from rescue_console.app import create_app
        if mode:
            with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
                ledger.record_mission_state(mission_state(mode, mode))
        app = create_app(self.db, accounts_path=self.accounts, secret_key=SECRET, site_state_path=self.site_db,
                         operations_url=OPS, command_transport=self.transport, cameras=CAMERAS, camera_stale_s=5.0)
        client = app.test_client()
        self.login(client, *account)
        return client

    def test_admin_sees_the_plugged_cameras_with_the_current_one_selected(self):
        page = self.client().get('/').get_data(as_text=True)
        self.assertIn(f'<option value="{USB}">', page)
        self.assertIn(f'<option value="{HP}" selected>', page)
        self.assertIn('카메라 바꾸기', page)
        self.assertEqual((OPS + WEBCAM_PATH, None), self.calls[0])

    def test_customer_sees_no_picker(self):
        page = self.client(CUSTOMER).get('/').get_data(as_text=True)
        self.assertNotIn('카메라 바꾸기', page)

    def test_disaster_mode_asks_once_more(self):
        page = self.client(mode='DISASTER').get('/').get_data(as_text=True)
        self.assertIn('임무 진행 중', page)
        self.assertIn('confirm(', page)

    def test_switch_is_sent_once_and_the_answer_shown(self):
        client = self.client()
        csrf = self.csrf(client, '/')
        self.calls.clear()
        response = client.post('/webcam', data={'csrf': csrf, 'device': USB})
        self.assertEqual(302, response.status_code)
        posts = [c for c in self.calls if c[1] is not None]
        self.assertEqual([(OPS + WEBCAM_PATH, {'device': USB, 'operator_id': 'admin'})], posts)
        page = client.get('/').get_data(as_text=True)
        self.assertIn('카메라 전환 — 지진 판정 다시 시작', page)

    def test_refusal_is_shown_as_given(self):
        self.reply = (400, json.dumps({'switched': False, 'reason': 'UNKNOWN_DEVICE', 'cameras': [HP]}).encode())
        client = self.client()
        client.post('/webcam', data={'csrf': self.csrf(client, '/'), 'device': USB})
        self.assertIn('UNKNOWN_DEVICE', client.get('/').get_data(as_text=True))

    def test_customer_cannot_switch(self):
        client = self.client(CUSTOMER)
        response = client.post('/webcam', data={'csrf': self.csrf(client, '/'), 'device': USB})
        self.assertEqual(403, response.status_code)
        self.assertEqual([], [c for c in self.calls if c[1] is not None])

    def test_operations_down_is_said_not_a_crash(self):
        from rescue_console.commands import CommandSendError

        def down(url, payload, timeout):
            raise CommandSendError('UNREACHABLE', 'connection refused', delivery='NOT_SENT')
        self.transport = down
        page = self.client().get('/').get_data(as_text=True)
        self.assertIn('카메라 목록을 못 받음', page)


if __name__ == '__main__':
    unittest.main()
