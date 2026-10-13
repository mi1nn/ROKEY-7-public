"""관제 웹에서 웹캠 고르기(OPS-183): operations 가 꽂힌 카메라 목록을 주고, 고른 장치로 webcam_node 를 바꾼다.

없는 장치는 거절하고(이전 장치로 조용히 두지 않음), 바꾼 것·거절된 것은 error_log 에 남는다(관제 웹 「명령·오류 기록」).
"""
import json
from pathlib import Path
import tempfile
import unittest

from rescue_operations.command_api import WebcamSwitch
from rescue_operations.site_state import SiteStateLedger, read_error_log

BY_ID = '/dev/v4l/by-id/'
HP, USB = 'usb-HP_Wide_Vision-video-index0', 'usb-Logitech_C920-video-index0'


class RealLedger:
    """실제 현장 상태 장부(error_log 는 C-09 공통 코드만 받는다). 가짜로는 그 검사가 빠진다(E3 에서 잡힘)."""

    def __init__(self):
        self.path = Path(tempfile.mkdtemp()) / 'site_state.sqlite3'
        self.ledger = SiteStateLedger(self.path, allowed_robot_ids=('robot_a', 'robot_b'))

    def record_error(self, row):
        return self.ledger.record_error(row)

    @property
    def errors(self):
        return list(reversed(read_error_log(self.path)['errors']))


class WebcamSwitchTests(unittest.TestCase):
    def setUp(self):
        self.ledger, self.current, self.calls = RealLedger(), BY_ID + HP, []
        self.addCleanup(self.ledger.ledger.close)
        self.answer = (True, '')

        def set_device(path):
            self.calls.append(path)
            if self.answer[0]:
                self.current = path
            return self.answer

        self.switch = WebcamSwitch(self.ledger, list_cameras=lambda: [HP, USB], get_device=lambda: self.current,
                                   set_device=set_device, by_id_dir=BY_ID)

    def post(self, **body):
        return self.switch.switch(json.dumps(body).encode(), received_at_ns=5)

    def test_state_lists_plugged_cameras_and_the_current_one(self):
        self.assertEqual((200, {'cameras': [HP, USB], 'current': HP}), self.switch.state())

    def test_switch_sets_the_by_id_path_and_logs_it(self):
        status, body = self.post(device=USB, operator_id='admin')
        self.assertEqual((200, True, USB), (status, body['switched'], body['current']))
        self.assertEqual([BY_ID + USB], self.calls)
        self.assertEqual('SENSOR_UNAVAILABLE', self.ledger.errors[0]['code'])   # 판정이 다시 시작되는 동안 비어 있음
        self.assertTrue(self.ledger.errors[0]['detail'].startswith('CAMERA_SWITCHED'))
        self.assertIn('admin', self.ledger.errors[0]['detail'])
        self.assertIn('지진 판정 다시 시작', self.ledger.errors[0]['detail'])

    def test_unplugged_device_is_refused_not_kept_quietly(self):
        status, body = self.post(device='usb-Gone-video-index0', operator_id='admin')
        self.assertEqual((400, False, 'UNKNOWN_DEVICE'), (status, body['switched'], body['reason']))
        self.assertEqual([HP, USB], body['cameras'])
        self.assertEqual([], self.calls)
        self.assertEqual('NOT_FOUND', self.ledger.errors[0]['code'])
        self.assertTrue(self.ledger.errors[0]['detail'].startswith('CAMERA_SWITCH_REFUSED'))

    def test_node_refusal_is_said(self):
        self.answer = (False, 'no such camera device')
        status, body = self.post(device=USB, operator_id='admin')
        self.assertEqual((200, False, 'no such camera device', HP), (status, body['switched'], body['reason'],
                                                                     body['current']))
        self.assertEqual('INVALID_INPUT', self.ledger.errors[0]['code'])
        self.assertTrue(self.ledger.errors[0]['detail'].startswith('CAMERA_SWITCH_REFUSED'))

    def test_bad_body_is_refused(self):
        for raw in (b'not json', json.dumps({'device': HP}).encode(), json.dumps({'device': 3, 'operator_id': 'a'}).encode()):
            with self.subTest(raw=raw):
                self.assertEqual(400, self.switch.switch(raw, received_at_ns=5)[0])
        self.assertEqual([], self.calls)

    def test_webcam_node_not_answering_is_503(self):
        from rescue_operations.command_api import CommandUnavailable

        def gone(_path):
            raise CommandUnavailable('webcam_node/set_parameters not available')
        self.switch.set_device = gone
        status, body = self.post(device=USB, operator_id='admin')
        self.assertEqual((503, False), (status, body['switched']))
        self.assertEqual('DRIVER_UNAVAILABLE', self.ledger.errors[0]['code'])


if __name__ == '__main__':
    unittest.main()
