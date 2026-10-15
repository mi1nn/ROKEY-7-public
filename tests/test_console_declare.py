"""재난 선포(DECLARE_DISASTER, 재범 결정·작전 설계 2026-10-15): 평시에 운영자가 지진 감지 대신 재난 모드로 넣는다.
지진 감지와 버튼 가운데 먼저 온 쪽이 재난 모드를 열고, 나중 것은 조정기가 ALREADY_DISASTER 로 거절한다.
버튼은 ADMIN 만, 평시(NORMAL)에만 켜고, 확인 창에서 무엇이 움직이는지 한 번 더 보인 뒤 한 번만 보낸다."""
import html
import json
import re
import unittest

from rescue_operations.site_state import SiteStateLedger
from rescue_console.commands import COMMAND_PATH, MODE_COMMAND_FIELDS, build_request, command_access
from rescue_console.render import render_fragment, render_page
from rescue_console.sources import load_ledger
from rescue_console.view import ConsoleError, build_view, mission_reason
from test_console_commands import COMMAND_ID, OPS, answer, mission_state
from test_console_view import ROBOTS, SECOND, T0, LedgerFixture, uid, write_site_state
from test_console_web import ADMIN, CUSTOMER, HAS_FLASK, SECRET, AccountsFixture

NAMES = {'robot_a': 'TB-6', 'robot_b': 'TB-5'}
USER = {'username': 'admin', 'display_name': 'admin', 'role': 'ADMIN'}


class DeclareRequestTests(unittest.TestCase):
    def test_carries_only_the_command_and_operator(self):
        payload = build_request({'csrf': 't', 'kind': 'DECLARE_DISASTER'}, operator_id='admin', command_id=COMMAND_ID)
        self.assertEqual(payload, {'command_id': COMMAND_ID, 'kind': 'DECLARE_DISASTER', 'operator_id': 'admin'})
        with self.assertRaises(ConsoleError):
            build_request({'kind': 'DECLARE_DISASTER', 'target_id': 't-1'}, operator_id='admin', command_id=COMMAND_ID)

    def test_fields_match_the_coordinator(self):
        from rescue_coordination.mission import MODE_COMMAND_FIELDS as GATE
        self.assertEqual({kind: set(fields) for kind, fields in MODE_COMMAND_FIELDS.items()},
                         {kind: set(fields) for kind, fields in GATE.items()})

    def test_refusal_and_trigger_have_words(self):
        # 거절 답은 TASK_BUSY 이고, ALREADY_DISASTER 는 명령 기록의 사유로 온다(#330).
        self.assertEqual(mission_reason('ALREADY_DISASTER', 'TB-5', 'TB-6')['text'], '이미 재난 모드입니다 — 재난 선포 거절')
        self.assertEqual(mission_reason('DISASTER_DECLARED_BY_OPERATOR', 'TB-5', 'TB-6')['text'],
                         '운영자 선포 — 지진 감지가 아니라 재난 선포 버튼으로 재난 모드')


class DeclareGateTests(LedgerFixture):
    def view_with(self, mission, search_start=None):
        status = dict(self.status, mission_state={'schema_version': '1.2', 'mission': mission})
        return build_view(load_ledger(self.db), status, built_at_ns=T0 + 30 * SECOND, robot_names=NAMES,
                          search_start=search_start)

    def gate(self, mission, search_start=None):
        return {c['name']: c for c in self.view_with(mission, search_start)['commands']}['DECLARE_DISASTER']

    def test_on_only_in_normal(self):
        gate = self.gate(mission_state('NORMAL', 'IDLE'))
        self.assertEqual((gate['enabled'], gate['group'], gate['label'], gate['args'], gate['choices']),
                         (True, 'approve', '재난 선포', {}, []))
        for mission, words in ((mission_state('DISASTER', 'DISASTER'), '이미 재난 모드'),
                               (mission_state('DISASTER', 'SEARCHING'), '이미 재난 모드'),
                               (mission_state('CLEARING', 'IDLE'), '평시 아님'),
                               (mission_state('MISSION_FINISHED', 'FINISHED'), '평시 아님'),
                               (None, '임무 상태')):
            with self.subTest(mission=mission and mission['mode']):
                gate = self.gate(mission)
                self.assertFalse(gate['enabled'])
                self.assertIn(words, gate['reason'])

    def test_confirm_says_what_moves(self):
        normal = mission_state('NORMAL', 'IDLE')
        self.assertIn('TB-6 이 바로 탐색을 시작합니다', self.gate(normal, 'AUTO')['confirm'])
        self.assertIn('수색 시작 버튼', self.gate(normal, 'OPERATOR')['confirm'])
        self.assertNotIn('TB-6', self.gate(normal)['confirm'])

    def test_auto_start_names_both_ways(self):
        view = self.view_with(mission_state('NORMAL', 'IDLE'), 'AUTO')
        start = {c['name']: c for c in view['commands']}['START_SEARCH']
        self.assertTrue(start['auto'])
        self.assertIn('재난 선포', start['reason'])

    def test_button_and_gate_reach_the_page(self):
        view = self.view_with(mission_state('NORMAL', 'IDLE'), 'AUTO')
        page = render_page(view, USER, csrf='tok', command_access=command_access('ADMIN', True))
        button = re.search(r'<button[^>]*data-command="DECLARE_DISASTER"[^>]*>', page).group(0)
        self.assertNotIn(' disabled', button)
        self.assertIn('gate.confirm ||', page)   # 확인 창 첫 줄은 관문의 confirm 문구
        gates = json.loads(html.unescape(re.search(r'data-gates="([^"]*)"', render_fragment(view)).group(1)))
        self.assertIn('TB-6 이 바로 탐색을 시작합니다', gates['DECLARE_DISASTER']['confirm'])
        self.assertNotIn('confirm', gates['START_SEARCH'])
        page = render_page(view, dict(USER, role='CUSTOMER'), csrf='tok', command_access=command_access('CUSTOMER', True))
        self.assertIn(' disabled', re.search(r'<button[^>]*data-command="DECLARE_DISASTER"[^>]*>', page).group(0))


def command_row(command_id, kind='DECLARE_DISASTER', accepted=True):
    return {'command_id': command_id, 'kind': kind, 'operator_id': 'admin', 'robot_id': None, 'target_id': None,
            'segment_index': None, 'route_revision': None, 'requested_at_ns': T0, 'accepted': accepted,
            'result': 'ACCEPTED' if accepted else 'REJECTED', 'reason': None if accepted else 'TASK_BUSY'}


class CauseTests(unittest.TestCase):
    """재난 출처(작전 2026-10-15): trigger_event_id 로 접수된 재난 선포 명령이나 지진 감지 사건을 찾는다."""

    class Sign:
        kind = 'DISASTER_SIGN'

        def __init__(self, event_id):
            self.event_id = event_id

    def test_operator_sign_or_unknown(self):
        from rescue_console.view import mission_cause
        self.assertEqual(mission_cause(COMMAND_ID, [], [command_row(COMMAND_ID)]), 'OPERATOR')
        self.assertEqual(mission_cause(COMMAND_ID, [self.Sign(COMMAND_ID)], []), 'SIGN')
        for commands in ([], [command_row(COMMAND_ID, accepted=False)], [command_row(COMMAND_ID, 'START_SEARCH')]):
            with self.subTest(commands=commands):
                self.assertEqual(mission_cause(COMMAND_ID, [self.Sign('other')], commands), 'UNKNOWN')
        self.assertIsNone(mission_cause(None, [], []))


class CauseBandTests(LedgerFixture):
    def band(self, **status):
        row = dict(mission_state('DISASTER', 'SEARCHING'), trigger_event_id=COMMAND_ID)
        view = build_view(load_ledger(self.db), dict(self.status, mission_state={'schema_version': '1.2', 'mission': row},
                                                      **status), built_at_ns=T0 + 30 * SECOND, robot_names=NAMES)
        return view['mission']['cause'], render_fragment(view)

    def test_operator_declared_is_said_on_the_band(self):
        for key in ('operator_commands', 'trigger_commands'):   # 최근 명령에 있든, 시작 시각 앞뒤에서 찾았든
            with self.subTest(key=key):
                cause, fragment = self.band(**{key: {'schema_version': '1.0', 'commands': [command_row(COMMAND_ID)]}})
                self.assertEqual(cause, 'OPERATOR')
                self.assertIn('운영자 선포 시각', fragment)
                self.assertIn('운영자 선포', fragment)

    def test_unknown_is_said(self):
        cause, fragment = self.band(operator_commands={'schema_version': '1.0', 'commands': []})
        self.assertEqual(cause, 'UNKNOWN')
        self.assertIn('출처 모름', fragment)


class TriggerLookupTests(LedgerFixture):
    """현장 상태 DB 에서: 최근 30건 밖으로 밀려난 재난 선포도 임무 시작 시각 앞뒤에서 찾는다."""

    def test_declare_out_of_the_recent_rows_is_found(self):
        from rescue_console.sources import HISTORY_ROWS, load_site_state
        site_db = self.dir / 'site_state.sqlite3'
        write_site_state(site_db, self.status)
        declare = COMMAND_ID
        with SiteStateLedger(site_db, allowed_robot_ids=ROBOTS) as ledger:
            ledger.record_operator_command(dict(command_row(declare), requested_at_ns=T0))
            for n in range(HISTORY_ROWS + 5):
                ledger.record_operator_command(dict(command_row(uid(f'later-{n}'), 'START_SEARCH', accepted=False),
                                                    requested_at_ns=T0 + (n + 1) * SECOND))
            ledger.record_mission_state(dict(mission_state('DISASTER', 'SEARCHING'), trigger_event_id=declare,
                                             started_at_ns=T0 + 1))
        status = load_site_state(site_db)
        self.assertNotIn(declare, [c['command_id'] for c in status['operator_commands']['commands']])
        self.assertEqual([c['command_id'] for c in status['trigger_commands']['commands']], [declare])
        view = build_view(load_ledger(self.db), status, built_at_ns=T0 + 60 * SECOND, robot_names=NAMES)
        self.assertEqual(view['mission']['cause'], 'OPERATOR')


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
class DeclareRouteTests(AccountsFixture):
    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        write_site_state(self.site_db, self.status)
        with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            ledger.record_mission_state(mission_state('NORMAL', 'IDLE'))
        self.calls = []
        self.reply = (200, answer())

    def transport(self, url, payload, timeout):
        self.calls.append((url, payload))
        return self.reply

    def client(self, account=ADMIN):
        from rescue_console.app import create_app
        app = create_app(self.db, accounts_path=self.accounts, secret_key=SECRET, site_state_path=self.site_db,
                         operations_url=OPS, command_transport=self.transport)
        client = app.test_client()
        self.login(client, *account)
        return client

    def post(self, client):
        return client.post('/commands', data={'kind': 'DECLARE_DISASTER', 'csrf': self.csrf(client, '/')})

    def test_admin_sends_once_through_the_command_api(self):
        result = self.post(self.client()).get_json()
        self.assertEqual((result['sent'], result['accepted']), (True, True))
        self.assertEqual(len(self.calls), 1)
        url, payload = self.calls[0]
        self.assertEqual(url, OPS + COMMAND_PATH)
        self.assertEqual(set(payload), {'command_id', 'kind', 'operator_id'})
        self.assertEqual((payload['kind'], payload['operator_id']), ('DECLARE_DISASTER', 'admin'))

    def test_refusal_is_shown_as_given(self):
        self.reply = (200, answer(False, 'TASK_BUSY'))   # 조정기: 이미 재난이면 TASK_BUSY(ALREADY_DISASTER)
        result = self.post(self.client()).get_json()
        self.assertEqual((result['accepted'], result['reason'], result['reason_label']),
                         (False, 'TASK_BUSY', '이미 재난 모드입니다'))

    def test_customer_cannot_send(self):
        response = self.post(self.client(CUSTOMER))
        self.assertEqual(response.status_code, 403)
        self.assertEqual(self.calls, [])


if __name__ == '__main__':
    unittest.main()
