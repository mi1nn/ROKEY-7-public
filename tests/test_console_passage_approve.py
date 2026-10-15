"""터널 통과 승인(#406 /rescue/passage_keepout/approve, 작전 2026-10-16): 관제 웹 지도에서 막힌 터널을 사람이 푼다.

웹은 ROS 노드가 아니다. 버튼 → 관제 웹 서버가 로그인 계정 이름을 operator_id 로 붙임 → 운영 API → 운영 노드가
서비스를 부른다. 브라우저가 보낸 이름은 받지 않는다. 고를 수 있는 터널은 passage_keepout 이 막힌 곳으로 칠하는
것(BLOCKED, 또는 UNKNOWN 가운데 NO_FLOOR_NEAR)뿐이다. 승인 기록(누가·언제)은 운영이 operator_commands 에
APPROVE_PASSAGE 로 남기고, 화면은 그 행을 사건 기록과 지도에 보인다.
"""
import json
import unittest

from rescue_console.commands import (COMMAND_PATH, PASSAGE_PATH, build_request, send_command)
from rescue_console.render import render_fragment, render_page
from rescue_operations.site_state import SiteStateLedger
from test_console_commands import COMMAND_ID, OPS, answer, mission_state
from test_console_live_map import LiveMapFixture, grid
from test_console_tunnel_map import SESSION, item
from test_console_view import ROBOTS, SECOND, T0, uid, write_site_state
from test_console_web import ADMIN, CUSTOMER, HAS_FLASK, SECRET, AccountsFixture
from rescue_operations.map_snapshot import write_snapshot

ITEM = item(2, 'BLOCKED', 0.40, 0.2, -1.2)['id']


def no_floor_near(ident, x, y):
    return dict(item(ident, 'UNKNOWN', None, x, y), reason='NO_FLOOR_NEAR')


def approval(item_id, *, accepted=True, reason=None, by='admin', offset=20):
    """운영이 남길 operator_commands 행(작전 2026-10-16 요청: kind APPROVE_PASSAGE, target_id 에 항목 id)."""
    return dict(command_id=uid(f'approve/{item_id}/{offset}'), kind='APPROVE_PASSAGE', operator_id=by,
                robot_id=None, target_id=item_id, segment_index=None, route_revision=None,
                requested_at_ns=T0 + offset * SECOND, accepted=accepted,
                result='ACCEPTED' if accepted else 'REJECTED', reason=reason)


class PassageRequestTests(unittest.TestCase):
    def build(self, **form):
        return build_request({'csrf': 'token', 'kind': 'APPROVE_PASSAGE', **form}, operator_id='admin',
                             command_id=COMMAND_ID)

    def test_item_id_and_the_session_name_only(self):
        self.assertEqual(self.build(item_id=ITEM), {'command_id': COMMAND_ID, 'kind': 'APPROVE_PASSAGE',
                                                    'operator_id': 'admin', 'item_id': ITEM})

    def test_a_browser_name_or_bad_item_is_refused(self):
        for form in ({'item_id': ITEM, 'operator_id': 'someone'}, {'item_id': ITEM, 'approved_by': 'someone'},
                     {'item_id': ''}, {'item_id': 'not-a-uuid'}, {}):
            with self.subTest(form=form), self.assertRaises(Exception) as caught:
                self.build(**form)
            self.assertEqual('INVALID_INPUT', caught.exception.detail_code)


class PassageSendTests(unittest.TestCase):
    PAYLOAD = {'command_id': COMMAND_ID, 'kind': 'APPROVE_PASSAGE', 'operator_id': 'admin', 'item_id': ITEM}

    def test_goes_to_the_approve_path_once_and_keeps_the_message(self):
        calls = []

        def transport(url, payload, timeout):
            calls.append(url)
            return 200, answer(False, 'NO_SUCH_ITEM', item_id=ITEM, message='no such overhead passage item')
        result = send_command(OPS, self.PAYLOAD, transport=transport)
        self.assertEqual('/api/passages/approve', PASSAGE_PATH)
        self.assertEqual([OPS + PASSAGE_PATH], calls)
        self.assertNotEqual(PASSAGE_PATH, COMMAND_PATH)
        self.assertEqual((True, False, 'NO_SUCH_ITEM'), (result['sent'], result['accepted'], result['reason']))
        self.assertEqual('no such overhead passage item', result['message'])
        self.assertNotEqual('NO_SUCH_ITEM', result['reason_label'])   # 한국어 풀이

    def test_other_commands_still_go_to_operator_commands(self):
        calls = []
        send_command(OPS, {'command_id': COMMAND_ID, 'kind': 'START_SEARCH', 'operator_id': 'admin'},
                     transport=lambda url, payload, timeout: (calls.append(url), (200, answer()))[1])
        self.assertEqual([OPS + COMMAND_PATH], calls)


class PassageGateTests(LiveMapFixture):
    def passages(self, items, commands=()):
        self.status['overhead_passages'] = {'schema_version': '1.5', 'robots': {'robot_a': {
            'robot_id': 'robot_a', 'generated_at_ns': 5, 'received_at_ns': 6, 'map_frame': 'map', 'clearance_m': 0.46,
            'floor_noise_m': 0.02, 'lidar_plane_m': 0.18, 'items': items}},
            'current_map': {'robot_id': 'robot_a', 'map_session_id': SESSION, 'map_revision': 2,
                            'generated_at_ns': 5, 'received_at_ns': 6}}
        self.status['operator_commands'] = {'schema_version': '1.5', 'commands': list(commands)}

    def drawn(self):
        write_snapshot(self.maps, **grid())
        self.received(0)
        view = self.view(self.live())
        return view, render_fragment(view)

    def gate(self, view):
        return next(c for c in view['commands'] if c['name'] == 'APPROVE_PASSAGE')

    def test_only_what_the_keepout_paints_can_be_chosen(self):
        self.passages([item(1, 'PASSABLE', 0.47, 0.0, -1.0), item(2, 'BLOCKED', 0.40, 0.2, -1.2),
                       item(3, 'UNKNOWN', None, 0.4, -1.4), no_floor_near(4, 0.6, -1.6)])
        gate = self.gate(self.drawn()[0])
        self.assertTrue(gate['enabled'])
        self.assertEqual('approve', gate['group'])
        self.assertEqual([item(2, 'BLOCKED', 0.4, 0, 0)['id'], no_floor_near(4, 0, 0)['id']],
                         [c['item_id'] for c in gate['choices']])
        self.assertTrue(all(c['label'] for c in gate['choices']))
        self.assertIn('통과', gate['confirm'])

    def test_no_painted_tunnel_turns_the_button_off(self):
        self.passages([item(1, 'PASSABLE', 0.47, 0.0, -1.0), item(3, 'UNKNOWN', None, 0.4, -1.4)])
        gate = self.gate(self.drawn()[0])
        self.assertFalse(gate['enabled'])
        self.assertEqual([], gate['choices'])
        self.assertIn('승인할 터널 없음', gate['reason'])

    def test_on_when_not_in_a_disaster(self):
        """지도 막힘은 임무 단계와 따로다. 재난 모드가 아니어도 켠다(조정기를 거치지 않는다)."""
        self.passages([item(2, 'BLOCKED', 0.40, 0.2, -1.2)])
        view = self.drawn()[0]
        self.assertTrue(self.gate(view)['enabled'])

    def test_an_approved_item_shows_who_and_when_and_leaves_the_choices(self):
        self.passages([item(2, 'BLOCKED', 0.40, 0.2, -1.2), no_floor_near(4, 0.6, -1.6)],
                      [approval(ITEM, by='jaebeom'), approval(no_floor_near(4, 0, 0)['id'], accepted=False,
                                                              reason='NO_SUCH_ITEM', offset=21)])
        view, html = self.drawn()
        approved = {row['id']: row['approved'] for row in view['passages']}
        self.assertEqual('jaebeom', approved[ITEM]['by'])
        self.assertEqual(T0 + 20 * SECOND, approved[ITEM]['at_ns'])
        self.assertIsNone(approved[no_floor_near(4, 0, 0)['id']])      # 거절은 승인 아님
        self.assertEqual([no_floor_near(4, 0, 0)['id']], [c['item_id'] for c in self.gate(view)['choices']])
        self.assertIn('승인 jaebeom', html)

    def test_map_marks_carry_the_item_to_click(self):
        self.passages([item(1, 'PASSABLE', 0.47, 0.0, -1.0), item(2, 'BLOCKED', 0.40, 0.2, -1.2)])
        html = self.drawn()[1]
        self.assertIn(f'data-passage="{ITEM}"', html)
        self.assertEqual(1, html.count('data-passage='))               # 통과 가능은 누를 것이 없다

    def test_approvals_are_in_the_event_log_without_kpi(self):
        self.passages([item(2, 'BLOCKED', 0.40, 0.2, -1.2)],
                      [approval(ITEM, by='jaebeom'), approval(ITEM, accepted=False, reason='NO_APPROVER', offset=21)])
        view, html = self.drawn()
        rows = [e for e in view['events'] if e['kind'] == 'PASSAGE_APPROVAL']
        self.assertEqual([(T0 + 21 * SECOND, 'admin'), (T0 + 20 * SECOND, 'jaebeom')],
                         [(r['event_ns'], r['source']) for r in rows])
        self.assertIn('터널 통과 승인', html)
        self.assertIn('data-kind="PASSAGE_APPROVAL"', html)
        self.assertRegex(html, r'<tr class="event passage_approval"[^>]*data-kpi="0"')

    def test_panel_has_the_button_and_the_script_knows_items(self):
        self.passages([item(2, 'BLOCKED', 0.40, 0.2, -1.2)])
        view = self.drawn()[0]
        page = render_page(view, {'username': 'admin', 'display_name': 'admin', 'role': 'ADMIN'}, csrf='t', command_access=(True, 'ok'))
        self.assertIn('data-command="APPROVE_PASSAGE"', page)
        self.assertIn('item_id', page)                                  # 확인 창이 item_id 를 보낸다
        self.assertIn('data-passage', page)                             # 지도 표시를 누르면 확인 창


@unittest.skipUnless(HAS_FLASK, 'Flask not installed')
class PassageRouteTests(AccountsFixture):
    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        write_site_state(self.site_db, self.status)
        with SiteStateLedger(self.site_db, allowed_robot_ids=ROBOTS) as ledger:
            ledger.record_mission_state(mission_state('DISASTER', 'DISASTER'))
        self.calls = []

    def transport(self, url, payload, timeout):
        self.calls.append((url, payload, timeout))
        return 200, answer(item_id=payload.get('item_id'), message='approved')

    def client(self, account=ADMIN):
        from rescue_console.app import create_app
        app = create_app(self.db, accounts_path=self.accounts, secret_key=SECRET, site_state_path=self.site_db,
                         operations_url=OPS, command_transport=self.transport)
        client = app.test_client()
        self.login(client, *account)
        return client

    def post(self, client, **form):
        form['csrf'] = self.csrf(client, '/')
        return client.post('/commands', data=form)

    def test_admin_approval_goes_to_operations_with_the_session_name(self):
        response = self.post(self.client(), kind='APPROVE_PASSAGE', item_id=ITEM)
        self.assertEqual(200, response.status_code)
        url, payload, _timeout = self.calls[0]
        self.assertEqual(OPS + PASSAGE_PATH, url)
        self.assertEqual(('APPROVE_PASSAGE', 'admin', ITEM), (payload['kind'], payload['operator_id'], payload['item_id']))
        self.assertEqual(1, len(self.calls))

    def test_a_browser_supplied_name_is_refused(self):
        response = self.post(self.client(), kind='APPROVE_PASSAGE', item_id=ITEM, operator_id='someone')
        self.assertEqual((400, 'INVALID_INPUT'), (response.status_code, response.get_json()['error']))
        self.assertEqual([], self.calls)

    def test_customer_cannot_approve(self):
        response = self.post(self.client(CUSTOMER), kind='APPROVE_PASSAGE', item_id=ITEM)
        self.assertEqual(403, response.status_code)
        self.assertEqual([], self.calls)


if __name__ == '__main__':
    unittest.main()
