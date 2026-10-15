"""회차 초기화 화면 모델과 요청. 순수 Python 이고 화면 모델 쪽은 I/O 와 시스템 시계를 쓰지 않는다.

초기화 범위는 (나) TB-6 SLAM 지도와 지도 고정까지 처음으로 되돌리기, 지난 기록은 회차별 보관이다(재범 결정
2026-10-15). 웹은 operations 의 ``POST /api/runs/reset`` 에 요청을 한 번 보내고 답을 그대로 보인다(#323). 다시 보내지
않는다. 로봇 쪽은 사람이 PC 별 순서로 다시 띄우고, 이 화면이 그 순서를 체크 목록으로 보인다. 자동 칸은
operations 의 ``read_reset_checklist`` 로, 사람 칸은 관제 웹 계정 DB 의 체크(새 회차 run_id 에 묶음)로 채운다.
"""

import json

from .commands import COMMAND_TIMEOUT_S, CommandSendError, post_json, read_reply
from .labels import label
from .view import ConsoleError

RESET_PATH = '/api/runs/reset'
# operations RunReset 의 거절 순서와 같다(command_api.RunReset.refusal, #323). 화면의 미리 보기용 거울이다.
# 서버가 같은 검사를 다시 하고, 보낸 뒤에는 서버의 답만 믿는다.
RESET_STALE_NS = 5_000_000_000
REFUSAL_HINT = '초기화는 두 로봇이 켜지고 도킹돼 있어야 됩니다(정지, 임무 없음).'
# PC 별 다시 띄우기 순서: docs/implementation/implementation-operations.md 「초기화」 표와 같다(작전 2026-10-15).
# 칸 종류는 셋이다. reply 는 초기화 답, coordinator·robot 은 read_reset_checklist 자동 확인, person 은 사람 확인.
# robot 칸은 표시 이름(--robot-name)이 TB-6·TB-5 인 로봇의 실행기 boot 가 바뀌었는지 본다.
RESET_STEPS = (
    (1, '운영 PC', '두 로봇이 도킹·정지·임무 없음일 때 이 화면에서 초기화를 보낸다', (('reply', 'sent', '초기화 접수, 새 회차 열림'),)),
    (2, '운영 PC', '조정기 launch 를 끈다. 운영 launch(db_writer·report_node)와 관제 웹은 그대로 둔다',
     (('person', 'coordinator_down', '조정기 끔'),)),
    (3, '로봇 담당 PC', '두 로봇의 launch 를 모두 끈다(SLAM, 지도 고정, 고정 지도 relay·AMCL, 출발 정렬, 실행기, 깊이 점, 터널 판정)',
     (('person', 'robots_down', '두 로봇 launch 모두 끔'),)),
    (4, '운영 PC', '조정기를 다시 띄운다', (('coordinator', 'coordinator_up', '조정기 다시 뜸'),)),
    (5, '로봇 담당 PC', 'TB-6 을 언도킹해 출발 표시에 둔다. 도킹한 채로 지도 고정 노드를 띄우지 않는다. '
     '그다음 SLAM, 실행기, 지도 고정(새 output_dir) 순으로 띄운다',
     (('person', 'tb6_start_mark', 'TB-6 언도킹, 출발 표시에 둠'),
      ('person', 'tb6_new_output_dir', '지도 고정을 새 output_dir 로 띄움'),
      ('robot', 'TB-6', 'TB-6 실행기 다시 뜸'))),
    (6, '로봇 담당 PC', 'TB-5 를 언도킹하고 고정 지도 위치 추정·출발 정렬·실행기를 띄운다',
     (('person', 'tb5_undock_align', 'TB-5 언도킹, 출발 정렬'), ('robot', 'TB-5', 'TB-5 실행기 다시 뜸'))),
)
PERSON_ITEMS = frozenset(key for _n, _pc, _text, items in RESET_STEPS for kind, key, _w in items if kind == 'person')


def robot_precheck(robots, now_ns):
    """두 로봇이 지금 초기화를 받을 수 있는지(서버 거절 순서의 거울). robots 는 read_robot_status 의 행이다.

    (로봇별 줄, 처음 걸린 이유 또는 None). 상태가 없는 배치 로봇은 robots 에 None 으로 넣는다.
    """
    checks = (('ROBOT_STATUS_STALE', lambda s: s is None or now_ns - s['received_at_ns'] > RESET_STALE_NS),
              ('ROBOT_MOVING', lambda s: s['motion_state'] != 'STOPPED'),
              ('TASK_ACTIVE', lambda s: s['active_task_id'] is not None),
              ('ROBOT_NOT_DOCKED', lambda s: s.get('dock_state') != 'DOCKED'))
    rows, first = [], None
    for robot_id, status in robots:
        failing = next((reason for reason, bad in checks if bad(status)), None)
        rows.append({'robot_id': robot_id, 'reason': failing, 'reason_label': label('reply', failing),
                     'motion_state': None if status is None else status['motion_state'],
                     'dock_state': None if status is None else status.get('dock_state'),
                     'age_ns': None if status is None else now_ns - status['received_at_ns']})
    for reason, _bad in checks:
        if any(row['reason'] == reason for row in rows):
            first = reason
            break
    return rows, first


def checklist(server, person_checks, names):
    """체크 목록 모델. ``server`` 는 read_reset_checklist, ``person_checks`` 는 계정 DB 의 그 회차 체크,
    ``names`` 는 robot_id -> 표시 이름. 초기화 전(previous_run_id None)에는 None 이다.

    robot 칸은 표시 이름으로 로봇을 찾는다. 그 이름의 로봇이 없으면 칸을 「모름」으로 두고 체크하지 않는다.
    """
    if server['previous_run_id'] is None:
        return None
    by_name = {names.get(robot_id, robot_id): robot_id for robot_id in server['robots']}
    steps = []
    for number, pc, text, items in RESET_STEPS:
        rows = []
        for kind, key, words in items:
            row = {'kind': kind, 'key': key, 'label': words, 'done': False, 'by_username': None, 'at_ns': None,
                   'unknown': False}
            if kind == 'reply':
                row['done'] = True   # previous_run_id 가 있으면 초기화가 접수돼 새 회차가 열린 것이다
            elif kind == 'coordinator':
                row['done'] = server['coordinator_restarted']
            elif kind == 'robot':
                robot_id = by_name.get(key)
                row['unknown'] = robot_id is None
                row['done'] = robot_id is not None and server['robots'][robot_id]['restarted']
            else:
                check = person_checks.get(key)
                if check is not None:
                    row.update(done=check['checked'], by_username=check['by_username'], at_ns=check['at_ns'])
            rows.append(row)
        steps.append({'number': number, 'pc': pc, 'text': text, 'items': rows})
    done = sum(row['done'] for step in steps for row in step['items'])
    total = sum(len(step['items']) for step in steps)
    return {'run_id': server['run_id'], 'steps': steps, 'done': done, 'total': total}


def build_reset_request(*, operator_id, command_id, note=None):
    """operations 초기화 요청 본문. note 는 비어 있으면 넣지 않는다."""
    body = {'command_id': command_id, 'operator_id': operator_id}
    if note:
        body['note'] = note
    return body


def send_reset(base_url, payload, *, transport=post_json, timeout=COMMAND_TIMEOUT_S):
    """초기화를 한 번 보내고 결과를 돌려준다. 다시 보내지 않는다(운영자 명령과 같은 규칙).

    답은 운영자 명령 모양(commands.read_reply)에 run_id 가 더 붙는다. 못 보냈거나 답을 못 읽으면 sent False 이고,
    delivery 가 UNKNOWN 이면 operations 가 받았을 수 있다(결과 모름).
    """
    head = {'command_id': payload['command_id'], 'kind': 'RESET_RUN'}
    try:
        status, body = transport(base_url + RESET_PATH, payload, timeout)
        reply = read_reply(status, body)
        run_id = json.loads(body).get('run_id')
        if run_id is not None and not (isinstance(run_id, str) and run_id):
            raise CommandSendError('BAD_RESPONSE', f'HTTP {status}: run_id must be a non-empty string')
    except CommandSendError as exc:
        return {'sent': False, **head, 'error': exc.code, 'message': exc.message, 'delivery': exc.delivery}
    return {'sent': True, **head, **reply, 'run_id': run_id, 'reason_label': label('reply', reply['reason'])}


def reset_notice(result):
    """보낸 결과 -> (분류, 문구). 분류는 success·danger·warning(결과 모름)."""
    if not result['sent']:
        if result['delivery'] == 'NOT_SENT':
            return 'danger', f'초기화를 보내지 못함({result["error"]}): {result["message"]}'
        return 'warning', (f'초기화 결과 모름({result["error"]}): {result["message"]} — 다시 누르기 전에 '
                           '리플레이에서 회차가 바뀌었는지 확인하세요')
    if result['accepted']:
        return 'success', '초기화 접수 — 새 회차가 열렸습니다. 아래 순서대로 다시 띄우세요'
    word = result['reason_label'] or result['reason']
    return 'danger', f'초기화 거절: {word} ({result["reason"]}). {REFUSAL_HINT}'


def check_item(item):
    """사람 확인 칸 이름 검사. 모르는 칸은 INVALID_INPUT."""
    if item not in PERSON_ITEMS:
        raise ConsoleError('INVALID_INPUT', 'not a reset checklist item')
    return item
