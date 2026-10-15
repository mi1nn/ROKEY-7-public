"""리플레이(지난 회차 다시 보기) 화면 모델. 순수 Python 이고 I/O 와 시스템 시계를 쓰지 않는다.

현장 상태 DB 에서 시각과 함께 쌓이는 표(사건·명령·오류·출구)만으로 시간 막대를 만들고, 고른 시각까지의
기록을 돌려준다. 첫 판은 DB 파일 하나를 회차 하나로 본다(작전 2026-10-15). 회차 구분(run_id)과
로봇 위치·인형 단계·지도의 이력 표는 operations 가 만든 뒤 붙인다. 그 전까지 그 칸은 「이력 없음」으로
드러내고 최신 값으로 채우지 않는다(최신 값은 과거 시점의 값이 아니다).
"""

from .view import COMMANDS, ConsoleError, _require, _section, _site_events

# 배속 버튼([제안] 관제·화면, 2026-10-15). 재생 시계는 브라우저가 돌리고 서버는 시점 상태만 답한다.
REPLAY_SPEEDS = (1, 2, 4, 8)
# operations 읽기 API 의 한 번 조회 상한(rescue_operations.ledger._limit). 이만큼 읽히면 잘렸다고 본다.
REPLAY_READ_LIMIT = 1000
# 시점 상태에 싣는 최근 기록 줄 수([제안] 관제·화면, 2026-10-15). 시간 막대의 점은 전부 싣는다.
REPLAY_ROWS = 50
# 아직 시간대별 기록이 없는 칸. operations 의 이력 표가 생기면 하나씩 뺀다.
NO_HISTORY = (('robots', '로봇 위치·배터리'), ('targets', '인형 구조 단계'), ('map', '지도'))
# 읽는 섹션: (status 키, 항목 키). sources.load_replay 가 이 순서로 채운다.
REPLAY_SECTIONS = (('site_events', 'site_events'), ('operator_commands', 'commands'),
                   ('error_log', 'errors'), ('exit_locations', 'exits'))


def _items(status, words):
    """네 섹션을 한 줄 모양으로 합친다. 빠진 섹션은 missing, 상한까지 찬 섹션은 truncated 에 이름을 남긴다."""
    items, missing, truncated = [], [], []
    for key, items_key in REPLAY_SECTIONS:
        section = _section(status, key, items_key)
        if section is None:
            missing.append(key)
            continue
        if len(section[items_key]) >= REPLAY_READ_LIMIT:
            truncated.append(key)
    events = _section(status, 'site_events', 'site_events')
    for event in [] if events is None else _site_events(events['site_events']):
        items.append({'at_ns': event.observed_at_ns, 'kind': 'EVENT', 'what': event.kind, 'label': event.kind,
                      'ref_id': event.event_id, 'subject': event.target_id, 'source': event.camera_id,
                      'accepted': None, 'reason': None, 'detail': event.detail or None})
    commands = _section(status, 'operator_commands', 'commands')
    for row in [] if commands is None else commands['commands']:
        items.append({'at_ns': row['requested_at_ns'], 'kind': 'COMMAND', 'what': row['kind'],
                      'label': words.get(row['kind'], row['kind']), 'ref_id': row['command_id'],
                      'subject': row['target_id'], 'source': row['operator_id'], 'accepted': row['accepted'],
                      'reason': row['reason'], 'detail': row['result']})
    errors = _section(status, 'error_log', 'errors')
    for row in [] if errors is None else errors['errors']:
        items.append({'at_ns': row['at_ns'], 'kind': 'ERROR', 'what': row['code'], 'label': row['code'],
                      'ref_id': str(row['seq']), 'subject': row['task_id'], 'source': row['robot_id'],
                      'accepted': None, 'reason': None, 'detail': row['detail']})
    exits = _section(status, 'exit_locations', 'exits')
    for row in [] if exits is None else exits['exits']:
        items.append({'at_ns': row['observed_at_ns'], 'kind': 'EXIT', 'what': row['class_label'], 'label': '출구 관측',
                      'ref_id': row['location_id'], 'subject': None, 'source': None, 'accepted': None,
                      'reason': None, 'detail': f"({row['x']:.2f}, {row['y']:.2f}) {row['mission_frame']}",
                      'x': row['x'], 'y': row['y'], 'frame_id': row['mission_frame'],
                      'map_revision': row['map_revision']})
    _require(all(type(item['at_ns']) is int for item in items), 'replay row time must be int ns')
    # 같은 시각은 종류와 ref_id 로 순서를 고정한다(같은 입력이면 같은 막대).
    items.sort(key=lambda item: (item['at_ns'], item['kind'], item['ref_id']))
    return items, missing, truncated


def build_timeline(status, *, words=None):
    """시간 막대 모델: 처음·끝 시각, 오래된 것부터 정렬한 전체 점, 빠지거나 잘린 섹션.

    기록이 하나도 없으면 start_ns·end_ns 가 None 이다(빈 회차를 0 시각으로 그리지 않는다).
    """
    items, missing, truncated = _items(status, dict(COMMANDS) if words is None else words)
    return {'start_ns': items[0]['at_ns'] if items else None, 'end_ns': items[-1]['at_ns'] if items else None,
            'items': items, 'missing': missing, 'truncated': truncated, 'speeds': REPLAY_SPEEDS,
            'no_history': [{'key': key, 'label': text} for key, text in NO_HISTORY]}


def replay_at(timeline, t_ns):
    """고른 시각 ``t_ns`` 까지의 상태. 범위 밖이면 가장자리로 당기지 않고 INVALID_INPUT 으로 끝낸다."""
    _require(type(t_ns) is int, 't must be int ns')
    start, end = timeline['start_ns'], timeline['end_ns']
    if start is None:
        raise ConsoleError('INVALID_INPUT', 'replay has no records')
    _require(start <= t_ns <= end, f't must be within {start}..{end}')
    seen = [item for item in timeline['items'] if item['at_ns'] <= t_ns]
    exits = [item for item in seen if item['kind'] == 'EXIT']
    passes = {item['ref_id'] for item in seen if item['kind'] == 'EVENT' and item['what'] == 'EXIT_PASS'}
    return {'t_ns': t_ns, 'offset_ns': t_ns - start, 'rows': seen[::-1][:REPLAY_ROWS], 'seen_count': len(seen),
            'exit': exits[-1] if exits else None, 'exit_pass_count': len(passes)}
