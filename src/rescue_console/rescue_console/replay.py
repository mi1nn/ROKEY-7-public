"""리플레이(지난 회차 다시 보기) 화면 모델. 순수 Python 이고 I/O 와 시스템 시계를 쓰지 않는다.

현장 상태 DB 스키마 1.5(#318)의 회차(run) 하나를 시각대로 다시 그린다. 쌓이는 표 넷(사건·명령·오류·출구)과
이력 표 둘(로봇 상태, 대상 상태)을 operations 의 회차 범위 읽기로 받는다. 고른 시각 t 의 로봇·인형 값은
t 이전의 가장 최근 이력 행이다(지금 값을 과거 시점에 그리지 않는다). 지도 그림은 operations 가
회차별로 남기기 전이라 「이력 없음」으로 둔다.
"""

import threading

from .view import COMMANDS, STAGES, ConsoleError, _require, _site_events, compose_stage

# 배속 버튼([제안] 관제·화면, 2026-10-15). 재생 시계는 브라우저가 돌리고 서버는 시점 상태만 답한다.
REPLAY_SPEEDS = (1, 2, 4, 8)
# 시점 상태에 싣는 최근 기록 줄 수([제안] 관제·화면, 2026-10-15). 시간 막대의 점은 전부 싣는다.
REPLAY_ROWS = 50
# 로봇 이력 행이 고른 시각보다 이만큼 넘게 앞서면 그 값을 낡음으로 표시한다. 관제 화면의 STATUS_STALE_S 와
# 같은 뜻([제안] 관제·화면, 2026-10-15). operations 는 로봇당 1초에 한 줄 남긴다(ROBOT_HISTORY_PERIOD_NS).
REPLAY_STALE_NS = 5_000_000_000
# 아직 시간대별 기록이 없는 칸. operations 가 지도를 회차별로 남기면 뺀다.
NO_HISTORY = (('map', '지도'),)
# 읽는 표: (모델 키, 범위 읽기 결과의 항목 키). sources.load_replay 가 이 키로 채운다.
REPLAY_SECTIONS = (('site_events', 'site_events'), ('operator_commands', 'commands'), ('error_log', 'errors'),
                   ('exit_locations', 'exits'), ('robot_history', 'history'), ('target_history', 'history'))


def _section(data, key, items_key):
    """범위 읽기 결과 하나. 빠지면 None. 모양이 다르면 그리지 않고 INVALID_INPUT."""
    section = data.get(key)
    if section is None:
        return None
    _require(isinstance(section, dict) and isinstance(section.get(items_key), list)
             and type(section.get('truncated')) is bool, f'{key} must be a range read result')
    return section


def _stage(state):
    stage, _rule, _warnings = compose_stage(state['detection_confirmed'], state['rescue_state'],
                                            state['support_state'], None)
    return stage


def _items(data, words):
    """사건·명령·오류·출구·인형 단계 바뀜을 한 줄 모양으로 합친다. 로봇 이력은 점으로 찍지 않는다(초당 한 줄)."""
    items = []
    events = _section(data, 'site_events', 'site_events')
    for event in [] if events is None else _site_events(events['site_events']):
        items.append({'at_ns': event.observed_at_ns, 'kind': 'EVENT', 'what': event.kind, 'label': event.kind,
                      'ref_id': event.event_id, 'subject': event.target_id, 'source': event.camera_id,
                      'accepted': None, 'reason': None, 'detail': event.detail or None})
    commands = _section(data, 'operator_commands', 'commands')
    for row in [] if commands is None else commands['commands']:
        items.append({'at_ns': row['requested_at_ns'], 'kind': 'COMMAND', 'what': row['kind'],
                      'label': words.get(row['kind'], row['kind']), 'ref_id': row['command_id'],
                      'subject': row['target_id'], 'source': row['operator_id'], 'accepted': row['accepted'],
                      'reason': row['reason'], 'detail': row['result']})
    errors = _section(data, 'error_log', 'errors')
    for row in [] if errors is None else errors['errors']:
        items.append({'at_ns': row['at_ns'], 'kind': 'ERROR', 'what': row['code'], 'label': row['code'],
                      'ref_id': str(row['seq']), 'subject': row['task_id'], 'source': row['robot_id'],
                      'accepted': None, 'reason': None, 'detail': row['detail']})
    exits = _section(data, 'exit_locations', 'exits')
    for row in [] if exits is None else exits['exits']:
        items.append({'at_ns': row['observed_at_ns'], 'kind': 'EXIT', 'what': row['class_label'], 'label': '출구 관측',
                      'ref_id': row['location_id'], 'subject': None, 'source': None, 'accepted': None,
                      'reason': None, 'detail': f"({row['x']:.2f}, {row['y']:.2f}) {row['mission_frame']}",
                      'x': row['x'], 'y': row['y'], 'frame_id': row['mission_frame'],
                      'map_revision': row['map_revision']})
    # 인형 단계는 바뀐 때만 한 줄(같은 단계로 다시 들어온 상태 행은 점을 늘리지 않는다).
    targets = _section(data, 'target_history', 'history')
    last = {}
    for row in [] if targets is None else targets['history']:
        stage = _stage(row['state'])
        if last.get(row['target_id']) != stage:
            items.append({'at_ns': row['updated_at_ns'], 'kind': 'TARGET', 'what': stage,
                          'label': f'인형 단계 → {STAGES[stage]}', 'ref_id': f"{row['target_id']}@{row['updated_at_ns']}",
                          'subject': row['target_id'], 'source': None, 'accepted': None, 'reason': None,
                          'detail': None})
        last[row['target_id']] = stage
    _require(all(type(item['at_ns']) is int for item in items), 'replay row time must be int ns')
    # 같은 시각은 종류와 ref_id 로 순서를 고정한다(같은 입력이면 같은 막대).
    items.sort(key=lambda item: (item['at_ns'], item['kind'], item['ref_id']))
    return items


def run_choices(runs):
    """회차 고르기 목록(오래된 것부터 번호 1, 2, …)과 처음 고를 회차. 열린 회차가 있으면 그것, 없으면 마지막 회차."""
    rows = [{'number': n, 'run_id': run['run_id'], 'started_at_ns': run['started_at_ns'],
             'ended_at_ns': run['ended_at_ns'], 'open': run['run_id'] == runs['open_run_id'], 'note': run['note']}
            for n, run in enumerate(runs['runs'], 1)]
    default = runs['open_run_id'] or (rows[-1]['run_id'] if rows else None)
    return rows, default


def build_timeline(data, *, run, words=None, robot_names=None):
    """회차 하나의 시간 막대 모델. ``run`` 은 run_choices 의 한 행이다.

    처음은 회차 시작과 첫 기록 중 이른 쪽, 끝은 닫힌 회차의 끝과 마지막 기록 중 늦은 쪽이다. 열린 회차는
    마지막 기록까지다(이 모듈은 시계를 읽지 않는다).
    """
    words = dict(COMMANDS) if words is None else words
    names = dict(robot_names or {})
    items = _items(data, words)
    robots = _section(data, 'robot_history', 'history')
    targets = _section(data, 'target_history', 'history')
    robot_rows = [] if robots is None else robots['history']
    target_rows = [] if targets is None else targets['history']
    times = [item['at_ns'] for item in items] + [row['at_ns'] for row in robot_rows] + [run['started_at_ns']]
    if run['ended_at_ns'] is not None:
        times.append(run['ended_at_ns'])
    missing = [key for key, items_key in REPLAY_SECTIONS if _section(data, key, items_key) is None]
    truncated = [key for key, items_key in REPLAY_SECTIONS
                 if key not in missing and data[key]['truncated']]
    return {'run': run, 'start_ns': min(times), 'end_ns': max(times), 'items': items,
            'robot_rows': robot_rows, 'target_rows': target_rows, 'names': names,
            'missing': missing, 'truncated': truncated, 'speeds': REPLAY_SPEEDS,
            'no_history': [{'key': key, 'label': text} for key, text in NO_HISTORY]}


def _latest(rows, time_key, id_key, t_ns):
    """id 별로 t 이전의 가장 최근 행(rows 는 오래된 것부터)."""
    latest = {}
    for row in rows:
        if row[time_key] > t_ns:
            break
        latest[row[id_key]] = row
    return latest


def replay_at(timeline, t_ns):
    """고른 시각 ``t_ns`` 까지의 상태. 범위 밖이면 가장자리로 당기지 않고 INVALID_INPUT 으로 끝낸다."""
    _require(type(t_ns) is int, 't must be int ns')
    start, end = timeline['start_ns'], timeline['end_ns']
    if not start <= t_ns <= end:
        raise ConsoleError('INVALID_INPUT', f't must be within {start}..{end}')
    seen = [item for item in timeline['items'] if item['at_ns'] <= t_ns]
    exits = [item for item in seen if item['kind'] == 'EXIT']
    passes = {item['ref_id'] for item in seen if item['kind'] == 'EVENT' and item['what'] == 'EXIT_PASS'}
    robots = []
    for robot_id, row in sorted(_latest(timeline['robot_rows'], 'at_ns', 'robot_id', t_ns).items()):
        status = row['status']
        robots.append({'robot_id': robot_id, 'name': timeline['names'].get(robot_id, robot_id), 'at_ns': row['at_ns'],
                       'age_ns': t_ns - row['at_ns'], 'stale': t_ns - row['at_ns'] > REPLAY_STALE_NS,
                       'pose': status.get('pose'), 'role': status.get('role'),
                       'motion_state': status.get('motion_state'), 'readiness': status.get('readiness'),
                       'battery_percent': status.get('battery_percent')})
    targets = []
    for target_id, row in sorted(_latest(timeline['target_rows'], 'updated_at_ns', 'target_id', t_ns).items()):
        stage = _stage(row['state'])
        targets.append({'target_id': target_id, 'stage': stage, 'stage_label': STAGES[stage],
                        'position': row['state'].get('position'), 'at_ns': row['updated_at_ns']})
    return {'t_ns': t_ns, 'offset_ns': t_ns - start, 'rows': seen[::-1][:REPLAY_ROWS], 'seen_count': len(seen),
            'exit': exits[-1] if exits else None, 'exit_pass_count': len(passes), 'robots': robots,
            'targets': targets}


class ClosedRunCache:
    """닫힌 회차의 시간 막대를 ``size`` 개까지 기억한다. 넘치면 가장 먼저 넣은 것을 버린다.

    Flask 는 요청을 스레드로 처리한다. 잠금이 없으면 두 요청이 같은 가장 오래된 회차를 동시에 버리려다
    KeyError 가 나거나, 한쪽이 넣는 동안 다른 쪽이 돌다 RuntimeError 가 난다(#322 검토 지적).
    ``lock`` 은 시험에서만 바꾼다.
    """

    def __init__(self, size, lock=None):
        self.size = size
        self._lock = threading.Lock() if lock is None else lock
        self._runs = {}

    def get(self, run_id):
        with self._lock:
            return self._runs.get(run_id)

    def put(self, run_id, timeline):
        with self._lock:
            if run_id not in self._runs and len(self._runs) >= self.size:
                oldest = next(iter(self._runs))
                self._between()
                del self._runs[oldest]
            self._runs[run_id] = timeline

    def _between(self):
        """가장 오래된 회차를 고른 뒤 지우기 전. 시험이 여기서 틈을 벌려 경합을 재현한다."""
