"""장부 + 상태 스냅숏 -> 화면 모델 하나. 순수 Python 이다. 입출력도 시계 읽기도 없다.

호출하는 쪽이 `built_at_ns` 를 넘기므로 같은 입력은 늘 같은 모델을 낸다.
없는 값은 None 으로 남고 화면에도 없음으로 보인다. 아무것도 채워 넣지 않는다.
"""

import re

from rescue_contracts import RESCUE_STATES, RescueError, SiteEvent

from .labels import label

# Q-20 (scope.md, 2026-10-08): Part I 3.7.2 의 대상 4단계를 여기, 표시 계층에서 조합한다.
# 계약 어휘(RESCUE_STATES, confirmation_result)는 늘리지 않는다.
STAGES = {
    'UNCONFIRMED': '미확정',
    'NOT_RESCUED': '미구조',
    'SUPPORTING': '지원 중',
    'RESCUED': '구출 완료',
}
# operations 읽기 API 가 생길 때까지 쓰는 임시 지원 상태 어휘다([제안] 관제·화면, 2026-10-08).
SUPPORT_STATES = frozenset({'NONE', 'ASSIGNED', 'ARRIVED', 'FAILED'})
ACTIVE_SUPPORT = frozenset({'ASSIGNED', 'ARRIVED'})

# 규칙 표. 위에서부터 처음 맞는 줄이 이긴다. 각 줄: (규칙 ID, 조건 글, 단계).
# 조건 글은 README 에 보이는 글이고, 아래 `compose_stage` 가 실행되는 형태다.
STAGE_RULES = (
    ('R1', 'detection_confirmed is false', 'UNCONFIRMED'),
    ('R2', 'rescue_state == RESCUE_CONFIRMED', 'RESCUED'),
    ('R3', 'support_state in {ASSIGNED, ARRIVED}', 'SUPPORTING'),
    ('R4', 'support_state unknown and approach_result == APPROACH_COMPLETE', 'SUPPORTING'),
    ('R5', 'otherwise (CONFIRMED, not rescued, no active support)', 'NOT_RESCUED'),
)

# 운영자 명령: console-and-db-plan 4절의 `POST /api/operator-commands` 종류다
# (Part I 3.7.6 에 임무 흐름의 사람 관문을 더한 것). 앞 다섯은 operations 로 보내고
# (rescue_console.commands), 나머지는 상태와 무관하게 여기서 꺼 둔다.
COMMANDS = (
    ('START_SEARCH', '수색 시작'),
    ('DISPATCH_SUPPORT', 'SUPPORT 출동 승인'),   # 상태가 SUPPORT 로봇을 알려 주면 그 로봇 이름으로 바뀐다(_command_words)
    ('CONTINUE_SEGMENT', '다음 구간 계속'),
    ('REQUEST_RECHECK', '재확인 지시'),
    ('RETURN_HOME', '복귀'),
    ('PAUSE', '일시정지'),
    ('RESUME', '재개'),
    ('CANCEL', '작업 취소'),
    ('STOP_ROBOT', '정지'),
)
# 버튼 판 묶음: 흐름의 사람 관문(#85 시작, 지원 출동, 구간마다 다음 구간)을 순서대로 먼저 두고,
# 정지는 나머지 모두와 떼어 둔다.
COMMAND_GROUPS = {
    'START_SEARCH': 'approve', 'DISPATCH_SUPPORT': 'approve', 'CONTINUE_SEGMENT': 'approve',
    'REQUEST_RECHECK': 'mission', 'RETURN_HOME': 'mission', 'PAUSE': 'mission', 'RESUME': 'mission',
    'CANCEL': 'mission',
    'STOP_ROBOT': 'stop',
}
# 콘솔이 보내지 않는 명령과 그 이유(작전 2026-10-12, #115).
HELD_COMMANDS = {
    'PAUSE': 'Q-22 재개 관문 전',
    'RESUME': 'Q-22 재개 관문 전',
    'CANCEL': 'Q-22 재개 관문 전',
    'STOP_ROBOT': 'RequestStop 별도 경로 — 위험하면 현장 비상 정지 장치',
}
# 임무 판정 뒤, 복귀 전의 phase(rescue_coordination.mission: RETURN_HOME 은 판정이 있어야 한다).
VERDICT_PHASES = frozenset({'RESCUE_CONFIRMED', 'RECHECK_NEEDED'})

# 사건 표의 행 상한. 더 오래된 행은 장부에 남는다([제안] 관제·화면, 2026-10-08).
EVENT_ROWS = 50
# 명령·오류 기록의 행 상한. 더 오래된 행은 현장 상태 DB 에 남는다([제안] 작전, 2026-10-12).
HISTORY_ROWS = 30

# 마지막 상태가 operations 에 닿은 지 이보다 오래된 로봇 행은 낡음으로 표시한다
# ([제안] 관제·화면, 2026-10-10). 표시용일 뿐 준비 판정(C-05)이 아니다. 프로파일 v2.1(#39)이
# main 에 들어가면 프로파일의 readiness max_age_s 로 바꾼다.
STATUS_STALE_S = 5.0

# 임무 배지 낱말: site_state MISSION_MODES(계약 S-00 모드 + SystemState MISSION_FINISHED).
MODES = {'NORMAL': '평시', 'DISASTER': '재난', 'CLEARING': '해제 중', 'MISSION_FINISHED': '임무 종료'}
DISPATCH_KEYS = ('target_id', 'target_revision', 'route_revision', 'segment_index', 'segment_count', 'segment_phase')


class ConsoleError(ValueError):
    """콘솔 입력·설정 실패를 고정된 코드와 함께 나타낸다.

    콘솔 안에서만 쓰고 RescueError 가 아니다. 계약 코드는 로봇·판정·보고 경로에만 쓰도록
    남겨 둔다(작전 결정 2026-10-08, README 「하지 않는 것」).
    """

    def __init__(self, code, message=''):
        self.detail_code, self.detail = code, message
        super().__init__(f'{code}: {message}')


def _require(ok, message, code='INVALID_INPUT'):
    if not ok:
        raise ConsoleError(code, message)


def compose_stage(detection_confirmed, rescue_state, support_state, approach_result):
    """대상 하나의 (단계, 규칙 ID, 경고)를 돌려준다. None 은 「보고 안 됨」이다."""
    # 엄격한 bool: 없는 값은 여기서 실패해야 한다. 「확정 안 됨」으로 읽혀 R1 에 떨어지면 안 된다.
    _require(type(detection_confirmed) is bool, 'detection_confirmed must be bool')
    _require(rescue_state is None or rescue_state in RESCUE_STATES, 'rescue_state outside RESCUE_STATES')
    _require(support_state is None or support_state in SUPPORT_STATES, 'support_state')
    warnings = []
    # 아래 분기는 STAGE_RULES R1..R5 를 순서대로 옮긴 것이다. 처음 맞는 분기가 돌려준다.
    if not detection_confirmed:
        # 서로 어긋나는 입력은 R1 에 두고 경고를 붙인다. RESCUED 나 SUPPORTING 으로 올리지 않는다.
        if rescue_state == 'RESCUE_CONFIRMED' or support_state in ACTIVE_SUPPORT:
            warnings.append('미확정 대상에 구출/지원 상태가 있음 — 입력 불일치')
        return 'UNCONFIRMED', 'R1', warnings
    if rescue_state == 'RESCUE_CONFIRMED':
        return 'RESCUED', 'R2', warnings
    if support_state in ACTIVE_SUPPORT:
        return 'SUPPORTING', 'R3', warnings
    # R4 는 지원 상태를 모를 때(None)만 쓴다. 보고된 NONE·FAILED 가 보고의 APPROACH_COMPLETE 보다 앞선다.
    if support_state is None and approach_result == 'APPROACH_COMPLETE':
        return 'SUPPORTING', 'R4', warnings
    return 'NOT_RESCUED', 'R5', warnings


def _section(status, key, items_key):
    """읽기 API 결과 ``status[key]`` 를 돌려준다. 키가 없으면(입력이 아직 없음) None 이다.

    있는 절은 문자열 ``schema_version`` 과 ``items_key`` 아래 목록을 가진 dict 여야 한다.
    그 밖이면 반쯤 읽은 채 그리지 않고 ``ConsoleError INVALID_INPUT`` 을 올린다.
    """
    section = status.get(key)
    if section is None:
        return None
    _require(isinstance(section, dict) and isinstance(section.get('schema_version'), str)
             and isinstance(section.get(items_key), list), f'{key} must be a read API result')
    return section


def _latest_reports(reports):
    """target_id -> (그 대상의 가장 새 장부 항목, 그 대상의 보고 수).

    가장 새 것은 (reported_at_ns, received_at_ns, report_id) 가 가장 큰 것이다. 보고 시각이 정하고,
    장부 수신 시각과 보고 ID 는 같을 때만 가르므로 시각이 같아도 답은 하나로 정해진다.
    """
    latest, counts = {}, {}
    for item in reports:
        payload = item['payload']
        target = payload['target_id']
        counts[target] = counts.get(target, 0) + 1
        key = (payload['reported_at_ns'], item['received_at_ns'], payload['report_id'])
        if target not in latest or key > latest[target][0]:
            latest[target] = (key, item)
    return {target: (entry[1], counts[target]) for target, entry in latest.items()}


def _position(position, source):
    """위치의 사본에 출처(``source``: 'target_state' 또는 'report')를 붙인다. None 은 None 으로 둔다."""
    if position is None:
        return None
    return {'x': position['x'], 'y': position['y'], 'frame_id': position['frame_id'],
            'map_revision': position['map_revision'], 'source': source}


def _latest_candidates(site_events):
    """target_id -> 그 대상을 가리키는 가장 새 TARGET_DISAPPEARED_CANDIDATE(후보일 뿐이고 로봇이 재확인한다)."""
    latest = {}
    for event in site_events:
        if event.kind != 'TARGET_DISAPPEARED_CANDIDATE':
            continue
        key = (event.observed_at_ns, event.event_id)
        if event.target_id not in latest or key > latest[event.target_id][0]:
            latest[event.target_id] = (key, {'event_id': event.event_id, 'observed_at_ns': event.observed_at_ns,
                                             'camera_id': event.camera_id})
    return {target: entry[1] for target, entry in latest.items()}


def _targets(ledger, target_states, candidates):
    """장부 보고나 대상 상태가 가리키는 대상마다 한 행을 만들고 target_id 로 정렬한다.

    ``ledger`` 가 None 이면 보고 장부가 주어지지 않은 것이다. 보고 칸은 None 으로 둔다(0 도 「보고 없음」도 아님).
    """
    reports = {} if ledger is None else _latest_reports(ledger['reports'])
    rows = []
    for target_id in sorted(set(reports) | set(target_states)):
        item, report_count = reports.get(target_id, (None, 0))
        payload = None if item is None else item['payload']
        state = target_states.get(target_id)
        warnings = []
        report_confirmed = None if payload is None else payload['confirmation_result'] == 'CONFIRMED'
        # 대상 상태(operations)가 이긴다. 최신 보고는 상태가 없을 때만 대신한다.
        # 어느 쪽이든 어긋남이나 빠진 입력은 행 경고가 된다. 말없이 하나를 고르지 않는다.
        if state is not None:
            confirmed = state['detection_confirmed']
            if report_confirmed is not None and report_confirmed != confirmed:
                warnings.append('장부 보고와 대상 상태의 확정 여부가 다름 — 대상 상태를 따름')
            position = _position(state['position'], 'target_state')
        else:
            confirmed = report_confirmed
            position = _position(payload['position'], 'report')
            warnings.append('구조 상태 입력 없음')
        rescue_state = None if state is None else state['rescue_state']
        # 운영자가 승인하는 revision(SubmitOperatorCommand expected_target_revision)이다. operations 가
        # 저장하면 대상 상태에 실려 온다. 그 전에는 None 이고 출동 버튼은 꺼진 채로 있다.
        revision = None if state is None else state.get('target_revision')
        _require(revision is None or (type(revision) is int and revision >= 0), 'target_revision')
        # 조정기가 그 revision 에 대해 PERSISTED 영수증을 가진 보고다(TargetEstimate
        # persisted_report_id, #118 선택 키). 없으면 조정기가 출동을 거절한다. 키가 생기기 전에 쓴 행도
        # None 으로 읽힌다. 아래의 장부 영수증으로 짐작하지 않는다.
        persisted = None if state is None else state.get('persisted_report_id')
        _require(persisted is None or (type(persisted) is str and persisted != ''), 'persisted_report_id')
        support_state = None if state is None else state['support_state']
        approach = None if payload is None else payload['approach_result']
        stage, rule, rule_warnings = compose_stage(confirmed, rescue_state, support_state, approach)
        if payload is None and ledger is not None:
            warnings.append('장부에 보고가 없는 대상')
        rows.append({
            'target_id': target_id,
            'target_revision': revision,
            'persisted_report_id': persisted,
            'stage': stage,
            'stage_label': STAGES[stage],
            'rule': rule,
            'position': position,
            'detection_confirmed': confirmed,
            'approach_result': approach,
            'rescue_state': rescue_state,
            'rescue_reasons': [] if state is None else list(state.get('rescue_reasons', [])),
            'support_state': support_state,
            'support_robot_id': None if state is None else state.get('support_robot_id'),
            'report_id': None if payload is None else payload['report_id'],
            'report_count': None if ledger is None else report_count,
            'receipt_status': None if item is None else item['receipt']['status'],
            'observed_at_ns': None if payload is None else payload['observed_at_ns'],
            'disappeared_candidate': candidates.get(target_id),
            'warnings': rule_warnings + warnings,
        })
    return rows


def _robots(allowed_robot_ids, robot_status, built_at_ns, warnings):
    """배치 로봇마다 ``allowed_robot_ids`` 순서로 한 행을 만든다. 경과 시간은 ``built_at_ns`` 에서 잰다.

    배치 밖 로봇의 상태는 ``warnings`` 에 문구를 더하고 버린다. 한 로봇의 상태가 둘이면
    ``ConsoleError INVALID_INPUT`` 을 올린다. 상태가 없는 배치 로봇도 행을 받으므로
    (``has_status`` False, 나머지 칸 None, ``stale`` True) 말없는 로봇이 가려지지 않는다.
    """
    by_id = {}
    for status in robot_status:
        rid = status.get('robot_id')
        if rid not in allowed_robot_ids:
            warnings.append(f'배치 밖 로봇 상태 무시: {rid!r}')
            continue
        _require(rid not in by_id, f'duplicate robot status for {rid}')
        by_id[rid] = status
    rows = []
    for rid in allowed_robot_ids:
        status = by_id.get(rid)
        if status is None:
            rows.append({'robot_id': rid, 'has_status': False, 'role': None, 'pose': None,
                         'motion_state': None, 'readiness': None, 'battery_percent': None,
                         'battery_age_s': None, 'active_task_id': None, 'stamp_ns': None, 'age_s': None,
                         'received_age_s': None, 'stale': True})
            continue
        stamp, battery_stamp = status.get('stamp_ns'), status.get('battery_stamp_ns')
        received = status.get('received_at_ns')
        received_age = None if received is None else (built_at_ns - received) / 1e9
        rows.append({
            'robot_id': rid,
            'has_status': True,
            'role': status.get('role'),
            'pose': status.get('pose'),
            'motion_state': status.get('motion_state'),
            'readiness': status.get('readiness'),
            'battery_percent': status.get('battery_percent'),
            # 경과 시간은 로봇 시각을 이 호스트 시계와 비교한다. 두 시계가 맞을 때만 뜻이 있다(C-03).
            'battery_age_s': None if battery_stamp is None else (built_at_ns - battery_stamp) / 1e9,
            'active_task_id': status.get('active_task_id'),
            'stamp_ns': stamp,
            'age_s': None if stamp is None else (built_at_ns - stamp) / 1e9,
            # 장부 시계 기준 경과 시간: 재부팅한 로봇의 시계가 뒤로 가도 여전히 뜻이 있다.
            'received_age_s': received_age,
            # 낡음은 로봇 시각이 아니라 수신 경과 시간으로 판정한다. 잴 수 없는 경과 시간은 낡음이다.
            'stale': received_age is None or received_age > STATUS_STALE_S,
        })
    return rows


def _site_events(raw_events):
    """날것의 현장 사건 dict 를 계약 ``SiteEvent`` 로 다시 만들어 계약의 검사가 여기서 돌게 한다.

    계약이 거절하는 행(모르는 칸·빠진 칸: TypeError, 잘못된 값: RescueError)은 화면 전체에 대해
    ``ConsoleError INVALID_INPUT`` 을 올린다. 건너뛰지 않는다.
    """
    events = []
    for raw in raw_events:
        try:
            events.append(SiteEvent(**raw))
        except (TypeError, RescueError) as exc:
            raise ConsoleError('INVALID_INPUT', f'site event: {exc}') from exc
    return events


def _events(ledger, site_events):
    """사건 표: 장부 보고와 현장 사건을 한 행 모양으로 담고 최신 순으로 둔다.

    (앞 EVENT_ROWS 행, 전체 행 수)를 돌려준다. 종류에 없는 칸은 None 으로 둔다. 현장 사건은 읽기 함수가
    ``received_at_ns`` 를 돌려주지 않으므로 그 값이 없다. ``event_ns`` 는 KPI-04 표시 지연을
    재는 기준인 관측 시각이다.
    """
    rows = []
    for item in [] if ledger is None else ledger['reports']:
        payload = item['payload']
        rows.append({
            'event_ns': payload['observed_at_ns'],
            'received_at_ns': item['received_at_ns'],
            'kind': 'REPORT',
            'source': payload['source_robot'],
            'subject': payload['target_id'],
            'detail': f"{payload['confirmation_result']} / {payload['approach_result']} / {item['receipt']['status']}",
            'confirmation': payload['confirmation_result'],
            'approach': payload['approach_result'],
            'receipt': item['receipt']['status'],
            'confidence': None,
            'note': None,
            'clock_domain_id': payload['clock_domain_id'],
            'ref_id': payload['report_id'],
        })
    for event in site_events:
        rows.append({
            'event_ns': event.observed_at_ns,
            'received_at_ns': None,
            'kind': event.kind,
            'source': event.camera_id,
            'subject': event.target_id,
            'detail': f'confidence {event.confidence:.2f}' + (f' / {event.detail}' if event.detail else ''),
            'confirmation': None,
            'approach': None,
            'receipt': None,
            'confidence': event.confidence,
            'note': event.detail or None,
            'clock_domain_id': event.clock_domain_id,
            'ref_id': event.event_id,
        })
    # 시각이 같으면 ref_id 로 가르므로 같은 시각의 행도 폴링마다 같은 순서를 지킨다.
    rows.sort(key=lambda row: (row['event_ns'], row['ref_id']), reverse=True)
    return rows[:EVENT_ROWS], len(rows)


MISSION_ROW_KEYS = frozenset({'mode', 'phase', 'trigger_event_id', 'started_at_ns', 'updated_at_ns', 'reason',
                              *DISPATCH_KEYS})


def _mission_row(status):
    """read_mission_state() 결과를 (절이 있는가, 행 또는 None)으로 돌려준다."""
    section = status.get('mission_state')
    if section is None:
        return False, None
    _require(isinstance(section, dict) and isinstance(section.get('schema_version'), str) and 'mission' in section,
             'mission_state must be a read API result')
    row = section['mission']
    _require(row is None or (isinstance(row, dict) and MISSION_ROW_KEYS <= set(row)), 'mission_state row')
    return True, row


# SystemState.reason 코드를 쉬운 한국어로 옮긴다(2026-10-13). {support}/{scout}: SUPPORT·SCOUT 로봇의
# 표시 이름. level: info(정상 진행), caution(보류), alert(빨강: 실패했고 다시 하지
# 않음). scope: 임무 띠 말고 또 보이는 곳(지원·정찰 카드, 출동한 대상).
MISSION_REASONS = {
    'SUPPORT_EXITING': ('{support} 출구로 이동 중', 'info'),
    'SUPPORT_AT_EXIT': ('{support} 출구 도착', 'info'),
    'SUPPORT_EXIT_CANCELED': ('{support} 출구 이동 취소(판정 종료·재난 해제)', 'info'),
    'SUPPORT_EXIT_FAILED': ('{support} 출구 이동 실패 — 다시 보내지 않음', 'alert'),
    'SUPPORT_POSE_UNKNOWN': ('{support} 위치 모름', 'caution'),
    # 자동 접근(#187): TB-5 를 보내지 않은 이유, 또는 실패했다는 것(스스로 다시 하지 않는다).
    'SUPPORT_POSE_BASIS_MISMATCH': ('{support} 위치의 지도 기준이 다름 — 자동 출동 보류', 'caution'),
    'SUPPORT_AT_TARGET': ('{support} 가 이미 대상 바로 옆 — 접근 지점을 정할 수 없음', 'caution'),
    'SUPPORT_NOT_READY': ('{support} 준비 안 됨 — arm 필요', 'caution'),   # #196: 낡지 않은 READY 상태가 없다
    'APPROACH_FAILED': ('{support} 대상 접근 실패 — 다시 보내지 않음', 'alert'),
    'MODE_NOT_DISASTER': ('재난 모드가 아님 — {support} 출동 안 함', 'info'),
    'ALREADY_DISPATCHED': ('{support} 이미 출동함', 'info'),
    'UNKNOWN_TARGET': ('모르는 대상 — {support} 출동 안 함', 'caution'),
    'TARGET_NOT_CONFIRMED': ('대상 미확정 — {support} 출동 대기', 'caution'),
    'TARGET_REVISION_STALE': ('대상 정보가 바뀜 — {support} 출동 다시 판단', 'caution'),
    'TARGET_ALREADY_JUDGED': ('이미 판정한 대상 — {support} 출동 안 함', 'info'),
    'SCOUT_RETURNING': ('{scout} 도킹 접근 지점으로 이동 중', 'info'),
    'SCOUT_AT_DOCK_APPROACH_DOCK_BY_HAND': ('{scout} 도킹 접근 지점 도착 — 도킹은 사람이 합니다', 'info'),
    'SCOUT_RETURN_FAILED': ('{scout} 도킹 접근 지점 이동 실패 — 다시 보내지 않음', 'alert'),   # #194
    # TargetLocation 시각 관문(#174, OPS-103): 이 경우에는 대상을 만들지 않는다.
    'TARGET_TIME_BASIS_UNUSABLE': ('대상 위치의 관측 시각을 쓸 수 없음(시계 미측정·오차 상한 없음)', 'caution'),
    'TARGET_CLOCK_DOMAIN_MISMATCH': ('대상 위치 관측의 시계 영역이 다름', 'caution'),
    'TARGET_OBSERVATION_FUTURE': ('대상 위치 관측 시각이 미래', 'caution'),
    'TARGET_OBSERVATION_STALE': ('대상 위치 관측이 오래됨', 'caution'),
    # #214 관문(OPS-127/129): 각 관문은 자기가 지키는 이동만 막는다 — TB-5 출동, TB-5 출구 이동, TB-6 복귀.
    # 막힌 지점을 다른 곳으로 옮기지 않는다.
    'MAP_FREEZE_WAITING_SCOUT_STOP': ('{scout} 이 멈추기를 기다림 — 멈추면 지도 고정', 'info'),
    'MAP_FREEZE_REQUESTED': ('지도 고정 요청함 — 고정 지도 확인 중', 'info'),
    'MAP_NOT_FROZEN': ('지도가 아직 고정되지 않음 — {support} 출동 대기', 'caution'),
    'MAP_FREEZE_FAILED': ('지도 고정 실패 — 다시 요청하지 않음', 'alert'),
    'TARGET_NOT_ON_FROZEN_MAP': ('고정 지도 기준 대상 위치가 아직 없음 — {support} 출동 대기', 'caution'),
    'SCOUT_NOT_LOCALIZED': ('{scout} 이 고정 지도에서 위치를 못 잡음 — {support} 출동 대기', 'caution'),
    'SUPPORT_NOT_LOCALIZED': ('{support} 가 고정 지도에서 위치를 못 잡음 — 출동 대기', 'caution'),
    'SUPPORT_APPROACH_NOT_FREE': ('{support} 접근 지점이 막힘 — 출동 보류', 'caution'),
    'SUPPORT_EXIT_NOT_FREE': ('{support} 출구 지점이 막힘 — 출구 이동 보류(대상 옆 대기)', 'caution'),
    'SCOUT_HOME_NOT_FREE': ('{scout} 도킹 접근 지점이 막힘 — {scout} 복귀 보류', 'caution'),
    'NO_HOME_POSITION': ('복귀 위치(출구·도킹 접근) 설정 없음 — 복귀 이동 안 함', 'caution'),
    # search_start AUTO(#228, OPS-140)와 지도 고정 전 출구 QR(OPS-136).
    'SEARCH_GOAL_NOT_SELECTED': ('{scout} 수색 목표 선택 대기', 'caution'),
    'AUTO_SEARCH_ENABLED': ('자동 수색 — 지진 감지로 시작, 수색 시작 버튼은 받지 않음', 'info'),
    'MAP_FREEZE_WAITING_EXIT': ('출구 QR 찾는 중 — {support} 출동 대기', 'info'),
    # #234 (OPS-147): QR 출구는 TB-5 출동과 출구 이동을 막고, 출발 자세는 순찰과 TB-6 복귀를 막는다.
    'TARGET_REBASED_ON_FREEZE': ('대상 위치를 고정 지도로 옮김 — 보고 저장 대기', 'info'),
    'EXIT_NOT_ON_FROZEN_MAP': ('출구(QR) 위치가 고정 지도 기준에 없음 — {support} 출동·출구 이동 보류', 'caution'),
    'SUPPORT_EXIT_UNREGISTERED': ('출구(QR) 미등록 — {support} 출동·출구 이동 보류', 'caution'),
    'START_POSE_UNKNOWN': ('{scout} 출발 자세 모름 — 순찰·{scout} 복귀 보류', 'caution'),
    'START_POSE_CONFLICT': ('{scout} 출발 자세가 다른 값으로 다시 옴 — 처음 값 유지, 확인 필요', 'alert'),
    'PATROL_EXHAUSTED': ('{scout} 순찰 끝(정한 바퀴 다 돎) — 더 돌지 않음', 'caution'),
}
# APPROACH_FAILED_<why> (#216): 실행기의 종료 이유, 그것이 없으면 실행 결과.
# 모르는 이유는 코드 그대로 보인다(OPS-127: 숨기지 않는다).
APPROACH_WHY = {
    'PASSAGE_BLOCKED': '통로 막힘', 'UNREACHABLE': '갈 수 없는 위치', 'FAILED': '실행 실패',
    'CANCELED': '취소됨', 'EXPIRED': '시간 초과', 'NAV_NOT_READY': '주행 준비 안 됨',
    'DEADLINE_EXPIRED': '시간 초과', 'STOP_LATCHED': '정지 유지 중', 'LOCAL_LEASE_EXPIRED': '명령 유효 시간 지남',
    'CLOCK_UNCERTAIN': '시계 불확실',
}
EXIT_COUNTDOWN = re.compile(r'SUPPORT_EXIT_IN_(\d+)S')
PATROL_ROUND = re.compile(r'PATROL_ROUND_(\d+)')
# RECHECK_NEEDED_<why>: 재확인이 구출을 확정하지 못했다(C-11 7-11행).
RECHECK_WHY = {
    'CANDIDATE_NOT_LINKED': '후보 연결 안 됨', 'FRAME_REVISION_MISMATCH': '지도 기준 다름',
    'CANDIDATE_TIME_BASIS_UNUSABLE': '시각 기준 못 씀', 'ORDER_VIOLATION': '시각 순서 틀림',
    'CAMERA_NOT_ABSENT': '인형이 아직 보임', 'LIDAR_NOT_CLEAR': '자리가 막힘', 'COVERAGE_NOT_VALID': '관측 범위 부족',
    'RECHECK_OBSERVATION_MISSING': '재확인 관측 없음 — 「재확인 지시」 버튼으로 다시 보낼 수 있음',
}
SCOPES = (('SUPPORT_', 'support'), ('SCOUT_', 'scout'), ('RECHECK_NEEDED_', 'target'))
# SUPPORT_ 접두어는 없지만 지원 로봇에 속하는 #187 의 접근 코드.
SUPPORT_CODES = frozenset({'APPROACH_FAILED', 'MODE_NOT_DISASTER', 'ALREADY_DISPATCHED', 'UNKNOWN_TARGET',
                           'TARGET_NOT_CONFIRMED', 'TARGET_REVISION_STALE', 'TARGET_ALREADY_JUDGED'})


def mission_reason(code, support_name, scout_name):
    """SystemState.reason 하나의 {code, text, level, scope} 를 만든다. 모르는 코드는 그대로 둔다."""
    if code is None:
        return None
    # 그 역할의 로봇을 알려 주는 상태가 없으면 robot_id 를 모른다. 일반 낱말 말고 역할 코드를 쓴다.
    names = {'support': support_name or 'SUPPORT', 'scout': scout_name or 'SCOUT'}
    support_code = code in SUPPORT_CODES or code.startswith('APPROACH_FAILED_')
    scope = 'support' if support_code else next((where for prefix, where in SCOPES if code.startswith(prefix)),
                                                           None)
    countdown = EXIT_COUNTDOWN.fullmatch(code)
    if countdown:
        text, level = f'{names["support"]} 출구 출발까지 {int(countdown.group(1))}초', 'info'
    elif PATROL_ROUND.fullmatch(code):
        text, level = f'{names["scout"]} 순찰 {int(code[len("PATROL_ROUND_"):])}바퀴째', 'info'
    elif code.startswith('APPROACH_FAILED_'):
        why = code[len('APPROACH_FAILED_'):]
        text, level = f'{names["support"]} 접근 실패: {APPROACH_WHY.get(why, why)} — 다시 보내지 않음', 'alert'
    elif code.startswith('RECHECK_NEEDED_'):
        why = code[len('RECHECK_NEEDED_'):]
        text, level = f'재확인 필요: {RECHECK_WHY.get(why, why)}', 'caution'
    elif code in MISSION_REASONS:
        text, level = MISSION_REASONS[code]
        text = text.format(**names)
    else:
        text, level = code, None
    return {'code': code, 'text': text, 'level': level, 'scope': scope}


def _mission(site_events, row):
    """배지 입력. mission_state 행(operations 가 조정기의 SystemState 를 쓴다, #111)이 있으면
    그 행을 쓴다. 첫 행 전에는 DISASTER_SIGN 사건에서 모드를 추정하고 추정이라고 표시한다
    (`source`). None 은 둘 다 없다는 뜻이다."""
    signs = sorted((e for e in site_events or () if e.kind == 'DISASTER_SIGN'),
                   key=lambda e: (e.observed_at_ns, e.event_id))
    base = {'sign_count': len(signs), 'phase': None, 'reason': None, 'dispatch': None, 'updated_at_ns': None}
    if row is not None:
        dispatch = None if row['target_id'] is None else {key: row[key] for key in DISPATCH_KEYS}
        return base | {'mode': row['mode'], 'label': label('mode', row['mode']), 'source': 'mission_state',
                       'trigger_event_id': row['trigger_event_id'], 'trigger_ns': row['started_at_ns'],
                       'phase': row['phase'], 'reason': row['reason'], 'dispatch': dispatch,
                       'updated_at_ns': row['updated_at_ns']}
    if site_events is None:
        return base | {'mode': None, 'label': '모름', 'source': None, 'trigger_event_id': None, 'trigger_ns': None}
    mode = 'DISASTER' if signs else 'NORMAL'
    return base | {'mode': mode, 'label': MODES[mode], 'source': 'site_events',
                   'trigger_event_id': signs[0].event_id if signs else None,
                   'trigger_ns': signs[0].observed_at_ns if signs else None}


SUPPORT_DISPATCH_MODES = ('AUTO', 'OPERATOR')   # --support-dispatch. 조정기와 같은 현장 값
SEARCH_START_MODES = ('AUTO', 'OPERATOR')       # --search-start. 조정기와 같은 현장 값(#228)


def _command_words(support_name):
    """명령 낱말. 출동 낱말에는 SUPPORT 로봇의 표시 이름을 넣는다(소유자 2026-10-13: TB-5)."""
    words = dict(COMMANDS)
    if support_name is not None:
        words['DISPATCH_SUPPORT'] = f'{support_name} 출동 승인'
    return words


def _history(status, words=None):
    """operations 가 기록한 운영자 명령(답과 사유, 거절 포함)과
    operations 의 error_log 를 최신 순으로 둔다. 두 입력이 다 없으면(상태 픽스처) None 이다.
    사건 행과 따로 둔다. 사건 행은 KPI-04 표시 지연을 싣고 이 행은 싣지 않는다."""
    commands = _section(status, 'operator_commands', 'commands')
    errors = _section(status, 'error_log', 'errors')
    if commands is None and errors is None:
        return None
    words = dict(COMMANDS) if words is None else words
    rows = []
    for item in [] if commands is None else commands['commands']:
        rows.append({'at_ns': item['requested_at_ns'], 'kind': 'COMMAND', 'ref_id': item['command_id'],
                     'what': item['kind'], 'what_label': words.get(item['kind'], item['kind']),
                     'accepted': item['accepted'], 'result': item['result'], 'reason': item['reason'],
                     'target_id': item['target_id'], 'operator_id': item['operator_id'], 'detail': None})
    for item in [] if errors is None else errors['errors']:
        rows.append({'at_ns': item['at_ns'], 'kind': 'ERROR', 'ref_id': str(item['seq']),
                     'what': item['code'], 'what_label': item['code'], 'accepted': None, 'result': None,
                     'reason': None, 'target_id': None, 'operator_id': None, 'detail': item['detail']})
    rows.sort(key=lambda row: (row['at_ns'], row['kind'], row['ref_id']), reverse=True)
    return {'rows': rows[:HISTORY_ROWS],
            'missing': [key for key, section in (('operator_commands', commands), ('error_log', errors))
                        if section is None]}


def _choice(target):
    """선택지 한 항목: 대상, 화면에 보인 revision(DISPATCH_SUPPORT 와 함께 보냄), 그 단계."""
    return {'target_id': target['target_id'], 'target_revision': target['target_revision'],
            'stage': target['stage'], 'stage_label': target['stage_label']}


def _gate(name, mission, targets):
    """명령 하나의 (enabled, reason, args, choices). rescue_coordination.mission.handle_command 의 관문 거울이다."""
    def off(reason):
        return False, reason, {}, []
    # 순서가 중요하다. 처음 걸리는 검사가 버튼 아래 보이는 사유가 된다.
    # 1. 보류 명령은 상태와 무관하게 끈다(commands.py 가 보내지 않는다).
    if name in HELD_COMMANDS:
        return off(HELD_COMMANDS[name])
    # 2. mission_state 행이 없으면 조정기 상태를 모른다. DISASTER_SIGN 사건에서 추정한 모드는
    #    (source 'site_events') 배지에는 보이지만 버튼을 켜지 않는다.
    if mission['source'] != 'mission_state':
        return off('임무 상태 없음 — operations 가 조정기 상태를 기록하면 켜짐')
    # 3. DISASTER 밖에서는 handle_command 가 모든 종류에 MODE_NOT_DISASTER 로 답한다.
    if mission['mode'] != 'DISASTER':
        return off(f"재난 모드 아님 ({mission['label']})")
    # 4. 명령별 규칙. 각각 그 종류에 대한 조정기 처리기를 따른다.
    phase, dispatch = mission['phase'], mission['dispatch']
    if name == 'START_SEARCH':
        # phase DISASTER 는 조정기의 「재난 감지, 아직 수색 시작 전」이다(mission.phase).
        return (True, '재난 감지 — 아직 수색 전', {}, []) if phase == 'DISASTER' else off('이미 수색을 시작함')
    if name == 'DISPATCH_SUPPORT':
        if dispatch is not None:
            return off('이미 출동함')
        # 확정이고 아직 구출 전인 대상: 지원 로봇을 아직 보낼 수 있는 대상이다.
        confirmed = [t for t in targets if t['detection_confirmed'] and t['stage'] != 'RESCUED']
        if not confirmed:
            return off('확정 대상 없음')
        # 운영자는 화면의 revision 을 승인한다. revision 이 없는 대상은 내놓지 않는다(null 로 보내지 않는다).
        revised = [t for t in confirmed if t['target_revision'] is not None]
        if not revised:
            return off('대상 revision 없음 — 대상 상태에 revision 이 기록되면 켜짐')
        # mission.py: 확정이면서 report_persisted 여야 하고, 아니면 TARGET_NOT_CONFIRMED.
        choices = [_choice(t) for t in revised if t['persisted_report_id'] is not None]
        if not choices:
            return off('보고 저장 대기 — 조정기가 저장 영수증을 받으면 켜짐')
        # 영수증이 없어 빠진 대상 수를 사유에 넣어, 선택지에 없는 대상의 까닭을 밝힌다.
        waiting = len(revised) - len(choices)
        return (True, f'확정 대상 {len(choices)}명 중 고름' + (f' (보고 저장 대기 {waiting}명 빠짐)' if waiting else ''),
                {}, choices)
    if name == 'CONTINUE_SEGMENT':
        if dispatch is None:
            return off('출동 전')
        if dispatch['segment_phase'] != 'AWAITING_APPROVAL':
            return off(f"승인 대기 아님 ({label('segment', dispatch['segment_phase'])})")
        # 화면의 값을 그대로 돌려보낸다. 그사이 바뀐 값이 있으면 조정기가 거절한다
        # (TARGET_REVISION_STALE, ROUTE_REVISION_STALE, SEGMENT_INDEX_MISMATCH).
        args = {key: dispatch[key] for key in ('target_id', 'target_revision', 'route_revision', 'segment_index')}
        return (True, f"구간 {dispatch['segment_index']}/{dispatch['segment_count']} 끝 — 다음 구간 승인 대기",
                args, [])
    if name == 'REQUEST_RECHECK':
        # RESCUE_CONFIRMED 로 판정되지 않은 모든 대상이다. 대상 상태가 없는 대상도 넣는다(C-11 10행:
        # 도착·출동은 조건이 아니다). UNKNOWN_TARGET 과 RECHECK_ALREADY_REQUESTED 는 여전히
        # 조정기가 답한다.
        open_targets = [_choice(t) for t in targets if t['rescue_state'] != 'RESCUE_CONFIRMED']
        if not open_targets:
            return off('재확인할 대상 없음')
        return True, f'대상 {len(open_targets)}명 중 고름', {}, open_targets
    if name == 'RETURN_HOME':
        # 조정기와 같은 순서: ALREADY_RETURNING, 그다음 NO_VERDICT(판정 phase 면 「판정이 있음」이다).
        if phase == 'RETURNING':
            return off('이미 복귀 중')
        if phase not in VERDICT_PHASES:
            return off('구출 판정 전')
        return True, '판정 뒤 — 두 로봇 복귀', {}, []
    # 여기 규칙이 없는 COMMANDS 항목은 사유 없는 버튼을 그리지 않고 드러나게 실패한다.
    raise ConsoleError('INVALID_INPUT', f'no gate for command {name}')


def command_gates(mission, targets, words=None, support_dispatch=None, support_name=None, search_start=None,
                  scout_name=None):
    """모든 명령과 그 버튼 상태. 화면용 조정기 관문 거울이지
    관문 자체가 아니다. 켜진 명령도 거절될 수 있고, 그 답은 온 그대로 보인다."""
    words = dict(COMMANDS) if words is None else words
    rows = []
    for name, _ in COMMANDS:
        enabled, reason, args, choices = _gate(name, mission, targets)
        if name == 'DISPATCH_SUPPORT' and support_dispatch == 'AUTO':
            # 조정기가 지원 로봇을 스스로 보내고 이 명령은 거절한다(NOT_AUTHORIZED).
            # 버튼이 아니라 상태다(소유자 결정 2026-10-13). 이 행에서는 위 _gate 결과를 버린다.
            # `auto: True` 이면 render.py 가 data-command 없는 상태 줄을 그린다.
            auto = f'{support_name or "SUPPORT"} 자동 출동'
            rows.append({'name': name, 'label': auto, 'group': COMMAND_GROUPS[name], 'enabled': False,
                         'reason': f'{auto} — 조정기가 보냄', 'args': {}, 'choices': [], 'auto': True})
            continue
        if name == 'START_SEARCH' and search_start == 'AUTO':
            # 재난 징후가 수색을 시작하고 조정기는 이 명령을 거절한다(AUTO_SEARCH_ENABLED).
            auto = f'{scout_name or "SCOUT"} 자동 출발'
            rows.append({'name': name, 'label': auto, 'group': COMMAND_GROUPS[name], 'enabled': False,
                         'reason': f'{auto} — 지진 감지로 조정기가 수색 시작', 'args': {}, 'choices': [],
                         'auto': True})
            continue
        rows.append({'name': name, 'label': words[name], 'group': COMMAND_GROUPS[name], 'enabled': enabled,
                     'reason': reason, 'args': args, 'choices': choices})
    return rows


def _exit(section):
    """가장 새 exit_qr 관측(#237, 최신 순으로 읽음, VALID 만) 또는 None. 관측일 뿐, 조정기가 등록한
    출구가 아니다(그것은 사유로만 보인다, 예: SUPPORT_EXIT_UNREGISTERED)."""
    if section is None or not section['exits']:
        return None
    row = section['exits'][0]
    return {'location_id': row['location_id'], 'x': row['x'], 'y': row['y'], 'frame_id': row['mission_frame'],
            'observed_at_ns': row['observed_at_ns'], 'map_revision': row['map_revision']}


def build_view(ledger, status, *, built_at_ns, robot_names=None, support_dispatch=None, search_start=None):
    """폴링 한 번의 입력으로 화면 모델 전체(`/api/view` JSON)를 만든다.

    ledger: read_reports() 결과. 보고 장부가 주어지지 않으면 None 이다(장부 없이 --site-state 만,
    2026-10-13). 그때는 보고 행과 건수를 보이지 않고 경고에 그렇다고 적는다.
    status: {'allowed_robots', 'robot_status', 'site_events', 'target_states', 'mission_state',
    'operator_commands', 'error_log', 'exit_locations'}. 각각 해당 operations 읽기 API 결과다(아직 없으면
    None). support_dispatch / search_start: 명령행으로 받은 조정기의 현장 방식 값이다
    (AUTO 면 버튼 대신 상태를 보인다).
    형식이 틀린 입력은 ``ConsoleError INVALID_INPUT`` 을 올린다. 없는 절은 알린다(``warnings`` 또는
    ``history['missing']``). 기본값으로 채우지 않는다.
    """
    _require(type(built_at_ns) is int and built_at_ns >= 0, 'built_at_ns must be int ns')
    names = dict(robot_names or {})   # robot_id -> 표시 이름(--robot-name). 없으면 ID 그대로
    _require(support_dispatch in (None,) + SUPPORT_DISPATCH_MODES,
             f'support_dispatch must be one of {SUPPORT_DISPATCH_MODES} or unset')
    _require(search_start in (None,) + SEARCH_START_MODES, f'search_start must be one of {SEARCH_START_MODES} or unset')
    warnings = []
    robot_section = _section(status, 'robot_status', 'robots')
    allowed_section = _section(status, 'allowed_robots', 'allowed_robot_ids')
    if ledger is None and allowed_section is not None:
        # 현장 상태 DB 가 배치 로봇을 정한다(read_allowed_robot_ids, #149).
        warnings.append('보고 장부 입력 없음 — 구조 보고 행·건수를 표시하지 않음')
        allowed = tuple(allowed_section['allowed_robot_ids'])
    elif ledger is None:
        # 로봇 목록이 없는 픽스처: 상태를 보낸 로봇만 그릴 수 있다.
        warnings.append('보고 장부 입력 없음 — 구조 보고 행·건수를 표시하지 않음, 로봇 목록은 상태가 들어온 로봇만')
        allowed = tuple(sorted({r.get('robot_id') for r in ([] if robot_section is None else robot_section['robots'])}))
    else:
        # 장부가 있으면 장부의 로봇 목록을 쓴다. 현장 상태 목록은 보지 않는다.
        allowed = tuple(ledger['allowed_robot_ids'])
    site_section = _section(status, 'site_events', 'site_events')
    target_section = _section(status, 'target_states', 'targets')
    has_mission, mission_row = _mission_row(status)
    # 없는 절은 빈 목록이 아니라 보이는 경고다(「입력 없음」이 「아무 일도 없음」처럼 보이면
    # 안 된다). `has_mission or None`: 행이 None 인 mission_state 절은 있는 것이므로 경고하지 않는다.
    for key, section in (('robot_status', robot_section), ('site_events', site_section),
                         ('target_states', target_section), ('mission_state', has_mission or None)):
        if section is None:
            warnings.append(f'{key} 입력 없음')
    target_states = {}
    for state in [] if target_section is None else target_section['targets']:
        _require(state.get('target_id') not in target_states, f"duplicate target state {state.get('target_id')}")
        target_states[state['target_id']] = state
    site_events = [] if site_section is None else _site_events(site_section['site_events'])
    candidates = _latest_candidates(site_events)
    targets = _targets(ledger, target_states, candidates)
    for target_id in sorted(set(candidates) - {row['target_id'] for row in targets}):
        warnings.append(f'소실 후보의 대상이 목록에 없음: {target_id}')
    # API 모델에는 두되 그리지 않는다. 고정 카메라가 하나라 EXIT_PASS 를 내는 곳이 없어
    # 수가 0 에 머문다(2026-10-09 결정, console-and-db-plan 3절, db-schema 2절). 3.5.5 의
    # 「구출 <= 출구 통과 수」 경고도 함께 뺐다. 구출 계기는 카메라의 후보 사건이다.
    exit_count = None if site_section is None else site_section['exit_pass_count']
    _require(exit_count is None or (type(exit_count) is int and exit_count >= 0), 'exit_pass_count')
    stage_counts = {stage: 0 for stage in STAGES}
    for row in targets:
        stage_counts[row['stage']] += 1
    events, event_total = _events(ledger, site_events)
    mission = _mission(None if site_section is None else site_events, mission_row)
    robots = _robots(allowed, [] if robot_section is None else robot_section['robots'], built_at_ns, warnings)
    for row in robots:
        row['name'] = names.get(row['robot_id'], row['robot_id'])
    # 역할의 표시 이름은 그 역할을 알리는 로봇이 정확히 하나일 때만 쓴다(역할은 상태에서 온다).
    # 없거나 여럿이면 None 이고, 짐작 대신 역할 코드(SUPPORT/SCOUT)를 보인다.
    support = [row['name'] for row in robots if row['role'] == 'SUPPORT']
    words = _command_words(support[0] if len(support) == 1 else None)
    scout = [row['name'] for row in robots if row['role'] == 'SCOUT']
    mission['reason_view'] = mission_reason(mission['reason'], support[0] if len(support) == 1 else None,
                                            scout[0] if len(scout) == 1 else None)
    history = _history(status, words)
    refused = [row['reason'] for row in (history or {'rows': []})['rows']] + [mission['reason']]
    if search_start != 'AUTO' and 'AUTO_SEARCH_ENABLED' in refused:
        # 조정기는 search_start AUTO 로 도는데, 이 콘솔은 조정기가 거절할 버튼을 아직 내놓는다.
        warnings.append('조정기는 자동 수색(search_start AUTO) — 관제 웹을 --search-start AUTO 로 다시 띄우세요')
    exits = _section(status, 'exit_locations', 'exits')
    return {
        'built_at_ns': built_at_ns,
        'exit': _exit(exits),
        'evidence_policy': None if ledger is None else ledger['evidence_policy'],
        'mission': mission,
        'robots': robots,
        'targets': targets,
        'stage_counts': stage_counts,
        'exit_count': exit_count,
        'events': events,
        'event_total': event_total,
        'map': {'available': False, 'note': '지도 스냅샷 미연결 — 로봇·지도 제공 뒤 연결'},
        'commands': command_gates(mission, targets, words, support_dispatch,
                                  support[0] if len(support) == 1 else None, search_start,
                                  scout[0] if len(scout) == 1 else None),
        'support_dispatch': support_dispatch,
        'search_start': search_start,
        'history': history,
        'warnings': warnings,
    }
