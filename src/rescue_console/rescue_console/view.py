"""Ledger + status snapshot -> one screen model. Pure Python; no I/O, no clock reads.

The caller passes `built_at_ns`, so the same inputs always give the same model.
Values that are missing stay None and are shown as missing; nothing is filled in.
"""

import re

from rescue_contracts import RESCUE_STATES, RescueError, SiteEvent

from .labels import label

# Q-20 (scope.md, 2026-10-08): the four Part I 3.7.2 stages are composed here, in the
# display layer. The contract vocabulary (RESCUE_STATES, confirmation_result) is not extended.
STAGES = {
    'UNCONFIRMED': '미확정',
    'NOT_RESCUED': '미구조',
    'SUPPORTING': '지원 중',
    'RESCUED': '구출 완료',
}
# Provisional support vocabulary until the operations read API exists ([제안] 관제·화면, 2026-10-08).
SUPPORT_STATES = frozenset({'NONE', 'ASSIGNED', 'ARRIVED', 'FAILED'})
ACTIVE_SUPPORT = frozenset({'ASSIGNED', 'ARRIVED'})

# Rule table, first match wins. Each row: (rule id, condition text, stage).
# The text is what README shows; `compose_stage` below is the executable form.
STAGE_RULES = (
    ('R1', 'detection_confirmed is false', 'UNCONFIRMED'),
    ('R2', 'rescue_state == RESCUE_CONFIRMED', 'RESCUED'),
    ('R3', 'support_state in {ASSIGNED, ARRIVED}', 'SUPPORTING'),
    ('R4', 'support_state unknown and approach_result == APPROACH_COMPLETE', 'SUPPORTING'),
    ('R5', 'otherwise (CONFIRMED, not rescued, no active support)', 'NOT_RESCUED'),
)

# Operator commands: the kinds of `POST /api/operator-commands` in console-and-db-plan 4절
# (Part I 3.7.6 plus the human gates of the mission flow). The first five are sent to operations
# (rescue_console.commands); the rest stay off here whatever the state.
COMMANDS = (
    ('START_SEARCH', '수색 시작'),
    ('DISPATCH_SUPPORT', 'SUPPORT 출동 승인'),   # the SUPPORT robot's name once a status names it (_command_words)
    ('CONTINUE_SEGMENT', '다음 구간 계속'),
    ('REQUEST_RECHECK', '재확인 지시'),
    ('RETURN_HOME', '복귀'),
    ('PAUSE', '일시정지'),
    ('RESUME', '재개'),
    ('CANCEL', '작업 취소'),
    ('STOP_ROBOT', '정지'),
)
# Panel grouping: the flow's human gates in order (#85 start, support dispatch, each next segment) first,
# the stop apart from everything else.
COMMAND_GROUPS = {
    'START_SEARCH': 'approve', 'DISPATCH_SUPPORT': 'approve', 'CONTINUE_SEGMENT': 'approve',
    'REQUEST_RECHECK': 'mission', 'RETURN_HOME': 'mission', 'PAUSE': 'mission', 'RESUME': 'mission',
    'CANCEL': 'mission',
    'STOP_ROBOT': 'stop',
}
# Commands the console never sends, and why (작전 2026-10-12, #115).
HELD_COMMANDS = {
    'PAUSE': 'Q-22 재개 관문 전',
    'RESUME': 'Q-22 재개 관문 전',
    'CANCEL': 'Q-22 재개 관문 전',
    'STOP_ROBOT': 'RequestStop 별도 경로 — 위험하면 현장 비상 정지 장치',
}
# Phases after the mission verdict, before the return (rescue_coordination.mission: RETURN_HOME needs a verdict).
VERDICT_PHASES = frozenset({'RESCUE_CONFIRMED', 'RECHECK_NEEDED'})

# Row cap for the event table; older rows stay in the ledger ([제안] 관제·화면, 2026-10-08).
EVENT_ROWS = 50
# Row cap for the command and error history; older rows stay in the site-state DB ([제안] 작전, 2026-10-12).
HISTORY_ROWS = 30

# A robot row whose last status reached operations longer ago than this is marked stale
# ([제안] 관제·화면, 2026-10-10). Display only, not readiness (C-05); replace with the profile's
# readiness max_age_s once profile v2.1 (#39) is on main.
STATUS_STALE_S = 5.0

# Mission badge words: site_state MISSION_MODES (contract S-00 modes + SystemState MISSION_FINISHED).
MODES = {'NORMAL': '평시', 'DISASTER': '재난', 'CLEARING': '해제 중', 'MISSION_FINISHED': '임무 종료'}
DISPATCH_KEYS = ('target_id', 'target_revision', 'route_revision', 'segment_index', 'segment_count', 'segment_phase')


class ConsoleError(ValueError):
    """Console input or setup failure with a stable code.

    Local to the console, not a RescueError: contract codes are reserved for the
    robot, verdict and report paths (작전 decision 2026-10-08, README 「하지 않는 것」).
    """

    def __init__(self, code, message=''):
        self.detail_code, self.detail = code, message
        super().__init__(f'{code}: {message}')


def _require(ok, message, code='INVALID_INPUT'):
    if not ok:
        raise ConsoleError(code, message)


def compose_stage(detection_confirmed, rescue_state, support_state, approach_result):
    """Return (stage, rule id, warnings) for one target. None means "not reported"."""
    _require(type(detection_confirmed) is bool, 'detection_confirmed must be bool')
    _require(rescue_state is None or rescue_state in RESCUE_STATES, 'rescue_state outside RESCUE_STATES')
    _require(support_state is None or support_state in SUPPORT_STATES, 'support_state')
    warnings = []
    if not detection_confirmed:
        if rescue_state == 'RESCUE_CONFIRMED' or support_state in ACTIVE_SUPPORT:
            warnings.append('미확정 대상에 구출/지원 상태가 있음 — 입력 불일치')
        return 'UNCONFIRMED', 'R1', warnings
    if rescue_state == 'RESCUE_CONFIRMED':
        return 'RESCUED', 'R2', warnings
    if support_state in ACTIVE_SUPPORT:
        return 'SUPPORTING', 'R3', warnings
    if support_state is None and approach_result == 'APPROACH_COMPLETE':
        return 'SUPPORTING', 'R4', warnings
    return 'NOT_RESCUED', 'R5', warnings


def _section(status, key, items_key):
    section = status.get(key)
    if section is None:
        return None
    _require(isinstance(section, dict) and isinstance(section.get('schema_version'), str)
             and isinstance(section.get(items_key), list), f'{key} must be a read API result')
    return section


def _latest_reports(reports):
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
    if position is None:
        return None
    return {'x': position['x'], 'y': position['y'], 'frame_id': position['frame_id'],
            'map_revision': position['map_revision'], 'source': source}


def _latest_candidates(site_events):
    """target_id -> the newest TARGET_DISAPPEARED_CANDIDATE naming it (a candidate only; the robot rechecks)."""
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
    """``ledger`` None: no report ledger was given; report fields stay None (not 0, not 'no report')."""
    reports = {} if ledger is None else _latest_reports(ledger['reports'])
    rows = []
    for target_id in sorted(set(reports) | set(target_states)):
        item, report_count = reports.get(target_id, (None, 0))
        payload = None if item is None else item['payload']
        state = target_states.get(target_id)
        warnings = []
        report_confirmed = None if payload is None else payload['confirmation_result'] == 'CONFIRMED'
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
        # The revision the operator approves (SubmitOperatorCommand expected_target_revision). It comes in the
        # target state once operations stores it; until then None, and the dispatch button stays off.
        revision = None if state is None else state.get('target_revision')
        _require(revision is None or (type(revision) is int and revision >= 0), 'target_revision')
        # The report of that revision the coordinator holds a PERSISTED receipt for (TargetEstimate
        # persisted_report_id, #118 선택 키). Without it the coordinator refuses the dispatch; a row written
        # before the key existed reads None too. Never guessed from the ledger receipt below.
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
            # Ages compare a robot stamp with this host's clock: valid only when clocks agree (C-03).
            'battery_age_s': None if battery_stamp is None else (built_at_ns - battery_stamp) / 1e9,
            'active_task_id': status.get('active_task_id'),
            'stamp_ns': stamp,
            'age_s': None if stamp is None else (built_at_ns - stamp) / 1e9,
            # Ledger-clock age: still meaningful when a rebooted robot's clock went backwards.
            'received_age_s': received_age,
            'stale': received_age is None or received_age > STATUS_STALE_S,
        })
    return rows


def _site_events(raw_events):
    events = []
    for raw in raw_events:
        try:
            events.append(SiteEvent(**raw))
        except (TypeError, RescueError) as exc:
            raise ConsoleError('INVALID_INPUT', f'site event: {exc}') from exc
    return events


def _events(ledger, site_events):
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
    rows.sort(key=lambda row: (row['event_ns'], row['ref_id']), reverse=True)
    return rows[:EVENT_ROWS], len(rows)


MISSION_ROW_KEYS = frozenset({'mode', 'phase', 'trigger_event_id', 'started_at_ns', 'updated_at_ns', 'reason',
                              *DISPATCH_KEYS})


def _mission_row(status):
    """The read_mission_state() result: (section present, row or None)."""
    section = status.get('mission_state')
    if section is None:
        return False, None
    _require(isinstance(section, dict) and isinstance(section.get('schema_version'), str) and 'mission' in section,
             'mission_state must be a read API result')
    row = section['mission']
    _require(row is None or (isinstance(row, dict) and MISSION_ROW_KEYS <= set(row)), 'mission_state row')
    return True, row


# SystemState.reason codes in plain Korean (2026-10-13). {support}/{scout}: the display names of the
# SUPPORT and SCOUT robots. level: info (normal progress), caution (held), alert (red: failed, not
# retried). scope: where besides the mission band it is shown (support/scout card, the dispatched target).
MISSION_REASONS = {
    'SUPPORT_EXITING': ('{support} 출구로 이동 중', 'info'),
    'SUPPORT_AT_EXIT': ('{support} 출구 도착', 'info'),
    'SUPPORT_EXIT_CANCELED': ('{support} 출구 이동 취소(판정 종료·재난 해제)', 'info'),
    'SUPPORT_EXIT_FAILED': ('{support} 출구 이동 실패 — 다시 보내지 않음', 'alert'),
    'SUPPORT_POSE_UNKNOWN': ('{support} 위치 모름', 'caution'),
    # Automatic approach (#187): why TB-5 has not been sent, or that it failed (never retried by itself).
    'SUPPORT_POSE_BASIS_MISMATCH': ('{support} 위치의 지도 기준이 다름 — 자동 출동 보류', 'caution'),
    'SUPPORT_AT_TARGET': ('{support} 가 이미 대상 바로 옆 — 접근 지점을 정할 수 없음', 'caution'),
    'SUPPORT_NOT_READY': ('{support} 준비 안 됨 — arm 필요', 'caution'),   # #196: no fresh READY status
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
    # TargetLocation time gate (#174, OPS-103): no target is made from these.
    'TARGET_TIME_BASIS_UNUSABLE': ('대상 위치의 관측 시각을 쓸 수 없음(시계 미측정·오차 상한 없음)', 'caution'),
    'TARGET_CLOCK_DOMAIN_MISMATCH': ('대상 위치 관측의 시계 영역이 다름', 'caution'),
    'TARGET_OBSERVATION_FUTURE': ('대상 위치 관측 시각이 미래', 'caution'),
    'TARGET_OBSERVATION_STALE': ('대상 위치 관측이 오래됨', 'caution'),
    # #214 gates (OPS-127/129): each holds only the move it guards — TB-5 dispatch, TB-5 exit or TB-6 return.
    # The blocked points are never moved elsewhere.
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
    # search_start AUTO (#228, OPS-140) and the exit QR before the freeze (OPS-136).
    'SEARCH_GOAL_NOT_SELECTED': ('{scout} 수색 목표 선택 대기', 'caution'),
    'AUTO_SEARCH_ENABLED': ('자동 수색 — 지진 감지로 시작, 수색 시작 버튼은 받지 않음', 'info'),
    'MAP_FREEZE_WAITING_EXIT': ('출구 QR 찾는 중 — {support} 출동 대기', 'info'),
    # #234 (OPS-147): the QR exit holds TB-5's dispatch and its exit move; the start pose holds patrol and TB-6's return.
    'TARGET_REBASED_ON_FREEZE': ('대상 위치를 고정 지도로 옮김 — 보고 저장 대기', 'info'),
    'EXIT_NOT_ON_FROZEN_MAP': ('출구(QR) 위치가 고정 지도 기준에 없음 — {support} 출동·출구 이동 보류', 'caution'),
    'SUPPORT_EXIT_UNREGISTERED': ('출구(QR) 미등록 — {support} 출동·출구 이동 보류', 'caution'),
    'START_POSE_UNKNOWN': ('{scout} 출발 자세 모름 — 순찰·{scout} 복귀 보류', 'caution'),
    'START_POSE_CONFLICT': ('{scout} 출발 자세가 다른 값으로 다시 옴 — 처음 값 유지, 확인 필요', 'alert'),
    'PATROL_EXHAUSTED': ('{scout} 순찰 끝(정한 바퀴 다 돎) — 더 돌지 않음', 'caution'),
}
# APPROACH_FAILED_<why> (#216): the executor's terminal reason, or its outcome when there is none.
# An unknown why is shown as its code (OPS-127: never hidden).
APPROACH_WHY = {
    'PASSAGE_BLOCKED': '통로 막힘', 'UNREACHABLE': '갈 수 없는 위치', 'FAILED': '실행 실패',
    'CANCELED': '취소됨', 'EXPIRED': '시간 초과', 'NAV_NOT_READY': '주행 준비 안 됨',
    'DEADLINE_EXPIRED': '시간 초과', 'STOP_LATCHED': '정지 유지 중', 'LOCAL_LEASE_EXPIRED': '명령 유효 시간 지남',
    'CLOCK_UNCERTAIN': '시계 불확실',
}
EXIT_COUNTDOWN = re.compile(r'SUPPORT_EXIT_IN_(\d+)S')
PATROL_ROUND = re.compile(r'PATROL_ROUND_(\d+)')
# RECHECK_NEEDED_<why>: the recheck did not confirm the rescue (C-11 rows 7-11).
RECHECK_WHY = {
    'CANDIDATE_NOT_LINKED': '후보 연결 안 됨', 'FRAME_REVISION_MISMATCH': '지도 기준 다름',
    'CANDIDATE_TIME_BASIS_UNUSABLE': '시각 기준 못 씀', 'ORDER_VIOLATION': '시각 순서 틀림',
    'CAMERA_NOT_ABSENT': '인형이 아직 보임', 'LIDAR_NOT_CLEAR': '자리가 막힘', 'COVERAGE_NOT_VALID': '관측 범위 부족',
    'RECHECK_OBSERVATION_MISSING': '재확인 관측 없음 — 「재확인 지시」 버튼으로 다시 보낼 수 있음',
}
SCOPES = (('SUPPORT_', 'support'), ('SCOUT_', 'scout'), ('RECHECK_NEEDED_', 'target'))
# Approach codes of #187 without the SUPPORT_ prefix that still belong to the support robot.
SUPPORT_CODES = frozenset({'APPROACH_FAILED', 'MODE_NOT_DISASTER', 'ALREADY_DISPATCHED', 'UNKNOWN_TARGET',
                           'TARGET_NOT_CONFIRMED', 'TARGET_REVISION_STALE', 'TARGET_ALREADY_JUDGED'})


def mission_reason(code, support_name, scout_name):
    """{code, text, level, scope} for a SystemState.reason; an unknown code stays as it is."""
    if code is None:
        return None
    # Without a status naming the role's robot its robot_id is not known: the role code, never a generic word.
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
    """Badge input. The mission_state row (operations writes the coordinator's SystemState, #111) when
    there is one. Before the first row the mode is inferred from DISASTER_SIGN events and marked so
    (`source`); None means neither is available."""
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


SUPPORT_DISPATCH_MODES = ('AUTO', 'OPERATOR')   # --support-dispatch; the coordinator's field value
SEARCH_START_MODES = ('AUTO', 'OPERATOR')       # --search-start; the coordinator's field value (#228)


def _command_words(support_name):
    """Command labels; the dispatch carries the SUPPORT robot's display name (owner 2026-10-13: TB-5)."""
    words = dict(COMMANDS)
    if support_name is not None:
        words['DISPATCH_SUPPORT'] = f'{support_name} 출동 승인'
    return words


def _history(status, words=None):
    """Operator commands as operations recorded them (answer and reason, rejections included) and
    operations' error_log, newest first. None when neither input is there (the status fixture).
    Kept apart from the event rows: those carry the KPI-04 display latency, these do not."""
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
    return {'target_id': target['target_id'], 'target_revision': target['target_revision'],
            'stage': target['stage'], 'stage_label': target['stage_label']}


def _gate(name, mission, targets):
    """(enabled, reason, args, choices) for one command, mirroring rescue_coordination.mission.handle_command."""
    def off(reason):
        return False, reason, {}, []
    if name in HELD_COMMANDS:
        return off(HELD_COMMANDS[name])
    if mission['source'] != 'mission_state':
        return off('임무 상태 없음 — operations 가 조정기 상태를 기록하면 켜짐')
    if mission['mode'] != 'DISASTER':
        return off(f"재난 모드 아님 ({mission['label']})")
    phase, dispatch = mission['phase'], mission['dispatch']
    if name == 'START_SEARCH':
        return (True, '재난 감지 — 아직 수색 전', {}, []) if phase == 'DISASTER' else off('이미 수색을 시작함')
    if name == 'DISPATCH_SUPPORT':
        if dispatch is not None:
            return off('이미 출동함')
        confirmed = [t for t in targets if t['detection_confirmed'] and t['stage'] != 'RESCUED']
        if not confirmed:
            return off('확정 대상 없음')
        # The operator approves the revision on screen; a target without one is not offered (never sent as null).
        revised = [t for t in confirmed if t['target_revision'] is not None]
        if not revised:
            return off('대상 revision 없음 — 대상 상태에 revision 이 기록되면 켜짐')
        # mission.py: confirmed AND report_persisted, else TARGET_NOT_CONFIRMED.
        choices = [_choice(t) for t in revised if t['persisted_report_id'] is not None]
        if not choices:
            return off('보고 저장 대기 — 조정기가 저장 영수증을 받으면 켜짐')
        waiting = len(revised) - len(choices)
        return (True, f'확정 대상 {len(choices)}명 중 고름' + (f' (보고 저장 대기 {waiting}명 빠짐)' if waiting else ''),
                {}, choices)
    if name == 'CONTINUE_SEGMENT':
        if dispatch is None:
            return off('출동 전')
        if dispatch['segment_phase'] != 'AWAITING_APPROVAL':
            return off(f"승인 대기 아님 ({label('segment', dispatch['segment_phase'])})")
        args = {key: dispatch[key] for key in ('target_id', 'target_revision', 'route_revision', 'segment_index')}
        return (True, f"구간 {dispatch['segment_index']}/{dispatch['segment_count']} 끝 — 다음 구간 승인 대기",
                args, [])
    if name == 'REQUEST_RECHECK':
        open_targets = [_choice(t) for t in targets if t['rescue_state'] != 'RESCUE_CONFIRMED']
        if not open_targets:
            return off('재확인할 대상 없음')
        return True, f'대상 {len(open_targets)}명 중 고름', {}, open_targets
    if name == 'RETURN_HOME':
        if phase == 'RETURNING':
            return off('이미 복귀 중')
        if phase not in VERDICT_PHASES:
            return off('구출 판정 전')
        return True, '판정 뒤 — 두 로봇 복귀', {}, []
    raise ConsoleError('INVALID_INPUT', f'no gate for command {name}')


def command_gates(mission, targets, words=None, support_dispatch=None, support_name=None, search_start=None,
                  scout_name=None):
    """Every command with the state of its button. A mirror of the coordinator gate for the screen,
    not the gate: an enabled command may still be rejected, and that answer is shown as it comes."""
    words = dict(COMMANDS) if words is None else words
    rows = []
    for name, _ in COMMANDS:
        enabled, reason, args, choices = _gate(name, mission, targets)
        if name == 'DISPATCH_SUPPORT' and support_dispatch == 'AUTO':
            # The coordinator sends the support robot itself and refuses this command (NOT_AUTHORIZED):
            # a state, not a button (owner decision 2026-10-13).
            auto = f'{support_name or "SUPPORT"} 자동 출동'
            rows.append({'name': name, 'label': auto, 'group': COMMAND_GROUPS[name], 'enabled': False,
                         'reason': f'{auto} — 조정기가 보냄', 'args': {}, 'choices': [], 'auto': True})
            continue
        if name == 'START_SEARCH' and search_start == 'AUTO':
            # The disaster sign starts the search and the coordinator refuses this command (AUTO_SEARCH_ENABLED).
            auto = f'{scout_name or "SCOUT"} 자동 출발'
            rows.append({'name': name, 'label': auto, 'group': COMMAND_GROUPS[name], 'enabled': False,
                         'reason': f'{auto} — 지진 감지로 조정기가 수색 시작', 'args': {}, 'choices': [],
                         'auto': True})
            continue
        rows.append({'name': name, 'label': words[name], 'group': COMMAND_GROUPS[name], 'enabled': enabled,
                     'reason': reason, 'args': args, 'choices': choices})
    return rows


def _exit(section):
    """The newest exit_qr observation (#237, read newest first; VALID only), or None. An observation, not the exit
    the coordinator registered (that one shows only as its reasons, e.g. SUPPORT_EXIT_UNREGISTERED)."""
    if section is None or not section['exits']:
        return None
    row = section['exits'][0]
    return {'location_id': row['location_id'], 'x': row['x'], 'y': row['y'], 'frame_id': row['mission_frame'],
            'observed_at_ns': row['observed_at_ns'], 'map_revision': row['map_revision']}


def build_view(ledger, status, *, built_at_ns, robot_names=None, support_dispatch=None, search_start=None):
    """ledger: read_reports() result, or None when no report ledger was given (--site-state without it,
    2026-10-13): report rows and counts are then not shown, said so in the warnings. status: {'robot_status', 'site_events', 'target_states', 'mission_state',
    'operator_commands', 'error_log'}, each the matching operations read API result (or None when not available yet)."""
    _require(type(built_at_ns) is int and built_at_ns >= 0, 'built_at_ns must be int ns')
    names = dict(robot_names or {})   # robot_id -> display name (--robot-name); none: the ID itself
    _require(support_dispatch in (None,) + SUPPORT_DISPATCH_MODES,
             f'support_dispatch must be one of {SUPPORT_DISPATCH_MODES} or unset')
    _require(search_start in (None,) + SEARCH_START_MODES, f'search_start must be one of {SEARCH_START_MODES} or unset')
    warnings = []
    robot_section = _section(status, 'robot_status', 'robots')
    allowed_section = _section(status, 'allowed_robots', 'allowed_robot_ids')
    if ledger is None and allowed_section is not None:
        # The site-state DB fixes the deployment's robots (read_allowed_robot_ids, #149).
        warnings.append('보고 장부 입력 없음 — 구조 보고 행·건수를 표시하지 않음')
        allowed = tuple(allowed_section['allowed_robot_ids'])
    elif ledger is None:
        # A fixture without the robot list: only robots that sent a status can be drawn.
        warnings.append('보고 장부 입력 없음 — 구조 보고 행·건수를 표시하지 않음, 로봇 목록은 상태가 들어온 로봇만')
        allowed = tuple(sorted({r.get('robot_id') for r in ([] if robot_section is None else robot_section['robots'])}))
    else:
        allowed = tuple(ledger['allowed_robot_ids'])
    site_section = _section(status, 'site_events', 'site_events')
    target_section = _section(status, 'target_states', 'targets')
    has_mission, mission_row = _mission_row(status)
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
    # Kept in the API model, not drawn: with one fixed camera there is no EXIT_PASS producer and the
    # count stays 0 (2026-10-09 decision; console-and-db-plan 3절, db-schema 2절). The 3.5.5
    # "rescued <= exit count" warning went with it; the rescue trigger is the camera's candidate event.
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
    support = [row['name'] for row in robots if row['role'] == 'SUPPORT']
    words = _command_words(support[0] if len(support) == 1 else None)
    scout = [row['name'] for row in robots if row['role'] == 'SCOUT']
    mission['reason_view'] = mission_reason(mission['reason'], support[0] if len(support) == 1 else None,
                                            scout[0] if len(scout) == 1 else None)
    history = _history(status, words)
    refused = [row['reason'] for row in (history or {'rows': []})['rows']] + [mission['reason']]
    if search_start != 'AUTO' and 'AUTO_SEARCH_ENABLED' in refused:
        # The coordinator runs search_start AUTO; this console still offers the button it refuses.
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
