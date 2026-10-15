"""장부 + 상태 스냅숏 -> 화면 모델 변환. 순수 함수형 Python 모듈로 I/O 및 시스템 시계 조회가 없다.

호출 측에서 `built_at_ns`를 명시적으로 주입하므로 동일한 입력에 대해 항상 결정론적인 모델을 생성한다.
누락된 값은 임의의 기본값으로 채우지 않고 None을 유지하여 화면에도 미수신 상태로 노출한다.
"""

import re

from rescue_contracts import RESCUE_STATES, RescueError, SiteEvent

from .labels import label

# Q-20 (scope.md, 2026-10-08): Part I 3.7.2의 요구사항인 대상 4단계를 프레젠테이션 계층에서 합성한다.
# 계약 어휘(RESCUE_STATES, confirmation_result)는 늘리지 않는다.
STAGES = {
    'UNCONFIRMED': '미확정',
    'NOT_RESCUED': '미구조',
    'SUPPORTING': '지원 중',
    'RESCUED': '구출 완료',
}
# operations 읽기 API가 구현될 때까지 사용할 임시 지원 상태 어휘 정의([제안] 관제·화면, 2026-10-08).
SUPPORT_STATES = frozenset({'NONE', 'ASSIGNED', 'ARRIVED', 'FAILED'})
ACTIVE_SUPPORT = frozenset({'ASSIGNED', 'ARRIVED'})

# 상태 판정 규칙 표. 위에서부터 최초로 조건을 만족하는 행이 적용된다: (규칙 ID, 조건 설명, 결과 단계).
# 조건 설명은 README 사양과 일치하며, 하단의 `compose_stage` 함수에서 실제 분기로 구현된다.
STAGE_RULES = (
    ('R1', 'detection_confirmed is false', 'UNCONFIRMED'),
    ('R2', 'rescue_state == RESCUE_CONFIRMED', 'RESCUED'),
    ('R3', 'support_state in {ASSIGNED, ARRIVED}', 'SUPPORTING'),
    ('R4', 'support_state unknown and approach_result == APPROACH_COMPLETE', 'SUPPORTING'),
    ('R5', 'otherwise (CONFIRMED, not rescued, no active support)', 'NOT_RESCUED'),
)

# 운영자 명령 목록: console-and-db-plan 4절의 `POST /api/operator-commands` 명세에 대응한다
# (Part I 3.7.6의 명령에 임무 상태 전이를 위한 사람 승인 관문을 통합한 구성). 상위 5개는 operations로 전달하며
# (rescue_console.commands), 하위 명령들은 상태와 무관하게 비활성화 상태를 유지한다.
COMMANDS = (
    ('DECLARE_DISASTER', '재난 선포'),   # 평시에 지진 감지 대신 운영자가 재난 모드로 넣는다(재범 결정 2026-10-15)
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
# 버튼 그룹 분류: 사람 승인 관문(#85 수색 시작, 지원 출동, 구간별 계속 진행)을 상단에 순서대로 배치하고,
# 비상 정지는 오작동 방지를 위해 다른 명령과 분리한다.
COMMAND_GROUPS = {
    'DECLARE_DISASTER': 'approve', 'START_SEARCH': 'approve', 'DISPATCH_SUPPORT': 'approve', 'CONTINUE_SEGMENT': 'approve',
    'REQUEST_RECHECK': 'mission', 'RETURN_HOME': 'mission', 'PAUSE': 'mission', 'RESUME': 'mission',
    'CANCEL': 'mission',
    'STOP_ROBOT': 'stop',
}
# 콘솔에서 전송을 차단하는 보류 명령 및 차단 사유(작전 2026-10-12, #115).
HELD_COMMANDS = {
    'PAUSE': 'Q-22 재개 관문 전',
    'RESUME': 'Q-22 재개 관문 전',
    'CANCEL': 'Q-22 재개 관문 전',
    'STOP_ROBOT': 'RequestStop 별도 경로 — 위험하면 현장 비상 정지 장치',
}
# 임무 판정 완료 후 복귀 전 단계(rescue_coordination.mission: RETURN_HOME은 판정 완료가 선행되어야 함).
VERDICT_PHASES = frozenset({'RESCUE_CONFIRMED', 'RECHECK_NEEDED'})

# 사건 테이블 최대 표시 행 수. 초과된 이전 이력은 원장에 보존된다([제안] 관제·화면, 2026-10-08).
EVENT_ROWS = 50
# 명령 및 오류 로그 최대 표시 행 수. 초과된 이전 이력은 현장 상태 DB에 보존된다([제안] 작전, 2026-10-12).
HISTORY_ROWS = 30

# operations 수신 후 경과 시간이 이 임계치를 초과하면 해당 로봇 행을 낡음(stale)으로 표시한다
# ([제안] 관제·화면, 2026-10-10). 이는 UI 표시용이며 노드 준비 상태 판정(C-05)과 무관하다. 프로파일 v2.1(#39)이
# main에 병합되면 프로파일의 readiness max_age_s 파라미터로 대체한다.
STATUS_STALE_S = 5.0

# 임무 배지 상태 레이블: site_state MISSION_MODES(계약 S-00 모드 + SystemState MISSION_FINISHED).
MODES = {'NORMAL': '평시', 'DISASTER': '재난', 'CLEARING': '해제 중', 'MISSION_FINISHED': '임무 종료'}
DISPATCH_KEYS = ('target_id', 'target_revision', 'route_revision', 'segment_index', 'segment_count', 'segment_phase')


class ConsoleError(ValueError):
    """콘솔 계층의 입력 검증 및 설정 오류를 고정된 오류 코드와 함께 표현한다.

    콘솔 내부 전용 예외이며 RescueError와 구분된다. 계약 계층 오류 코드는 로봇·판정·보고 경로에만
    한정하여 사용한다(작전 결정 2026-10-08, README 「하지 않는 것」).
    """

    def __init__(self, code, message=''):
        self.detail_code, self.detail = code, message
        super().__init__(f'{code}: {message}')


def _require(ok, message, code='INVALID_INPUT'):
    if not ok:
        raise ConsoleError(code, message)


def compose_stage(detection_confirmed, rescue_state, support_state, approach_result):
    """단일 대상의 상태를 합성하여 (단계, 규칙 ID, 경고 목록)을 반환한다. None은 '미보고' 상태를 의미한다."""
    # 엄격한 bool 타입 검사: 누락된 값이 '미확정'으로 오인되어 R1으로 잘못 분류되는 오류를 차단한다.
    _require(type(detection_confirmed) is bool, 'detection_confirmed must be bool')
    _require(rescue_state is None or rescue_state in RESCUE_STATES, 'rescue_state outside RESCUE_STATES')
    _require(support_state is None or support_state in SUPPORT_STATES, 'support_state')
    warnings = []
    # 아래 분기는 STAGE_RULES R1..R5를 순차 평가하며, 최초 일치하는 규칙의 결과를 즉시 반환한다.
    if not detection_confirmed:
        # 상호 모순된 입력은 R1으로 처리하고 경고를 부착한다. 모순 상태에서 RESCUED나 SUPPORTING으로 승격하지 않는다.
        if rescue_state == 'RESCUE_CONFIRMED' or support_state in ACTIVE_SUPPORT:
            warnings.append('미확정 대상에 구출/지원 상태가 있음 — 입력 불일치')
        return 'UNCONFIRMED', 'R1', warnings
    if rescue_state == 'RESCUE_CONFIRMED':
        return 'RESCUED', 'R2', warnings
    if support_state in ACTIVE_SUPPORT:
        return 'SUPPORTING', 'R3', warnings
    # R4는 지원 상태를 알 수 없는 경우(None)에 한해 평가한다. 명시적 보고(NONE/FAILED)가 APPROACH_COMPLETE보다 우선한다.
    if support_state is None and approach_result == 'APPROACH_COMPLETE':
        return 'SUPPORTING', 'R4', warnings
    return 'NOT_RESCUED', 'R5', warnings


def _section(status, key, items_key):
    """읽기 API 응답 ``status[key]`` 섹션을 반환한다. 키가 없으면(데이터 미수신) None을 반환한다.

    존재하는 섹션은 문자열 ``schema_version``과 ``items_key`` 리스트를 포함하는 dict여야 한다.
    규격에 맞지 않으면 불완전한 상태로 렌더링하지 않고 ``ConsoleError INVALID_INPUT`` 예외를 발생시킨다.
    """
    section = status.get(key)
    if section is None:
        return None
    _require(isinstance(section, dict) and isinstance(section.get('schema_version'), str)
             and isinstance(section.get(items_key), list), f'{key} must be a read API result')
    return section


def _latest_reports(reports):
    """target_id -> (해당 대상의 최신 장부 항목, 해당 대상의 누적 보고 건수) 매핑을 생성한다.

    최신 보고는 (reported_at_ns, received_at_ns, report_id) 튜플의 최댓값으로 결정된다. 보고 시각이 우선하며,
    수신 시각과 고유 ID를 순차 비교하므로 동일 시각이라도 항상 유일한 최신 항목이 결정론적으로 선택된다.
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
    """위치 정보 딕셔너리를 복사하고 데이터 출처(``source``: 'target_state' 또는 'report')를 명시한다."""
    if position is None:
        return None
    return {'x': position['x'], 'y': position['y'], 'frame_id': position['frame_id'],
            'map_revision': position['map_revision'], 'source': source}


def _latest_candidates(site_events):
    """target_id -> 대상을 가리키는 최신 TARGET_DISAPPEARED_CANDIDATE 이벤트(후보 단계이며 로봇 재확인 필요)."""
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
    """장부 보고 및 대상 상태를 취합하여 대상별 1행을 구성하고 target_id 기준으로 정렬한다.

    ``ledger``가 None이면 보고 장부가 비활성화된 상태다. 보고 관련 열은 0이 아닌 None으로 설정한다.
    """
    reports = {} if ledger is None else _latest_reports(ledger['reports'])
    rows = []
    for target_id in sorted(set(reports) | set(target_states)):
        item, report_count = reports.get(target_id, (None, 0))
        payload = None if item is None else item['payload']
        state = target_states.get(target_id)
        warnings = []
        report_confirmed = None if payload is None else payload['confirmation_result'] == 'CONFIRMED'
        # 대상 상태(operations)가 우선권을 갖는다. 최신 보고 데이터는 상태가 존재하지 않을 때만 대체한다.
        # 양자 간 불일치나 누락된 입력은 행 단위 경고로 기록하며, 묵묵히 한쪽을 취사선택하지 않는다.
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
        # 운영자 승인 대상 revision(SubmitOperatorCommand expected_target_revision). operations가
        # 원장에 영속화하면 대상 상태에 반영되며, 미기록 상태에서는 None으로 출동 버튼이 비활성화된다.
        revision = None if state is None else state.get('target_revision')
        _require(revision is None or (type(revision) is int and revision >= 0), 'target_revision')
        # 해당 revision에 대해 PERSISTED 영수증이 발급된 보고 식별자(TargetEstimate
        # persisted_report_id, #118 선택 필드). 영수증 부재 시 조정기가 출동을 거부한다. 필드 도입 전 레코드는
        # None으로 파싱되며, 아래 장부 영수증을 임의로 유추하지 않는다.
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
    """배치 로봇 목록(``allowed_robot_ids``) 순서대로 로봇 상태 행을 구성한다. 경과 시간은 ``built_at_ns`` 기준이다.

    배치 외 로봇 상태는 ``warnings``에 기록하고 무시한다. 동일 로봇의 상태가 중복 수신되면
    ``ConsoleError INVALID_INPUT`` 예외를 발생시킨다. 미보고 배치 로봇도 행을 유지하여
    (``has_status`` False, 필드 None, ``stale`` True) 통신 두절 로봇이 UI에서 누락되지 않도록 한다.
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
            # 배터리 경과 시간은 로봇 발행 시각과 호스트 시계를 비교한다. 시계 동기화가 전제되어야 유효하다(C-03).
            'battery_age_s': None if battery_stamp is None else (built_at_ns - battery_stamp) / 1e9,
            'active_task_id': status.get('active_task_id'),
            'stamp_ns': stamp,
            'age_s': None if stamp is None else (built_at_ns - stamp) / 1e9,
            # 장부 수신 시각 기준 경과 시간: 로봇 재부팅으로 로봇 시계가 역행하더라도 신뢰할 수 있는 단조 증가 기준이다.
            'received_age_s': received_age,
            # 낡음 판정은 로봇 시각이 아닌 장부 수신 시각을 기준으로 수행한다. 수신 시각 미확인 항목은 stale로 처리한다.
            'stale': received_age is None or received_age > STATUS_STALE_S,
        })
    return rows


def _site_events(raw_events):
    """원시 현장 이벤트 dict를 계약 ``SiteEvent`` 객체로 인스턴스화하여 계약 유효성 검증을 강제한다.

    계약 규격에 어긋나는 행(정의되지 않은 필드, 누락 필드: TypeError, 값 범위 위반: RescueError)은
    개별 건너뛰기 대신 전체 화면에 대해 ``ConsoleError INVALID_INPUT`` 예외를 발생시킨다.
    """
    events = []
    for raw in raw_events:
        try:
            events.append(SiteEvent(**raw))
        except (TypeError, RescueError) as exc:
            raise ConsoleError('INVALID_INPUT', f'site event: {exc}') from exc
    return events


def _events(ledger, site_events):
    """사건 이력 테이블: 장부 보고와 현장 이벤트를 단일 행 스키마로 통합하여 최신 순으로 정렬한다.

    (상위 EVENT_ROWS개 행, 전체 건수)를 반환한다. 해당 이벤트 종류에 없는 필드는 None으로 채운다. 현장 이벤트는
    ``received_at_ns``가 제공되지 않으므로 그 값이 없다. ``event_ns``는 KPI-04 표시 지연 측정 기준인
    관측 시각이다.
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
    # 시각이 동일한 경우 ref_id를 2차 정렬 키로 사용하여 매 폴링마다 결정론적인 정렬 순서를 보장한다.
    rows.sort(key=lambda row: (row['event_ns'], row['ref_id']), reverse=True)
    return rows[:EVENT_ROWS], len(rows)


MISSION_ROW_KEYS = frozenset({'mode', 'phase', 'trigger_event_id', 'started_at_ns', 'updated_at_ns', 'reason',
                              *DISPATCH_KEYS})


def _mission_row(status):
    """read_mission_state() 조회 결과를 (섹션 존재 여부, 미션 행 또는 None) 튜플로 반환한다."""
    section = status.get('mission_state')
    if section is None:
        return False, None
    _require(isinstance(section, dict) and isinstance(section.get('schema_version'), str) and 'mission' in section,
             'mission_state must be a read API result')
    row = section['mission']
    _require(row is None or (isinstance(row, dict) and MISSION_ROW_KEYS <= set(row)), 'mission_state row')
    return True, row


# SystemState.reason 코드를 사용자 친화적 한국어 기술 용어로 변환한다(2026-10-13). {support}/{scout}: 지원/정찰 로봇의
# 화면 표시명 매핑. level: info(정상 진행), caution(주의/대기), alert(심각: 실패 및 재시도 불가).
# scope: 미션 배너 외 추가 표시 영역(지원/정찰 카드, 출동 대상 등).
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
    'AUTO_SEARCH_ENABLED': ('자동 수색 — 지진 감지나 재난 선포로 시작, 수색 시작 버튼은 받지 않음', 'info'),
    # 운영자 재난 선포(DECLARE_DISASTER, #330). 거절 답은 TASK_BUSY 이고 이 사유는 명령 기록·임무 띠에 실린다.
    'DISASTER_DECLARED_BY_OPERATOR': ('운영자 선포 — 지진 감지가 아니라 재난 선포 버튼으로 재난 모드', 'info'),
    'ALREADY_DISASTER': ('이미 재난 모드입니다 — 재난 선포 거절', 'info'),
    'MAP_FREEZE_WAITING_EXIT': ('출구 QR 찾는 중 — {support} 출동 대기', 'info'),
    # #234 (OPS-147): QR 출구는 TB-5 출동과 출구 이동을 막고, 출발 자세는 순찰과 TB-6 복귀를 막는다.
    'TARGET_REBASED_ON_FREEZE': ('대상 위치를 고정 지도로 옮김 — 보고 저장 대기', 'info'),
    'EXIT_NOT_ON_FROZEN_MAP': ('출구(QR) 위치가 고정 지도 기준에 없음 — {support} 출동·출구 이동 보류', 'caution'),
    'SUPPORT_EXIT_UNREGISTERED': ('출구(QR) 미등록 — {support} 출동·출구 이동 보류', 'caution'),
    'START_POSE_UNKNOWN': ('{scout} 출발 자세 모름 — 순찰·{scout} 복귀 보류', 'caution'),
    'START_POSE_CONFLICT': ('{scout} 출발 자세가 다른 값으로 다시 옴 — 처음 값 유지, 확인 필요', 'alert'),
    'PATROL_EXHAUSTED': ('{scout} 순찰 끝(정한 바퀴 다 돎) — 더 돌지 않음', 'caution'),
    # #247/#248 (OPS-174): 프런티어 탐색, 완성 지도(두 번째 고정), TB-5 출구 길(격자 경로).
    'MAP_NOT_FINAL': ('완성 지도(두 번째 고정) 대기 — {scout} 복귀·{support} 출구 이동 보류', 'caution'),
    'SUPPORT_START_UNKNOWN': ('출구(QR) 없음·{support} 시작점 모름 — {support} 출구 이동 보류', 'caution'),
    'SUPPORT_EXIT_UNREACHABLE': ('출구로 갈 길 없음 — {support} 출구 이동 보류', 'caution'),
    'NO_SURVIVOR_FOUND': ('생존자 없음 — 탐색 완료, {support} 출동 안 함', 'info'),
    'EXPLORATION_COMPLETE': ('{scout} 지도 전체 탐색 완료', 'info'),
    'EXPLORATION_TIMEOUT': ('탐색 시간 초과 — 탐색 종료', 'caution'),
    'EXPLORATION_MAP_UNKNOWN': ('{scout} 탐색 지도 아직 없음 — 탐색 보류', 'caution'),
    'SCOUT_POSE_UNKNOWN': ('{scout} 위치 모름 — 탐색 보류', 'caution'),
    # 두 로봇 교통 관제(#272, OPS-235). 두 로봇이 다 멈추는 YIELD_NO_SPACE·STOP_TIMEOUT 은 빨강(#272 웹 요청).
    # SEPARATION_UNCHECKED 는 멈춤이 아니라 간격 검사가 꺼진 것을 알리는 표시다.
    'TRAFFIC_SUPPORT_WAITING_SCOUT_STOP': ('{scout} 정지 확인 기다림 — {support} 대기', 'caution'),
    # 정지를 확인하지 못했을 뿐이라 TB-6 이 아직 움직일 수 있다(OPS-237): 현장 확인을 문구에 넣는다.
    'TRAFFIC_SCOUT_STOP_TIMEOUT': ('{scout} 정지 확인 안 됨(시간 초과) — {support} 대기, 현장 확인', 'alert'),
    'TRAFFIC_SUPPORT_WAITING_SCOUT_YIELD': ('{scout} 이 {support} 경로 밖으로 비키는 중 — {support} 대기', 'caution'),
    'TRAFFIC_SCOUT_YIELD_NO_SPACE': ('{scout} 이 비킬 자리 없음 — 두 로봇 모두 정지', 'alert'),
    'TRAFFIC_SCOUT_HELD_FOR_SUPPORT': ('{support} 경로와 겹쳐 {scout} 명령 보류', 'caution'),
    'TRAFFIC_ROBOTS_TOO_CLOSE': ('두 로봇이 너무 가까움 — 새 명령 보류', 'caution'),
    'TRAFFIC_SUPPORT_PATH_UNKNOWN': ('고정 지도에서 {support} 경로를 찾지 못함 — 대기', 'caution'),
    'TRAFFIC_SCOUT_POSE_UNKNOWN': ('{scout} 위치 확인 안 됨 — 대기', 'caution'),
    'TRAFFIC_SUPPORT_POSE_UNKNOWN': ('{support} 위치 확인 안 됨 — 대기', 'caution'),
    'TRAFFIC_SEPARATION_UNCHECKED': ('{support} 출발 자리 미측정 — 두 로봇 간격 검사 꺼짐', 'info'),
}
# APPROACH_FAILED_<why> (#216): 실행기 종료 사유, 미정의 시 실행 결과 코드.
# 정의되지 않은 사유 코드는 코드명을 그대로 노출하여 정보를 은폐하지 않는다(OPS-127).
APPROACH_WHY = {
    'PASSAGE_BLOCKED': '통로 막힘', 'UNREACHABLE': '갈 수 없는 위치', 'FAILED': '실행 실패',
    'CANCELED': '취소됨', 'EXPIRED': '시간 초과', 'NAV_NOT_READY': '주행 준비 안 됨',
    'DEADLINE_EXPIRED': '시간 초과', 'STOP_LATCHED': '정지 유지 중', 'LOCAL_LEASE_EXPIRED': '명령 유효 시간 지남',
    'CLOCK_UNCERTAIN': '시계 불확실',
}
EXIT_COUNTDOWN = re.compile(r'SUPPORT_EXIT_IN_(\d+)S')
PATROL_ROUND = re.compile(r'PATROL_ROUND_(\d+)')
# RECHECK_NEEDED_<why>: 재확인 결과 구출을 확정하지 못한 원인(C-11 7-11행).
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
    """SystemState.reason 코드를 {code, text, level, scope} 구조로 변환한다. 미등록 코드는 원본을 유지한다."""
    if code is None:
        return None
    # 해당 역할의 로봇이 식별되지 않으면 robot_id를 알 수 없으므로 임의 명칭 대신 역할 코드(SUPPORT/SCOUT)를 사용한다.
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


CAUSE_WORDS = {'SIGN': '지진 감지', 'OPERATOR': '운영자 선포', 'UNKNOWN': '출처 모름'}


def mission_cause(trigger_event_id, site_events, commands):
    """재난 모드를 연 것(작전 2026-10-15). trigger_event_id 와 같은 command_id 의 DECLARE_DISASTER 명령이
    접수됐으면 OPERATOR, 같은 event_id 의 DISASTER_SIGN 사건이 있으면 SIGN, 둘 다 아니면 UNKNOWN 이다.
    SystemState 의 reason(DISASTER_DECLARED_BY_OPERATOR)은 다른 사유가 있으면 실리지 않아 쓰지 않는다."""
    if trigger_event_id is None:
        return None
    if any(c['command_id'] == trigger_event_id and c['kind'] == 'DECLARE_DISASTER' and c['accepted'] is True
           for c in commands):
        return 'OPERATOR'
    if any(e.event_id == trigger_event_id and e.kind == 'DISASTER_SIGN' for e in site_events or ()):
        return 'SIGN'
    return 'UNKNOWN'


def _mission(site_events, row, commands=()):
    """임무 배지 데이터 모델 구성. mission_state 레코드(operations가 SystemState를 기록함, #111)가 있으면
    해당 레코드를 적용한다. 최초 레코드 수신 전에는 DISASTER_SIGN 이벤트로 모드를 추정하고 출처(`source`)를 명시한다.
    둘 다 없으면 미수신 상태(None)를 반환한다."""
    signs = sorted((e for e in site_events or () if e.kind == 'DISASTER_SIGN'),
                   key=lambda e: (e.observed_at_ns, e.event_id))
    base = {'sign_count': len(signs), 'phase': None, 'reason': None, 'dispatch': None, 'updated_at_ns': None}
    if row is not None:
        dispatch = None if row['target_id'] is None else {key: row[key] for key in DISPATCH_KEYS}
        cause = mission_cause(row['trigger_event_id'], site_events, commands)
        return base | {'mode': row['mode'], 'label': label('mode', row['mode']), 'source': 'mission_state',
                       'trigger_event_id': row['trigger_event_id'], 'trigger_ns': row['started_at_ns'], 'cause': cause,
                       'phase': row['phase'], 'reason': row['reason'], 'dispatch': dispatch,
                       'updated_at_ns': row['updated_at_ns']}
    if site_events is None:
        return base | {'mode': None, 'label': '모름', 'source': None, 'trigger_event_id': None, 'trigger_ns': None,
                       'cause': None}
    mode = 'DISASTER' if signs else 'NORMAL'
    return base | {'mode': mode, 'label': MODES[mode], 'source': 'site_events',
                   'trigger_event_id': signs[0].event_id if signs else None,
                   'trigger_ns': signs[0].observed_at_ns if signs else None, 'cause': 'SIGN' if signs else None}


SUPPORT_DISPATCH_MODES = ('AUTO', 'OPERATOR')   # --support-dispatch: 조정기와 공유하는 현장 운영 정책
SEARCH_START_MODES = ('AUTO', 'OPERATOR')       # --search-start: 조정기와 공유하는 현장 운영 정책(#228)


def _command_words(support_name):
    """명령 레이블 구성. 출동 승인 명령에는 SUPPORT 로봇의 화면 표시명을 반영한다(소유자 2026-10-13: TB-5)."""
    words = dict(COMMANDS)
    if support_name is not None:
        words['DISPATCH_SUPPORT'] = f'{support_name} 출동 승인'
    return words


def _history(status, words=None):
    """operations가 기록한 운영자 명령 이력(응답 및 거절 사유 포함)과
    operations의 error_log를 통합하여 최신 순으로 정렬한다. 두 입력 모두 없으면(픽스처 모드) None 반환.
    사건 이력과 분리 운영된다: 사건 이력은 KPI-04 지연 메트릭을 포함하지만 명령 이력은 포함하지 않는다."""
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
    """선택지 항목 구성: 대상 ID, 화면에 표시된 revision(DISPATCH_SUPPORT 전달용), 대상 단계 정보."""
    return {'target_id': target['target_id'], 'target_revision': target['target_revision'],
            'stage': target['stage'], 'stage_label': target['stage_label']}


def _gate(name, mission, targets):
    """단일 명령의 (enabled, reason, args, choices) 판정. rescue_coordination.mission.handle_command의 게이트 로직을 반영한다."""
    def off(reason):
        return False, reason, {}, []
    # 평가 순서가 핵심이다: 최초로 걸린 검사 조건이 버튼 하단 안내 사유로 표시된다.
    # 1. 보류 명령은 상태와 무관하게 즉시 비활성화한다(commands.py 전송 차단).
    if name in HELD_COMMANDS:
        return off(HELD_COMMANDS[name])
    # 2. mission_state 레코드가 없으면 조정기 상태를 알 수 없다. DISASTER_SIGN 이벤트 기반 추정 모드는
    #    (source 'site_events') 배지에는 표시되나 명령 버튼을 활성화하지 않는다.
    if mission['source'] != 'mission_state':
        return off('임무 상태 없음 — operations 가 조정기 상태를 기록하면 켜짐')
    # 재난 선포는 평시(NORMAL)에만 켠다. 조정기는 이미 재난이면 TASK_BUSY(사유 ALREADY_DISASTER)로 거절한다(작전 2026-10-15).
    if name == 'DECLARE_DISASTER':
        if mission['mode'] == 'NORMAL':
            return True, '평시 — 누르면 재난 모드로 들어감', {}, []
        return off('이미 재난 모드' if mission['mode'] == 'DISASTER' else f"평시 아님 ({mission['label']})")
    # 3. DISASTER 모드가 아니면 handle_command가 모든 요청에 대해 MODE_NOT_DISASTER로 거부한다.
    if mission['mode'] != 'DISASTER':
        return off(f"재난 모드 아님 ({mission['label']})")
    # 4. 개별 명령별 전이 조건 검증. 각 명령에 대응하는 조정기 핸들러 로직과 일치시킨다.
    phase, dispatch = mission['phase'], mission['dispatch']
    if name == 'START_SEARCH':
        # phase DISASTER는 조정기의 '재난 감지 후 수색 시작 전' 초기 단계다(mission.phase).
        return (True, '재난 감지 — 아직 수색 전', {}, []) if phase == 'DISASTER' else off('이미 수색을 시작함')
    if name == 'DISPATCH_SUPPORT':
        if dispatch is not None:
            return off('이미 출동함')
        # 확정 상태이며 아직 구출되지 않은 대상: 지원 로봇 출동이 허용되는 대상 목록이다.
        confirmed = [t for t in targets if t['detection_confirmed'] and t['stage'] != 'RESCUED']
        if not confirmed:
            return off('확정 대상 없음')
        # 운영자는 화면에 표시된 revision을 승인한다. revision 정보가 없는 대상은 선택지로 제시하지 않는다(null 전송 방지).
        revised = [t for t in confirmed if t['target_revision'] is not None]
        if not revised:
            return off('대상 revision 없음 — 대상 상태에 revision 이 기록되면 켜짐')
        # mission.py: 확정 상태이면서 report_persisted 영수증이 존재해야 하며, 미충족 시 TARGET_NOT_CONFIRMED.
        choices = [_choice(t) for t in revised if t['persisted_report_id'] is not None]
        if not choices:
            return off('보고 저장 대기 — 조정기가 저장 영수증을 받으면 켜짐')
        # 영수증 누락으로 제외된 대상 수를 사유 메시지에 표기하여 선택지 누락 원인을 명확히 안내한다.
        waiting = len(revised) - len(choices)
        return (True, f'확정 대상 {len(choices)}명 중 고름' + (f' (보고 저장 대기 {waiting}명 빠짐)' if waiting else ''),
                {}, choices)
    if name == 'CONTINUE_SEGMENT':
        if dispatch is None:
            return off('출동 전')
        if dispatch['segment_phase'] != 'AWAITING_APPROVAL':
            return off(f"승인 대기 아님 ({label('segment', dispatch['segment_phase'])})")
        # 화면의 파라미터를 그대로 회신한다. 그 사이 상태가 변경되었으면 조정기가 안전하게 거절한다
        # (TARGET_REVISION_STALE, ROUTE_REVISION_STALE, SEGMENT_INDEX_MISMATCH).
        args = {key: dispatch[key] for key in ('target_id', 'target_revision', 'route_revision', 'segment_index')}
        return (True, f"구간 {dispatch['segment_index']}/{dispatch['segment_count']} 끝 — 다음 구간 승인 대기",
                args, [])
    if name == 'REQUEST_RECHECK':
        # RESCUE_CONFIRMED로 최종 확정되지 않은 모든 대상이다. 대상 상태 레코드가 없는 대상도 포함한다(C-11 10행:
        # 도착 및 출동 여부는 재확인 지시의 필수 선행 조건이 아님). UNKNOWN_TARGET 및 RECHECK_ALREADY_REQUESTED는
        # 조정기가 최종 판정하여 거절한다.
        open_targets = [_choice(t) for t in targets if t['rescue_state'] != 'RESCUE_CONFIRMED']
        if not open_targets:
            return off('재확인할 대상 없음')
        return True, f'대상 {len(open_targets)}명 중 고름', {}, open_targets
    if name == 'RETURN_HOME':
        # 조정기와 동일한 검증 순서: ALREADY_RETURNING 검사 후 NO_VERDICT 검사(판정 phase 진입 시 판정 완료로 간주).
        if phase == 'RETURNING':
            return off('이미 복귀 중')
        if phase not in VERDICT_PHASES:
            return off('구출 판정 전')
        return True, '판정 뒤 — 두 로봇 복귀', {}, []
    # 등록되지 않은 COMMANDS 항목은 안내 없는 버튼을 생성하지 않고 즉시 예외를 발생시킨다.
    raise ConsoleError('INVALID_INPUT', f'no gate for command {name}')


def _declare_confirm(search_start, scout_name):
    scout = scout_name or 'SCOUT'
    if search_start == 'AUTO':
        return f'재난 선포 — 재난 모드로 들어가고 {scout} 이 바로 탐색을 시작합니다'
    if search_start == 'OPERATOR':
        return f'재난 선포 — 재난 모드로 들어갑니다. {scout} 탐색은 수색 시작 버튼으로 시작합니다'
    return '재난 선포 — 재난 모드로 들어갑니다'


def command_gates(mission, targets, words=None, support_dispatch=None, support_name=None, search_start=None,
                  scout_name=None):
    """모든 운영자 명령의 버튼 활성화 상태를 계산한다. 이는 프론트엔드 안내용 게이트 복제본이며
    권한 인가 그 자체가 아니다. 활성화된 명령도 조정기에서 거절될 수 있으며 해당 응답은 그대로 표시된다."""
    words = dict(COMMANDS) if words is None else words
    rows = []
    for name, _ in COMMANDS:
        enabled, reason, args, choices = _gate(name, mission, targets)
        if name == 'DISPATCH_SUPPORT' and support_dispatch == 'AUTO':
            # 조정기가 지원 로봇을 자동 파견하므로 수동 명령은 거부된다(NOT_AUTHORIZED).
            # 버튼이 아닌 자동 상태 표시로 전환된다(소유자 결정 2026-10-13). 본 행에서는 _gate 결과를 대체한다.
            # `auto: True` 설정 시 render.py가 data-command 없는 안내 텍스트로 렌더링한다.
            auto = f'{support_name or "SUPPORT"} 자동 출동'
            rows.append({'name': name, 'label': auto, 'group': COMMAND_GROUPS[name], 'enabled': False,
                         'reason': f'{auto} — 조정기가 보냄', 'args': {}, 'choices': [], 'auto': True})
            continue
        if name == 'START_SEARCH' and search_start == 'AUTO':
            # 지진 감지나 재난 선포로 재난 모드가 되면 수색이 자동 시작되므로 수동 명령은 거부된다(AUTO_SEARCH_ENABLED).
            auto = f'{scout_name or "SCOUT"} 자동 출발'
            rows.append({'name': name, 'label': auto, 'group': COMMAND_GROUPS[name], 'enabled': False,
                         'reason': f'{auto} — 지진 감지나 재난 선포로 조정기가 수색 시작', 'args': {}, 'choices': [],
                         'auto': True})
            continue
        row = {'name': name, 'label': words[name], 'group': COMMAND_GROUPS[name], 'enabled': enabled,
               'reason': reason, 'args': args, 'choices': choices}
        if name == 'DECLARE_DISASTER':
            # 확인 창 첫 줄. 무엇이 움직이는지 보내기 전에 한 번 더 보인다(작전 2026-10-15).
            row['confirm'] = _declare_confirm(search_start, scout_name)
        rows.append(row)
    return rows


PASSAGE_WORDS = {'PASSABLE': '통과 가능', 'BLOCKED': '통과 불가', 'UNKNOWN': '판정 불가'}   # OPS-196 지도·카메라 공통


def _passages(section, warnings):
    """터널 판정(#262 read_overhead_passages, OPS-189·196): 지도에 그릴 행. 지금 지도(current_map 의 session·revision)와
    같은 항목은 current True(진하게), 다르면 False(흐리게). current_map 을 모르면 전부 진하게 그리고 그 사실을 적는다."""
    if section is None:
        return []
    current = section.get('current_map')
    rows = []
    for listing in section['robots'].values():
        for item in listing['items']:
            word = PASSAGE_WORDS.get(item['state'], item['state'])
            label = word if item['lowest_m'] is None or item['state'] == 'UNKNOWN' else f'{word} ↑{item["lowest_m"]:.2f} m'
            rows.append({'id': item['id'], 'robot_id': listing['robot_id'], 'state': item['state'], 'label': label,
                         'x': item['x'], 'y': item['y'], 'frame_id': listing['map_frame'],
                         'map_revision': item['map_revision'], 'last_seen_ns': item['last_seen_ns'],
                         'current': current is None or (item['map_session_id'], item['map_revision'])
                         == (current['map_session_id'], current['map_revision'])})
    if rows and current is None:
        warnings.append('현재 지도 revision 모름 — 터널 판정을 모두 진하게 그림')
    return rows


def _exit(section):
    """최신 exit_qr 관측 정보(#237, 최신 순 정렬, VALID 판정만) 또는 None 반환. 단순 센서 관측치이며
    조정기에 등록 확정된 출구와는 구별된다(미등록 시 SUPPORT_EXIT_UNREGISTERED 사유로 노출됨)."""
    if section is None or not section['exits']:
        return None
    row = section['exits'][0]
    return {'location_id': row['location_id'], 'x': row['x'], 'y': row['y'], 'frame_id': row['mission_frame'],
            'observed_at_ns': row['observed_at_ns'], 'map_revision': row['map_revision']}


def _apply_watch(robots, watch, built_at_ns, warnings):
    """--robot-watch: 로봇 기본 토픽 직접 관측을 행마다 ``watch`` 로 붙인다(재범 2026-10-13). 프로젝트 상태가 먼저이고,
    상태가 없는 로봇은 낡지 않은 직접 관측 위치를 ``pose`` 로 쓴다. 낡음은 이 호스트 시계로 잰다(관측 노드와 같은 PC).
    직접 관측의 'map' 은 로봇마다 자기 localization 의 map 이다. 보이는 지도의 주인(``map_robot``)이 아닌 로봇은
    frame 을 ``<ns> map`` 으로 바꿔 지도에 그리지 않는다(현장 2026-10-14, TB-5 가 TB-6 지도 원점에 겹침)."""
    for row in robots:
        row['watch'] = None
    if watch is None:
        return
    data, stale_s = watch['data'], watch['stale_s']
    if data is None:
        warnings.append(watch['problem'])
        return
    written_age = (built_at_ns - data['written_at_ns']) / 1e9
    dead = written_age > stale_s
    if dead:
        warnings.append(f'로봇 직접 관측 파일이 {written_age:.0f}초 동안 갱신 안 됨 — robot_watch 노드 확인')
    deployed = {row['robot_id'] for row in robots}
    for rid in sorted(set(data['robots']) - deployed):
        warnings.append(f'배치 밖 로봇 직접 관측 무시: {rid!r}')   # 빠진 것도 보인다(배치 밖 로봇 상태와 같음)
    for row in robots:
        seen = data['robots'].get(row['robot_id'])
        if seen is None:
            continue
        age = None if seen.get('last_seen_ns') is None else (built_at_ns - seen['last_seen_ns']) / 1e9
        pose_age = None if seen.get('pose_seen_ns') is None else (built_at_ns - seen['pose_seen_ns']) / 1e9
        stale = dead or age is None or age > stale_s
        pose = seen.get('pose') if not dead and pose_age is not None and pose_age <= stale_s else None
        own_map = pose is not None and row['robot_id'] != watch['map_robot']
        if own_map:
            pose = dict(pose, frame_id=f"{seen.get('namespace')} {pose['frame_id']}")   # 옮기지 않음
        row['watch'] = {'namespace': seen.get('namespace'), 'age_s': age, 'stale': stale, 'own_map': own_map,
                        'battery_percent': seen.get('battery_percent'), 'docked': seen.get('docked'), 'pose': pose}
        if not row['has_status'] and pose is not None:
            row['pose'] = pose


def build_view(ledger, status, *, built_at_ns, robot_names=None, support_dispatch=None, search_start=None,
               robot_watch=None):
    """단일 폴링 주기의 모든 입력 소스를 종합하여 전체 화면 모델(`/api/view` JSON)을 빌드한다.

    ledger: read_reports() 결과. 보고 원장이 미제공된 경우 None(원장 없이 --site-state 단독 실행,
    2026-10-13). 이 경우 보고 행 및 건수를 비활성화하고 warnings에 사유를 명시한다.
    status: {'allowed_robots', 'robot_status', 'site_events', 'target_states', 'mission_state',
    'operator_commands', 'error_log', 'exit_locations'}. operations 읽기 API 결과 딕셔너리다(미수신 시
    None). support_dispatch / search_start: 명령행으로 전달된 조정기 현장 운영 모드 플래그다
    (AUTO 설정 시 버튼 대신 자동 상태 메시지를 표시한다).
    입력 형식 위반 시 ``ConsoleError INVALID_INPUT`` 예외를 발생시킨다. 누락된 섹션은 경고로 보고하며
    (``warnings`` 또는 ``history['missing']``), 임의의 기본값으로 채우지 않는다.
    """
    _require(type(built_at_ns) is int and built_at_ns >= 0, 'built_at_ns must be int ns')
    names = dict(robot_names or {})   # robot_id -> 화면 표시명(--robot-name). 미설정 시 robot_id 원본 사용
    _require(support_dispatch in (None,) + SUPPORT_DISPATCH_MODES,
             f'support_dispatch must be one of {SUPPORT_DISPATCH_MODES} or unset')
    _require(search_start in (None,) + SEARCH_START_MODES, f'search_start must be one of {SEARCH_START_MODES} or unset')
    warnings = []
    robot_section = _section(status, 'robot_status', 'robots')
    allowed_section = _section(status, 'allowed_robots', 'allowed_robot_ids')
    if ledger is None and allowed_section is not None:
        # 현장 상태 DB가 배치 로봇을 결정한다(read_allowed_robot_ids, #149).
        warnings.append('보고 장부 입력 없음 — 구조 보고 행·건수를 표시하지 않음')
        allowed = tuple(allowed_section['allowed_robot_ids'])
    elif ledger is None:
        # 로봇 목록이 정의되지 않은 픽스처: 상태가 수신된 로봇만으로 구성한다.
        warnings.append('보고 장부 입력 없음 — 구조 보고 행·건수를 표시하지 않음, 로봇 목록은 상태가 들어온 로봇만')
        allowed = tuple(sorted({r.get('robot_id') for r in ([] if robot_section is None else robot_section['robots'])}))
    else:
        # 원장이 존재하면 원장에 정의된 로봇 목록을 우선 사용하며 현장 상태 목록은 참조하지 않는다.
        allowed = tuple(ledger['allowed_robot_ids'])
    site_section = _section(status, 'site_events', 'site_events')
    target_section = _section(status, 'target_states', 'targets')
    has_mission, mission_row = _mission_row(status)
    # 누락된 섹션은 빈 리스트가 아닌 명시적 경고로 노출한다('데이터 미수신'이 '정상 공백'으로 오인되는
    # 현상을 방지함). `has_mission or None`: 행이 None이라도 mission_state 섹션 자체가 존재하면 경고를 생략한다.
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
    # API 모델에는 보존하되 화면에 렌더링하지 않는다. 고정 카메라 1대 환경에서 EXIT_PASS 이벤트 발행처가 없어
    # 카운트가 0에 고정된다(2026-10-09 결정, console-and-db-plan 3절, db-schema 2절). 사양 3.5.5의
    # '구출 수 <= 출구 통과 수' 불일치 경고도 제외되었다. 구출 상태 전이는 카메라 소실 후보 이벤트로 트리거된다.
    exit_count = None if site_section is None else site_section['exit_pass_count']
    _require(exit_count is None or (type(exit_count) is int and exit_count >= 0), 'exit_pass_count')
    stage_counts = {stage: 0 for stage in STAGES}
    for row in targets:
        stage_counts[row['stage']] += 1
    events, event_total = _events(ledger, site_events)
    seen_commands = [row for key in ('operator_commands', 'trigger_commands')
                     for row in (_section(status, key, 'commands') or {'commands': []})['commands']]
    mission = _mission(None if site_section is None else site_events, mission_row, seen_commands)
    robots = _robots(allowed, [] if robot_section is None else robot_section['robots'], built_at_ns, warnings)
    _apply_watch(robots, robot_watch, built_at_ns, warnings)
    for row in robots:
        row['name'] = names.get(row['robot_id'], row['robot_id'])
    # 역할 표시명은 해당 역할을 보고한 로봇이 단 1대일 때만 적용한다(역할 정보는 robot_status에서 수신).
    # 없거나 복수 대인 경우 추측하지 않고 기본 역할 코드(SUPPORT/SCOUT)를 노출한다.
    support = [row['name'] for row in robots if row['role'] == 'SUPPORT']
    words = _command_words(support[0] if len(support) == 1 else None)
    scout = [row['name'] for row in robots if row['role'] == 'SCOUT']
    mission['reason_view'] = mission_reason(mission['reason'], support[0] if len(support) == 1 else None,
                                            scout[0] if len(scout) == 1 else None)
    history = _history(status, words)
    refused = [row['reason'] for row in (history or {'rows': []})['rows']] + [mission['reason']]
    if search_start != 'AUTO' and 'AUTO_SEARCH_ENABLED' in refused:
        # 조정기는 search_start AUTO 모드로 동작 중이나, 콘솔은 수동 시작 버튼을 제시하는 불일치 상황 안내.
        warnings.append('조정기는 자동 수색(search_start AUTO) — 관제 웹을 --search-start AUTO 로 다시 띄우세요')
    exits = _section(status, 'exit_locations', 'exits')
    passages = _passages(status.get('overhead_passages'), warnings)
    return {
        'built_at_ns': built_at_ns,
        'exit': _exit(exits),
        'passages': passages,
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
