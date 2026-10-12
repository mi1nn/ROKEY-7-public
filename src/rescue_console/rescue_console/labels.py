"""Korean display words for the codes the screen shows. Pure data; no I/O.

The code itself is never hidden: `label` returns an unknown code unchanged, so a value
added upstream shows up as its raw code instead of disappearing or being renamed.

Where each vocabulary comes from (2026-10-10, main 7a940e9):
- role: `rescue_contracts.ROBOT_ROLES` + `UNKNOWN` (`rescue_operations.site_state._robot_status`).
- motion: free text in site_state (<= 64 chars, no producer yet). Values from the SDD robot
  status (`docs/design/sdd-node-interface-spec-v0.1.md`: STOPPED/MOVING/STOPPING/STOP_UNCONFIRMED)
  and `UNKNOWN` (`tests/test_operations_site_state.py`).
- readiness: free text in site_state, no producer yet. READY / NOT_READY / UNKNOWN from
  `tests/test_operations_site_state.py` and `examples/status.example.json`.
- rescue: `rescue_contracts.RESCUE_STATES`.
- support: `rescue_operations.site_state.SUPPORT_STATES`.
- approach / confirmation: report schema checks in `rescue_operations.schema` (inline tuples).
- receipt: `rescue_operations.ledger` receipt states (RECEIVED / PENDING_EVIDENCE / PERSISTED).
- event: `rescue_contracts.SITE_EVENT_KINDS` (TARGET_DISAPPEARED_CANDIDATE from #48, labelled ahead
  of its merge) + `REPORT`, the console's own row kind for ledger reports (`view._events`).
- mode: `rescue_operations.site_state.MISSION_MODES` (`rescue_contracts.SYSTEM_MODES`, S-01, plus
  SystemState MISSION_FINISHED).
- phase / segment: `rescue_operations.site_state.MISSION_PHASES` / `SEGMENT_PHASES` (SystemState, #88·#111).
- reply: the C-09 common codes that the coordinator puts in `SubmitOperatorCommand.reason_code`
  (`rescue_coordination.ros_node.REASON_CODES`, OPS-58) and NAV_NOT_READY for START_SEARCH (#98).
  The code is shown next to the word; operations may answer with other codes, shown as they come.
The Korean words are [제안] 관제·화면, 2026-10-10 (phase / segment / reply 2026-10-12).
"""

LABELS = {
    'role': {'SCOUT': '정찰', 'SUPPORT': '지원', 'UNKNOWN': '모름'},
    'motion': {'MOVING': '이동 중', 'STOPPING': '정지 중', 'STOPPED': '정지',
               'STOP_UNCONFIRMED': '정지 미확인', 'UNKNOWN': '모름'},
    'readiness': {'READY': '준비됨', 'NOT_READY': '준비 안 됨', 'UNKNOWN': '모름'},
    'rescue': {'NOT_CHECKED': '확인 전', 'RECHECK_NEEDED': '재확인 필요', 'RESCUE_CONFIRMED': '구출 확인'},
    'support': {'NONE': '없음', 'ASSIGNED': '배정', 'ARRIVED': '도착', 'FAILED': '실패'},
    'approach': {'NOT_ATTEMPTED': '접근 전', 'APPROACH_COMPLETE': '접근 완료', 'UNREACHABLE': '도달 불가',
                 'FAILED': '접근 실패', 'CANCELED': '접근 취소'},
    'confirmation': {'CONFIRMED': '확정', 'UNCONFIRMED': '미확정'},
    'receipt': {'RECEIVED': '수신', 'PENDING_EVIDENCE': '증거 대기', 'PERSISTED': '저장 완료'},
    'event': {'REPORT': '구조 보고', 'DISASTER_SIGN': '진동 감지', 'EXIT_PASS': '출구 통과',
              'TARGET_DISAPPEARED_CANDIDATE': '대상 소실 후보'},
    'mode': {'NORMAL': '평시', 'DISASTER': '재난', 'CLEARING': '해제 중', 'MISSION_FINISHED': '임무 종료'},
    'phase': {'IDLE': '대기', 'DISASTER': '재난 감지', 'SEARCHING': '수색 중', 'TARGET_FOUND': '요구조자 발견',
              'DISPATCH_APPROVED': '출동 승인됨', 'SEGMENT_MOVING': '구간 이동 중', 'AWAITING_CONTINUE': '계속 승인 대기',
              'ARRIVED': '도착', 'CANDIDATE': '구조 완료 후보', 'RECHECKING': '재확인 중',
              'RESCUE_CONFIRMED': '구출 확인', 'RECHECK_NEEDED': '재확인 필요', 'RETURNING': '복귀 중',
              'FINISHED': '종료'},
    'segment': {'IDLE': '대기', 'EVALUATING': '판단 중', 'AWAITING_APPROVAL': '승인 대기', 'EXECUTING': '이동 중',
                'STOPPING': '정지 중', 'STOPPED': '정지', 'UNKNOWN': '모름'},
    'reply': {'NOT_AUTHORIZED': '재난 모드가 아님', 'NOT_FOUND': '대상이나 복귀 위치를 모름',
              'PENDING_EVIDENCE': '확정·경로·판정이 아직 없음', 'TASK_BUSY': '이미 진행 중',
              'INVALID_INPUT': '화면의 값이 지금 상태와 다름', 'FRAME_REVISION_MISMATCH': '지도 기준이 다름',
              'NAV_NOT_READY': '로봇 준비 안 됨 — 준비 뒤 다시 누르세요', 'ID_CONTENT_CONFLICT': '같은 ID 에 다른 내용'},
}


def label(kind, code):
    """Korean word for `code`; the code unchanged when unknown; None stays None (shown as missing).

    An unknown `kind` is a programming error and raises KeyError.
    """
    table = LABELS[kind]
    if code is None:
        return None
    return table.get(code, code)
