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
- mode: `rescue_contracts.SYSTEM_MODES` (`docs/design/state-machines.md` S-01).
The Korean words are [제안] 관제·화면, 2026-10-10.
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
    'mode': {'NORMAL': '평시', 'DISASTER': '재난', 'CLEARING': '해제 중'},
}


def label(kind, code):
    """Korean word for `code`; the code unchanged when unknown; None stays None (shown as missing).

    An unknown `kind` is a programming error and raises KeyError.
    """
    table = LABELS[kind]
    if code is None:
        return None
    return table.get(code, code)
