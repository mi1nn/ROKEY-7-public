"""화면에 표시할 상태 코드의 한국어 명칭 매핑 모듈이다. I/O 작업 없는 순수 데이터 모듈이다.

원래 코드를 숨기지 않는다: `label`은 정의되지 않은 코드를 그대로 반환하므로, 상위 계층에서 새로 추가된
값도 누락이나 변형 없이 원본 코드 문자열 형태로 UI에 노출된다.

각 어휘의 출처(2026-10-10, main 7a940e9):
- role: `rescue_contracts.ROBOT_ROLES` 에 `UNKNOWN` 을 더한 것(`rescue_operations.site_state._robot_status`).
- motion: site_state 의 자유 문자열(<= 64자)이고, operations 가 RobotStatus.motion_state 에서 옮겨 쓴다
  (`rescue_operations.ros_ingest.robot_status_from_msg`). 값은 SDD 로봇 상태 명세
  (`docs/design/sdd-node-interface-spec-v0.1.md`: STOPPED/MOVING/STOPPING/STOP_UNCONFIRMED)에서 왔고,
  `UNKNOWN` 은 `tests/test_operations_site_state.py` 에서 왔다.
- readiness: site_state 의 자유 문자열이고, 같은 방식으로 RobotStatus.readiness 에서 옮겨 쓴다. READY / NOT_READY /
  UNKNOWN 은 `tests/test_operations_site_state.py` 와 `examples/status.example.json` 에서 왔다. RobotStatus 는
  DEGRADED 도 정의한다(`src/rescue_interfaces/msg/RobotStatus.msg`). 여기에는 아직 낱말이 없어 원본 코드 그대로 표시된다.
- rescue: `rescue_contracts.RESCUE_STATES`.
- support: `rescue_operations.site_state.SUPPORT_STATES`.
- approach / confirmation: `rescue_operations.schema` 의 보고 스키마 검사(코드 안에 적은 튜플).
- receipt: `rescue_operations.ledger` 의 영수증 상태(RECEIVED / PENDING_EVIDENCE / PERSISTED).
- event: `rescue_contracts.SITE_EVENT_KINDS`(TARGET_DISAPPEARED_CANDIDATE 는 #48 에서 왔고, 머지 전에
  낱말을 먼저 붙였다)에 `REPORT` 를 더한 것. `REPORT` 는 장부 보고에 쓰는 관제 웹 자체의 행 종류다(`view._events`).
- mode: `rescue_operations.site_state.MISSION_MODES`(`rescue_contracts.SYSTEM_MODES`, S-01 에
  SystemState 의 MISSION_FINISHED 를 더한 것).
- phase / segment: `rescue_operations.site_state.MISSION_PHASES` / `SEGMENT_PHASES` (SystemState, #88·#111).
- reply: 조정기가 `SubmitOperatorCommand.reason_code` 에 넣는 C-09 공통 코드
  (`rescue_coordination.ros_node.REASON_CODES`, OPS-58)와 START_SEARCH 의 NAV_NOT_READY(#98).
  화면에는 한국어 명칭과 원본 코드가 함께 표기된다. operations가 미정의 코드로 응답하면 수신된 코드를 그대로 표시한다.
한국어 명칭 정의: [제안] 관제·화면, 2026-10-10 (phase / segment / reply는 2026-10-12).
"""

# kind -> {코드: 한국어 명칭}. 각 kind는 상단에 명시된 도메인별 어휘 범주를 나타낸다.
LABELS = {
    'role': {'SCOUT': '정찰', 'SUPPORT': '지원', 'UNKNOWN': '모름'},
    'motion': {'MOVING': '이동 중', 'STOPPING': '정지 중', 'STOPPED': '정지',
               'STOP_UNCONFIRMED': '정지 미확인', 'UNKNOWN': '모름'},
    'readiness': {'READY': '준비됨', 'NOT_READY': '준비 안 됨', 'UNKNOWN': '모름'},
    'rescue': {'NOT_CHECKED': '확인 전', 'RECHECK_NEEDED': '재확인 필요', 'RESCUE_CONFIRMED': '구출 완료'},
    'support': {'NONE': '없음', 'ASSIGNED': '배정', 'ARRIVED': '도착', 'FAILED': '실패'},
    'approach': {'NOT_ATTEMPTED': '접근 전', 'APPROACH_COMPLETE': '접근 완료', 'UNREACHABLE': '도달 불가',
                 'FAILED': '접근 실패', 'CANCELED': '접근 취소'},
    'confirmation': {'CONFIRMED': '확정', 'UNCONFIRMED': '미확정'},
    'receipt': {'RECEIVED': '수신', 'PENDING_EVIDENCE': '증거 대기', 'PERSISTED': '저장 완료'},
    'event': {'REPORT': '구조 보고', 'DISASTER_SIGN': '진동 감지', 'EXIT_PASS': '출구 통과',
              'TARGET_DISAPPEARED_CANDIDATE': '대상 소실 후보',
              'PASSAGE_APPROVAL': '터널 통과 승인'},   # operator_commands APPROVE_PASSAGE 행(#406 승인 서비스)
    'mode': {'NORMAL': '평시', 'DISASTER': '재난', 'CLEARING': '해제 중', 'MISSION_FINISHED': '임무 종료'},
    'phase': {'IDLE': '대기', 'DISASTER': '재난 감지', 'SEARCHING': '수색 중', 'TARGET_FOUND': '요구조자 발견',
              'DISPATCH_APPROVED': '출동 승인됨', 'SEGMENT_MOVING': '구간 이동 중', 'AWAITING_CONTINUE': '계속 승인 대기',
              'ARRIVED': '도착', 'CANDIDATE': '구조 완료 후보', 'RECHECKING': '재확인 중',
              'RESCUE_CONFIRMED': '구출 완료', 'RECHECK_NEEDED': '재확인 필요', 'RETURNING': '복귀 중',
              'FINISHED': '종료'},
    'segment': {'IDLE': '대기', 'EVALUATING': '판단 중', 'AWAITING_APPROVAL': '승인 대기', 'EXECUTING': '이동 중',
                'STOPPING': '정지 중', 'STOPPED': '정지', 'UNKNOWN': '모름'},
    'reply': {'NOT_AUTHORIZED': '재난 모드가 아님', 'NOT_FOUND': '대상이나 복귀 위치를 모름',
              'PENDING_EVIDENCE': '확정·경로·판정이 아직 없음', 'TASK_BUSY': '이미 진행 중',
              'INVALID_INPUT': '화면의 값이 지금 상태와 다름', 'FRAME_REVISION_MISMATCH': '지도 기준이 다름',
              'NAV_NOT_READY': '로봇 준비 안 됨 — 준비 뒤 다시 누르세요', 'ID_CONTENT_CONFLICT': '같은 ID 에 다른 내용',
              # 회차 초기화 거절(operations RunReset, #323)
              'ROBOT_STATUS_STALE': '로봇 상태가 없거나 5초 넘게 끊김(로봇이 꺼져 있음)',
              'ROBOT_MOVING': '로봇이 정지 상태가 아님', 'TASK_ACTIVE': '로봇에 임무가 남아 있음',
              'ROBOT_NOT_DOCKED': '로봇이 도킹돼 있지 않음',
              # 터널 통과 승인 거절(#406 ApproveOverheadPassage reason_code)
              'NO_SUCH_ITEM': '지금 터널 목록에 없는 항목', 'NO_APPROVER': '승인한 사람 이름 없음'},
}


def label(kind, code):
    """`code`에 대응하는 한국어 명칭을 반환한다. 미정의 코드는 원본 유지, None은 None 반환(UI상 빈 값 처리).

    지원하지 않는 `kind` 전달은 프로그램 오류로 간주하여 KeyError를 발생시킨다.
    """
    table = LABELS[kind]
    if code is None:
        return None
    return table.get(code, code)
