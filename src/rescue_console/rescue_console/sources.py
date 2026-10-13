"""읽기 전용 데이터 소스 모듈: operations 장부, 현장 상태 DB 또는 상태 픽스처 연동.

파일 생성이나 쓰기 작업을 수행하지 않는 읽기 전용 접근만 수행한다.
"""

import json
from pathlib import Path

from rescue_operations.ledger import OperationsError, read_reports
from rescue_operations.site_state import (read_allowed_robot_ids, read_error_log, read_exit_locations, read_mission_state,
                                          read_operator_commands, read_robot_status, read_site_events, read_target_states)

from .view import HISTORY_ROWS, ConsoleError

# read_reports 및 read_site_events는 1회 조회 시 최대 1000개 행으로 제한된다(rescue_operations.ledger._limit).
LEDGER_READ_LIMIT = 1000
# read_site_events는 관측 시각 오름차순으로 반환하므로(ORDER BY observed_at_ns LIMIT n), 관제 웹은
# 최대 한도까지 요청한다. 현장 사건이 1000건을 초과하면 최신 사건이 누락될 수 있다(README 「후속」).
SITE_EVENT_READ_LIMIT = 1000
# 상태 픽스처 허용 최상위 키 목록: load_site_state 반환 키와 일치하여 build_view에서 동일하게 처리된다.
STATUS_KEYS = frozenset({'allowed_robots', 'robot_status', 'site_events', 'target_states', 'mission_state', 'operator_commands',
                         'error_log', 'exit_locations'})


def load_ledger(db_path):
    """operations 읽기 전용 API(mode=ro, query_only)를 통해 보고서 장부 스냅샷을 조회한다."""
    try:
        return read_reports(db_path, limit=LEDGER_READ_LIMIT)
    except OperationsError as exc:
        raise ConsoleError('STORAGE_ERROR', f'ledger read failed: {exc}') from exc


def load_status(path):
    """상태 픽스처(E2 모의 환경, operations 미실행): operations 읽기 API 규격을 따르는 JSON 파일 파싱.

    키와 대응하는 읽기 함수: allowed_robots (read_allowed_robot_ids), robot_status (read_robot_status), site_events
    (read_site_events), target_states (read_target_states), mission_state (read_mission_state),
    operator_commands (read_operator_commands), error_log (read_error_log), exit_locations (read_exit_locations).
    누락된 키는 임의로 채우지 않으며 build_view에서 해당 항목 없음으로 처리된다(allowed_robots가 없으면 로봇 목록은 장부
    또는 상태 보고 로봇에서 파생). 매 폴링 시점마다 다시 읽으며, 읽기 실패 또는
    정의되지 않은 키 포함 시 INVALID_INPUT 예외가 발생한다.
    """
    if path is None:
        return {}   # 장부만 있고 상태 소스는 없는 경우: 화면에 상태 데이터 없음 안내가 표시된다
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:   # ValueError는 잘못된 JSON 형식 및 UTF-8 디코딩 오류를 모두 포착한다
        raise ConsoleError('INVALID_INPUT', f'status fixture unreadable: {exc}') from exc
    # 누락된 키는 허용한다(항목 없음 처리). 반면 미정의 키는 차단하여, 오타가 발생한 키가 조용히
    # 누락 처리되는 오류를 방지한다.
    if not isinstance(data, dict) or not set(data) <= STATUS_KEYS:
        raise ConsoleError('INVALID_INPUT', f'status fixture keys must be a subset of {sorted(STATUS_KEYS)}')
    return data


def load_site_state(db_path):
    """현장 상태 읽기 API 결과(#34, mission_state #111, 명령·오류 #125)를 읽기 전용으로 순차
    조회하여 딕셔너리로 취합한다. 명령과 오류 이력은 최근 HISTORY_ROWS 건을 최신순으로 정렬한다.

    상태 픽스처와 데이터 구조가 동일하므로 `build_view`는 두 소스를 구분 없이 동일하게 처리한다.
    각 항목은 단일 트랜잭션이 아닌 개별 연결로 조회되므로, 조회 도중 기록된 변경사항은 특정 테이블에만
    일시적으로 반영될 수 있다. 명령 버튼 상태는 mission_state에 연동되며,
    발행된 모든 명령은 coordinator(조정기)가 수신 시점에 자체 상태와 대조하여 다시 검증한다.
    """
    try:
        return {
            'allowed_robots': read_allowed_robot_ids(db_path),   # #149: 상태 수신 전에도 사전 정의된 배치 로봇 목록
            'robot_status': read_robot_status(db_path),
            'site_events': read_site_events(db_path, limit=SITE_EVENT_READ_LIMIT),
            'target_states': read_target_states(db_path),
            'mission_state': read_mission_state(db_path),
            'operator_commands': read_operator_commands(db_path, limit=HISTORY_ROWS),
            'error_log': read_error_log(db_path, limit=HISTORY_ROWS),
            'exit_locations': read_exit_locations(db_path, limit=1),   # #237: 최신 exit_qr 관측치 1건 조회
        }
    except OperationsError as exc:
        # 단일 조회라도 실패하면 전체 폴링이 실패 처리된다(503 응답, 불완전하게 부분 채움된 UI는 제공하지 않음). 다른 용도나
        # 스키마의 DB 파일 역시 읽기 API에 의해 거부되며, 동일하게 STORAGE_ERROR 예외로 변환된다.
        raise ConsoleError('STORAGE_ERROR', f'site-state read failed: {exc}') from exc
