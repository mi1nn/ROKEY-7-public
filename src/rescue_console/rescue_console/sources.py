"""읽기 전용 입력: operations 장부, 현장 상태 DB, 또는 상태 픽스처.

파일에 쓰거나 파일을 만들지 않는다.
"""

import json
from pathlib import Path

from rescue_operations.ledger import OperationsError, read_reports
from rescue_operations.site_state import (read_allowed_robot_ids, read_error_log, read_exit_locations, read_mission_state,
                                          read_operator_commands, read_robot_status, read_site_events, read_target_states)

from .view import HISTORY_ROWS, ConsoleError

# read_reports 와 read_site_events 는 한 번 읽기를 1000 행으로 자른다(rescue_operations.ledger._limit).
LEDGER_READ_LIMIT = 1000
# read_site_events 는 가장 오래된 행부터 돌려준다(ORDER BY observed_at_ns LIMIT n). 그래서 관제 웹은
# 상한만큼 요청한다. 현장 사건이 1000 개를 넘으면 가장 새것이 빠진다(README 「후속」).
SITE_EVENT_READ_LIMIT = 1000
# 상태 픽스처가 가질 수 있는 최상위 키: load_site_state 가 돌려주는 키와 같아서 build_view 가 둘을 똑같이 읽는다.
STATUS_KEYS = frozenset({'allowed_robots', 'robot_status', 'site_events', 'target_states', 'mission_state', 'operator_commands',
                         'error_log', 'exit_locations'})


def load_ledger(db_path):
    """operations 의 읽기 전용 API(mode=ro, query_only)로 장부 스냅숏을 읽는다."""
    try:
        return read_reports(db_path, limit=LEDGER_READ_LIMIT)
    except OperationsError as exc:
        raise ConsoleError('STORAGE_ERROR', f'ledger read failed: {exc}') from exc


def load_status(path):
    """상태 픽스처(E2, operations 를 띄우지 않음): operations 읽기 API 의 모양을 따르는 JSON 파일이다.

    키와 대응하는 읽기 함수: allowed_robots (read_allowed_robot_ids), robot_status (read_robot_status), site_events
    (read_site_events), target_states (read_target_states), mission_state (read_mission_state),
    operator_commands (read_operator_commands), error_log (read_error_log), exit_locations (read_exit_locations).
    빠진 키는 채우지 않는다: build_view 가 입력 없음으로 보인다(allowed_robots 가 없으면 로봇 목록은 장부에서,
    또는 상태를 보낸 로봇에서 온다). 폴링마다 다시 읽고, 읽을 수 없는 파일이나
    모르는 키는 INVALID_INPUT 이다.
    """
    if path is None:
        return {}   # 장부만 있고 상태 원천은 없음: 화면이 상태 입력 없음을 알린다
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:   # ValueError 는 잘못된 JSON 과 잘못된 UTF-8 을 둘 다 잡는다
        raise ConsoleError('INVALID_INPUT', f'status fixture unreadable: {exc}') from exc
    # 빠진 키는 허용한다(입력 없음으로 보인다). 모르는 키는 거부해서, 철자가 틀린 키가 조용히
    # 입력 없음으로 읽히지 않게 한다.
    if not isinstance(data, dict) or not set(data) <= STATUS_KEYS:
        raise ConsoleError('INVALID_INPUT', f'status fixture keys must be a subset of {sorted(STATUS_KEYS)}')
    return data


def load_site_state(db_path):
    """현장 상태 읽기 API 결과(#34, mission_state #111, 명령·오류 #125)를 읽기 전용으로 하나씩 차례로
    열어 모은다. 명령과 오류는 각각 가장 새 HISTORY_ROWS 행이고, 새것부터 놓인다.

    상태 픽스처와 모양이 같아서 `build_view` 는 어느 쪽을 받았는지 모른다.
    읽기마다 따로 열고 한 트랜잭션이 아니다: 그 사이에 쓰인 행은 한 폴링 동안 한 표에는 보이고
    다른 표에는 아직 안 보일 수 있다. 명령 버튼은 임무 행을 따르고,
    어차피 조정기가 모든 명령을 자기 상태와 맞춰 다시 검사한다.
    """
    try:
        return {
            'allowed_robots': read_allowed_robot_ids(db_path),   # #149: 상태가 오기 전에도 있는 배치 로봇 목록
            'robot_status': read_robot_status(db_path),
            'site_events': read_site_events(db_path, limit=SITE_EVENT_READ_LIMIT),
            'target_states': read_target_states(db_path),
            'mission_state': read_mission_state(db_path),
            'operator_commands': read_operator_commands(db_path, limit=HISTORY_ROWS),
            'error_log': read_error_log(db_path, limit=HISTORY_ROWS),
            'exit_locations': read_exit_locations(db_path, limit=1),   # #237: 가장 새 exit_qr 관측 하나만
        }
    except OperationsError as exc:
        # 읽기 하나가 실패하면 폴링 전체가 실패한다(503 이고, 일부만 채운 화면은 내지 않는다). 다른 역할이나
        # schema 의 파일도 읽기 API 가 거부하며, 여기서 똑같이 STORAGE_ERROR 로 보인다.
        raise ConsoleError('STORAGE_ERROR', f'site-state read failed: {exc}') from exc
