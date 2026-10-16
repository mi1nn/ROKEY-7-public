"""읽기 전용 데이터 소스 모듈: operations 장부, 현장 상태 DB 또는 상태 픽스처 연동.

파일 생성이나 쓰기 작업을 수행하지 않는 읽기 전용 접근만 수행한다.
"""

import json
from pathlib import Path
import threading

from rescue_operations.ledger import OperationsError, read_reports
from rescue_operations.site_state import (list_runs, read_error_log_range, read_exit_locations_range,
                                          read_operator_commands_range, read_reset_checklist, read_robot_status_history,
                                          read_site_events_range, read_target_state_history)
from rescue_operations.site_state import (read_allowed_robot_ids, read_error_log, read_exit_locations, read_mission_state,
                                          read_operator_commands, read_overhead_passages, read_robot_status, read_site_events,
                                          read_target_states)

from .view import HISTORY_ROWS, ConsoleError

# read_reports 및 read_site_events는 1회 조회 시 최대 1000개 행으로 제한된다(rescue_operations.ledger._limit).
LEDGER_READ_LIMIT = 1000
# read_site_events는 관측 시각 오름차순으로 반환하므로(ORDER BY observed_at_ns LIMIT n), 관제 웹은
# 최대 한도까지 요청한다. 현장 사건이 1000건을 초과하면 최신 사건이 누락될 수 있다(README 「후속」).
SITE_EVENT_READ_LIMIT = 1000
# 리플레이에서 표 하나를 이어 읽는 쪽 수 상한([제안] 관제·화면, 2026-10-15). 1000건 x 30쪽이면 로봇 둘의
# 초당 이력으로 약 4시간이다. 넘으면 화면에 「일부만 보임」.
REPLAY_PAGES = 30
# 상태 픽스처 허용 최상위 키 목록: load_site_state 반환 키와 일치하여 build_view에서 동일하게 처리된다.
STATUS_KEYS = frozenset({'allowed_robots', 'robot_status', 'site_events', 'target_states', 'mission_state', 'operator_commands',
                         'error_log', 'exit_locations', 'overhead_passages', 'trigger_commands'})
# 재난 출처 찾기: 임무의 trigger_event_id 와 같은 command_id 의 명령을 임무 시작 시각 앞뒤 이만큼에서 찾는다
# ([제안] 관제·화면, 2026-10-15). 최근 HISTORY_ROWS 건 밖으로 밀려난 재난 선포도 찾기 위해서다.
TRIGGER_WINDOW_NS = 60_000_000_000


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


def load_robot_watch(path):
    """(data, problem): robot_watch 노드가 쓴 직접 관측 파일(--robot-watch). 없거나 못 읽으면 data None 과 이유.
    임시 파일 + rename 으로 써서 반쯤 쓴 파일은 보지 않는다."""
    try:
        text = Path(path).read_text(encoding='utf-8')
    except FileNotFoundError:
        return None, '로봇 직접 관측 파일 없음'
    except OSError as exc:
        return None, f'로봇 직접 관측 파일 읽기 실패: {exc}'
    try:
        data = json.loads(text)
    except ValueError as exc:
        return None, f'로봇 직접 관측 파일 읽기 실패: {exc}'
    if not isinstance(data, dict) or not isinstance(data.get('robots'), dict) or type(data.get('written_at_ns')) is not int:
        return None, '로봇 직접 관측 파일 읽기 실패: 모양이 다름'
    return data, None


def load_site_state(db_path):
    """현장 상태 읽기 API 결과(#34, mission_state #111, 명령·오류 #125)를 읽기 전용으로 순차
    조회하여 딕셔너리로 취합한다. 명령과 오류 이력은 최근 HISTORY_ROWS 건을 최신순으로 정렬한다.

    상태 픽스처와 데이터 구조가 동일하므로 `build_view`는 두 소스를 구분 없이 동일하게 처리한다.
    각 항목은 단일 트랜잭션이 아닌 개별 연결로 조회되므로, 조회 도중 기록된 변경사항은 특정 테이블에만
    일시적으로 반영될 수 있다. 명령 버튼 상태는 mission_state에 연동되며,
    발행된 모든 명령은 coordinator(조정기)가 수신 시점에 자체 상태와 대조하여 다시 검증한다.
    """
    try:
        status = {
            'allowed_robots': read_allowed_robot_ids(db_path),   # #149: 상태 수신 전에도 사전 정의된 배치 로봇 목록
            'robot_status': read_robot_status(db_path),
            'site_events': read_site_events(db_path, limit=SITE_EVENT_READ_LIMIT),
            'target_states': read_target_states(db_path),
            'mission_state': read_mission_state(db_path),
            'operator_commands': read_operator_commands(db_path, limit=HISTORY_ROWS),
            'error_log': read_error_log(db_path, limit=HISTORY_ROWS),
            'exit_locations': read_exit_locations(db_path, limit=1),   # #237: 최신 exit_qr 관측치 1건 조회
            'overhead_passages': read_overhead_passages(db_path),      # #262: 로봇별 터널 판정 + 지금 지도
        }
        status['trigger_commands'] = {'schema_version': status['operator_commands']['schema_version'],
                                      'commands': _trigger_commands(db_path, status)}
        return status
    except OperationsError as exc:
        # 단일 조회라도 실패하면 전체 폴링이 실패 처리된다(503 응답, 불완전하게 부분 채움된 UI는 제공하지 않음). 다른 용도나
        # 스키마의 DB 파일 역시 읽기 API에 의해 거부되며, 동일하게 STORAGE_ERROR 예외로 변환된다.
        raise ConsoleError('STORAGE_ERROR', f'site-state read failed: {exc}') from exc



def _trigger_commands(db_path, status):
    """임무를 연 명령(재난 선포)의 기록 행. 최근 명령에 있으면 그것을 쓰고, 없으면 임무 시작 시각 앞뒤를 읽는다.
    재난이 아니거나 trigger 가 없으면 빈 목록이다. 출처 판단은 view.mission_cause 가 한다(작전 2026-10-15)."""
    mission = status['mission_state']['mission']
    if mission is None or mission['trigger_event_id'] is None:
        return []
    trigger = mission['trigger_event_id']
    found = [row for row in status['operator_commands']['commands'] if row['command_id'] == trigger]
    if found:
        return found
    run_id = list_runs(db_path)['open_run_id']
    if run_id is None:
        return []
    started = mission['started_at_ns']
    page = read_operator_commands_range(db_path, run_id=run_id, start_ns=max(0, started - TRIGGER_WINDOW_NS),
                                        end_ns=started + TRIGGER_WINDOW_NS)
    return [row for row in page['commands'] if row['command_id'] == trigger]


def _range_all(read, items_key, time_key, db_path, run_id):
    """회차 범위 읽기를 쪽마다 이어 읽는다(operations 한도 1000건, 오래된 것부터, 넘치면 최신 쪽이 잘림).

    잘린 쪽은 마지막 시각의 행을 버리고 그 시각부터 다시 읽어 겹치거나 빠지는 행이 없다. 쪽 수는
    REPLAY_PAGES 까지이고, 그래도 남으면 truncated 를 참으로 둔다. 한 시각에 1000건이 몰리면 더 나아갈 수 없어 거기서 멈춘다.
    """
    rows, start = [], None
    for _ in range(REPLAY_PAGES):
        page = read(db_path, run_id=run_id, start_ns=start)
        items = page[items_key]
        if not page['truncated']:
            return {'truncated': False, items_key: rows + items}
        last = items[-1][time_key]
        keep = [row for row in items if row[time_key] != last]
        if not keep:
            return {'truncated': True, items_key: rows + items}
        rows += keep
        start = last
    return {'truncated': True, items_key: rows}


def load_runs(db_path):
    """회차 목록(operations list_runs, mode=ro)."""
    try:
        return list_runs(db_path)
    except OperationsError as exc:
        raise ConsoleError('STORAGE_ERROR', f'run list read failed: {exc}') from exc


class RunIndex:
    """리플레이 회차 목록의 원천: 지금 현장 상태 DB 와 ``root`` 아래 한 단계 폴더의 같은 이름 DB(재범 2026-10-16).

    시뮬(run_full.sh)과 현장(런북)은 launch 마다 새 폴더에 DB 를 만든다. 그래서 지난 회차는 옆 폴더의 다른 파일에
    있다. 지금 DB 는 매번 읽고, 못 읽으면 STORAGE_ERROR 다. 다른 파일은 (크기, 수정 시각, -wal 크기)가 같으면
    전에 읽은 목록을 쓴다(재생 중 조각마다 수십 개를 다시 열지 않게). 못 읽은 다른 파일은 건너뛰고 (폴더 이름, 사유)를
    남긴다. 모두 operations 읽기(mode=ro)로만 연다. Flask 스레드가 함께 쓰므로 기억 저장소를 잠근다.
    """

    def __init__(self, live, root):
        self.live, self.root = Path(live), Path(root)
        self._seen = {}
        self._lock = threading.Lock()

    @staticmethod
    def _key(path):
        """(DB 크기, 수정 시각, -wal 크기). 읽기 전용으로 열 때 빈 -wal 이 생길 수 있어 없는 -wal 은 크기 0 으로 본다."""
        stat = path.stat()
        try:
            wal = path.with_name(path.name + '-wal').stat().st_size
        except OSError:
            wal = 0
        return stat.st_size, stat.st_mtime_ns, wal

    def read(self):
        """([(DB 경로, 폴더 이름, 지금 DB 인가, list_runs 결과)], [(폴더 이름, 못 읽은 사유)]). 지금 DB 가 맨 앞."""
        sources = [(self.live, self.live.parent.name, True, load_runs(self.live))]
        problems = []
        live = self.live.resolve()
        for path in sorted(self.root.glob(f'*/{self.live.name}')):
            if not path.is_file() or path.resolve() == live:
                continue
            try:
                key = self._key(path)
            except OSError as exc:   # 목록을 본 뒤 지워진 파일
                problems.append((path.parent.name, exc.strerror or str(exc)))
                continue
            with self._lock:
                cached = self._seen.get(path)
            if cached is not None and cached[0] == key:
                runs, problem = cached[1], cached[2]
            else:
                try:
                    runs, problem = list_runs(path), None
                except OperationsError as exc:
                    runs, problem = None, str(exc)
                with self._lock:
                    self._seen[path] = (key, runs, problem)
            if runs is None:
                problems.append((path.parent.name, problem))
            else:
                sources.append((path, path.parent.name, False, runs))
        return sources, problems


def load_replay(db_path, run_id):
    """회차 하나의 쌓이는 표 넷과 이력 표 둘을 operations 회차 범위 읽기로 끝까지 읽는다(mode=ro)."""
    reads = (('site_events', read_site_events_range, 'site_events', 'observed_at_ns'),
             ('operator_commands', read_operator_commands_range, 'commands', 'requested_at_ns'),
             ('error_log', read_error_log_range, 'errors', 'at_ns'),
             ('exit_locations', read_exit_locations_range, 'exits', 'observed_at_ns'),
             ('robot_history', read_robot_status_history, 'history', 'at_ns'),
             ('target_history', read_target_state_history, 'history', 'updated_at_ns'))
    try:
        return {key: _range_all(read, items_key, time_key, db_path, run_id) for key, read, items_key, time_key in reads}
    except OperationsError as exc:
        # 회차 ID 는 app 이 list_runs 목록과 맞춰 본 뒤에 넘기므로 여기 실패는 저장소 쪽 문제다.
        raise ConsoleError('STORAGE_ERROR', f'replay read failed: {exc}') from exc


def load_reset_state(db_path):
    """초기화 화면 입력: 배치 로봇, 로봇 상태(미리 보기 거울), 회차 목록, 다시 띄우기 자동 확인(mode=ro)."""
    try:
        return {'allowed_robots': read_allowed_robot_ids(db_path)['allowed_robot_ids'],
                'robots': read_robot_status(db_path)['robots'],
                'runs': list_runs(db_path),
                'checklist': read_reset_checklist(db_path)}
    except OperationsError as exc:
        raise ConsoleError('STORAGE_ERROR', f'reset state read failed: {exc}') from exc
