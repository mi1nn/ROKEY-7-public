"""1차 지도 고정(R1) 전 TB-5 추정 위치. 화면 전용이고 I/O 는 현장 파일 읽기 하나뿐이다.

TB-5 는 고정 지도 위 AMCL(FROZEN_MAP_AMCL)이라 R1 전에는 실제 위치가 없다. 재범님은 처음부터 TB-5 가 보이길
원한다(재범 결정 2026-10-15 17:15, 작전 설계). 그래서 TB-6 출발 자세(/rescue/map/start_pose/<지도 주인>, map 기준,
robot_watch 가 받아 파일에 적음)에 현장 파일 ``robots.<id>.initial_pose_xy_yaw``(TB-6 출발 자세 기준 상대값)를
``compose_planar`` 로 더한 한 점을 그린다. odom 누적은 하지 않는다. TB-5 는 R1 전에 움직이지 않고, 누적하면
Wi-Fi 구독이 늘고 오차가 실제처럼 보이기 때문이다.

조건(작전 2026-10-15): 출발 자세나 시작 값이 없으면 그리지 않는다. 지도 frame 의 실제 위치가 생기면 바로 그 값을
쓰고, 바뀐 때를 로그에 남긴다(EstimateLog). 추정은 pose 의 ``estimated: True`` 로 표시하고 지도·카드가 다르게
그린다. 조정기·실행기·출동 판단·DB 로봇 상태에는 쓰지 않는다(이 모듈 결과는 화면 모델에만 들어간다).
"""

import json
import math
from pathlib import Path
import threading

from rescue_contracts.occupancy import compose_planar

from .mapimage import MAP_FRAME
from .view import ConsoleError

# 고정 전 SLAM 지도의 revision. 지도 고정은 이 값 + 1(R1), + 2(R2)를 낸다(map_freeze FREEZES, contracts C-04a).
# 배치 프로파일 map_basis.map_revision 이 0 이다([설정] config/deployment*.json). 작전 OPS-281: 이것이 확인될 때만 그린다.
PRE_FREEZE_MAP_REVISION = 0
OFFSET_KEY = 'initial_pose_xy_yaw'   # 현장 파일 robots.<id> 의 키(scripts/field/capture_initial_pose.py 가 잰 값)


def load_start_offsets(path):
    """현장 파일에서 {robot_id: (x, y, yaw)}. 키가 없는 로봇은 빠진다. 파일이나 값 모양이 틀리면 ConsoleError."""
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise ConsoleError('INVALID_INPUT', f'field file not readable: {exc}') from exc
    robots = data.get('robots') if isinstance(data, dict) else None
    if not isinstance(robots, dict):
        raise ConsoleError('INVALID_INPUT', 'field file needs a robots object')
    offsets = {}
    for robot_id, section in robots.items():
        value = section.get(OFFSET_KEY) if isinstance(section, dict) else None
        if value is None:
            continue
        if (not isinstance(value, list) or len(value) != 3
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value)):
            raise ConsoleError('INVALID_INPUT', f'robots.{robot_id}.{OFFSET_KEY} must be [x, y, yaw]')
        offsets[robot_id] = tuple(float(v) for v in value)
    return offsets


def before_first_freeze(current_map, map_robot, dispatch):
    """「R1 전」이 확인되는가(작전 OPS-281). 지도 주인 로봇의 지금 지도가 고정 전 revision 이고 출동 구간이 없어야
    한다. 지금 지도를 모르면 아니다(모르면 그리지 않음). R1 뒤나 출동 뒤 TB-5 위치가 비었을 때 움직이는 TB-5 가
    출발점에 다시 나타나지 않게 한다."""
    return (dispatch is None and current_map is not None and current_map['robot_id'] == map_robot
            and current_map['map_revision'] == PRE_FREEZE_MAP_REVISION)


def apply_estimate(robots, watch, built_at_ns, *, current_map=None, dispatch=None):
    """지도 frame 위치가 없는 로봇에 추정 위치를 넣는다. ``watch`` 는 build_view 의 robot_watch 입력이고,
    ``start_offsets`` 가 있어야 한다. 직접 관측 파일이 낡았거나 출발 자세가 map frame 이 아니면 넣지 않는다.
    「R1 전」(before_first_freeze)이 확인되지 않거나 그 로봇이 임무 중(active_task_id)이면 넣지 않는다."""
    offsets = (watch or {}).get('start_offsets')
    data = None if watch is None else watch['data']
    if not offsets or data is None or (built_at_ns - data['written_at_ns']) / 1e9 > watch['stale_s']:
        return
    if not before_first_freeze(current_map, watch['map_robot'], dispatch):
        return
    start = (data['robots'].get(watch['map_robot']) or {}).get('start_pose')
    if not isinstance(start, dict) or start.get('frame_id') != MAP_FRAME:
        return
    for row in robots:
        relative = offsets.get(row['robot_id'])
        pose = row.get('pose')
        if (relative is None or row['robot_id'] == watch['map_robot'] or (pose and pose['frame_id'] == MAP_FRAME)
                or row.get('active_task_id') is not None):
            continue
        x, y, yaw = compose_planar((start['x'], start['y'], start['yaw']), relative)
        row['pose'] = {'x': x, 'y': y, 'yaw': yaw, 'frame_id': MAP_FRAME, 'estimated': True}


class EstimateLog:
    """로봇별로 추정 ↔ 실제가 바뀐 때 한 줄씩 돌려준다(app 이 로그에 남김). 처음 본 상태도 한 줄 남긴다."""

    def __init__(self):
        self.last = {}
        self.lock = threading.Lock()   # 요청 스레드 여럿이 같은 기록을 본다

    def changes(self, robots):
        with self.lock:
            return self._changes(robots)

    def _changes(self, robots):
        lines = []
        for row in robots:
            pose = row.get('pose')
            now = None if pose is None else ('추정' if pose.get('estimated') else '실제')
            before = self.last.get(row['robot_id'])
            if now != before and now is not None:
                lines.append(f'{row.get("name") or row["robot_id"]} 지도 위치: {before or "없음"} -> {now}')
            self.last[row['robot_id']] = now
        return lines
