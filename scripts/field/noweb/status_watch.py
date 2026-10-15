#!/usr/bin/env python3
"""The console's cards as one line a second, for runs without the web console (웹 없는 트랙, 2026-10-15).

    status_watch.py --db <site_state.sqlite3> [--log <status.log>] [--interval 1.0] [--once]

Reads the operations DB read-only with the site_state readers (the file is never created or written; no ROS)
and prints, on one line separated by " | ":
- the time and the open run (회차 없음 between a reset and the next run)
- the mission: mode/phase, the coordinator's reason, the support dispatch's segment
- each configured robot: role, readiness, motion, dock, whether it has a pose and a task, battery
- the SLAM robot's live map revision and the frozen maps kept in this run (map_snapshots frozen-r<rev>).
  Only db_writer's map archive (operations.launch map_topic, map_dir) writes those rows. A run with no kept
  map at all prints 「지도 보관 꺼짐 — 고정 여부 모름」: the archive is off, or it has not kept its first live
  map yet (about ARCHIVE_PERIOD after the first MapState). 「고정 지도 없음」 only when live maps are kept.
- the targets: rescue state / support state
- after a reset, the restart checklist (read_reset_checklist)
Each value with a time says how old it is; older than STALE_S it is marked 낡음. A value the DB does not have
is printed as missing, never filled in. Navigation stack state is not in this DB (RobotStatus carries only
readiness); look at the robot's own log for it.
"""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

REPO = Path(__file__).resolve().parents[3]
try:
    import rescue_operations  # noqa: F401  (installed or on PYTHONPATH)
except ImportError:  # run straight from the checkout
    for package in ('rescue_contracts', 'rescue_operations'):
        sys.path.insert(0, str(REPO / 'src' / package))

from rescue_operations import site_state  # noqa: E402
from rescue_operations.schema import OperationsError  # noqa: E402

# Same as the console's stale robot card and the reset's status check (command_api.RESET_STATUS_STALE_NS).
STALE_S = 5
SECOND = 1_000_000_000


def _age(at_ns, now_ns):
    if at_ns is None:
        return '(시각 없음)'
    seconds = (now_ns - at_ns) // SECOND
    if seconds < 0:
        return f'({-seconds}초 앞선 시각)'
    return f'({seconds}초 전, 낡음)' if seconds > STALE_S else f'({seconds}초 전)'


def _mission(db, now_ns, run):
    m = site_state.read_mission_state(db)['mission']
    if m is None:
        return '임무 기록 없음'
    text = f'임무 {m["mode"]}/{m["phase"]} 이유={m["reason"] or "-"}'
    if m['target_id'] is not None:
        text += f' 구간 {m["segment_index"]}/{m["segment_count"]} {m["segment_phase"]}'
    return f'{text} {_age(m["updated_at_ns"], now_ns)}'


def _robots(db, now_ns, run):
    heard = {r['robot_id']: r for r in site_state.read_robot_status(db)['robots']}
    parts = []
    for robot in site_state.read_allowed_robot_ids(db)['allowed_robot_ids']:
        r = heard.get(robot)
        if r is None:
            parts.append(f'{robot} 상태 없음')
            continue
        battery = '-' if r['battery_percent'] is None else f'{r["battery_percent"]:.0f}%'
        parts.append(f'{robot}({r["role"]}) 준비={r["readiness"]} 이동={r["motion_state"]} '
                     f'도킹={r["dock_state"] or "모름"} 자세={"있음" if r["pose"] else "없음"} '
                     f'작업={"있음" if r["active_task_id"] else "없음"} 배터리={battery} '
                     f'{_age(r["received_at_ns"], now_ns)}')
    return ' | '.join(parts)


def _kept_maps(db, run):
    """Every kept map of the open run: the oldest RANGE_LIMIT_MAX and the newest 100, so neither an early
    frozen map nor a late one drops out of a long run."""
    if run is None:
        return []
    oldest = site_state.read_map_snapshots_range(db, run_id=run)['maps']
    return oldest + site_state.read_map_snapshots(db)['maps']


def _maps(db, now_ns, run):
    current = site_state.read_overhead_passages(db)['current_map']
    live = '지도 상태 없음' if current is None else \
        f'지도 r{current["map_revision"]} {_age(current["received_at_ns"], now_ns)}'
    kept = _kept_maps(db, run)
    if not kept:
        return f'{live} | 지도 보관 꺼짐 — 고정 여부 모름'
    frozen = sorted({m['revision'] for m in kept if Path(m['path']).name.startswith('frozen-')})
    return f'{live} | ' + ('고정 지도 없음' if not frozen else '고정 지도 ' + ','.join(f'r{r}' for r in frozen))


def _targets(db, now_ns, run):
    targets = site_state.read_target_states(db)['targets']
    if not targets:
        return '대상 없음'
    return f'대상 {len(targets)}개 ' + ', '.join(
        f'{t["target_id"][:8]} {t["rescue_state"]}/{t["support_state"]}' for t in targets)


def _checklist(db, now_ns, run):
    c = site_state.read_reset_checklist(db)
    if c['previous_run_id'] is None:
        return None
    done = {True: '다시 켜짐', False: '아직'}
    robots = ' '.join(f'{robot}={done[item["restarted"]]}' for robot, item in c['robots'].items())
    return f'초기화 확인 조정기={done[c["coordinator_restarted"]]} {robots}'


def snapshot(db, *, now_ns):
    """One status line. A part the DB cannot give says so; the line is always printed."""
    clock = datetime.fromtimestamp(now_ns / SECOND).strftime('%H:%M:%S')
    try:
        run = site_state.list_runs(db)['open_run_id']
    except OperationsError as exc:
        return f'{clock} DB 읽기 실패: {exc.detail}'
    parts = [f'{clock} ' + ('회차 없음' if run is None else f'회차 {run[:8]}')]
    for part in (_mission, _robots, _maps, _targets, _checklist):
        try:
            text = part(db, now_ns, run)
        except OperationsError as exc:
            text = f'DB 읽기 실패: {exc.detail}'
        if text is not None:
            parts.append(text)
    return ' | '.join(parts)


def main(argv=None):
    parser = argparse.ArgumentParser(description='관제 웹 카드 대신 운영 DB 의 상태를 한 줄씩 보여 준다')
    parser.add_argument('--db', type=Path, required=True, help='운영 DB site_state.sqlite3 (읽기 전용으로 연다)')
    parser.add_argument('--log', type=Path, help='같은 줄을 덧붙여 쓸 파일')
    parser.add_argument('--interval', type=float, default=1.0, help='초 (기본 1)')
    parser.add_argument('--once', action='store_true', help='한 줄만 쓰고 끝낸다')
    args = parser.parse_args(argv)
    if args.log is not None:
        args.log.parent.mkdir(parents=True, exist_ok=True)
    try:
        while True:
            line = snapshot(args.db, now_ns=time.time_ns())
            print(line, flush=True)
            if args.log is not None:
                with args.log.open('a', encoding='utf-8') as log:
                    log.write(line + '\n')
            if args.once:
                return 0
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
