"""Ledger + status snapshot -> one screen model. Pure Python; no I/O, no clock reads.

The caller passes `built_at_ns`, so the same inputs always give the same model.
Values that are missing stay None and are shown as missing; nothing is filled in.
"""

from rescue_contracts import RESCUE_STATES, RescueError, SiteEvent

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

# Part I 3.7.6 operator commands. Placeholders only: no transport exists until Q-22
# fixes their safety gates. Enabling one is a separate PR with its own tests.
COMMANDS = (
    ('PAUSE', '일시정지'),
    ('RESUME', '재개'),
    ('CANCEL_TASK', '작업 취소'),
    ('RETURN_HOME', '복귀'),
)
COMMAND_DISABLED_REASON = 'Q-22 안전 관문 결정 전 — 명령 전송 미연결'

# Row cap for the event table; older rows stay in the ledger ([제안] 관제·화면, 2026-10-08).
EVENT_ROWS = 50

# A robot row whose last status reached operations longer ago than this is marked stale
# ([제안] 관제·화면, 2026-10-10). Display only, not readiness (C-05); replace with the profile's
# readiness max_age_s once profile v2.1 (#39) is on main.
STATUS_STALE_S = 5.0

# Mission mode shown in the badge. Until site_state 1.1 adds `mission_state`
# (console-and-db-plan 3절), the mode is inferred from the site events alone.
MODES = {'NORMAL': '평시', 'DISASTER': '재난'}


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


def _targets(ledger, target_states):
    reports = _latest_reports(ledger['reports'])
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
        support_state = None if state is None else state['support_state']
        approach = None if payload is None else payload['approach_result']
        stage, rule, rule_warnings = compose_stage(confirmed, rescue_state, support_state, approach)
        if payload is None:
            warnings.append('장부에 보고가 없는 대상')
        rows.append({
            'target_id': target_id,
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
            'report_count': report_count,
            'receipt_status': None if item is None else item['receipt']['status'],
            'observed_at_ns': None if payload is None else payload['observed_at_ns'],
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
    for item in ledger['reports']:
        payload = item['payload']
        rows.append({
            'event_ns': payload['observed_at_ns'],
            'received_at_ns': item['received_at_ns'],
            'kind': 'REPORT',
            'source': payload['source_robot'],
            'subject': payload['target_id'],
            'detail': f"{payload['confirmation_result']} / {payload['approach_result']} / {item['receipt']['status']}",
            'clock_domain_id': payload['clock_domain_id'],
            'ref_id': payload['report_id'],
        })
    for event in site_events:
        rows.append({
            'event_ns': event.observed_at_ns,
            'received_at_ns': None,
            'kind': event.kind,
            'source': event.camera_id,
            'subject': None,
            'detail': f'confidence {event.confidence:.2f}' + (f' / {event.detail}' if event.detail else ''),
            'clock_domain_id': event.clock_domain_id,
            'ref_id': event.event_id,
        })
    rows.sort(key=lambda row: (row['event_ns'], row['ref_id']), reverse=True)
    return rows[:EVENT_ROWS], len(rows)


def _mission(site_events):
    """Badge input inferred from DISASTER_SIGN events; None means site events are not available."""
    if site_events is None:
        return {'mode': None, 'label': '모름', 'source': None, 'trigger_event_id': None,
                'trigger_ns': None, 'sign_count': 0}
    signs = sorted((e for e in site_events if e.kind == 'DISASTER_SIGN'),
                   key=lambda e: (e.observed_at_ns, e.event_id))
    mode = 'DISASTER' if signs else 'NORMAL'
    return {'mode': mode, 'label': MODES[mode], 'source': 'site_events',
            'trigger_event_id': signs[0].event_id if signs else None,
            'trigger_ns': signs[0].observed_at_ns if signs else None, 'sign_count': len(signs)}


def build_view(ledger, status, *, built_at_ns):
    """ledger: read_reports() result. status: {'robot_status', 'site_events', 'target_states'},
    each the matching operations read API result (or None when not available yet)."""
    _require(type(built_at_ns) is int and built_at_ns >= 0, 'built_at_ns must be int ns')
    warnings = []
    allowed = tuple(ledger['allowed_robot_ids'])
    robot_section = _section(status, 'robot_status', 'robots')
    site_section = _section(status, 'site_events', 'site_events')
    target_section = _section(status, 'target_states', 'targets')
    for key, section in (('robot_status', robot_section), ('site_events', site_section),
                         ('target_states', target_section)):
        if section is None:
            warnings.append(f'{key} 입력 없음')
    target_states = {}
    for state in [] if target_section is None else target_section['targets']:
        _require(state.get('target_id') not in target_states, f"duplicate target state {state.get('target_id')}")
        target_states[state['target_id']] = state
    targets = _targets(ledger, target_states)
    site_events = [] if site_section is None else _site_events(site_section['site_events'])
    # Kept in the API model, not drawn: with one fixed camera there is no EXIT_PASS producer and the
    # count stays 0 (2026-10-09 decision; console-and-db-plan 3절, db-schema 2절). The 3.5.5
    # "rescued <= exit count" warning went with it; the rescue trigger is the camera's candidate event.
    exit_count = None if site_section is None else site_section['exit_pass_count']
    _require(exit_count is None or (type(exit_count) is int and exit_count >= 0), 'exit_pass_count')
    stage_counts = {stage: 0 for stage in STAGES}
    for row in targets:
        stage_counts[row['stage']] += 1
    events, event_total = _events(ledger, site_events)
    return {
        'built_at_ns': built_at_ns,
        'evidence_policy': ledger['evidence_policy'],
        'mission': _mission(None if site_section is None else site_events),
        'robots': _robots(allowed, [] if robot_section is None else robot_section['robots'],
                          built_at_ns, warnings),
        'targets': targets,
        'stage_counts': stage_counts,
        'exit_count': exit_count,
        'events': events,
        'event_total': event_total,
        'map': {'available': False, 'note': '지도 스냅샷 미연결 — 로봇·지도 제공 뒤 연결'},
        'commands': [{'name': name, 'label': label, 'enabled': False, 'reason': COMMAND_DISABLED_REASON}
                     for name, label in COMMANDS],
        'warnings': warnings,
    }


def request_command(name):
    """Every operator command is refused here; nothing is sent to a robot or written to a DB."""
    _require(name in dict(COMMANDS), f'unknown command {name!r}')
    raise ConsoleError('NOT_CONNECTED', f'{name}: {COMMAND_DISABLED_REASON}')
