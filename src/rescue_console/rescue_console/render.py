"""Screen model -> HTML. Standard library only, so the page is tested without Flask."""

from datetime import datetime
from html import escape

# Browser polling period. KPI-04 budget is 3 s (BRD v0.1); 1 s polling leaves 2 s for
# ledger write, read and transfer ([제안] 관제·화면, 2026-10-08; README 「KPI-04 측정」).
POLL_INTERVAL_MS = 1000
MISSING = '—'


def _text(value):
    return MISSING if value is None else escape(str(value))


def _time(ns):
    if ns is None:
        return MISSING
    stamp = datetime.fromtimestamp(ns / 1e9).strftime('%H:%M:%S.%f')[:-3]
    return f'<span title="{ns}">{stamp}</span>'


def _age(seconds):
    return MISSING if seconds is None else f'{seconds:.1f}s'


def _pose(pose):
    if pose is None:
        return MISSING
    yaw = pose.get('yaw')
    return escape(f"({pose['x']:.2f}, {pose['y']:.2f}"
                  + (f", {yaw:.2f}rad" if yaw is not None else '') + f") {pose['frame_id']}")


def _robots(view):
    rows = []
    for robot in view['robots']:
        battery = MISSING if robot['battery_percent'] is None else f"{robot['battery_percent']:.0f}%"
        rows.append(
            f'<tr class="{"" if robot["has_status"] else "missing"}"><td>{_text(robot["robot_id"])}</td>'
            f'<td>{_text(robot["role"])}</td><td>{_pose(robot["pose"])}</td>'
            f'<td>{_text(robot["motion_state"])}</td><td>{_text(robot["readiness"])}</td>'
            f'<td>{battery} <small>({_age(robot["battery_age_s"])})</small></td>'
            f'<td>{_text(robot["active_task_id"])}</td><td>{_age(robot["age_s"])}</td>'
            f'<td>{_age(robot["received_age_s"])}</td></tr>')
    return ('<table id="robots"><caption>로봇</caption><tr><th>ID</th><th>역할</th><th>위치</th>'
            '<th>동작</th><th>준비</th><th>배터리 (나이)</th><th>작업</th><th>상태 나이</th><th>수신 나이</th></tr>'
            + ''.join(rows) + '</table>')


def _targets(view):
    rows = []
    for target in view['targets']:
        position = target['position']
        where = MISSING if position is None else escape(
            f"({position['x']:.2f}, {position['y']:.2f}) {position['frame_id']} r{position['map_revision']}")
        notes = '; '.join(target['warnings'] + target['rescue_reasons'])
        rows.append(
            f'<tr data-stage="{target["stage"]}"><td>{_text(target["target_id"])}</td>'
            f'<td class="stage {target["stage"].lower()}">{escape(target["stage_label"])}'
            f' <small>{target["rule"]}</small></td><td>{where}</td>'
            f'<td>{_text(target["rescue_state"])}</td><td>{_text(target["support_state"])}'
            f' {_text(target["support_robot_id"]) if target["support_robot_id"] else ""}</td>'
            f'<td>{_text(target["approach_result"])}</td><td>{_text(target["receipt_status"])}'
            f' ×{target["report_count"]}</td><td>{escape(notes)}</td></tr>')
    return ('<table id="targets"><caption>사람 좌표와 구조 상태</caption><tr><th>대상</th><th>표시 단계</th>'
            '<th>좌표</th><th>구조 상태</th><th>지원</th><th>접근</th><th>보고</th><th>비고</th></tr>'
            + ''.join(rows) + '</table>')


def _summary(view):
    counts = ''.join(f'<li>{escape(label)} <b>{view["stage_counts"][stage]}</b></li>'
                     for stage, label in (('UNCONFIRMED', '미확정'), ('NOT_RESCUED', '미구조'),
                                          ('SUPPORTING', '지원 중'), ('RESCUED', '구출 완료')))
    return (f'<section id="summary"><div class="exit">출구 누적 <b id="exit-count">{_text(view["exit_count"])}</b></div>'
            f'<ul class="stages">{counts}</ul></section>')


def _events(view):
    rows = []
    for event in view['events']:
        rows.append(
            f'<tr class="event" data-ref="{escape(event["ref_id"])}" data-event-ns="{event["event_ns"]}">'
            f'<td>{_time(event["event_ns"])}</td><td>{_time(event["received_at_ns"])}</td>'
            f'<td class="rendered">{MISSING}</td><td class="latency">{MISSING}</td>'
            f'<td>{escape(event["kind"])}</td><td>{escape(event["source"])}</td><td>{_text(event["subject"])}</td>'
            f'<td>{escape(event["detail"])}</td><td>{escape(event["clock_domain_id"])}</td></tr>')
    shown = f'{len(view["events"])} / {view["event_total"]}'
    return (f'<table id="events"><caption>이벤트 ({shown})</caption><tr><th>이벤트 시각</th><th>장부 기록</th>'
            '<th>화면 표시</th><th>지연</th><th>종류</th><th>출처</th><th>대상</th><th>내용</th><th>시계</th></tr>'
            + ''.join(rows) + '</table>')


def _commands(view):
    buttons = ''.join(
        f'<button type="button" disabled data-command="{escape(c["name"])}" title="{escape(c["reason"])}">'
        f'{escape(c["label"])}</button>' for c in view['commands'])
    reason = escape(view['commands'][0]['reason']) if view['commands'] else ''
    return f'<section id="commands">{buttons}<small>{reason}</small></section>'


def render_fragment(view):
    """The part of the page that the browser replaces on every poll."""
    warnings = ''.join(f'<li>{escape(w)}</li>' for w in view['warnings'])
    return (f'<div id="built" data-built-ns="{view["built_at_ns"]}">서버 집계 {_time(view["built_at_ns"])}'
            f' · 증거 정책 {escape(view["evidence_policy"])}</div>'
            + (f'<ul id="warnings">{warnings}</ul>' if warnings else '')
            + f'<div class="grid"><section id="map" class="placeholder">{escape(view["map"]["note"])}</section>'
            + _summary(view) + '</div>' + _robots(view) + _targets(view) + _events(view))


_STYLE = '''
body{font-family:system-ui,sans-serif;margin:16px;background:#fafafa;color:#222}
table{border-collapse:collapse;width:100%;margin:12px 0;font-size:14px}
caption{text-align:left;font-weight:600;padding:4px 0}
th,td{border:1px solid #ccc;padding:4px 6px;text-align:left;vertical-align:top}
tr.missing td{color:#999}
.grid{display:grid;grid-template-columns:2fr 1fr;gap:12px}
.placeholder{min-height:180px;border:2px dashed #aaa;display:flex;align-items:center;justify-content:center;color:#777}
.exit{font-size:28px}.stages{list-style:none;padding:0}
.stage.unconfirmed{background:#eee}.stage.not_rescued{background:#fdd}
.stage.supporting{background:#ffd}.stage.rescued{background:#dfd}
#warnings{color:#a00}#commands button{margin-right:6px}
'''

# Stamps each event row with the browser time it first appeared (KPI-04). Only GET requests.
_SCRIPT = '''
const firstSeen = new Map();
window.kpi04 = [];
function stamp() {
  const now = Date.now();
  for (const row of document.querySelectorAll('tr.event')) {
    const ref = row.dataset.ref, eventMs = Number(row.dataset.eventNs) / 1e6;
    if (!firstSeen.has(ref)) {
      firstSeen.set(ref, now);
      window.kpi04.push({ref_id: ref, event_ns: row.dataset.eventNs, rendered_ms: now});
    }
    const shown = firstSeen.get(ref);
    row.dataset.renderedMs = shown;
    row.querySelector('.rendered').textContent = new Date(shown).toLocaleTimeString();
    row.querySelector('.latency').textContent = ((shown - eventMs) / 1000).toFixed(2) + 's';
  }
}
async function poll() {
  try {
    const response = await fetch('fragment', {cache: 'no-store'});
    if (!response.ok) throw new Error('HTTP ' + response.status);
    document.getElementById('live').innerHTML = await response.text();
    document.getElementById('poll-error').textContent = '';
    stamp();
  } catch (error) {
    document.getElementById('poll-error').textContent = '갱신 실패: ' + error.message + ' — 표시 값이 낡았을 수 있음';
  }
}
stamp();
setInterval(poll, POLL_MS);
'''


def render_page(view):
    script = _SCRIPT.replace('POLL_MS', str(POLL_INTERVAL_MS))
    return ('<!doctype html><html lang="ko"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>구조 관제</title><style>{_STYLE}</style></head><body>'
            '<h1>구조 관제 <small>(읽기 전용)</small></h1><div id="poll-error" style="color:#a00"></div>'
            + _commands(view) + f'<div id="live">{render_fragment(view)}</div>'
            f'<script>{script}</script></body></html>')
