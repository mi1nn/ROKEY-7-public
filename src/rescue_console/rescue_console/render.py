"""Screen model -> HTML. Standard library only, so the page is tested without Flask."""

from datetime import datetime
from html import escape
import math

from .labels import label
from .mapimage import MAP_FRAME
from .view import STAGES

# Browser polling period. KPI-04 budget is 3 s (BRD v0.1); 1 s polling leaves 2 s for
# ledger write, read and transfer ([제안] 관제·화면, 2026-10-08; README 「KPI-04 측정」).
POLL_INTERVAL_MS = 1000
# KPI-04 budget: event time -> shown on the operator screen (BRD v0.1 KPI-04, 3 s).
KPI04_BUDGET_S = 3.0
MISSING = '—'
# Map marker sizes in map pixels (0.05 m/px for the pre-quake map): robot arrow half-width and
# nose length, victim V half-width. Display only ([제안] 관제·화면, 2026-10-10).
ROBOT_MARK_PX, HEADING_PX, TARGET_MARK_PX = 2.6, 4.8, 2.0


def _text(value):
    return MISSING if value is None else escape(str(value))


def _time(ns, ref_ns=None):
    """Clock time of a ns stamp; the date is added when it is not the day of `ref_ns`."""
    if ns is None:
        return MISSING
    moment = datetime.fromtimestamp(ns / 1e9)
    same_day = ref_ns is None or moment.date() == datetime.fromtimestamp(ref_ns / 1e9).date()
    stamp = moment.strftime('%H:%M:%S.%f' if same_day else '%Y-%m-%d %H:%M:%S.%f')[:-3]
    return f'<span title="{ns}">{stamp}</span>'


def _age(seconds):
    """Short age text: 0.4s, 12.0s, 3분 05초, 2시간 10분, 41일. Negative means a clock ahead of this host."""
    if seconds is None:
        return MISSING
    sign, value = ('-', -seconds) if seconds < 0 else ('', seconds)
    if value < 60:
        text = f'{value:.1f}s'
    elif value < 3600:
        text = f'{int(value // 60)}분 {int(value % 60):02d}초'
    elif value < 86400:
        text = f'{int(value // 3600)}시간 {int(value % 3600 // 60)}분'
    else:
        text = f'{int(value // 86400)}일'
    return sign + text


# Role -> the unit name the team uses (console-and-db-plan 1절: AMR1 정찰, AMR2 지원 출동).
ROLE_UNIT = {'SCOUT': 'AMR1', 'SUPPORT': 'AMR2'}
# Characters of a UUID shown on screen; the full ID stays in the title tooltip.
SHORT_ID = 8
# Battery bar colours switch below these percentages ([제안] 관제·화면, 2026-10-10; display only).
BATTERY_LOW, BATTERY_WARN = 25, 50
# Survey grid spacing on the map, metres.
GRID_M = 1.0
STAGE_ORDER = ('UNCONFIRMED', 'NOT_RESCUED', 'SUPPORTING', 'RESCUED')


def _short(ident):
    if ident is None:
        return MISSING
    text = str(ident)
    shown = text if len(text) <= SHORT_ID + 1 else text[:SHORT_ID]
    return f'<span class="id" title="{escape(text)}">{escape(shown)}</span>'


def _word(kind, code):
    return _text(label(kind, code))


def _elapsed(seconds):
    if seconds is None or seconds < 0:
        return MISSING
    seconds = int(seconds)
    if seconds < 3600:
        return f'{seconds // 60}분 {seconds % 60:02d}초'
    return f'{seconds // 3600}시간 {seconds % 3600 // 60:02d}분'


def _pose(pose):
    if pose is None:
        return MISSING
    text = f"({pose['x']:.2f}, {pose['y']:.2f}) m"
    if pose.get('yaw') is not None:
        text += f" {math.degrees(pose['yaw']):.0f}°"
    if pose['frame_id'] != MAP_FRAME:
        text += f" [{pose['frame_id']}]"
    return escape(text)


def _victim_path(x, y, size):
    """INSARAG-style victim mark: a V whose point sits on the reported position."""
    return (f'M{x - size:.2f},{y - 1.6 * size:.2f} L{x:.2f},{y:.2f} '
            f'L{x + size:.2f},{y - 1.6 * size:.2f}')


def _victim_icon(stage):
    """Small V for lists and the legend, same shape and colour as on the map."""
    strike = '<line x1="2" y1="7" x2="14" y2="7"/>' if stage == 'RESCUED' else ''
    return (f'<svg class="vicon {stage.lower()}" viewBox="0 0 16 16" aria-hidden="true">'
            f'<path d="{_victim_path(8, 13, 5)}"/>{strike}</svg>')


# ---- band (fragment) ------------------------------------------------------------------------

def _band(view):
    """Mission mode across the top. The exit count is not drawn: one fixed camera (db-schema 2절)."""
    mission = view['mission']
    mode = mission['mode']
    built = view['built_at_ns']
    facts = []
    if mode == 'DISASTER':
        elapsed = (built - mission['trigger_ns']) / 1e9
        facts = [('트리거', _time(mission['trigger_ns'], built)), ('경과', _elapsed(elapsed)),
                 ('진동 감지', f'{mission["sign_count"]}건')]
        note = '현장 카메라의 진동 감지로 판단'
    elif mode == 'NORMAL':
        note = '진동 감지 없음'
    else:
        note = '현장 사건을 읽지 못해 모드를 알 수 없음'
    dl = ''.join(f'<div><dt>{name}</dt><dd>{value}</dd></div>' for name, value in facts)
    return (f'<section id="mission" class="band {(mode or "unknown").lower()}" data-mode="{escape(mode or "")}">'
            f'<h1 class="mode-word">{escape(mission["label"])}</h1>'
            f'<dl class="facts">{dl}</dl><p class="mode-note">{note}</p>'
            f'<p id="built" data-built-ns="{built}">서버 집계 {_time(built)}</p></section>')


def _warnings(view):
    if not view['warnings']:
        return '<section id="warnings" class="empty"></section>'
    items = ''.join(f'<li>{escape(w)}</li>' for w in view['warnings'])
    return f'<section id="warnings"><h2>확인할 것 {len(view["warnings"])}</h2><ul>{items}</ul></section>'


# ---- map (fragment) -------------------------------------------------------------------------

def _grid(layer):
    """Metre grid and axis numbers in image pixels (survey-plan look)."""
    width, height, res = layer['width_px'], layer['height_px'], layer['resolution']
    x0, y0 = layer['origin'][0], layer['origin'][1]
    lines, numbers = [], []
    for m in range(math.ceil(x0 / GRID_M), math.floor((x0 + width * res) / GRID_M) + 1):
        px = round((m * GRID_M - x0) / res, 2)
        lines.append(f'<line class="{"axis" if m == 0 else "grid"}" x1="{px}" y1="0" x2="{px}" y2="{height}"/>')
        numbers.append(f'<text class="tick" x="{px}" y="{height + 3.2}" text-anchor="middle">{m * GRID_M:g}</text>')
    for m in range(math.ceil(y0 / GRID_M), math.floor((y0 + height * res) / GRID_M) + 1):
        py = round(height - (m * GRID_M - y0) / res, 2)
        lines.append(f'<line class="{"axis" if m == 0 else "grid"}" x1="0" y1="{py}" x2="{width}" y2="{py}"/>')
        numbers.append(f'<text class="tick" x="-1.2" y="{py + 0.8}" text-anchor="end">{m * GRID_M:g}</text>')
    bar = GRID_M / res
    scale = (f'<g class="scale"><line x1="0" y1="{height + 6}" x2="{bar:g}" y2="{height + 6}"/>'
             f'<text x="{bar + 1:g}" y="{height + 6.8}">{GRID_M:g} m</text></g>')
    return ''.join(lines) + ''.join(numbers) + scale


def _map(view):
    """Pre-quake map as the background of one SVG, so marks and image always scale together."""
    layer = view['map']
    if not layer.get('available'):
        return ('<section id="map" class="placeholder"><h2>지도</h2>'
                '<p>기준 지도가 없습니다. 실행할 때 <code>--map assets/maps/pre_quake/map.yaml</code> 을 주세요.</p>'
                '</section>')
    width, height = layer['width_px'], layer['height_px']
    marks = []
    for target in layer['targets']:
        stage = target['stage']
        x, y = target['px'], target['py']
        strike = (f'<line class="strike" x1="{x - TARGET_MARK_PX * 1.2:.2f}" y1="{y - TARGET_MARK_PX * 0.8:.2f}" '
                  f'x2="{x + TARGET_MARK_PX * 1.2:.2f}" y2="{y - TARGET_MARK_PX * 0.8:.2f}"/>') if stage == 'RESCUED' else ''
        marks.append(
            f'<g class="victim {stage.lower()}"><title>{escape(target["id"])} {escape(STAGES[stage])}</title>'
            f'<path class="mark" d="{_victim_path(x, y, TARGET_MARK_PX)}"/>{strike}'
            f'<text class="label" x="{x + TARGET_MARK_PX + 0.5:.2f}" y="{y - TARGET_MARK_PX:.2f}">'
            f'{escape(target["id"][:4])}</text></g>')
    for robot in layer['robots']:
        x, y = robot['px'], robot['py']
        role = robot.get('role') or 'UNKNOWN'
        unit = ROLE_UNIT.get(role, robot['id'])
        if robot['yaw'] is None:
            body = f'<circle class="body" cx="{x}" cy="{y}" r="{ROBOT_MARK_PX}"/>'
        else:
            # Image rows grow downwards, so +yaw (counter-clockwise in the map) is -y here.
            points = []
            for angle, reach in ((0.0, HEADING_PX), (2.5, ROBOT_MARK_PX), (-2.5, ROBOT_MARK_PX)):
                a = robot['yaw'] + angle
                points.append(f'{x + reach * math.cos(a):.2f},{y - reach * math.sin(a):.2f}')
            body = f'<polygon class="heading" points="{" ".join(points)}"/>'
        marks.append(
            f'<g class="robot {escape(role.lower())}"><title>{escape(robot["id"])}</title>{body}'
            f'<text class="label" x="{x + ROBOT_MARK_PX + 1:.2f}" y="{y + ROBOT_MARK_PX + 2.4:.2f}">'
            f'{escape(unit)}</text></g>')
    unplotted = [f'{w["id"][:SHORT_ID]}: 좌표계 {w["frame_id"]} — 지도 좌표계가 아님' for w in layer['frame_mismatch']]
    unplotted += [f'{w["id"][:SHORT_ID]}: 지도 밖 ({w["x"]:.2f}, {w["y"]:.2f}) m' for w in layer['off_map']]
    notes = ''.join(f'<li>{escape(text)}</li>' for text in unplotted)
    legend = ''.join(f'<li>{_victim_icon(stage)}{escape(STAGES[stage])}</li>' for stage in STAGE_ORDER)
    legend += '<li><svg class="vicon robot" viewBox="0 0 16 16" aria-hidden="true"><polygon points="14,8 3,2.5 3,13.5"/></svg>로봇과 진행 방향</li>'
    pad_left, pad_top, pad_bottom = 5, 2, 9
    return (f'<section id="map" class="map"><header><h2>지도</h2><ul class="legend">{legend}</ul></header>'
            f'<svg class="plan" viewBox="{-pad_left} {-pad_top} {width + pad_left + 2} {height + pad_top + pad_bottom}" '
            f'preserveAspectRatio="xMidYMid meet" role="img" aria-label="기준 지도 위 로봇과 요구조자 위치">'
            f'<image href="{escape(layer["url"])}" x="0" y="0" width="{width}" height="{height}" '
            f'preserveAspectRatio="none"/>'
            f'<rect class="frame" x="0" y="0" width="{width}" height="{height}"/>'
            f'{_grid(layer)}{"".join(marks)}</svg>'
            f'<p class="map-note" title="{escape(layer["note"])}">재난 전 기준 지도, 격자 한 칸 {GRID_M:g} m. '
            '좌표가 같은 지도 세션인지는 아직 확인하지 않습니다 (Q-09).</p>'
            + (f'<ul class="unplotted"><li class="head">지도에 그리지 못한 위치</li>{notes}</ul>' if notes else '')
            + '</section>')


# ---- units and victims (fragment) -----------------------------------------------------------

def _battery(robot):
    percent = robot['battery_percent']
    if percent is None:
        return '<div class="battery none"><span class="value">배터리 값 없음</span></div>'
    level = 'low' if percent < BATTERY_LOW else 'warn' if percent < BATTERY_WARN else 'ok'
    width = max(0.0, min(100.0, percent))
    return (f'<div class="battery {level}"><span class="bar"><span style="width:{width:.0f}%"></span></span>'
            f'<span class="value">{percent:.0f}%</span>'
            f'<span class="age">{_age(robot["battery_age_s"])} 전</span></div>')


def _robots(view):
    units = []
    for robot in view['robots']:
        role = robot['role']
        unit = ROLE_UNIT.get(role, '')
        head = (f'<h3><span class="unit-name">{escape(unit or robot["robot_id"])}</span>'
                f'<span class="unit-role">{_word("role", role) if role else ""}</span>'
                f'<span class="unit-id">{escape(robot["robot_id"])}</span></h3>')
        if not robot['has_status']:
            units.append(f'<section class="unit missing" data-robot="{escape(robot["robot_id"])}">{head}'
                         '<p class="alert">상태를 한 번도 받지 못했습니다. 로봇 상태 발행과 operations 연결을 확인하세요.</p>'
                         '</section>')
            continue
        motion = robot['motion_state']
        stale = (f'<p class="alert">낡음: {_age(robot["received_age_s"])} 동안 새 상태가 없습니다. '
                 '표시 값은 그때의 것입니다.</p>') if robot['stale'] else ''
        units.append(
            f'<section class="unit {(role or "unknown").lower()}{" stale" if robot["stale"] else ""}" '
            f'data-robot="{escape(robot["robot_id"])}">{head}'
            f'<p class="state"><span class="motion {escape((motion or "unknown").lower())}">{_word("motion", motion)}</span>'
            f'<span class="ready">{_word("readiness", robot["readiness"])}</span></p>'
            + _battery(robot)
            + f'<dl><div><dt>위치</dt><dd>{_pose(robot["pose"])}</dd></div>'
            f'<div><dt>작업</dt><dd>{_short(robot["active_task_id"])}</dd></div>'
            f'<div><dt>수신</dt><dd>{_age(robot["received_age_s"])} 전</dd></div></dl>'
            + stale + '</section>')
    return f'<section id="robots"><h2>로봇</h2>{"".join(units)}</section>'


def _targets(view):
    counts = ''.join(
        f'<li class="{stage.lower()}">{_victim_icon(stage)}<span>{escape(STAGES[stage])}</span>'
        f'<b>{view["stage_counts"][stage]}</b></li>' for stage in STAGE_ORDER)
    rows = []
    for target in view['targets']:
        position = target['position']
        where = MISSING if position is None else escape(f"({position['x']:.2f}, {position['y']:.2f}) m")
        support = ''
        if target['support_state'] not in (None, 'NONE'):
            support = (f'<span class="support">지원 {_word("support", target["support_state"])}'
                       + (f' {escape(target["support_robot_id"])}' if target['support_robot_id'] else '') + '</span>')
        notes = ''.join(f'<li>{escape(n)}</li>' for n in target['warnings'] + target['rescue_reasons'])
        rows.append(
            f'<li class="victim-row {target["stage"].lower()}" data-stage="{target["stage"]}">'
            f'{_victim_icon(target["stage"])}'
            f'<div class="who">{_short(target["target_id"])}<span class="stage-word">{escape(target["stage_label"])}</span>'
            f'<span class="rule" title="표시 규칙 (README 구조 상태 4단계)">{target["rule"]}</span></div>'
            f'<div class="what"><span>{where}</span><span>구조 {_word("rescue", target["rescue_state"])}</span>'
            f'<span>{_word("approach", target["approach_result"])}</span>{support}'
            f'<span class="receipt">보고 {target["report_count"]}건 {_word("receipt", target["receipt_status"])}</span></div>'
            + (f'<ul class="notes">{notes}</ul>' if notes else '') + '</li>')
    body = ''.join(rows) or '<li class="none">아직 보고된 요구조자가 없습니다.</li>'
    return (f'<section id="targets"><h2>요구조자 {len(view["targets"])}명</h2>'
            f'<ul class="stage-counts">{counts}</ul><ul class="victims">{body}</ul></section>')


# ---- event log (fragment) -------------------------------------------------------------------

def _event_text(event):
    if event['kind'] == 'REPORT':
        return (f'{_word("confirmation", event["confirmation"])}, {_word("approach", event["approach"])}, '
                f'{_word("receipt", event["receipt"])}')
    text = '' if event['confidence'] is None else f'신뢰도 {event["confidence"]:.2f}'
    if event['note']:
        text += f' {escape(event["note"])}'
    return text.strip() or MISSING


def _events(view):
    rows = []
    built = view['built_at_ns']
    for event in view['events']:
        rows.append(
            f'<tr class="event {event["kind"].lower()}" data-ref="{escape(event["ref_id"])}" '
            f'data-event-ns="{event["event_ns"]}" data-received-ns="{event["received_at_ns"] or ""}" '
            f'data-kind="{escape(event["kind"])}" data-clock="{escape(event["clock_domain_id"])}">'
            f'<td class="t" title="시계 {escape(event["clock_domain_id"])}">{_time(event["event_ns"], built)}</td>'
            f'<td class="kind">{_word("event", event["kind"])}</td>'
            f'<td>{escape(event["source"])}</td><td>{_short(event["subject"])}</td>'
            f'<td class="text">{_event_text(event)}</td>'
            f'<td class="t">{_time(event["received_at_ns"], built)}</td>'
            f'<td class="t rendered">{MISSING}</td><td class="t latency">{MISSING}</td></tr>')
    shown = len(view['events'])
    more = f' (최근 {shown}건 표시)' if view['event_total'] > shown else ''
    return (f'<section id="events"><h2>사건 기록 {view["event_total"]}건{more}</h2><div class="scroll">'
            '<table><thead><tr><th>발생</th><th>종류</th><th>출처</th><th>대상</th><th>내용</th>'
            '<th>장부 기록</th><th>화면 표시</th><th title="화면 표시 − 발생 (KPI-04)">지연</th></tr></thead>'
            f'<tbody>{"".join(rows)}</tbody></table></div></section>')


def render_fragment(view):
    """The part of the page that the browser replaces on every poll (its children sit in the page grid)."""
    return (_band(view) + _warnings(view) + _map(view) + '<div id="middle">' + _robots(view) + _targets(view)
            + '</div>' + _events(view))


# ---- page-level parts (outside the polled fragment) -----------------------------------------

def _commands(view):
    """Buttons grouped by consequence. All disabled until the operations command API exists."""
    groups = {'approve': [], 'mission': [], 'stop': []}
    for command in view['commands']:
        groups[command['group']].append(
            f'<button type="button" class="cmd {command["group"]}" disabled data-command="{escape(command["name"])}" '
            f'title="{escape(command["reason"])}">{escape(command["label"])}</button>')
    reason = escape(view['commands'][0]['reason']) if view['commands'] else ''
    return (f'<section id="commands"><h2>명령</h2><p class="cmd-note">{reason}</p>'
            f'<div class="cmd-group approve">{"".join(groups["approve"])}</div>'
            f'<div class="cmd-group mission">{"".join(groups["mission"])}</div>'
            f'<div class="cmd-group stop">{"".join(groups["stop"])}</div>'
            '<p class="estop">위험할 때는 현장 비상 정지 장치를 쓰세요. 이 화면의 정지는 보조 수단입니다.</p>'
            '</section>')


_STYLE = """
:root{
  --concrete:#dde2e1; --panel:#f4f6f5; --ink:#17252b; --ink-2:#4a5a60; --rule:#b4bebd;
  --signal:#c8102e; --rescue:#d9480f; --rescue-tint:#fbe1d3; --info:#005a9c; --info-tint:#dbe9f4;
  --safe:#00784a; --safe-tint:#d7eee3; --pending:#7a878a; --caution:#f2b600; --caution-tint:#fff3c4;
  --font:"Noto Sans CJK KR","Noto Sans KR","Pretendard","Apple SD Gothic Neo","Malgun Gothic",system-ui,sans-serif;
}
*{box-sizing:border-box}
html,body{margin:0}
body{font-family:var(--font);font-size:14px;line-height:1.45;color:var(--ink);background:var(--concrete);
  font-variant-numeric:tabular-nums}
h1,h2,h3,p{margin:0}
h2{font-size:15px;font-weight:700;margin-bottom:8px}
code{font-size:13px;background:var(--panel);padding:0 4px}
a{color:var(--info)}
:focus-visible{outline:3px solid var(--info);outline-offset:2px}
.id{font-variant-numeric:tabular-nums;letter-spacing:.02em}
dl{margin:0}

/* ---- console grid ---- */
.console{display:grid;gap:10px;padding:10px;min-height:100vh;
  grid-template-columns:minmax(0,1fr) 400px 470px;
  grid-template-rows:auto auto minmax(0,1fr) 230px;
  grid-template-areas:"band band top" "warn warn side" "map middle side" "log log side"}
#live{display:contents}
#mission{grid-area:band} #warnings{grid-area:warn} #map{grid-area:map} #middle{grid-area:middle}
#events{grid-area:log} #topbar{grid-area:top} #side{grid-area:side}
@media (min-width:1400px) and (min-height:820px){ .console{height:100vh;overflow:hidden} }
@media (max-width:1399px){
  .console{grid-template-columns:minmax(0,1fr);grid-template-rows:none;
    grid-template-areas:"top" "band" "warn" "map" "side" "middle" "log"}
}
#map,#middle>section,#events,#side>section{background:var(--panel);border:1px solid var(--rule);padding:10px 12px}
#middle{display:flex;flex-direction:column;gap:10px;min-height:0}
#side{display:flex;flex-direction:column;gap:10px;min-height:0}

/* ---- band ---- */
.band{display:flex;align-items:center;gap:28px;padding:10px 18px;border:1px solid var(--rule);background:var(--panel)}
.band .mode-word{font-size:30px;font-weight:700;letter-spacing:.04em;min-width:3.2em}
.band .facts{display:flex;gap:24px}
.band dt{font-size:12px;color:var(--ink-2)} .band dd{margin:0;font-size:18px;font-weight:500}
.band .mode-note{color:var(--ink-2)} #built{margin-left:auto;font-size:12px;color:var(--ink-2)}
.band.disaster{background:var(--signal);border-color:var(--signal);color:#fff}
.band.disaster dt,.band.disaster .mode-note,.band.disaster #built{color:#ffd9de}
.band.unknown .mode-word{color:var(--pending)}
#topbar{display:grid;grid-template-columns:auto 1fr;align-content:center;align-items:center;gap:6px 10px;
  padding:8px 12px;background:var(--panel);border:1px solid var(--rule)}
#topbar .who{grid-column:2;grid-row:1;justify-self:end;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
#topbar .kpi{grid-column:1/-1;grid-row:2}
#poll{font-size:13px;display:flex;gap:6px;align-items:center}
#poll::before{content:"";width:9px;height:9px;border-radius:50%;background:var(--safe)}
#poll.lost{color:var(--signal);font-weight:700} #poll.lost::before{background:var(--signal)}
#poll.wait::before{background:var(--pending)}
#topbar .kpi{display:flex;gap:8px;align-items:center;font-size:12.5px;color:var(--ink-2)}
#kpi.ok{color:var(--safe)} #kpi.over{color:var(--signal);font-weight:700}
.mini{font:inherit;font-size:12px;padding:2px 8px;border:1px solid var(--rule);background:#fff;color:var(--ink);border-radius:3px;cursor:pointer}

/* ---- warnings ---- */
#warnings{background:var(--caution-tint);border:1px solid var(--caution);border-left:8px solid var(--caution);padding:6px 12px}
#warnings.empty{display:none}
#warnings h2{display:inline;margin-right:12px;font-size:14px}
#warnings ul{display:inline;margin:0;padding:0;list-style:none}
#warnings li{display:inline}
#warnings li+li::before{content:"／";margin:0 8px;color:var(--ink-2)}

/* ---- map ---- */
#map{display:flex;flex-direction:column;min-height:0}
#map header{display:flex;align-items:baseline;gap:16px}
.legend{display:flex;gap:14px;margin:0;padding:0;list-style:none;font-size:12.5px;color:var(--ink-2)}
.legend li{display:flex;align-items:center;gap:4px}
.plan{flex:1;min-height:0;width:100%;background:#fff}
.plan image{image-rendering:pixelated;opacity:.5}
.plan .frame{fill:none;stroke:var(--ink);stroke-width:.25}
.plan .grid{stroke:var(--info);stroke-width:.08;opacity:.45}
.plan .axis{stroke:var(--info);stroke-width:.16;opacity:.7}
.plan .tick{font-size:2.1px;fill:var(--ink-2)}
.plan .scale line{stroke:var(--ink);stroke-width:.5} .plan .scale text{font-size:2.3px;fill:var(--ink)}
.plan .label{font-size:2.6px;font-weight:700;paint-order:stroke;stroke:#fff;stroke-width:.6}
.victim .mark,.victim .strike{fill:none;stroke-width:.75;stroke-linecap:round;stroke-linejoin:round}
.victim.unconfirmed .mark{stroke:var(--pending);stroke-dasharray:.9 .6} .victim.unconfirmed .label{fill:var(--pending)}
.victim.not_rescued .mark{stroke:var(--rescue)} .victim.not_rescued .label{fill:var(--rescue)}
.victim.supporting .mark{stroke:var(--info)} .victim.supporting .label{fill:var(--info)}
.victim.rescued .mark,.victim.rescued .strike{stroke:var(--safe)} .victim.rescued .label{fill:var(--safe)}
.robot .heading,.robot .body{fill:var(--ink);stroke:#fff;stroke-width:.35}
.robot.support .heading,.robot.support .body{fill:var(--info)}
.robot .label{fill:var(--ink)} .robot.support .label{fill:var(--info)}
.map-note{font-size:12px;color:var(--ink-2);margin-top:4px}
.unplotted{margin:6px 0 0;padding:6px 10px;list-style:none;background:var(--caution-tint);border-left:6px solid var(--caution);font-size:13px}
.unplotted .head{font-weight:700}
#map.placeholder p{margin-top:40px;text-align:center;color:var(--ink-2)}

/* ---- small V icon ---- */
.vicon{width:16px;height:16px;flex:none;vertical-align:-3px}
.vicon path,.vicon line{fill:none;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
.vicon.unconfirmed path{stroke:var(--pending);stroke-dasharray:2.2 1.6}
.vicon.not_rescued path{stroke:var(--rescue)} .vicon.supporting path{stroke:var(--info)}
.vicon.rescued path,.vicon.rescued line{stroke:var(--safe)}
.vicon.robot polygon{fill:var(--ink)}

/* ---- robots ---- */
.unit{border-left:6px solid var(--ink);padding:6px 0 8px 10px}
.unit+.unit{margin-top:8px;border-top:1px solid var(--rule);padding-top:10px}
.unit.support{border-left-color:var(--info)}
.unit h3{display:flex;align-items:baseline;gap:8px}
.unit-name{font-size:20px;font-weight:700} .unit-role{font-weight:500} .unit-id{color:var(--ink-2);font-size:12.5px;margin-left:auto}
.unit .state{display:flex;gap:12px;margin:2px 0 6px;font-size:15px}
.motion{font-weight:700} .motion.stop_unconfirmed{color:var(--signal)}
.battery{display:flex;align-items:center;gap:8px;margin-bottom:6px}
.battery .bar{flex:1;height:10px;background:#fff;border:1px solid var(--rule);overflow:hidden}
.battery .bar span{display:block;height:100%;background:var(--safe)}
.battery.warn .bar span{background:var(--caution)} .battery.low .bar span{background:var(--signal)}
.battery.low .value{color:var(--signal);font-weight:700}
.battery .value{min-width:3.2em;text-align:right;font-weight:500} .battery .age{font-size:12px;color:var(--ink-2)}
.battery.none .value{color:var(--ink-2);text-align:left}
.unit dl{display:grid;grid-template-columns:auto 1fr;column-gap:10px;font-size:13px}
.unit dl div{display:contents} .unit dt{color:var(--ink-2)} .unit dd{margin:0}
.unit .alert{margin-top:6px;padding:4px 8px;background:var(--caution-tint);font-size:13px}
.unit.stale,.unit.missing{border-left-color:var(--caution)}
.unit.stale .state,.unit.stale .battery,.unit.stale dl{opacity:.55}

/* ---- victims ---- */
#targets{display:flex;flex-direction:column;min-height:0;flex:1}
.stage-counts{display:grid;grid-template-columns:repeat(2,1fr);gap:4px;margin:0 0 8px;padding:0;list-style:none}
.stage-counts li{display:flex;align-items:center;gap:6px;padding:3px 8px;background:#fff;border-left:4px solid var(--pending);font-size:13px}
.stage-counts li b{margin-left:auto;font-size:18px}
.stage-counts .not_rescued{border-left-color:var(--rescue)} .stage-counts .supporting{border-left-color:var(--info)}
.stage-counts .rescued{border-left-color:var(--safe)}
.victims{margin:0;padding:0;list-style:none;overflow:auto;min-height:0}
.victim-row{display:grid;grid-template-columns:20px 1fr;column-gap:6px;padding:6px 0;border-top:1px solid var(--rule)}
.victim-row .vicon{grid-row:span 3;margin-top:2px}
.victim-row .who{display:flex;gap:8px;align-items:baseline}
.victim-row .stage-word{font-weight:700} .victim-row .rule{font-size:11px;color:var(--ink-2)}
.victim-row.not_rescued .stage-word{color:var(--rescue)} .victim-row.supporting .stage-word{color:var(--info)}
.victim-row.rescued .stage-word{color:var(--safe)} .victim-row.unconfirmed .stage-word{color:var(--pending)}
.victim-row .what{display:flex;flex-wrap:wrap;column-gap:12px;font-size:13px;color:var(--ink-2)}
.victim-row .support{color:var(--info);font-weight:500}
.victim-row .notes{grid-column:2;margin:4px 0 0;padding:2px 8px;list-style:none;background:var(--caution-tint);font-size:12.5px}
.victims .none{padding:12px 0;color:var(--ink-2)}

/* ---- events ---- */
#events{display:flex;flex-direction:column;min-height:0}
#events .scroll{overflow:auto;min-height:0;flex:1}
#events table{border-collapse:collapse;width:100%;font-size:13px}
#events th{position:sticky;top:0;background:var(--panel);text-align:left;font-weight:500;color:var(--ink-2);
  border-bottom:1px solid var(--ink);padding:2px 8px;white-space:nowrap}
#events td{padding:3px 8px;border-bottom:1px solid #d5dbda;vertical-align:top}
#events td.t{white-space:nowrap}
#events tr.disaster_sign td.kind{color:var(--signal);font-weight:700}
#events tr.target_disappeared_candidate td.kind{color:var(--safe);font-weight:700}
#events tr.fresh td{background:var(--caution-tint)}

/* ---- side: cameras and commands ---- */
#cameras figure{margin:0;position:relative}
#cameras img{display:block;width:100%;aspect-ratio:16/9;object-fit:contain;background:#26343a;color:transparent}
#cameras .nosignal{display:none;position:absolute;left:0;right:0;top:0;aspect-ratio:16/9;align-items:center;
  justify-content:center;text-align:center;padding:0 24px;color:#e8eeee;font-size:14px}
#cameras figure.lost .nosignal{display:flex}
#cameras figure.lost img{visibility:hidden} #cameras figure{background:#26343a}
#cameras figcaption{display:flex;justify-content:space-between;font-size:13px;padding-top:4px;background:var(--panel)}
#cameras figcaption .lost{color:var(--signal);font-weight:700}
.camera-empty p{color:var(--ink-2);font-size:13px}
#commands .cmd-note{font-size:12.5px;color:var(--ink-2);margin-bottom:8px}
.cmd-group{display:grid;gap:6px;margin-bottom:8px}
.cmd-group.approve{grid-template-columns:1fr}
.cmd-group.mission{grid-template-columns:repeat(3,1fr)}
.cmd{font:inherit;border:1px solid var(--ink);background:#fff;color:var(--ink);padding:8px 6px;border-radius:3px;cursor:pointer}
.cmd.approve{font-size:17px;font-weight:700;padding:12px;background:var(--info);border-color:var(--info);color:#fff}
.cmd.stop{width:100%;font-size:18px;font-weight:700;padding:12px;background:var(--signal);border-color:var(--signal);color:#fff}
.cmd:disabled{cursor:not-allowed;opacity:.38}
.estop{font-size:12.5px;padding:6px 8px;border-left:6px solid var(--signal);background:#fff}

/* ---- forms (login, accounts) ---- */
.card{max-width:420px;margin:12vh auto;padding:24px;background:var(--panel);border:1px solid var(--rule)}
.card h1{font-size:22px;margin-bottom:12px} .card button{margin-top:16px}
.card label{display:block;margin:12px 0 4px}.card input{width:100%;padding:8px;font:inherit;border:1px solid var(--rule)}
.card button,form.inline button{font:inherit;padding:8px 14px;border:1px solid var(--ink);background:var(--ink);color:#fff;border-radius:3px;cursor:pointer}
#topbar form.inline button{padding:3px 10px;background:#fff;color:var(--ink)}
.flash{padding:8px 12px;margin:8px 0;border-left:6px solid var(--ink-2);background:#fff}
.flash.danger{border-color:var(--signal)}.flash.success{border-color:var(--safe)}.flash.info{border-color:var(--info)}
form.inline{display:inline}form.inline input{width:9em;padding:4px;font:inherit}form.inline select{margin-right:4px;font:inherit}
.admin{max-width:1200px;margin:0 auto;padding:16px}
.admin table{border-collapse:collapse;width:100%;background:var(--panel);margin-top:12px}
.admin th,.admin td{border-bottom:1px solid var(--rule);padding:6px 8px;text-align:left}
"""

# Stamps each event row with the browser time it first appeared (KPI-04). Only GET requests.
_SCRIPT = """
const firstSeen = new Map();
window.kpi04 = [];
const FRESH_MS = 3000;
let lastOk = Date.now(), loaded = false;
// Same formats as render.py _time / _age, so the server and browser columns read alike.
function clock(ms) {
  const d = new Date(ms), pad = (n, w) => String(n).padStart(w || 2, '0');
  return pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds()) + '.' + pad(d.getMilliseconds(), 3);
}
function age(s) {
  const sign = s < 0 ? '-' : '', v = Math.abs(s);
  if (v < 60) return sign + v.toFixed(2) + 's';
  if (v < 3600) return sign + Math.floor(v / 60) + '분 ' + String(Math.floor(v % 60)).padStart(2, '0') + '초';
  if (v < 86400) return sign + Math.floor(v / 3600) + '시간 ' + Math.floor(v % 3600 / 60) + '분';
  return sign + Math.floor(v / 86400) + '일';
}
function stamp() {
  const now = Date.now();
  for (const row of document.querySelectorAll('tr.event')) {
    const ref = row.dataset.ref, eventMs = Number(row.dataset.eventNs) / 1e6;
    if (!firstSeen.has(ref)) {
      firstSeen.set(ref, now);
      // live: the row arrived while this page was open. Rows already there at load are history,
      // not display latency, and stay out of the KPI-04 summary.
      window.kpi04.push({ref_id: ref, kind: row.dataset.kind, clock_domain_id: row.dataset.clock,
                         event_ns: row.dataset.eventNs, received_at_ns: row.dataset.receivedNs || '',
                         rendered_ms: now, latency_s: (now - eventMs) / 1000, live: loaded});
    }
    const shown = firstSeen.get(ref);
    row.dataset.renderedMs = shown;
    row.querySelector('.rendered').textContent = clock(shown);
    row.querySelector('.latency').textContent = age((shown - eventMs) / 1000);
    // Rows that arrived while the page was open stay highlighted for a moment (no animation).
    row.classList.toggle('fresh', loaded && now - shown < FRESH_MS);
  }
  loaded = true;
  kpiSummary();
}
function kpiSummary() {
  const el = document.getElementById('kpi');
  if (!el) return;
  const live = window.kpi04.filter(r => r.live);
  if (!live.length) {
    el.className = '';
    el.textContent = 'KPI-04: 페이지를 연 뒤 들어온 사건 없음';
    return;
  }
  const within = live.filter(r => r.latency_s <= KPI_BUDGET_S).length;
  const worst = Math.max(...live.map(r => r.latency_s));
  el.className = within === live.length ? 'ok' : 'over';
  el.textContent = 'KPI-04: 실시간 사건 ' + live.length + '건 중 ' + KPI_BUDGET_S + '초 이내 ' + within + '건, 최대 ' + age(worst);
}
function csvCell(value) {
  const text = String(value);
  return /[",\\n]/.test(text) ? '"' + text.replace(/"/g, '""') + '"' : text;
}
// Saves the records as a CSV file in the browser (Blob); nothing is sent to the server.
function saveKpi() {
  const keys = ['ref_id', 'kind', 'clock_domain_id', 'event_ns', 'received_at_ns', 'rendered_ms', 'latency_s', 'live'];
  const lines = window.kpi04.map(r => keys.map(k => csvCell(k === 'latency_s' ? r[k].toFixed(3) : r[k])).join(','));
  const url = URL.createObjectURL(new Blob([[keys.join(',')].concat(lines).join('\\n') + '\\n'], {type: 'text/csv'}));
  const link = document.createElement('a');
  link.href = url;
  link.download = 'kpi04-' + new Date().toISOString().replace(/[:.]/g, '-') + '.csv';
  link.click();
  URL.revokeObjectURL(url);
}
function pollStatus(ok, message) {
  const el = document.getElementById('poll');
  if (ok) {
    lastOk = Date.now();
    el.className = '';
    el.textContent = '실시간 갱신 ' + clock(lastOk).slice(0, 8);
  } else {
    el.className = 'lost';
    el.textContent = '갱신 끊김: ' + message + ', ' + age((Date.now() - lastOk) / 1000) + ' 전 값을 보는 중';
  }
}
let timer = null;
async function poll() {
  try {
    const response = await fetch('fragment', {cache: 'no-store'});
    if (response.status === 401 || response.redirected) {
      // Session ended (logout elsewhere, account changed): stop polling and go to the login page
      // instead of pasting whatever came back into the console (AST-50-02).
      clearInterval(timer);
      location.href = response.status === 401 ? '/login' : response.url;
      return;
    }
    if (!response.ok) throw new Error('HTTP ' + response.status);
    document.getElementById('live').innerHTML = await response.text();
    stamp();
    pollStatus(true);
  } catch (error) {
    pollStatus(false, error.message);
  }
}
document.getElementById('kpi-save').addEventListener('click', saveKpi);
stamp();
pollStatus(true);
timer = setInterval(poll, POLL_MS);
"""


def _head(title):
    return ('<!doctype html><html lang="ko"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>{escape(title)}</title><style>{_STYLE}</style></head><body>')


def _flashes(messages):
    return ''.join(f'<div class="flash {escape(category)}">{escape(text)}</div>' for category, text in messages)


def _user_nav(user, csrf):
    """Who is logged in and where they may go. The only form on the dashboard is logout (POST + CSRF)."""
    if user is None:
        return ''
    admin = ' <a href="/admin/accounts">계정 관리</a>' if user['role'] == 'ADMIN' else ''
    return (f'<nav class="who"><span>{escape(user["display_name"])} ({escape(user["role"])})</span>{admin}'
            f' <form class="inline" method="post" action="/logout"><input type="hidden" name="csrf" value="{escape(csrf or "")}">'
            '<button type="submit">로그아웃</button></form></nav>')


def _header(user, csrf):
    """Plain page header for the account pages."""
    return f'<header class="admin-head"><h1>구조 관제</h1>{_user_nav(user, csrf)}</header>'


def _cameras(cameras):
    """Camera streams embedded straight from the camera PC's MJPEG gateway (IF-082).

    Outside the polled fragment, so a poll never restarts a stream. The console does not relay
    video. A broken stream is shown as lost, not hidden.
    """
    if not cameras:
        return ('<section class="camera-empty"><h2>웹캠</h2><p>웹캠 주소가 없습니다. 실행할 때 '
                '<code>--camera cam_a=http://…</code> 로 카메라 PC 의 영상 주소를 주세요.</p></section>')
    figures = ''.join(
        f'<figure><img src="{escape(url)}" alt="{escape(camera_id)} 영상" '
        'onerror="var f=this.parentNode,s=f.querySelector(\'figcaption\').lastChild;'
        'f.classList.add(\'lost\');s.textContent=\'영상 끊김\';s.className=\'lost\'">'
        '<p class="nosignal">영상이 들어오지 않습니다. 카메라 PC 의 영상 서버와 주소를 확인하세요.</p>'
        f'<figcaption><span>{escape(camera_id)}</span><span></span></figcaption></figure>'
        for camera_id, url in cameras)
    return f'<section id="cameras"><h2>웹캠</h2>{figures}</section>'


def render_page(view, user=None, csrf=None, cameras=()):
    """`cameras`: ((camera_id, http(s) URL), ...) from the launch arguments."""
    script = _SCRIPT.replace('POLL_MS', str(POLL_INTERVAL_MS)).replace('KPI_BUDGET_S', f'{KPI04_BUDGET_S:g}')
    return (_head('구조 관제') + '<main class="console">'
            + f'<section id="topbar"><p id="poll" class="wait">연결 중</p><div id="poll-error" hidden></div>'
            + '<div class="kpi"><p id="kpi" title="발생 시각과 이 화면에 처음 그린 시각의 차이. 두 PC 의 시계가 맞을 때만 의미가 있습니다 (C-03)">'
            'KPI-04: 페이지를 연 뒤 들어온 사건 없음</p>'
            '<button type="button" id="kpi-save" class="mini">지연 기록 저장</button></div>'
            + _user_nav(user, csrf) + '</section>'
            + f'<aside id="side">{_cameras(cameras)}{_commands(view)}</aside>'
            + f'<div id="live">{render_fragment(view)}</div></main>'
            f'<script>{script}</script></body></html>')


def render_login(csrf, messages=()):
    """Login page (tutor day5 requirement: login page backed by login data in SQLite)."""
    return (_head('구조 관제 로그인') + '<div class="card"><h1>구조 관제 로그인</h1>' + _flashes(messages)
            + '<form method="post" action="/login">'
            f'<input type="hidden" name="csrf" value="{escape(csrf)}">'
            '<label for="username">아이디</label><input id="username" name="username" autocomplete="username" required>'
            '<label for="password">비밀번호</label>'
            '<input id="password" name="password" type="password" autocomplete="current-password" required>'
            '<p><button type="submit">로그인</button></p></form></div></body></html>')


def render_accounts(accounts, user, csrf, messages=()):
    """ADMIN only: list accounts, create a customer, update or delete one."""
    hidden = f'<input type="hidden" name="csrf" value="{escape(csrf)}">'
    rows = []
    for account in accounts:
        u = escape(account['username'])
        self_row = account['username'] == user['username']
        role_options = ''.join(f'<option value="{r}"{" selected" if account["role"] == r else ""}>{r}</option>'
                               for r in ('ADMIN', 'CUSTOMER'))
        status_options = ''.join(f'<option value="{st}"{" selected" if account["status"] == st else ""}>{st}</option>'
                                 for st in ('ACTIVE', 'INACTIVE'))
        rows.append(
            f'<tr><td>{u}</td><td>'
            f'<form class="inline" method="post" action="/admin/accounts/{u}">{hidden}'
            '<input type="hidden" name="_action" value="update">'
            f'<input name="display_name" value="{escape(account["display_name"])}" placeholder="표시 이름">'
            f'<input name="email" value="{escape(account["email"])}" placeholder="이메일">'
            f'<select name="role">{role_options}</select><select name="status">{status_options}</select>'
            '<input name="password" type="password" placeholder="새 비밀번호(선택)" autocomplete="new-password">'
            '<button type="submit">저장</button></form></td><td>'
            + ('<em>본인</em>' if self_row else
               f'<form class="inline" method="post" action="/admin/accounts/{u}">{hidden}'
               '<input type="hidden" name="_action" value="delete">'
               '<button type="submit" onclick="return confirm(\'삭제할까요?\')">삭제</button></form>')
            + f'</td><td>{_time(account["updated_at_ns"])}</td></tr>')
    return (_head('계정 관리') + '<div class="admin">' + _header(user, csrf) + _flashes(messages)
            + '<h2>계정 추가</h2><form class="inline" method="post" action="/admin/accounts">' + hidden
            + '<input name="username" placeholder="아이디(소문자)" required>'
              '<input name="display_name" placeholder="표시 이름">'
              '<input name="email" placeholder="이메일">'
              '<select name="role"><option value="CUSTOMER" selected>CUSTOMER</option><option value="ADMIN">ADMIN</option></select>'
              '<input name="password" type="password" placeholder="비밀번호(8자 이상)" autocomplete="new-password" required>'
              '<button type="submit">추가</button></form>'
            + '<table id="accounts"><caption>계정</caption><thead><tr><th>아이디</th><th>정보 · 역할 · 상태 · 비밀번호</th>'
              '<th>삭제</th><th>갱신</th></tr></thead><tbody>' + ''.join(rows) + '</tbody></table>'
            + '<p><a href="/">관제 화면으로</a></p></div></body></html>')
