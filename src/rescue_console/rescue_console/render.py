"""화면 모델 -> HTML. 표준 라이브러리만 쓰므로 Flask 없이 페이지를 시험한다."""

from datetime import datetime
from html import escape
import json
import math

from .labels import label
from .mapimage import MAP_FRAME
from .view import STAGES, ConsoleError

# 브라우저 폴링 주기. KPI-04 예산은 3 s(BRD v0.1)이고, 1 s 폴링이면 2 s 가 남아
# 장부 기록·읽기·전송에 쓴다([제안] 관제·화면, 2026-10-08; README 「KPI-04 측정」).
POLL_INTERVAL_MS = 1000
# KPI-04 예산: 사건 시각부터 운영자 화면에 보일 때까지(BRD v0.1 KPI-04, 3 s).
KPI04_BUDGET_S = 3.0
MISSING = '—'
# 지도 표식 크기, 지도 픽셀 단위(재난 전 지도는 0.05 m/px): 로봇 화살표의 반폭과
# 앞코 길이, 요구조자 V 의 반폭. 표시 전용이다([제안] 관제·화면, 2026-10-10).
ROBOT_MARK_PX, HEADING_PX, TARGET_MARK_PX = 2.6, 4.8, 2.0


def _text(value):
    """아무 값이나 이스케이프한 글자로 바꾼다. None 은 MISSING 으로 보인다."""
    return MISSING if value is None else escape(str(value))


def _time(ns, ref_ns=None):
    """ns 시각을 시계 시각으로 보인다. `ref_ns` 와 날짜가 다르면 날짜를 붙인다."""
    if ns is None:
        return MISSING
    moment = datetime.fromtimestamp(ns / 1e9)
    same_day = ref_ns is None or moment.date() == datetime.fromtimestamp(ref_ns / 1e9).date()
    stamp = moment.strftime('%H:%M:%S.%f' if same_day else '%Y-%m-%d %H:%M:%S.%f')[:-3]
    return f'<span title="{ns}">{stamp}</span>'


def _age(seconds):
    """짧은 나이 글자: 0.4s, 12.0s, 3분 05초, 2시간 10분, 41일. 음수는 시계가 이 호스트보다 앞섰다는 뜻이다."""
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


# 화면에 보이는 UUID 글자 수. 전체 ID 는 title 툴팁에 남는다.
SHORT_ID = 8
# 배터리 막대 색은 이 백분율 아래에서 바뀐다([제안] 관제·화면, 2026-10-10; 표시 전용).
BATTERY_LOW, BATTERY_WARN = 25, 50
# 지도 위 측량 격자 간격, 미터 단위.
GRID_M = 1.0
STAGE_ORDER = ('UNCONFIRMED', 'NOT_RESCUED', 'SUPPORTING', 'RESCUED')
# 콘솔이 입력을 읽지 못할 때 운영자가 확인할 것, ConsoleError 코드별.
FAILURE_HINTS = {
    'STORAGE_ERROR': '운영 장부나 현장 상태 DB 를 읽지 못했습니다. 실행 인자의 파일 경로와 operations 프로세스가 돌고 있는지 확인하세요.',
    'INVALID_INPUT': '읽은 데이터의 형식이 화면이 기대한 것과 다릅니다. operations 와 콘솔이 같은 버전인지 확인하세요.',
    'POLICY_MISMATCH': '연 파일이 기대한 DB 가 아닙니다(역할·스키마가 다름). 실행 인자의 경로를 확인하세요.',
}
FAILURE_HINT_OTHER = '콘솔을 띄운 터미널의 로그를 확인하세요.'
# 실패 페이지는 이 주기로 스스로 다시 읽으므로, 입력을 읽을 수 있게 되면 화면이 돌아온다.
FAILURE_RETRY_S = 5


def _short(ident):
    """표에 쓰는 짧은 ID: SHORT_ID + 1 자까지는 그대로 보이고, 더 길면 SHORT_ID 자로 자른다.

    전체 ID 는 title 툴팁에 남고, None 은 MISSING 으로 보인다. 페이지 스크립트의 shortId 도 같은 규칙을 쓴다.
    """
    if ident is None:
        return MISSING
    text = str(ident)
    shown = text if len(text) <= SHORT_ID + 1 else text[:SHORT_ID]
    return f'<span class="id" title="{escape(text)}">{escape(shown)}</span>'


def _word(kind, code):
    """코드를 한국어 낱말(labels.label)로 바꿔 이스케이프한다. 모르는 코드는 그대로, None 은 MISSING 으로 보인다."""
    return _text(label(kind, code))


def _elapsed(seconds):
    """임무 경과 시간, 초 단위 정수: 1시간 미만은 분/초, 하루 미만은 시간/분, 그 이상은 일/시간.

    `_age` 와 달리 None 이나 음수(트리거가 집계 시각보다 늦음)는 MISSING 으로 보인다.
    """
    if seconds is None or seconds < 0:
        return MISSING
    seconds = int(seconds)
    if seconds < 3600:
        return f'{seconds // 60}분 {seconds % 60:02d}초'
    if seconds < 86400:
        return f'{seconds // 3600}시간 {seconds % 3600 // 60:02d}분'
    return f'{seconds // 86400}일 {seconds % 86400 // 3600}시간'


def _pose(pose):
    """자세 글자 `(x, y) m`. yaw 를 알면 정수 도로 붙이고 이스케이프한다. None 은 MISSING 으로 보인다.

    MAP_FRAME 이 아닌 좌표계는 대괄호 안에 이름을 적어, 숫자를 지도 좌표로 읽지 않게 한다.
    """
    if pose is None:
        return MISSING
    text = f"({pose['x']:.2f}, {pose['y']:.2f}) m"
    if pose.get('yaw') is not None:
        text += f" {math.degrees(pose['yaw']):.0f}°"
    if pose['frame_id'] != MAP_FRAME:
        text += f" [{pose['frame_id']}]"
    return escape(text)


def _victim_path(x, y, size):
    """INSARAG 식 요구조자 표식: 꼭짓점이 보고된 위치에 놓인 V."""
    return (f'M{x - size:.2f},{y - 1.6 * size:.2f} L{x:.2f},{y:.2f} '
            f'L{x + size:.2f},{y - 1.6 * size:.2f}')


def _victim_icon(stage):
    """목록과 범례에 쓰는 작은 V. 모양과 색은 지도 위와 같다."""
    strike = '<line x1="2" y1="7" x2="14" y2="7"/>' if stage == 'RESCUED' else ''
    return (f'<svg class="vicon {stage.lower()}" viewBox="0 0 16 16" aria-hidden="true">'
            f'<path d="{_victim_path(8, 13, 5)}"/>{strike}</svg>')


# ---- 임무 띠 (조각) ------------------------------------------------------------------------------

def _reason(reason):
    """조정기 사유를 낱말로 보이고, 낱말이 코드와 다르면 옆에 코드를 붙인다."""
    level = f' {reason["level"]}' if reason['level'] else ''
    code = '' if reason['text'] == reason['code'] else f' <code>{escape(reason["code"])}</code>'
    return f'<span class="reason{level}" title="조정기 SystemState.reason">{escape(reason["text"])}</span>{code}'


def _scoped_reason(view, *scopes, tag='p'):
    """조정기 사유를 요소 하나로 만든다. 범위가 `scopes` 중 하나일 때만 만들고, 아니면 ''."""
    reason = view['mission'].get('reason_view')
    if reason is None or reason['scope'] not in scopes:
        return ''
    return f'<{tag} class="scoped-reason">{_reason(reason)}</{tag}>'


def _band(view):
    """맨 위를 가로지르는 임무 모드 띠. 출구 누적은 그리지 않는다: 고정 카메라가 하나다(db-schema 2절)."""
    mission = view['mission']
    mode, source = mission['mode'], mission['source']
    built = view['built_at_ns']
    facts = []
    if mission['trigger_ns'] is not None:
        facts += [('트리거', _time(mission['trigger_ns'], built)),
                  ('경과', _elapsed((built - mission['trigger_ns']) / 1e9))]
    if mode == 'DISASTER' and mission['sign_count']:
        facts.append(('진동 감지', f'{mission["sign_count"]}건'))
    if mission['phase'] is not None:
        facts.append(('단계', _word('phase', mission['phase'])))
    dispatch = mission['dispatch']
    if dispatch is not None:
        facts.append(('구간', f'<span title="끝낸 구간 / 전체 구간">{dispatch["segment_index"]}/{dispatch["segment_count"]}</span> '
                              f'{_word("segment", dispatch["segment_phase"])}'))
    if mission.get('reason_view') is not None:
        facts.append(('사유', _reason(mission['reason_view'])))
    if source == 'mission_state':
        note = f'조정기 임무 상태, {_age((built - mission["updated_at_ns"]) / 1e9)} 전 바뀜'
    elif mode == 'DISASTER':
        note = '임무 상태 기록 전 — 현장 카메라의 진동 감지로 추정'
    elif mode == 'NORMAL':
        note = '임무 상태 기록 전 — 진동 감지 없음'
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


# ---- 지도 (조각) --------------------------------------------------------------------------------

def _grid(layer):
    """미터 격자와 축 숫자를 이미지 픽셀 단위로 그린다(측량 도면 모양)."""
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
    """재난 전 지도를 SVG 하나의 바탕으로 깔아, 표식과 이미지가 늘 함께 확대·축소되게 한다."""
    layer = view['map']
    live = layer.get('live')
    if not layer.get('available') and live is not None:
        return (f'<section id="map" class="placeholder"><h2>지도</h2><p>{escape(live["message"])}</p>'
                '<p>운영 launch 의 <code>map_topic</code>·<code>map_dir</code> 와 SLAM 지도 발행을 확인하세요.</p></section>')
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
    for exit_ in layer.get('exits', ()):
        x, y = exit_['px'], exit_['py']
        seen = f'{exit_["id"]} 관측 {datetime.fromtimestamp(exit_["observed_at_ns"] / 1e9):%H:%M:%S}'
        half = TARGET_MARK_PX
        marks.append(
            f'<g class="exit"><title>{escape(seen)} · map_revision {exit_["map_revision"]} '
            f'(조정기 등록 출구 아님 — 관측)</title>'
            f'<rect class="mark" x="{x - half:.2f}" y="{y - half:.2f}" width="{2 * half:.2f}" height="{2 * half:.2f}"/>'
            f'<text class="label" x="{x + half + 0.5:.2f}" y="{y - half:.2f}">{escape(seen)}</text></g>')
    for robot in layer['robots']:
        x, y = robot['px'], robot['py']
        role = robot.get('role') or 'UNKNOWN'
        unit = robot.get('name') or robot['id']
        if robot['yaw'] is None:
            body = f'<circle class="body" cx="{x}" cy="{y}" r="{ROBOT_MARK_PX}"/>'
        else:
            # 이미지 행은 아래로 자라므로, +yaw(지도에서 반시계 방향)는 여기서 -y 다.
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
    unplotted += [f'{w["name"]}: 위치 미확정' for w in layer.get('no_pose', ())]
    notes = ''.join(f'<li>{escape(text)}</li>' for text in unplotted)
    legend = ''.join(f'<li>{_victim_icon(stage)}{escape(STAGES[stage])}</li>' for stage in STAGE_ORDER)
    legend += '<li><svg class="vicon robot" viewBox="0 0 16 16" aria-hidden="true"><polygon points="14,8 3,2.5 3,13.5"/></svg>로봇과 진행 방향</li>'
    pad_left, pad_top, pad_bottom = 5, 2, 9
    if live is None:
        note = ('재난 전 기준 지도, 격자 한 칸 {grid:g} m. 좌표가 같은 지도 세션인지는 아직 확인하지 않습니다 (Q-09).'
                .format(grid=GRID_M))
        state = ''
    else:
        received = '받은 시각 없음' if live['received_at_ns'] is None else (
            f'받은 시각 {datetime.fromtimestamp(live["received_at_ns"] / 1e9):%H:%M:%S} ({live["age_s"]:g}초 전)')
        note = (f'실시간 SLAM 지도 · {received} · {width}×{height} px · {layer["resolution"]:g} m/px · '
                f'해시 {layer["pgm_sha256"][:12]} · 격자 한 칸 {GRID_M:g} m'
                + (f' — {live["message"]}' if live['message'] else ''))
        state = '' if live['state'] == 'OK' else ' stale'
    return (f'<section id="map" class="map{state}"><header><h2>지도</h2><ul class="legend">{legend}</ul></header>'
            f'<svg class="plan" viewBox="{-pad_left} {-pad_top} {width + pad_left + 2} {height + pad_top + pad_bottom}" '
            f'preserveAspectRatio="xMidYMid meet" role="img" aria-label="기준 지도 위 로봇과 요구조자 위치">'
            f'<image href="{escape(layer["url"])}" x="0" y="0" width="{width}" height="{height}" '
            f'preserveAspectRatio="none"/>'
            f'<rect class="frame" x="0" y="0" width="{width}" height="{height}"/>'
            f'{_grid(layer)}{"".join(marks)}</svg>'
            f'<p class="map-note" title="{escape(layer["note"])}">{escape(note)}</p>'
            + (f'<ul class="unplotted"><li class="head">지도에 그리지 못한 위치</li>{notes}</ul>' if notes else '')
            + '</section>')


# ---- 로봇과 요구조자 (조각) --------------------------------------------------------------------------

def _battery(robot):
    """로봇 카드의 배터리 막대: BATTERY_LOW / BATTERY_WARN 에 따른 색, 값, 그 측정값의 나이.

    값이 없으면 빈 막대나 0 % 막대가 아니라 「배터리 값 없음」을 보인다. 막대 폭만 0-100 으로 자르고,
    숫자는 받은 값 그대로 보인다.
    """
    percent = robot['battery_percent']
    if percent is None:
        return '<div class="battery none"><span class="value">배터리 값 없음</span></div>'
    level = 'low' if percent < BATTERY_LOW else 'warn' if percent < BATTERY_WARN else 'ok'
    width = max(0.0, min(100.0, percent))
    return (f'<div class="battery {level}"><span class="bar"><span style="width:{width:.0f}%"></span></span>'
            f'<span class="value">{percent:.0f}%</span>'
            f'<span class="age">{_age(robot["battery_age_s"])} 전</span></div>')


def _watch(robot):
    """--robot-watch 한 줄: 로봇 기본 토픽 직접 관측(수신 경과, 배터리, 도킹)."""
    w = robot['watch']
    battery = '배터리 값 없음' if w['battery_percent'] is None else f'배터리 {w["battery_percent"]:.0f}%'
    dock = {True: '도킹', False: '도킹 아님', None: '도킹 모름'}[w['docked']]
    age = '수신 없음' if w['age_s'] is None else f'{_age(w["age_s"])} 전'
    flag = '직접 관측 낡음' if w['stale'] else '직접 관측'
    return (f'<p class="watch{" alert" if w["stale"] else ""}" title="{escape(w["namespace"] or "")} 구독만">'
            f'{flag} · {age} · {battery} · {dock}</p>')


def _robots(view):
    """배치 순서(view['robots'])대로 로봇 카드를 그린다.

    아직 상태가 없는 로봇은 빈 값이 아니라 경고 카드를 받는다. 그 밖에는 이동·준비 낱말,
    배터리, 위치, 진행 중 작업, operations 가 상태를 받은 뒤 지난 시간, 역할 범위에 맞는 조정기
    사유, 상태가 오래됐으면 낡음 안내(마지막 값은 남기고 `stale` 클래스로 흐리게 한다)를 보인다.
    """
    units = []
    for robot in view['robots']:
        role = robot['role']
        head = (f'<h3><span class="unit-name">{escape(robot["name"])}</span>'
                f'<span class="unit-role">{_word("role", role) if role else ""}</span>'
                f'<span class="unit-id">{escape(robot["robot_id"])}</span></h3>')
        if not robot['has_status'] and robot.get('watch'):
            units.append(f'<section class="unit watch{" stale" if robot["watch"]["stale"] else ""}" '
                         f'data-robot="{escape(robot["robot_id"])}">{head}'
                         '<p class="note">프로젝트 상태 없음 — 로봇 기본 토픽 직접 관측만 보임</p>'
                         + _watch(robot) + f'<dl><div><dt>위치</dt><dd>{_pose(robot["pose"])}</dd></div></dl></section>')
            continue
        if not robot['has_status']:
            units.append(f'<section class="unit missing" data-robot="{escape(robot["robot_id"])}">{head}'
                         '<p class="alert">상태를 한 번도 받지 못했습니다. 로봇 상태 발행과 operations 연결을 확인하세요.</p>'
                         '</section>')
            continue
        motion = robot['motion_state']
        stale = (f'<p class="alert" title="표시 값은 마지막으로 받은 상태입니다">낡음: '
                 f'{_age(robot["received_age_s"])} 동안 새 상태 없음</p>') if robot['stale'] else ''
        units.append(
            f'<section class="unit {(role or "unknown").lower()}{" stale" if robot["stale"] else ""}" '
            f'data-robot="{escape(robot["robot_id"])}">{head}'
            f'<p class="state"><span class="motion {escape((motion or "unknown").lower())}">{_word("motion", motion)}</span>'
            f'<span class="ready">{_word("readiness", robot["readiness"])}</span></p>'
            + _battery(robot)
            + f'<dl><div><dt>위치</dt><dd>{_pose(robot["pose"])}</dd></div>'
            f'<div><dt>작업</dt><dd>{_short(robot["active_task_id"])}</dd></div>'
            f'<div><dt>수신</dt><dd>{_age(robot["received_age_s"])} 전</dd></div></dl>'
            + _scoped_reason(view, {'SUPPORT': 'support', 'SCOUT': 'scout'}.get(role)) + stale
            + (_watch(robot) if robot.get('watch') else '') + '</section>')
    return f'<section id="robots"><h2>로봇</h2>{"".join(units)}</section>'


def _targets(view):
    """요구조자 칸: 단계별 개수, 그 아래에 화면 모델 순서대로 대상마다 한 줄.

    한 줄에는 단계와 그 규칙 ID, 위치, 구조·접근 낱말, 지원, 소실 후보(고정 카메라의 단서일 뿐이고
    구출은 로봇 재확인이 판정한다), 보고 건수를 보인다. 보고 건수는 장부가 없으면 0 이 아니라
    「보고 장부 입력 없음」이다. 조정기 사유는 출동한 대상에 붙이고, AUTO 출동인데 아직 출동한
    대상이 없으면 NOT_RESCUED 대상마다 붙인다.
    """
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
        dispatched = view['mission'].get('dispatch') or {}
        if dispatched.get('target_id') == target['target_id']:
            support += _scoped_reason(view, 'support', 'target', tag='span')
        elif not dispatched and view.get('support_dispatch') == 'AUTO' and target['stage'] == 'NOT_RESCUED':
            support += _scoped_reason(view, 'support', tag='span')   # 자동 출동이 아직 나가지 않은 이유
        receipt = ('<span class="receipt">보고 장부 입력 없음</span>' if target['report_count'] is None else
                   f'<span class="receipt">보고 {target["report_count"]}건 {_word("receipt", target["receipt_status"])}</span>')
        mark = target['disappeared_candidate']
        candidate = ('' if mark is None else
                     f'<span class="candidate" title="고정 웹캠 {escape(mark["camera_id"])} 사건 '
                     f'{escape(mark["event_id"])} — 후보일 뿐, 로봇 재확인으로 구출을 판정">'
                     f'소실 후보 {_time(mark["observed_at_ns"], view["built_at_ns"])}</span>')
        rows.append(
            f'<li class="victim-row {target["stage"].lower()}" data-stage="{target["stage"]}">'
            f'{_victim_icon(target["stage"])}'
            f'<div class="who">{_short(target["target_id"])}<span class="stage-word">{escape(target["stage_label"])}</span>'
            f'<span class="rule" title="표시 규칙 (README 구조 상태 4단계)">{target["rule"]}</span></div>'
            f'<div class="what"><span>{where}</span><span>구조 {_word("rescue", target["rescue_state"])}</span>'
            f'<span>{_word("approach", target["approach_result"])}</span>{support}{candidate}{receipt}</div>'
            + (f'<ul class="notes">{notes}</ul>' if notes else '') + '</li>')
    body = ''.join(rows) or '<li class="none">아직 보고된 요구조자가 없습니다.</li>'
    return (f'<section id="targets"><h2>요구조자 {len(view["targets"])}명</h2>'
            f'<ul class="stage-counts">{counts}</ul><ul class="victims">{body}</ul></section>')


# ---- 사건 기록 (조각) -----------------------------------------------------------------------------

def _event_text(event):
    """사건 행의 내용 칸.

    REPORT 는 확정·접근·영수증 낱말을, 현장 사건은 신뢰도와 메모를 보인다.
    보일 것이 없으면 MISSING 이다.
    """
    if event['kind'] == 'REPORT':
        return (f'{_word("confirmation", event["confirmation"])}, {_word("approach", event["approach"])}, '
                f'{_word("receipt", event["receipt"])}')
    text = '' if event['confidence'] is None else f'신뢰도 {event["confidence"]:.2f}'
    if event['note']:
        text += f' {escape(event["note"])}'
    return text.strip() or MISSING


def _events(view):
    """사건 기록 표, 화면 모델 순서(최신이 위). 최근 행만 보일 때는 제목에 그렇게 적는다.

    행마다 페이지 스크립트가 쓸 data-* 속성(ref, 사건·수신 ns, 종류, 시계 영역)을 단다. 스크립트가
    화면 표시·지연 칸(KPI-04)을 채우고 폴링 사이에도 유지한다. 서버는 그 칸에 MISSING 을 보낸다.
    """
    rows = []
    built = view['built_at_ns']
    for event in view['events']:
        rows.append(
            f'<tr class="event {event["kind"].lower()}" data-ref="{escape(event["ref_id"])}" '
            # 현장 사건 행에는 장부 시각이 없어(read_site_events 가 돌려주지 않음) 빈 값으로 둔다.
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


# ---- 명령·오류 기록 (조각) --------------------------------------------------------------------------

def _reply(code):
    """사유 낱말과 그 코드. 모르는 코드는 코드 그대로 한 번만 보인다."""
    if code is None:
        return ''
    word = label('reply', code)
    return escape(code) if word == code else f'{escape(word)} <code>{escape(code)}</code>'


def _history(view):
    """운영자 명령과 operations 오류 기록(--site-state 에서 읽은 operator_commands 와 error_log).

    None(예: --status 고정 입력)이면 빈 표 대신 기록을 보는 방법을 보인다. 명령 행은
    접수/거절과 사유 낱말·코드, 대상을 보이고, 오류 행은 코드와 상세를 보인다.
    없는 입력은 이름으로 나열한다.
    """
    history = view['history']
    head = '<h2>명령·오류 기록</h2>'
    if history is None:
        return (f'<section id="history" class="empty">{head}<p>기록 입력 없음 — <code>--site-state</code> 로 '
                '띄우면 운영자 명령의 접수·거절과 operations 오류가 보입니다.</p></section>')
    built, rows = view['built_at_ns'], []
    for row in history['rows']:
        if row['kind'] == 'COMMAND':
            outcome, css = ('접수', 'accepted') if row['accepted'] else ('거절', 'refused')
            target = '' if row['target_id'] is None else f' 대상 {_short(row["target_id"])}'
            rows.append(f'<tr class="hist command {css}" data-ref="{escape(row["ref_id"])}">'
                        f'<td class="t">{_time(row["at_ns"], built)}</td>'
                        f'<td>{escape(row["what_label"])}</td>'
                        f'<td><b class="outcome" title="{escape(row["result"] or "")}">{outcome}</b> '
                        f'{_reply(row["reason"])}{target}</td></tr>')
        else:
            rows.append(f'<tr class="hist error" data-ref="{escape(row["ref_id"])}">'
                        f'<td class="t">{_time(row["at_ns"], built)}</td><td>오류</td>'
                        f'<td><b class="outcome">{escape(row["what"])}</b> {escape(row["detail"])}</td></tr>')
    missing = ''.join(f'<p class="missing">{escape(key)} 입력 없음</p>' for key in history['missing'])
    body = ('<table><thead><tr><th>시각</th><th>무엇</th><th>결과 · 사유</th></tr></thead>'
            f'<tbody>{"".join(rows)}</tbody></table>') if rows else '<p>아직 기록 없음</p>'
    return f'<section id="history">{head}{missing}{body}</section>'


def _gates(view):
    """조각 밖 버튼을 위한 명령 관문 거울. 페이지 스크립트가 폴링마다 적용한다."""
    gates = {c['name']: {key: c[key] for key in ('enabled', 'reason', 'args', 'choices')} for c in view['commands']}
    return f'<div id="gates" hidden data-gates="{escape(json.dumps(gates, ensure_ascii=False))}"></div>'


def render_fragment(view):
    """브라우저가 폴링마다 갈아 끼우는 페이지 부분(그 자식들이 페이지 격자에 놓인다)."""
    return (_band(view) + _warnings(view) + _map(view) + '<div id="middle">' + _robots(view) + _targets(view)
            + '</div>' + _events(view) + _history(view) + _gates(view))


# ---- 페이지 단위 부분 (폴링 조각 밖) --------------------------------------------------------------------

def _commands(view, access, csrf):
    """결과의 무게에 따라 묶은 버튼, 확인 칸, 이 페이지에서 보낸 명령의 답.

    `access` 는 로그인한 역할의 (allowed, reason)이다(commands.command_access). 버튼이 켜지려면
    권한과 그 관문(view.command_gates)이 모두 있어야 하고, 페이지 스크립트가 폴링마다 관문을
    다시 적용한다. 이 칸은 폴링 조각 밖에 있어서 폴링이 커서 아래에서 다시 그리지 않는다.
    """
    allowed, access_reason = access
    groups = {'approve': [], 'mission': [], 'stop': []}
    for command in view['commands']:
        if command.get('auto'):   # 자동 지원 출동: 버튼 없이 상태 줄만 둔다(data-command 없음)
            groups[command['group']].append(f'<p class="cmd-auto" title="{escape(command["reason"])}">'
                                            f'{escape(command["label"])}</p>')
            continue
        name, reason = escape(command['name']), escape(command['reason'])
        off = '' if allowed and command['enabled'] else ' disabled'
        groups[command['group']].append(
            f'<button type="button" class="cmd {command["group"]}"{off} data-command="{name}" title="{reason}">'
            f'<span class="cmd-label">{escape(command["label"])}</span><span class="why">{reason}</span></button>')
    data = f' data-allowed="1" data-csrf="{escape(csrf or "")}"' if allowed else ' data-allowed="0"'
    return (f'<section id="commands"{data}><header><h2>명령</h2>'
            f'<p class="cmd-access {"on" if allowed else "off"}">{escape(access_reason)}</p></header>'
            f'<div class="cmd-group approve">{"".join(groups["approve"])}</div>'
            f'<div class="cmd-group mission">{"".join(groups["mission"])}</div>'
            f'<div class="cmd-group stop">{"".join(groups["stop"])}</div>'
            '<div id="cmd-confirm" hidden><p class="what"></p>'
            '<label id="cmd-pick-row" hidden>대상 <select id="cmd-pick"></select></label>'
            '<dl class="args"></dl><div class="row">'
            '<button type="button" id="cmd-send" class="cmd send">보내기</button>'
            '<button type="button" id="cmd-cancel" class="cmd">취소</button></div></div>'
            '<ol id="cmd-log" aria-live="polite"></ol>'
            '<p class="estop">위험할 때는 현장 비상 정지 장치를 쓰세요. 이 화면의 명령은 보조 수단이고, '
            '받을지는 조정기가 정합니다.</p></section>')


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

/* ---- 관제 화면 격자 ---- */
.console{display:grid;gap:10px;padding:10px;min-height:100vh;
  grid-template-columns:minmax(0,1fr) 400px 470px;
  grid-template-rows:auto auto minmax(0,1fr) fit-content(230px);
  grid-template-areas:"band band top" "warn warn side" "map middle side" "log hist side"}
#live{display:contents}
#mission{grid-area:band} #warnings{grid-area:warn} #map{grid-area:map} #middle{grid-area:middle}
#events{grid-area:log} #history{grid-area:hist} #topbar{grid-area:top} #side{grid-area:side}
@media (min-width:1400px) and (min-height:820px){ .console{height:100vh;overflow:hidden} }
@media (max-width:1399px){
  .console{grid-template-columns:minmax(0,1fr);grid-template-rows:none;
    grid-template-areas:"top" "band" "warn" "map" "side" "middle" "log" "hist"}
}
#map,#middle>section,#events,#history,#side>section{background:var(--panel);border:1px solid var(--rule);padding:10px 12px}
#middle{display:flex;flex-direction:column;gap:10px;min-height:0}
/* 칸이 모자라면 요구조자 목록보다 로봇 카드가 먼저 줄고 그 안에서 스크롤된다(#64). */
#robots{min-height:0;overflow:auto}
#side{display:flex;flex-direction:column;gap:10px;min-height:0}

/* ---- 임무 띠 ---- */
.band{display:flex;align-items:center;gap:28px;padding:10px 18px;border:1px solid var(--rule);background:var(--panel)}
.band .mode-word{font-size:30px;font-weight:700;letter-spacing:.04em;min-width:3.2em}
.band .facts{display:flex;gap:24px}
.band dt{font-size:12px;color:var(--ink-2)} .band dd{margin:0;font-size:18px;font-weight:500}
.band .mode-note{color:var(--ink-2)} #built{margin-left:auto;font-size:12px;color:var(--ink-2)}
.band.disaster{background:var(--signal);border-color:var(--signal);color:#fff}
.band.disaster dt,.band.disaster .mode-note,.band.disaster #built{color:#ffd9de}
.band.unknown .mode-word{color:var(--pending)}
.band.clearing{border-left:10px solid var(--caution)}
.band.mission_finished{border-left:10px solid var(--safe)} .band.mission_finished .mode-word{color:var(--safe)}
.band .reason{font-size:15px;font-weight:700}
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

/* ---- 확인할 것 ---- */
.reason.alert{color:var(--signal);font-weight:700} .reason.caution{color:var(--ink);background:var(--caution-tint);padding:0 4px}
.scoped-reason{margin:4px 0 0;font-weight:600}
.cmd-auto{margin:0;padding:10px 12px;border:1px dashed var(--info);color:var(--info);font-weight:700}
#warnings{background:var(--caution-tint);border:1px solid var(--caution);border-left:8px solid var(--caution);padding:6px 12px}
#warnings.empty{display:none}
#warnings h2{display:inline;margin-right:12px;font-size:14px}
#warnings ul{display:inline;margin:0;padding:0;list-style:none}
#warnings li{display:inline}
#warnings li+li::before{content:"／";margin:0 8px;color:var(--ink-2)}

/* ---- 지도 ---- */
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
.exit .mark{fill:none;stroke:var(--safe);stroke-width:.75} .exit .label{fill:var(--safe)}
.robot.support .heading,.robot.support .body{fill:var(--info)}
.robot .label{fill:var(--ink)} .robot.support .label{fill:var(--info)}
.map-note{font-size:12px;color:var(--ink-2);margin-top:4px}
#map.stale .plan{opacity:.45} #map.stale .map-note{color:var(--signal);font-weight:700}
.unplotted{margin:6px 0 0;padding:6px 10px;list-style:none;background:var(--caution-tint);border-left:6px solid var(--caution);font-size:13px}
.unplotted .head{font-weight:700}
#map.placeholder p{margin-top:40px;text-align:center;color:var(--ink-2)}

/* ---- 작은 V 표식 ---- */
.vicon{width:16px;height:16px;flex:none;vertical-align:-3px}
.vicon path,.vicon line{fill:none;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
.vicon.unconfirmed path{stroke:var(--pending);stroke-dasharray:2.2 1.6}
.vicon.not_rescued path{stroke:var(--rescue)} .vicon.supporting path{stroke:var(--info)}
.vicon.rescued path,.vicon.rescued line{stroke:var(--safe)}
.vicon.robot polygon{fill:var(--ink)}

/* ---- 로봇 ---- */
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
.unit dl{display:grid;grid-template-columns:auto 1fr auto 1fr;column-gap:10px;font-size:13px}
.unit dl div{display:contents} .unit dt{color:var(--ink-2)} .unit dd{margin:0}
.unit .alert{margin-top:6px;padding:4px 8px;background:var(--caution-tint);font-size:13px}
.unit.stale,.unit.missing{border-left-color:var(--caution)}
.unit.stale .state,.unit.stale .battery,.unit.stale dl{opacity:.55}

/* ---- 요구조자 ---- */
#targets{display:flex;flex-direction:column;min-height:240px;flex:1}
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
.victim-row .candidate{color:var(--ink);background:var(--caution-tint);border:1px solid var(--caution);padding:0 6px;font-weight:600}
.victim-row .notes{grid-column:2;margin:4px 0 0;padding:2px 8px;list-style:none;background:var(--caution-tint);font-size:12.5px}
.victims .none{padding:12px 0;color:var(--ink-2)}

/* ---- 사건 기록 ---- */
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
/* ---- 명령·오류 기록: 사건 기록 옆, 요구조자 아래. 안에서 스크롤 ---- */
#history{min-height:0;overflow:auto}
#history table{border-collapse:collapse;width:100%;font-size:13px}
#history th{position:sticky;top:0;background:var(--panel);text-align:left;font-weight:500;color:var(--ink-2)}
#history td{padding:3px 6px;border-bottom:1px solid #d5dbda;vertical-align:top}
#history td.t,#history td:nth-child(2){white-space:nowrap}
#history tr.refused .outcome,#history tr.error .outcome{color:var(--signal);font-weight:700}
#history tr.accepted .outcome{color:var(--safe)}
#history .missing,#history.empty p{color:var(--ink-2)}

/* ---- 옆 칸: 웹캠과 명령 ---- */
#cameras figure{margin:0;position:relative}
#cameras img{display:block;width:100%;aspect-ratio:16/9;object-fit:contain;background:#26343a;color:transparent}
#cameras .nosignal{display:none;position:absolute;left:0;right:0;top:0;aspect-ratio:16/9;align-items:center;
  justify-content:center;text-align:center;padding:0 24px;color:#e8eeee;font-size:14px}
#cameras figure.lost .nosignal{display:flex}
#cameras figure.lost img{visibility:hidden} #cameras figure{background:#26343a}
#cameras figcaption{display:flex;justify-content:space-between;font-size:13px;padding-top:4px;background:var(--panel)}
#cameras figcaption .lost{color:var(--signal);font-weight:700}
.camera-empty p{color:var(--ink-2);font-size:13px}
#commands{overflow:auto;min-height:0}
#commands header{display:flex;align-items:baseline;gap:10px;margin-bottom:8px}
#commands header h2{margin:0}
.cmd-access{font-size:12.5px;color:var(--ink-2)} .cmd-access.off{color:var(--signal);font-weight:500}
.cmd-group{display:grid;gap:6px;margin-bottom:8px}
.cmd-group.approve{grid-template-columns:1fr}
.cmd-group.mission{grid-template-columns:repeat(3,1fr)}
.cmd{font:inherit;border:1px solid var(--ink);background:#fff;color:var(--ink);padding:6px 8px;border-radius:3px;
  cursor:pointer;display:flex;flex-direction:column;align-items:flex-start;gap:1px;text-align:left}
.cmd .cmd-label{font-weight:700}
.cmd .why{font-size:11.5px;line-height:1.3;color:var(--ink-2)}
.cmd.approve{padding:8px 12px;background:var(--info);border-color:var(--info);color:#fff}
.cmd.approve .cmd-label{font-size:17px} .cmd.approve .why{font-size:12.5px;color:#dbe9f4}
.cmd.stop{width:100%;padding:8px 12px;background:var(--signal);border-color:var(--signal);color:#fff}
.cmd.stop .cmd-label{font-size:17px} .cmd.stop .why{color:#ffd9de}
/* 꺼진 버튼도 읽히게 둔다: 낱말 아래 꺼진 이유가 핵심이다. */
.cmd:disabled{cursor:not-allowed;background:#e9edec;border-color:var(--rule);color:var(--ink-2)}
.cmd:disabled .why{color:var(--ink-2)}
/* 확인 칸이 열려 있는 동안 누른 명령. */
.cmd.armed:disabled{background:var(--info-tint);border:2px solid var(--info);color:var(--info)}
#cmd-confirm{border:2px solid var(--info);background:#fff;padding:8px 10px;margin-bottom:8px}
#cmd-confirm .what{font-weight:700;margin-bottom:6px}
#cmd-confirm label{display:flex;gap:8px;align-items:center;margin-bottom:6px;font-size:13px}
#cmd-confirm label[hidden]{display:none}
#cmd-confirm select{font:inherit;flex:1}
#cmd-confirm .args{display:grid;grid-template-columns:auto 1fr;column-gap:10px;font-size:13px;margin-bottom:8px}
#cmd-confirm .args dt{color:var(--ink-2)} #cmd-confirm .args dd{margin:0}
#cmd-confirm .row{display:flex;gap:8px}
#cmd-confirm .row .cmd{align-items:center;flex:1}
.cmd.send{background:var(--info);border-color:var(--info);color:#fff;font-weight:700}
#cmd-log{list-style:none;margin:0 0 8px;padding:0;font-size:12.5px}
#cmd-log li{padding:3px 8px;border-left:5px solid var(--pending);background:#fff;margin-bottom:3px}
#cmd-log li time{color:var(--ink-2);margin-right:6px}
#cmd-log li.accepted{border-left-color:var(--safe)} #cmd-log li.rejected{border-left-color:var(--caution)}
#cmd-log li.failed{border-left-color:var(--signal);font-weight:500}
.estop{font-size:12.5px;padding:6px 8px;border-left:6px solid var(--signal);background:#fff}

/* ---- 폼(로그인, 계정) ---- */
.card{max-width:420px;margin:12vh auto;padding:24px;background:var(--panel);border:1px solid var(--rule)}
.card h1{font-size:22px;margin-bottom:12px}
.failure{border-left:8px solid var(--signal)} .failure p{margin-top:10px}
.failure .detail{font-size:13px;color:var(--ink-2);word-break:break-all} .failure .retry{font-size:13px} .card button{margin-top:16px}
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

# 사건 행마다 처음 나타난 브라우저 시각을 찍고(KPI-04), 명령 칸을 돌린다.
# 조각은 GET 으로 받고, 유일한 POST 는 이 콘솔의 /commands 로 보내는 운영자 명령이다.
_SCRIPT = """
// 페이지 상태: firstSeen 은 사건 ref_id 를 그 행을 처음 그린 브라우저 시각에 잇는다. window.kpi04 는
// 사건마다 지연 기록 하나를 담는다(saveKpi 가 CSV 로 저장). loaded 는 첫 stamp() 뒤에 참이 된다.
// lastOk 는 마지막 성공 폴링 시각, FRESH_MS 는 새로 들어온 행을 강조해 두는 시간이다.
const firstSeen = new Map();
window.kpi04 = [];
const FRESH_MS = 3000;
let lastOk = Date.now(), loaded = false;
// render.py _time / _age 와 같은 형식이라 서버 칸과 브라우저 칸이 똑같이 읽힌다.
function clock(ms) {
  const d = new Date(ms), pad = (n, w) => String(n).padStart(w || 2, '0');
  return pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds()) + '.' + pad(d.getMilliseconds(), 3);
}
// render.py _age 와 같은 나이 글자. 다만 1분 미만은 소수 한 자리가 아니라 두 자리로 쓴다.
function age(s) {
  const sign = s < 0 ? '-' : '', v = Math.abs(s);
  if (v < 60) return sign + v.toFixed(2) + 's';
  if (v < 3600) return sign + Math.floor(v / 60) + '분 ' + String(Math.floor(v % 60)).padStart(2, '0') + '초';
  if (v < 86400) return sign + Math.floor(v / 3600) + '시간 ' + Math.floor(v % 3600 / 60) + '분';
  return sign + Math.floor(v / 86400) + '일';
}
// 모든 사건 행의 화면 표시·지연 칸을 채운다. 새 조각은 그 칸이 빈 채(—)로 오므로 폴링마다 돈다.
// 처음 본 ref_id 는 firstSeen 에 지금 브라우저 시각을 받고 KPI-04 기록 하나를 얻는다. 그래서 사건의
// 지연은 그 행이 처음 나타날 때 한 번만 잰다. 지연은 브라우저 시계에서 사건 시계를 뺀 값이라,
// 두 시계가 맞을 때만 의미가 있다(C-03).
function stamp() {
  const now = Date.now();
  for (const row of document.querySelectorAll('tr.event')) {
    const ref = row.dataset.ref, eventMs = Number(row.dataset.eventNs) / 1e6;
    if (!firstSeen.has(ref)) {
      firstSeen.set(ref, now);
      // live: 이 페이지가 열려 있는 동안 들어온 행이다. 로드 때 이미 있던 행은 표시 지연이 아니라
      // 지난 기록이므로 KPI-04 요약에서 뺀다.
      window.kpi04.push({ref_id: ref, kind: row.dataset.kind, clock_domain_id: row.dataset.clock,
                         event_ns: row.dataset.eventNs, received_at_ns: row.dataset.receivedNs || '',
                         rendered_ms: now, latency_s: (now - eventMs) / 1000, live: loaded});
    }
    const shown = firstSeen.get(ref);
    row.dataset.renderedMs = shown;
    row.querySelector('.rendered').textContent = clock(shown);
    row.querySelector('.latency').textContent = age((shown - eventMs) / 1000);
    // 페이지가 열린 뒤 들어온 행은 잠시 강조해 둔다(애니메이션 없음).
    row.classList.toggle('fresh', loaded && now - shown < FRESH_MS);
  }
  loaded = true;
  kpiSummary();
}
// #kpi 줄: live 기록만 대상으로, KPI-04 예산 안에 든 건수와 가장 긴 지연을 보인다.
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
// CSV 칸 하나. 따옴표·쉼표·줄바꿈이 있으면 따옴표로 감싼다(안의 따옴표는 두 번 쓴다).
function csvCell(value) {
  const text = String(value);
  return /[",\\n]/.test(text) ? '"' + text.replace(/"/g, '""') + '"' : text;
}
// 기록을 브라우저 안에서 CSV 파일(Blob)로 저장한다. 서버로는 아무것도 보내지 않는다.
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
// 탭 제목에도 상태를 실어, 뒤에 깔린 탭도 재난이나 갱신 끊김을 알린다.
function title(lost) {
  const band = document.getElementById('mission');
  const tag = lost ? '[갱신 끊김] ' : band && band.dataset.mode === 'DISASTER' ? '[재난] ' : '';
  document.title = tag + '구조 관제';
}
// 폴링 결과를 탭 제목과 #poll 에 보인다. 갱신 끊김이면 모든 명령 버튼이 꺼져야 하므로 명령 관문도
// 다시 적용한다. 실패 때는 lastOk 를 그대로 두어, 화면이 얼마나 오래된 값인지 문구가 말해 준다.
function pollStatus(ok, message) {
  title(!ok);
  screenLost = !ok;
  applyGates();
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
// 폴링 한 번. 맨 아래 setInterval 이 앞 폴링이 끝나기를 기다리지 않고 시작한다: 조각을 GET 하고
// #live 를 바꾼 뒤 사건 행에 시각을 찍고 새 관문을 읽고 화면을 실시간으로 표시한다. 401·다른 주소로 재지정이면
// 폴링을 멈추고 로그인 화면(재지정이면 그 주소)으로 간다. 그 밖의 실패는 갱신 끊김 표시뿐이고 마지막 조각은 남는다.
async function poll() {
  try {
    const response = await fetch('fragment', {cache: 'no-store'});
    if (response.status === 401 || response.redirected) {
      // 세션이 끝났다(다른 곳에서 로그아웃, 계정 변경): 돌아온 내용을 콘솔에 붙이지 않고
      // 폴링을 멈춘 뒤 로그인 페이지로 간다(AST-50-02).
      clearInterval(timer);
      location.href = response.status === 401 ? '/login' : response.url;
      return;
    }
    if (!response.ok) {
      // 서버가 무엇이 실패했는지 알려 준다(render.failure_text). 상태 코드만 보이지 말고 그 글을 보인다.
      const reason = (await response.text()).slice(0, 200);
      throw new Error('HTTP ' + response.status + (reason ? ' ' + reason : ''));
    }
    document.getElementById('live').innerHTML = await response.text();
    stamp();
    readGates();
    pollStatus(true);
  } catch (error) {
    pollStatus(false, error.message);
  }
}
document.getElementById('kpi-save').addEventListener('click', saveKpi);

// ---- 운영자 명령 ----
// 관문은 조각마다 함께 온다(#gates). 버튼이 켜지려면 이 페이지의 권한(ADMIN, operations 주소)과
// 그 관문이 있어야 하고, 화면이 갱신되지 않는 동안에는 모든 버튼이 꺼진다. 한 번 누르면 보낼 내용을
// 보이는 확인 칸이 열리고, 보내기만 한 번 보낸다. 자동으로 다시 보내는 일은 없다.
const panel = document.getElementById('commands');
const allowed = panel.dataset.allowed === '1';
const confirmBox = document.getElementById('cmd-confirm'), pick = document.getElementById('cmd-pick');
const sendButton = document.getElementById('cmd-send'), cancelButton = document.getElementById('cmd-cancel');
const LOG_ROWS = 6;
// gates: 최신 관문 거울. armed: 확인 칸이 열린 명령({kind, name, JSON 문자열인 args,
// choices}). sending: POST 가 가는 중. screenLost: 마지막 폴링이 실패했다.
let gates = {}, armed = null, sending = false, screenLost = false;
// 조각의 숨은 #gates 에서 관문 JSON 을 읽는다. 거울이 없거나 읽히지 않으면 관문이 없는 것으로 보고,
// 이전 관문을 유지하지 않고 모든 버튼을 끈다.
function readGates() {
  const el = document.getElementById('gates');
  try { gates = el ? JSON.parse(el.dataset.gates) : {}; } catch (error) { gates = {}; }
}
// 명령 종류의 버튼 글자(확인 칸과 답 목록에 쓴다). 버튼이 없으면 종류 그대로.
function nameOf(kind) {
  const button = panel.querySelector('button[data-command="' + kind + '"] .cmd-label');
  return button ? button.textContent : kind;
}
// render.py _short 와 같은 줄임 규칙.
function shortId(id) { return String(id).length > 9 ? String(id).slice(0, 8) : String(id); }
// 확인 칸에서 고른 대상. 열린 명령에 선택지가 없으면 null.
function chosen() { return armed && armed.choices.length ? armed.choices[pick.selectedIndex] : null; }
// 열린 확인 칸이 아직 최신 관문과 맞으면 참: 관문이 켜져 있고, args 가 그대로이고,
// 고른 대상이 아직 선택지에 있다. 아니면 운영자가 확인한 내용이 낡은 것이다.
function sameArmed(gate) {
  if (!gate || !gate.enabled || JSON.stringify(gate.args) !== armed.args) return false;
  const choice = chosen();
  return !choice || gate.choices.some(c => JSON.stringify(c) === JSON.stringify(choice));
}
// 모든 명령 버튼을 그 관문에 맞춰 켜고 끄며 아래에 사유를 쓴다. 버튼은 이 페이지에 권한이 있고,
// 관문이 켜져 있고, 화면이 실시간이고, 보내는 중이 아니고, 확인 칸이 닫혀 있을 때만 켜진다.
function applyGates() {
  for (const button of panel.querySelectorAll('button[data-command]')) {
    const gate = gates[button.dataset.command] || {enabled: false, reason: '관문 정보 없음'};
    // 폴링이 실패하는 동안 관문은 낡았을 수 있으므로, 최신이 아닌 화면에서는 명령을 내주지 않는다.
    const reason = screenLost ? '갱신 끊김 — 화면이 최신이 아니라 꺼 둠' : gate.reason;
    button.disabled = !allowed || !gate.enabled || screenLost || sending || armed !== null;
    button.classList.toggle('armed', armed !== null && armed.kind === button.dataset.command);
    button.querySelector('.why').textContent = reason;
    button.title = reason;
  }
  // 열린 확인 칸은 그 관문이 바뀌었거나 화면이 낡으면 취소하고, 운영자에게 다시 누르라고 알린다.
  // 보내는 중에는 취소하지 않는다: POST 는 이미 나갔고 취소해도 멈추지 않는다.
  if (armed && !sending && (screenLost || !sameArmed(gates[armed.kind]))) {
    disarm('상태가 바뀌어 ' + armed.name + ' 확인을 취소했습니다. 다시 누르세요.');
  }
}
// 확인 칸에 고른 대상 말고 명령과 함께 가는 값을 나열한다: DISPATCH_SUPPORT 면 대상 revision,
// 그리고 관문 arg 마다 한국어 이름표를 붙여 보인다. 다른 대상을 고르면 다시 돈다.
// 값은 textContent 로 넣으므로 관문에서 온 어떤 값도 HTML 로 해석되지 않는다.
function showArgs() {
  const rows = [], args = JSON.parse(armed.args), choice = chosen();
  const labels = {target_id: '대상', target_revision: '대상 revision', route_revision: '경로 revision',
                  segment_index: '승인할 구간'};
  if (choice && armed.kind === 'DISPATCH_SUPPORT') rows.push(['대상 revision', choice.target_revision]);
  for (const key of Object.keys(args)) {
    // segment_index 는 끝낸 구간 수이므로, 다음 구간을 0부터 센 번호이기도 하다.
    rows.push([labels[key] || key, key === 'segment_index'
      ? (args[key] + 1) + '번째 구간 (segment_index ' + args[key] + ')' : args[key]]);
  }
  if (!rows.length && !choice) rows.push(['보내는 값', '명령 종류와 운영자 아이디만']);
  const list = confirmBox.querySelector('.args');
  list.replaceChildren();
  for (const [name, value] of rows) {
    const dt = document.createElement('dt'), dd = document.createElement('dd');
    dt.textContent = name;
    dd.textContent = String(value);
    list.append(dt, dd);
  }
}
// 첫 번째 누름: 명령 하나의 확인 칸을 열고 모든 명령 버튼을 끈다. 관문의 args 와 choices 를 지금
// armed 에 복사하므로 뒤의 폴링이 보낼 내용을 바꿀 수 없다(바뀌면 sameArmed 가 대신 취소한다).
function arm(kind) {
  const gate = gates[kind];
  if (!allowed || screenLost || sending || !gate || !gate.enabled) return;
  armed = {kind: kind, name: nameOf(kind), args: JSON.stringify(gate.args), choices: gate.choices};
  confirmBox.querySelector('.what').textContent = armed.name + ' — 아래 내용으로 보냅니다';
  pick.replaceChildren(...gate.choices.map(c => {
    const option = document.createElement('option');
    option.textContent = shortId(c.target_id) + ' ' + c.stage_label;
    option.title = c.target_id;
    return option;
  }));
  document.getElementById('cmd-pick-row').hidden = !gate.choices.length;
  showArgs();
  confirmBox.hidden = false;
  applyGates();
  sendButton.focus();
}
// 확인 칸을 닫고 그 버튼을 되돌린다. message 가 있으면 왜 닫았는지 답 목록에 적는다.
function disarm(message) {
  armed = null;
  confirmBox.hidden = true;
  sendButton.disabled = cancelButton.disabled = false;
  sendButton.textContent = '보내기';
  if (message) log('info', message);
  applyGates();
}
// 이 페이지의 답 목록 맨 위에 시각을 붙인 줄 하나(클래스 accepted, rejected, failed, info)를 더하고
// 최신 LOG_ROWS 줄만 남긴다. 이 페이지에서 누른 것만 보이고, 함께 보는 기록은 명령·오류 기록 표다.
function log(kind, text) {
  const item = document.createElement('li'), time = document.createElement('time');
  item.className = kind;
  time.textContent = clock(Date.now()).slice(0, 8);
  item.append(time, text);
  const list = document.getElementById('cmd-log');
  list.prepend(item);
  while (list.children.length > LOG_ROWS) list.lastChild.remove();
}
// 콘솔의 답을 답 목록용 [클래스, 글자]로 바꾼다. sent === true: operations 가 답했다
// (accepted 는 접수이지 완료가 아니다). sent === false: 전달 안 됨이거나, 전달 여부를 모르고 찾아볼
// command_id 가 함께 온다. 그 밖(예상 못 한 HTTP 답)은 전달 여부를 모르는 실패다.
function describe(name, status, result) {
  if (result && result.sent === true) {
    const id = ' — request_id ' + result.request_id + (result.request_state ? ' (' + result.request_state + ')' : '');
    if (result.accepted) return ['accepted', name + ': 접수됨 (실행 완료 아님)' + id];
    const word = result.reason_label && result.reason_label !== result.reason ? ' (' + result.reason_label + ')' : '';
    return ['rejected', name + ': 거절 ' + result.reason + word + id];
  }
  if (result && result.sent === false) {
    const where = result.delivery === 'NOT_SENT' ? ' · 전달 안 됨'
      : result.command_id ? ' · 전달됐는지 모름 — command_id ' + result.command_id + ' 로 명령 기록을 확인한 뒤 다시 누르세요' : '';
    return ['failed', name + ': 보내지 못함 — ' + result.error + (result.message ? ': ' + result.message : '') + where];
  }
  return ['failed', name + ': 보내지 못함 — 콘솔 응답 HTTP ' + status + ', 전달됐는지 모름'];
}
// 두 번째 누름(보내기): 이 콘솔의 commands 경로로 CSRF 토큰, kind, 고른 대상, args 를 담아 정확히
// 한 번 POST 한다. 답이나 실패를 기록에 적고 확인 칸을 닫는다. 다시 시도하지 않는다
// (AGENTS.md 7절): 콘솔은 POST 마다 새 command_id 를 주므로, 다시 보내면 두 번째 명령이 된다.
async function send() {
  if (!armed || sending) return;
  const body = new URLSearchParams({csrf: panel.dataset.csrf, kind: armed.kind});
  const args = JSON.parse(armed.args), choice = chosen();
  if (choice) {
    body.append('target_id', choice.target_id);
    if (armed.kind === 'DISPATCH_SUPPORT') body.append('target_revision', String(choice.target_revision));
  }
  for (const key of Object.keys(args)) body.append(key, String(args[key]));
  const name = armed.name;
  sending = true;
  sendButton.disabled = cancelButton.disabled = true;
  sendButton.textContent = '보내는 중…';
  applyGates();
  let entry;
  try {
    const response = await fetch('commands', {method: 'POST', body: body, cache: 'no-store'});
    let result = null;
    try { result = await response.json(); } catch (error) { result = null; }
    entry = response.status === 401 ? ['failed', name + ': 보내지 못함 — 로그인이 끝났습니다. 다시 로그인하세요.']
      : describe(name, response.status, result);
  } catch (error) {
    entry = ['failed', name + ': 보내지 못함 — 콘솔 서버에 닿지 못함 (' + error.message + '), 전달됐는지 모름'];
  }
  log(entry[0], entry[1]);
  sending = false;
  disarm();
}
// 모든 명령 버튼을 위한 클릭 리스너 하나를 칸에 단다(이벤트 위임). 취소와 Escape 는 확인 칸을
// 닫지만, 명령을 보내는 중에는 Escape 가 아무것도 하지 않는다.
panel.addEventListener('click', event => {
  const button = event.target.closest('button[data-command]');
  if (button && !button.disabled) arm(button.dataset.command);
});
pick.addEventListener('change', showArgs);
sendButton.addEventListener('click', send);
cancelButton.addEventListener('click', () => disarm());
confirmBox.addEventListener('keydown', event => { if (event.key === 'Escape' && !sending) disarm(); });

// 시작: 페이지는 서버가 그린 조각을 품고 오므로, 그 행에 시각을 찍고(live 가 아닌 지난 기록으로)
// 관문을 적용한 뒤 일정한 간격으로 폴링한다.
stamp();
readGates();
pollStatus(true);
timer = setInterval(poll, POLL_MS);
"""


def _head(title):
    """스타일시트를 안에 넣은 문서 머리. 페이지는 바깥에서 아무것도 읽지 않는다(현장에 인터넷이 없을 수 있다)."""
    return ('<!doctype html><html lang="ko"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>{escape(title)}</title><style>{_STYLE}</style></head><body>')


def _flashes(messages):
    return ''.join(f'<div class="flash {escape(category)}">{escape(text)}</div>' for category, text in messages)


def _user_nav(user, csrf):
    """누가 로그인했고 어디로 갈 수 있는지. 관제 화면의 유일한 폼은 로그아웃(POST + CSRF)이다."""
    if user is None:
        return ''
    admin = ' <a href="/admin/accounts">계정 관리</a>' if user['role'] == 'ADMIN' else ''
    return (f'<nav class="who"><span>{escape(user["display_name"])} ({escape(user["role"])})</span>{admin}'
            f' <form class="inline" method="post" action="/logout"><input type="hidden" name="csrf" value="{escape(csrf or "")}">'
            '<button type="submit">로그아웃</button></form></nav>')


def _header(user, csrf):
    """계정 페이지에 쓰는 단순한 페이지 머리."""
    return f'<header class="admin-head"><h1>구조 관제</h1>{_user_nav(user, csrf)}</header>'


def camera_stale_ms(cameras, stale_s):
    """영상을 끊김으로 보이기 전까지 새 프레임 없이 기다리는 ms. 카메라가 있으면 꼭 필요하고 기본값은 없다."""
    if not cameras:
        return None
    if stale_s is None:
        raise ConsoleError('CONFIG_MISSING', 'cameras need --camera-stale-s (seconds without a new frame)')
    if type(stale_s) is not float or not math.isfinite(stale_s) or stale_s <= 0:
        raise ConsoleError('INVALID_INPUT', f'camera stale time must be finite seconds > 0: {stale_s!r}')
    return round(stale_s * 1000)


# 정상으로 끝난 multipart 영상은 오류를 내지 않고 마지막 프레임을 화면에 남긴다(측정
# 2026-10-13, Firefox: 프레임마다 load 이벤트 하나, 닫힌 뒤에는 없음). 그래서 프레임마다 load 이벤트
# 시각을 재고, 끊김 판정 시간을 넘기면 칸에 「영상 끊김」을 보이고 캐시를 피하는 쿼리를 붙여 영상을
# 다시 연다. 다시 열기는 끊김 판정 시간마다 많아야 한 번이다. onerror 는 여전히 바로 표시한다.
_CAMERA_SCRIPT = """
function watchCameras() {
  document.querySelectorAll('#cameras figure[data-stale-ms]').forEach(figure => {
    const img = figure.querySelector('img'), note = figure.querySelector('figcaption').lastChild;
    const stale = Number(figure.dataset.staleMs), base = img.getAttribute('src');
    let last = Date.now(), retried = Date.now();
    // 프레임의 load 이벤트가 살아 있다는 신호이고, 끊김 표시도 지운다.
    img.addEventListener('load', () => {
      last = Date.now();
      if (figure.classList.contains('lost')) {
        figure.classList.remove('lost'); note.textContent = ''; note.className = '';
      }
    });
    // stale / 2 ms 마다 검사하되, 100-500 ms 안으로 둔다.
    setInterval(() => {
      const now = Date.now();
      if (now - last < stale) return;
      figure.classList.add('lost'); note.textContent = '영상 끊김'; note.className = 'lost';
      if (now - retried >= stale) {
        retried = now;
        img.src = base + (base.includes('?') ? '&' : '?') + '_r=' + now;
      }
    }, Math.max(100, Math.min(500, stale / 2)));
  });
}
watchCameras();
"""


def _cameras(cameras, stale_ms=None):
    """카메라 PC 의 MJPEG 게이트웨이(IF-082)에서 바로 받는 영상을 넣는다.

    폴링 조각 밖에 있으므로 폴링이 영상을 다시 시작하지 않는다. 콘솔은 영상을 중계하지
    않는다. 깨지거나 멈춘 영상은 숨기지 않고 끊김으로 보이며 다시 연결한다(_CAMERA_SCRIPT).
    """
    if not cameras:
        return ('<section class="camera-empty"><h2>웹캠</h2><p>웹캠 주소가 없습니다. 실행할 때 '
                '<code>--camera cam_a=http://…</code> 로 카메라 PC 의 영상 주소를 주세요.</p></section>')
    figures = ''.join(
        f'<figure data-stale-ms="{stale_ms}"><img src="{escape(url)}" '
        f'alt="{escape(camera_id)} 영상" '
        'onerror="var f=this.parentNode,s=f.querySelector(\'figcaption\').lastChild;'
        'f.classList.add(\'lost\');s.textContent=\'영상 끊김\';s.className=\'lost\'">'
        '<p class="nosignal">영상이 들어오지 않습니다. 카메라 PC 의 영상 서버와 주소를 확인하세요.</p>'
        f'<figcaption><span>{escape(camera_id)}</span><span></span></figcaption></figure>'
        for camera_id, url in cameras)
    return f'<section id="cameras"><h2>웹캠</h2>{figures}</section>'


def page_title(view):
    """로드 때의 탭 제목. 그 뒤에는 페이지 스크립트의 title() 이 폴링마다 최신으로 유지한다."""
    return ('[재난] ' if view['mission']['mode'] == 'DISASTER' else '') + '구조 관제'


def failure_text(code, message):
    """갱신 상태에 쓸 한 줄: 코드, 확인할 것, 원문 상세."""
    return f'{code}: {FAILURE_HINTS.get(code, FAILURE_HINT_OTHER)} ({message})'


def render_failure(code, message):
    """페이지 전체 실패(503): 무엇이 실패했고 무엇을 확인할지 보이고, FAILURE_RETRY_S 마다 다시 시도한다.

    비어 있으면서 정상처럼 보이는 콘솔은 절대 내지 않는다(AGENTS.md 7절: 실패는 드러나야 한다).
    """
    return (_head('[화면 오류] 구조 관제').replace(
                '<meta charset="utf-8">', f'<meta charset="utf-8"><meta http-equiv="refresh" content="{FAILURE_RETRY_S}">')
            + '<main class="card failure"><h1>화면을 그릴 수 없습니다</h1>'
            f'<p class="hint">{escape(FAILURE_HINTS.get(code, FAILURE_HINT_OTHER))}</p>'
            f'<p class="detail"><code>{escape(code)}</code> {escape(message)}</p>'
            f'<p class="retry">{FAILURE_RETRY_S}초마다 다시 시도합니다. 로봇은 이 화면과 상관없이 동작하므로, '
            '위험하면 현장 비상 정지 장치를 쓰세요.</p></main></body></html>')


def render_page(view, user=None, csrf=None, cameras=(), command_access=None, camera_stale_s=None):
    """`cameras`: 실행 인자에서 온 ((camera_id, http(s) URL), ...). 이것이 있으면 `camera_stale_s` 도
    필요하다(「영상 끊김」 전까지 새 프레임 없이 기다리는 초, 기본값 없음). `command_access`:
    commands.command_access 의 (allowed, reason). 없으면 이 페이지에서 명령을 보낼 수 없다."""
    access = command_access or (False, '명령 연결 없음')
    stale_ms = camera_stale_ms(cameras, camera_stale_s)
    script = _SCRIPT.replace('POLL_MS', str(POLL_INTERVAL_MS)).replace('KPI_BUDGET_S', f'{KPI04_BUDGET_S:g}')
    if cameras:
        script += _CAMERA_SCRIPT
    return (_head(page_title(view)) + '<main class="console">'
            + f'<section id="topbar"><p id="poll" class="wait">연결 중</p><div id="poll-error" hidden></div>'
            + '<div class="kpi"><p id="kpi" title="발생 시각과 이 화면에 처음 그린 시각의 차이. 두 PC 의 시계가 맞을 때만 의미가 있습니다 (C-03)">'
            'KPI-04: 페이지를 연 뒤 들어온 사건 없음</p>'
            '<button type="button" id="kpi-save" class="mini">지연 기록 저장</button></div>'
            + _user_nav(user, csrf) + '</section>'
            + f'<aside id="side">{_cameras(cameras, stale_ms)}{_commands(view, access, csrf)}</aside>'
            + f'<div id="live">{render_fragment(view)}</div></main>'
            f'<script>{script}</script></body></html>')


def render_login(csrf, messages=()):
    """로그인 페이지(강사 day5 요구: SQLite 의 로그인 데이터를 쓰는 로그인 페이지)."""
    return (_head('구조 관제 로그인') + '<div class="card"><h1>구조 관제 로그인</h1>' + _flashes(messages)
            + '<form method="post" action="/login">'
            f'<input type="hidden" name="csrf" value="{escape(csrf)}">'
            '<label for="username">아이디</label><input id="username" name="username" autocomplete="username" required>'
            '<label for="password">비밀번호</label>'
            '<input id="password" name="password" type="password" autocomplete="current-password" required>'
            '<p><button type="submit">로그인</button></p></form></div></body></html>')


def render_accounts(accounts, user, csrf, messages=()):
    """ADMIN 전용: 계정 목록, 고객 계정 만들기, 계정 수정·삭제."""
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
