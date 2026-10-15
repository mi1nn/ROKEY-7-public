"""화면 모델 -> HTML 렌더링. 표준 라이브러리 기반으로 구현하여 Flask 의존성 없이 단위 테스트가 가능하다."""

from datetime import datetime
from html import escape
import json
import math

from .labels import label
from .mapimage import MAP_FRAME
from .view import CAUSE_WORDS, STAGES, ConsoleError

# 브라우저 폴링 주기. KPI-04 예산은 3초(BRD v0.1)이며, 1초 주기로 폴링하면 남은 2초를
# 운영 장부 기록·조회·전송 처리에 활용한다([제안] 관제·화면, 2026-10-08; README 「KPI-04 측정」).
POLL_INTERVAL_MS = 1000
# KPI-04 레이턴시 예산: 사건 발생 시각부터 운영자 화면에 렌더링될 때까지의 시간(BRD v0.1 KPI-04, 3초).
KPI04_BUDGET_S = 3.0
MISSING = '—'
# 화면 위에 보이는 시스템 이름: 팀 BRD Part I 제목(docs/requirements/brd-part1-v0.1.md, 범위 RQ-09).
# 탭 제목·「구조 관제」 문구는 화면 이름이라 그대로 둔다(2026-10-15 사용자 지시: 「시스템명은 문서에 명시된 이름으로」).
SYSTEM_NAME = '지진 재난 현장 대응형 실내 탐색 로봇'
# 지도 표식 크기(지도 픽셀 단위, 재난 전 기준 지도는 0.05 m/px): 로봇 화살표 반폭 및
# 전방 꼭짓점 길이, 요구조자 V 표식 반폭. 화면 표시 전용이다([제안] 관제·화면, 2026-10-10).
ROBOT_MARK_PX, HEADING_PX, TARGET_MARK_PX = 2.6, 4.8, 2.0


def _text(value):
    """임의의 값을 HTML 이스케이프된 문자열로 변환한다. None은 MISSING으로 표시된다."""
    return MISSING if value is None else escape(str(value))


def _time(ns, ref_ns=None):
    """나노초 단위 타임스탬프를 시각 문자열로 표시한다. `ref_ns`와 날짜가 다르면 날짜도 함께 병기한다."""
    if ns is None:
        return MISSING
    moment = datetime.fromtimestamp(ns / 1e9)
    same_day = ref_ns is None or moment.date() == datetime.fromtimestamp(ref_ns / 1e9).date()
    stamp = moment.strftime('%H:%M:%S' if same_day else '%Y-%m-%d %H:%M:%S')
    # 밀리초는 따로 감싸 화면에서는 숨긴다(2026-10-15 화면 개선). 전체 값은 title(ns)에 있다.
    return f'<span title="{ns}">{stamp}<span class="ms">.{moment.microsecond // 1000:03d}</span></span>'


def _age(seconds):
    """경과 시간 문자열(0.4s, 12.0s, 3분 05초, 2시간 10분, 41일). 음수는 원격 시계가 호스트보다 앞섬을 의미한다."""
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


# 화면에 표시할 UUID 글자 수. 전체 UUID는 title 툴팁에 보존된다.
SHORT_ID = 8
# 배터리 잔량 표시 색상 전환 임계값(단위: %)([제안] 관제·화면, 2026-10-10; UI 표시 전용).
BATTERY_LOW, BATTERY_WARN = 25, 50
# 지도 평면 상의 측량 그리드 간격(단위: 미터).
GRID_M = 1.0
STAGE_ORDER = ('UNCONFIRMED', 'NOT_RESCUED', 'SUPPORTING', 'RESCUED')
# 콘솔이 입력 데이터를 읽지 못할 때 표시할 조치 가이드(ConsoleError 코드별 대응).
FAILURE_HINTS = {
    'STORAGE_ERROR': '운영 장부나 현장 상태 DB 를 읽지 못했습니다. 실행 인자의 파일 경로와 operations 프로세스가 돌고 있는지 확인하세요.',
    'INVALID_INPUT': '읽은 데이터의 형식이 화면이 기대한 것과 다릅니다. operations 와 콘솔이 같은 버전인지 확인하세요.',
    'POLICY_MISMATCH': '연 파일이 기대한 DB 가 아닙니다(역할·스키마가 다름). 실행 인자의 경로를 확인하세요.',
}
FAILURE_HINT_OTHER = '콘솔을 띄운 터미널의 로그를 확인하세요.'
# 장애 화면 자동 재시도 폴링 주기(초). 정상 입력 복구 시 화면이 자동으로 복원된다.
FAILURE_RETRY_S = 5


def _short(ident):
    """테이블 표시용 단축 식별자. SHORT_ID + 1자 이하는 그대로 표시하고 초과 시 잘라낸다.

    전체 식별자는 title 툴팁에 보존되며, None은 MISSING으로 표시된다. 프론트엔드 shortId()도 동일한 규칙을 따른다.
    """
    if ident is None:
        return MISSING
    text = str(ident)
    shown = text if len(text) <= SHORT_ID + 1 else text[:SHORT_ID]
    return f'<span class="id" title="{escape(text)}">{escape(shown)}</span>'


def _word(kind, code):
    """도메인 코드를 한국어 레이블(labels.label)로 변환해 이스케이프한다. 미등록 코드는 원본 유지, None은 MISSING 표기."""
    return _text(label(kind, code))


def _elapsed(seconds):
    """임무 경과 시간(초 단위 정수). 1시간 미만은 분/초, 24시간 미만은 시간/분, 초과 시 일/시간 단위 포맷.

    `_age`와 달리 None이거나 음수(트리거 시각이 집계 시각보다 미래)인 경우 MISSING으로 표시된다.
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
    """로봇 자세 문자열 `(x, y) m`. yaw 정보가 포함되어 있으면 도(°) 단위로 변환해 표기한다.

    MAP_FRAME이 아닌 로컬 좌표계는 대괄호 안에 frame_id를 명시하여 지도 좌표와 오인되지 않도록 방지한다.
    """
    if pose is None:
        return MISSING
    text = f"({pose['x']:.2f}, {pose['y']:.2f}) m"
    if pose.get('yaw') is not None:
        text += f" {math.degrees(pose['yaw']):.0f}°"
    if pose['frame_id'] != MAP_FRAME:
        text += f" [{pose['frame_id']}]"
    if pose.get('estimated'):
        text += ' 추정(TB-6 출발 자세 + 현장 파일 시작 값)'
    return escape(text)


def _victim_path(x, y, size):
    """INSARAG 규격 요구조자 표식 SVG 패스(꼭짓점이 대상 좌표에 닿는 V자 형태)."""
    return (f'M{x - size:.2f},{y - 1.6 * size:.2f} L{x:.2f},{y:.2f} '
            f'L{x + size:.2f},{y - 1.6 * size:.2f}')


def _victim_icon(stage):
    """요구조자 목록 및 범례용 축소 V 아이콘(지도 표식과 색상·도안 일치)."""
    strike = '<line x1="2" y1="7" x2="14" y2="7"/>' if stage == 'RESCUED' else ''
    return (f'<svg class="vicon {stage.lower()}" viewBox="0 0 16 16" aria-hidden="true">'
            f'<path d="{_victim_path(8, 13, 5)}"/>{strike}</svg>')


# ---- 임무 상태 띠 배너 (조각 렌더링) -------------------------------------------------------------

def _reason(reason):
    """조정기 사유를 읽기 쉬운 레이블로 렌더링하며, 코드가 다른 경우 <code>로 병기한다."""
    level = f' {reason["level"]}' if reason['level'] else ''
    code = '' if reason['text'] == reason['code'] else f' <code>{escape(reason["code"])}</code>'
    return f'<span class="reason{level}" title="조정기가 알려 준 지금 상태의 이유">{escape(reason["text"])}</span>{code}'


def _scoped_reason(view, *scopes, tag='p'):
    """조정기 사유 엘리먼트를 생성한다. 사유의 적용 범위가 `scopes`에 포함될 때만 출력한다."""
    reason = view['mission'].get('reason_view')
    if reason is None or reason['scope'] not in scopes:
        return ''
    return f'<{tag} class="scoped-reason">{_reason(reason)}</{tag}>'


def _band(view):
    """화면 최상단 임무 모드 배너. 출구 누적은 생략한다(단일 고정 카메라 환경, db-schema 2절)."""
    mission = view['mission']
    mode, source = mission['mode'], mission['source']
    built = view['built_at_ns']
    facts = []
    if mission['trigger_ns'] is not None:
        # 재난 출처(작전 2026-10-15): 지진 감지와 운영자 선포를 구분하고, 둘 다 아니면 「출처 모름」을 드러낸다.
        cause = mission.get('cause')
        when = '운영자 선포 시각' if cause == 'OPERATOR' else '재난 감지 시각' if cause in (None, 'SIGN') else '재난 시작 시각'
        facts += [(when, _time(mission['trigger_ns'], built)),
                  ('경과 시간', _elapsed((built - mission['trigger_ns']) / 1e9))]
        if cause is not None:
            facts.append(('재난 출처', escape(CAUSE_WORDS[cause])))
    if mode == 'DISASTER' and mission['sign_count']:
        facts.append(('진동 감지 횟수', f'{mission["sign_count"]}건'))
    if mission['phase'] is not None:
        facts.append(('임무 단계', _word('phase', mission['phase'])))
    dispatch = mission['dispatch']
    if dispatch is not None:
        facts.append(('이동 구간', f'<span title="끝낸 구간 수 / 전체 구간 수">{dispatch["segment_index"]}/{dispatch["segment_count"]}</span> '
                              f'{_word("segment", dispatch["segment_phase"])}'))
    if mission.get('reason_view') is not None:
        facts.append(('상태 사유', _reason(mission['reason_view'])))
    if source == 'mission_state':
        note = f'조정기 임무 상태, {_age((built - mission["updated_at_ns"]) / 1e9)} 전 바뀜'
    elif mode == 'DISASTER':
        note = '임무 상태 기록 전 — 현장 카메라의 진동 감지로 추정'
    elif mode == 'NORMAL':
        note = '임무 상태 기록 전 — 진동 감지 없음'
    else:
        note = '현장 사건을 읽지 못해 모드를 알 수 없음'
    dl = ''.join(f'<div><dt>{name}</dt><dd>{value}</dd></div>' for name, value in facts)
    return (f'<section id="mission" class="band {(mode or "unknown").lower()}" data-mode="{escape(mode or "")}" '
            f'title="{escape(note)}">'
            f'<h1 class="mode-word">{escape(mission["label"])}</h1>'
            f'<dl class="facts">{dl}</dl><p class="mode-note">{note}</p>'
            f'<p id="built" data-built-ns="{built}">서버 기준 시각 {_time(built)}</p></section>')


def _warnings(view):
    if not view['warnings']:
        return '<section id="warnings" class="empty"></section>'
    items = ''.join(f'<li>{escape(w)}</li>' for w in view['warnings'])
    return f'<section id="warnings"><h2>확인 필요 {len(view["warnings"])}건</h2><ul>{items}</ul></section>'


# ---- 지도 뷰 (조각 렌더링) ----------------------------------------------------------------------

def _grid(layer):
    """미터 단위 측량 그리드 및 축 눈금을 이미지 픽셀 좌표계로 렌더링한다."""
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
    """재난 전 기준 지도를 SVG 배경으로 배치하여 표식과 이미지가 항상 동일 비율로 확대·축소되도록 렌더링한다."""
    layer = view['map']
    live = layer.get('live')
    if not layer.get('available') and live is not None:
        return (f'<section id="map" class="placeholder"><h2>현장 지도</h2><p>{escape(live["message"])}</p>'
                '<p>운영 launch 의 <code>map_topic</code>·<code>map_dir</code> 와 SLAM 지도 발행을 확인하세요.</p></section>')
    if not layer.get('available'):
        return ('<section id="map" class="placeholder"><h2>현장 지도</h2>'
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
    for passage in layer.get('passages', ()):
        x, y, h = passage['px'], passage['py'], TARGET_MARK_PX
        # 막힌 곳으로 칠해진 터널은 눌러서 통과 승인 확인 창을 연다(#406, data-passage 는 항목 id).
        click = (f' data-passage="{escape(passage["id"])}"' if passage.get('approvable') else '')
        marks.append(   # 터널 판정: 기둥 둘과 지붕(∏), 초록 통과 가능 / 빨강 통과 불가 / 회색 판정 불가
            f'<g class="passage {passage["state"].lower()} {"current" if passage["current"] else "old"}'
            f'{" approvable" if click else ""}"{click}>'
            f'<title>{escape(passage["label"])} · {escape(passage["robot_id"])} · map_revision {passage["map_revision"]}'
            f'{"" if passage["current"] else " (이전 지도)"}{" · 누르면 통과 승인" if click else ""}</title>'
            f'<path class="mark" d="M{x - h:.2f},{y + h:.2f} V{y - h:.2f} H{x + h:.2f} V{y + h:.2f}"/>'
            f'<text class="label" x="{x + h + 0.5:.2f}" y="{y + h:.2f}">{escape(passage["label"])}</text></g>')
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
            # SVG y축은 아래로 증가하므로, 지도 상의 반시계 방향(+yaw)은 SVG 상에서 -y 방향이다.
            points = []
            for angle, reach in ((0.0, HEADING_PX), (2.5, ROBOT_MARK_PX), (-2.5, ROBOT_MARK_PX)):
                a = robot['yaw'] + angle
                points.append(f'{x + reach * math.cos(a):.2f},{y - reach * math.sin(a):.2f}')
            body = f'<polygon class="heading" points="{" ".join(points)}"/>'
        # 추정 위치(R1 전 TB-5): 점선·흐린 색에 「추정」 글자를 붙여 실제 위치와 구분한다(작전 2026-10-15).
        estimated = robot.get('estimated')
        why = ' — 추정: TB-6 출발 자세 + 현장 파일 시작 값, 실제 위치 아님' if estimated else ''
        marks.append(
            f'<g class="robot {escape(role.lower())}{" estimated" if estimated else ""}">'
            f'<title>{escape(robot["id"])}{why}</title>{body}'
            f'<text class="label" x="{x + ROBOT_MARK_PX + 1:.2f}" y="{y + ROBOT_MARK_PX + 2.4:.2f}">'
            f'{escape(unit)}{" 추정" if estimated else ""}</text></g>')
    unplotted = [f'{w["name"][:SHORT_ID]}: 좌표계 {w["frame_id"]} — 지도 좌표계가 아님' for w in layer['frame_mismatch']]
    unplotted += [f'{w.get("label") or w["name"][:SHORT_ID]}: 지도 밖 ({w["x"]:.2f}, {w["y"]:.2f}) m' for w in layer['off_map']]
    unplotted += [f'{w["name"]}: 위치 미확정' for w in layer.get('no_pose', ())]
    notes = ''.join(f'<li>{escape(text)}</li>' for text in unplotted)
    legend = ''.join(f'<li>{_victim_icon(stage)}{escape(STAGES[stage])}</li>' for stage in STAGE_ORDER)
    legend += '<li><svg class="vicon robot" viewBox="0 0 16 16" aria-hidden="true"><polygon points="14,8 3,2.5 3,13.5"/></svg>로봇 · 진행 방향</li>'
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
    return (f'<section id="map" class="map{state}"><header><h2>현장 지도</h2><ul class="legend">{legend}</ul></header>'
            f'<svg class="plan" viewBox="{-pad_left} {-pad_top} {width + pad_left + 2} {height + pad_top + pad_bottom}" '
            f'preserveAspectRatio="xMidYMid meet" role="img" aria-label="기준 지도 위 로봇과 요구조자 위치">'
            f'{_MAP_TONE}<image href="{escape(layer["url"])}" x="0" y="0" width="{width}" height="{height}" '
            f'preserveAspectRatio="none"/>'
            f'<rect class="frame" x="0" y="0" width="{width}" height="{height}"/>'
            f'{_grid(layer)}{"".join(marks)}</svg>'
            f'<p class="map-note" title="{escape(layer["note"])}">{escape(note)}</p>'
            + (f'<ul class="unplotted"><li class="head">지도에 그리지 못한 위치</li>{notes}</ul>' if notes else '')
            + '</section>')


# 지도 그림(PGM 회색조)의 화면 색: 벽(0) 차콜, 미탐색(205) 지도 바탕과 같은 연회색, 빈 곳(254) 흰색.
# 지도 값은 바꾸지 않고 화면 색만 정한다(2026-10-15 화면 개선). SVG 안에 두어 외부 파일이 없다.
_MAP_TONE = ('<defs><filter id="map-tone" color-interpolation-filters="sRGB"><feColorMatrix type="saturate" values="0"/>'
             '<feComponentTransfer><feFuncR type="table" tableValues=".25 .42 .6 .77 .91 1"/>'
             '<feFuncG type="table" tableValues=".28 .45 .63 .8 .925 1"/>'
             '<feFuncB type="table" tableValues=".32 .49 .66 .82 .937 1"/></feComponentTransfer></filter></defs>')


# ---- 로봇 및 요구조자 현황 (조각 렌더링) --------------------------------------------------------

def _battery(robot):
    """로봇 카드의 배터리 게이지: 임계값별 색상 클래스, 잔량(%), 측정값 경과 시간.

    배터리 정보가 없으면 빈 게이지나 0% 대신 「배터리 값 없음」으로 명시한다. 프로그레스 너비는 0-100%로
    제한하되 수치 레이블은 수신값 그대로 표시한다.
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
            f'{flag} · {age} · {battery} · {dock}{" · 위치는 자기 지도 기준, 이 지도와 다름" if w["own_map"] else ""}</p>')


def _robots(view):
    """배치 설정 순서(view['robots'])대로 로봇 상태 카드를 렌더링한다.

    아직 수신된 상태가 없는 로봇은 빈 카드 대신 경고 카드로 표시된다. 그 외에는 주행/준비 상태,
    배터리 잔량, 좌표 자세, 실행 중 작업, operations 수신 경과 시간, 로봇 역할에 해당하는 조정기 사유를
    표시하며, 상태가 지연된 경우 stale 경고(마지막 수신값 유지 및 stale 스타일 적용)를 제공한다.
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
        stale = (f'<p class="alert" title="새 상태가 오지 않아 마지막으로 받은 값을 보여 주는 중입니다">낡음: '
                 f'{_age(robot["received_age_s"])} 동안 새 상태 없음</p>') if robot['stale'] else ''
        units.append(
            f'<section class="unit {(role or "unknown").lower()}{" stale" if robot["stale"] else ""}" '
            f'data-robot="{escape(robot["robot_id"])}">{head}'
            f'<p class="state"><span class="motion {escape((motion or "unknown").lower())}">{_word("motion", motion)}</span>'
            f'<span class="ready">{_word("readiness", robot["readiness"])}</span></p>'
            + _battery(robot)
            + f'<dl><div><dt>현재 위치</dt><dd>{_pose(robot["pose"])}</dd></div>'
            f'<div><dt>진행 작업</dt><dd>{_short(robot["active_task_id"])}</dd></div>'
            f'<div><dt>마지막 수신</dt><dd>{_age(robot["received_age_s"])} 전</dd></div></dl>'
            + _scoped_reason(view, {'SUPPORT': 'support', 'SCOUT': 'scout'}.get(role)) + stale
            + (_watch(robot) if robot.get('watch') else '') + '</section>')
    return f'<section id="robots"><h2>로봇 상태</h2>{"".join(units)}</section>'


def _targets(view):
    """요구조자 섹션: 구조 단계별 통계 및 개별 대상 상세 목록.

    각 행에는 구조 단계, 상태 판정 규칙 ID, 추정 좌표, 구조/접근 상태, 지원 출동 현황, 소실 후보 표시
    (고정 카메라 단서이며 최종 구조 완료는 로봇 현장 재확인으로 확정), 보고 누적 건수를 표시한다.
    보고 장부 데이터가 없으면 0건 대신 「보고 장부 입력 없음」으로 명시한다. 조정기 사유는 출동 대상에 표기하며,
    자동 출동 대기 상태인 경우 NOT_RESCUED 대상들에 미출동 사유를 안내한다.
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
            f'<span class="rule" title="이 단계를 정한 규칙 번호(R1 ~ R4, 관제 웹 README 「구조 상태 4단계」)">{target["rule"]}</span></div>'
            f'<div class="what"><span>{where}</span><span>구조 {_word("rescue", target["rescue_state"])}</span>'
            f'<span>{_word("approach", target["approach_result"])}</span>{support}{candidate}{receipt}</div>'
            + (f'<ul class="notes">{notes}</ul>' if notes else '') + '</li>')
    body = ''.join(rows) or '<li class="none">아직 보고된 요구조자가 없습니다.</li>'
    return (f'<section id="targets"><h2>요구조자 {len(view["targets"])}명</h2>'
            f'<ul class="stage-counts">{counts}</ul><ul class="victims">{body}</ul></section>')


# ---- 사건 기록 (조각 렌더링) ---------------------------------------------------------------------

def _event_text(event):
    """사건 행의 상세 내용 텍스트 생성.

    REPORT 사건은 확정/접근/보고접수 상태를 요약하고, 현장 사건은 신뢰도 및 메모를 표시한다.
    표시할 정보가 없으면 MISSING으로 대체된다.
    """
    if event['kind'] == 'REPORT':
        return (f'{_word("confirmation", event["confirmation"])}, {_word("approach", event["approach"])}, '
                f'{_word("receipt", event["receipt"])}')
    text = '' if event['confidence'] is None else f'신뢰도 {event["confidence"]:.2f}'
    if event['note']:
        text += f' {escape(event["note"])}'
    return text.strip() or MISSING


def _events(view):
    """사건 기록 테이블(최신순 정렬). 부분 조회 시 건수 요약이 함께 표시된다.

    각 행에는 프론트엔드 스크립트가 활용할 data-* 메타데이터(ref, 타임스탬프, 종류, 시계 도메인)를 주입한다.
    브라우저 스크립트가 렌더링 시각 및 지연 시간(KPI-04)을 계산하여 채우며, 서버 초기값은 MISSING으로 전달된다.
    """
    rows = []
    built = view['built_at_ns']
    for event in view['events']:
        rows.append(
            f'<tr class="event {event["kind"].lower()}" data-ref="{escape(event["ref_id"])}" '
            # 현장 사건 행에는 장부 시각이 없어(read_site_events 가 돌려주지 않음) 빈 값으로 둔다.
            f'data-event-ns="{event["event_ns"]}" data-received-ns="{event["received_at_ns"] or ""}" '
            f'data-kind="{escape(event["kind"])}" data-clock="{escape(event["clock_domain_id"])}"'
            # 터널 통과 승인 행은 사람이 누른 기록이라 KPI-04(사건 표시 지연)에 넣지 않는다
            f'{"" if event.get("kpi", True) else " data-kpi=\"0\""}>'
            f'<td class="t" title="시계 {escape(event["clock_domain_id"])}">{_time(event["event_ns"], built)}</td>'
            f'<td class="kind">{_word("event", event["kind"])}</td>'
            f'<td>{escape(event["source"])}</td><td>{_short(event["subject"])}</td>'
            f'<td class="text">{_event_text(event)}</td>'
            f'<td class="t">{_time(event["received_at_ns"], built)}</td>'
            f'<td class="t rendered">{MISSING}</td><td class="t latency">{MISSING}</td></tr>')
    shown = len(view['events'])
    more = f' (최근 {shown}건 표시)' if view['event_total'] > shown else ''
    return (f'<section id="events"><h2>사건 기록 {view["event_total"]}건{more}</h2><div class="scroll">'
            '<table><thead><tr><th>발생 시각</th><th>종류</th><th>보낸 곳</th><th>대상</th><th>내용</th>'
            '<th title="운영 장부에 기록된 시각">기록 시각</th><th title="이 화면에 처음 그린 시각">화면 표시</th>'
            '<th title="발생부터 이 화면에 뜨기까지 걸린 시간 (목표 3초, KPI-04)">표시 지연</th></tr></thead>'
            f'<tbody>{"".join(rows)}</tbody></table></div></section>')


# ---- 명령 및 오류 이력 (조각 렌더링) -------------------------------------------------------------

def _reply(code):
    """응답 사유 레이블 및 식별 코드 포맷. 레이블과 코드가 동일한 경우 단일 표기한다."""
    if code is None:
        return ''
    word = label('reply', code)
    return escape(code) if word == code else f'{escape(word)} <code>{escape(code)}</code>'


def _history(view):
    """운영자 명령 및 operations 오류 이력(--site-state 소스의 operator_commands, error_log 테이블).

    입력 소스가 없거나 고정 파일 모드인 경우 안내 문구를 출력한다.
    명령 행은 접수/거절 여부 및 사유 코드, 대상을 표시하며, 오류 행은 오류 코드와 세부 내용을 표시한다.
    누락된 입력 소스가 있으면 명칭을 함께 안내한다.
    """
    history = view['history']
    head = '<h2>보낸 명령 · 오류</h2>'
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
    body = ('<table><thead><tr><th>시각</th><th>내용</th><th>결과 · 이유</th></tr></thead>'
            f'<tbody>{"".join(rows)}</tbody></table>') if rows else '<p>아직 기록 없음</p>'
    return f'<section id="history">{head}{missing}{body}</section>'


def _gates(view):
    """명령 패널 동기화를 위한 게이트 상태 미러. 브라우저 스크립트가 폴링마다 동기화한다."""
    gates = {c['name']: {key: c[key] for key in ('enabled', 'reason', 'args', 'choices', 'confirm') if key in c}
             for c in view['commands']}
    return f'<div id="gates" hidden data-gates="{escape(json.dumps(gates, ensure_ascii=False))}"></div>'


def render_fragment(view):
    """브라우저가 폴링마다 교체 렌더링하는 본문 프래그먼트 HTML."""
    return (_band(view) + _warnings(view) + _map(view) + '<div id="middle">' + _robots(view) + _targets(view)
            + '</div>' + _events(view) + _history(view) + _gates(view))


# ---- 정적 페이지 레이아웃 (폴링 영역 외곽) -------------------------------------------------------

def _commands(view, access, csrf):
    """명령 성격별 그룹 버튼, 2단계 승인 대화상자, 명령 전송 결과 로그 패널.

    `access`는 현재 사용자 권한의 (allowed, reason) 튜플이다(commands.command_access).
    버튼 활성화를 위해서는 사용자 권한과 도메인 게이트 조건(view.command_gates)이 모두 충족되어야 한다.
    이 패널은 폴링 프래그먼트 외부에 위치하므로 갱신 주기마다 포커스나 입력 상태가 초기화되지 않는다.
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
    return (f'<section id="commands"{data}><header><h2>운영 명령</h2>'
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
            '<p class="estop">위험할 때는 현장 비상 정지 장치를 쓰세요.</p></section>')


_STYLE = """
/* 2026-10-15 사용자 지시 「지진 재난 감지·대응 시스템 Web UI 개선」과 후속(「레이아웃은 기존처럼」, 「현대적이고 모던하게, 색 바꿔도 됨」,
   군더더기 문구 빼기): 밝은 바탕 위 그림자 카드, 인디고 강조, 또렷한 숫자, 부드러운 색 배지.
   색의 뜻(빨강 재난·정지, 주황 미구조, 파랑 지원·승인, 초록 구출, 회색 미확정, 노랑 주의)과 칸 배치·클래스·ID·스크립트는 그대로다.
   외부 글꼴·CSS 없음(오프라인). */
:root{
  --bg:#f3f5f9; --card:#ffffff; --soft:#f6f8fb; --raise:#eef2f7; --line:#e7ecf2; --line-2:#d8dfe8;
  --ink:#0f172a; --ink-2:#475569; --ink-3:#6b778a;   /* --ink-3: 흰 바탕 대비 약 4.6:1(작은 라벨도 읽히게) */ --accent:#4f46e5; --accent-soft:#eef0ff;
  --blue:#2563eb; --blue-hover:#1d4ed8; --map-bg:#eef2f6;
  --signal:#e5484d; --signal-strong:#dc2f35; --rescue:#f76b15; --info:#2563eb; --safe:#16a34a; --pending:#94a3b8; --caution:#d99a06;
  --signal-tint:#fff0f0; --rescue-tint:#fff3eb; --info-tint:#eef4ff; --safe-tint:#ecfaf1; --caution-tint:#fff8e6; --pending-tint:#f1f5f9;
  --shadow:0 1px 2px rgba(15,23,42,.04),0 2px 8px rgba(15,23,42,.05);
  --r:14px; --r-in:10px; --pill:999px; --gap:14px;
  --font:"Pretendard","Noto Sans CJK KR","Noto Sans KR","Apple SD Gothic Neo","Malgun Gothic",system-ui,sans-serif;
}
*{box-sizing:border-box}
html,body{margin:0}
body{font-family:var(--font);font-size:14px;line-height:1.5;color:var(--ink);background:var(--bg);letter-spacing:-.005em;
  font-variant-numeric:tabular-nums;-webkit-font-smoothing:antialiased}
h1,h2,h3,p{margin:0}
h2{font-size:15px;font-weight:700;color:var(--ink);letter-spacing:-.01em;margin-bottom:12px}
code{font-size:12.5px;background:var(--raise);color:var(--ink-2);padding:1px 6px;border-radius:6px}
a{color:var(--accent);text-decoration:none;font-weight:600} a:hover{text-decoration:underline}
:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
.id{font-variant-numeric:tabular-nums}
.ms{display:none}   /* 밀리초는 화면에서 숨김(전체 값은 마우스를 올리면 title) */
dl{margin:0}
select,input{font:inherit;color:var(--ink);background:#fff;border:1px solid var(--line-2);border-radius:8px}
select{padding:5px 10px}
select:focus,input:focus{border-color:var(--accent);outline:none;box-shadow:0 0 0 3px rgba(79,70,229,.15)}
button{font-family:inherit}
::-webkit-scrollbar{width:8px;height:8px} ::-webkit-scrollbar-thumb{background:#d4dbe5;border-radius:4px}
::-webkit-scrollbar-track{background:transparent}

/* ---- 관제 화면 격자 (배치는 기존 그대로, 맨 위에 이름 줄) ---- */
.console{display:grid;gap:var(--gap);padding:0 16px 16px;min-height:100vh;
  grid-template-columns:minmax(0,1fr) 390px 470px;
  grid-template-rows:auto auto auto minmax(0,1fr) fit-content(230px);
  grid-template-areas:"head head head" "band band top" "warn warn side" "map middle side" "log middle side"}
#live{display:contents}
/* 보낸 명령 · 오류 칸은 평소 접어 두고(그 자리는 로봇 상태에), 위쪽 막대의 「보낸 명령 · 오류」를 누르면 편다.
   펼침 상태는 갱신 영역(#live) 밖 체크박스라 1초 갱신에도 유지된다. 스크립트는 바꾸지 않는다(2026-10-15 사용자 요청). */
#history{display:none}
.console:has(#show-history:checked){grid-template-areas:"head head head" "band band top" "warn warn side" "map middle side" "log hist side"}
.console:has(#show-history:checked) #history{display:block}
#show-history{position:absolute;opacity:0;width:1px;height:1px;pointer-events:none}
.history-toggle{font-size:12.5px;font-weight:600;padding:4px 12px;border-radius:var(--pill);background:var(--raise);color:var(--ink-2);
  cursor:pointer;user-select:none;white-space:nowrap;transition:background-color .15s}
.history-toggle:hover{background:var(--line-2)}
#show-history:focus-visible+.history-toggle{outline:2px solid var(--accent);outline-offset:2px}
#show-history:checked+.history-toggle{background:var(--accent);color:#fff}
#masthead{grid-area:head}
#mission{grid-area:band} #warnings{grid-area:warn} #map{grid-area:map} #middle{grid-area:middle}
#events{grid-area:log} #history{grid-area:hist} #topbar{grid-area:top} #side{grid-area:side}
@media (min-width:1400px) and (min-height:820px){ .console{height:100vh;overflow:hidden}
  /* 한 화면 고정 배치: 카메라 영상 높이를 화면 높이에 묶어 명령 칸이 남는 높이를 쓰게 한다. */
  #cameras img,#robot-camera img,#cameras .nosignal,#robot-camera .nosignal{max-height:16vh}
  /* 로봇 카메라 칸이 없으면 웹캠을 크게 */
  #side:not(:has(#robot-camera)) #cameras img,#side:not(:has(#robot-camera)) #cameras .nosignal{max-height:27vh} }
@media (max-width:1399px){
  .console{grid-template-columns:minmax(0,1fr);grid-template-rows:none;
    grid-template-areas:"head" "top" "band" "warn" "map" "side" "middle" "log" "hist"}
  .console:has(#show-history:checked){grid-template-areas:"head" "top" "band" "warn" "map" "side" "middle" "log" "hist"}
}
#map,#middle>section,#events,#history,#side>section{background:var(--card);border-radius:var(--r);padding:16px 18px;box-shadow:var(--shadow)}
#middle{display:flex;flex-direction:column;gap:var(--gap);min-height:0}
/* 칸이 부족하면 요구조자 목록보다 로봇 카드가 먼저 축소되며 내부 스크롤된다(#64). */
#robots{min-height:0;overflow:auto}
#side{display:flex;flex-direction:column;gap:var(--gap);min-height:0}
#side>section{flex:none}

/* ---- 이름 줄 ---- */
#masthead{display:flex;align-items:center;gap:12px;margin:0 -16px;padding:0 24px;height:54px;background:#fff;
  border-bottom:1px solid var(--line);min-width:0}
#masthead .sys{order:1;font-size:15px;font-weight:700;letter-spacing:-.01em;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
#masthead .sys::before{content:"";display:inline-block;width:22px;height:22px;margin-right:10px;border-radius:7px;vertical-align:-6px;
  background:var(--accent);box-shadow:inset 0 0 0 6px var(--accent),inset 0 0 0 9px #fff}
#masthead .replay-link{order:3;font-size:12.5px;white-space:nowrap}
#masthead .screen{order:2;font-size:12.5px;font-weight:600;padding:3px 10px;border-radius:var(--pill);background:var(--accent-soft);color:var(--accent);white-space:nowrap}

/* ---- 임무 카드 ---- */
.band{display:grid;grid-template-columns:auto;grid-auto-flow:column;grid-auto-columns:minmax(0,1fr);align-items:center;
  padding:14px 8px 14px 18px;background:var(--card);border-radius:var(--r);box-shadow:var(--shadow)}
.band .mode-word{display:flex;align-items:center;gap:8px;font-size:19px;font-weight:800;padding:8px 18px;border-radius:var(--pill);
  background:var(--pending-tint);color:var(--ink-2)}
.band .mode-word::before{content:"";width:9px;height:9px;border-radius:50%;background:currentColor}
/* 지표 줄: 모드 배지 뒤로 값마다 같은 폭의 칸과 얇은 구분선(값 개수가 바뀌어도 띠가 고르게 찬다) */
.band .facts{display:contents}
.band .facts>div,#built{min-width:0;padding:2px 18px;border-left:1px solid var(--line)}
.band .mode-word{margin-right:16px;justify-self:start}
.band dt{font-size:12.5px;font-weight:500;color:var(--ink-3);margin-bottom:2px}
.band dd{margin:0;font-size:21px;font-weight:700;letter-spacing:-.02em;line-height:1.25;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.band .mode-note{display:none}   /* 모드 출처 설명은 띠에 마우스를 올리면(title) */
/* 서버 기준 시각은 다른 값처럼 「작은 라벨 + 큰 값」으로 사실 줄 바로 뒤에 둔다(오른쪽 끝으로 밀지 않는다). */
#built{display:flex;flex-direction:column;font-size:12.5px;font-weight:500;color:var(--ink-3);line-height:1.25}
#built>span{display:block;margin-top:2px;font-size:21px;font-weight:700;letter-spacing:-.02em;color:var(--ink);white-space:nowrap}
.band.normal .mode-word{background:var(--safe-tint);color:var(--safe)}
.band.disaster{background:#fff4f4;box-shadow:var(--shadow),inset 0 0 0 1px #ffd7d8}
.band.disaster .facts>div,.band.disaster #built{border-left-color:#ffdcdd}
.band.disaster .mode-word{background:var(--signal);color:#fff}
.band.unknown .mode-word{color:var(--pending)}
.band.clearing .mode-word{background:var(--caution-tint);color:var(--caution)}
.band.mission_finished .mode-word{background:var(--safe-tint);color:var(--safe)}
.band .reason{font-size:15px;font-weight:600}
/* 좁은 화면(폰·세로 태블릿): 지표를 같은 폭 칸으로 줄 바꿔 채우고 모드 배지는 한 줄을 다 쓴다. 위 넓은 화면 규칙보다
   뒤에 있어야 이긴다 — 앞에 두었더니 폰에서 일곱 칸이 한 줄에 끼어 글자가 세로로 섰다(2026-10-15). */
@media (max-width:1399px){
  .band{grid-auto-flow:row;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:12px 0;padding:14px}
  .band .mode-word{grid-column:1/-1;margin:0 0 2px}
  .band dd{white-space:normal;overflow-wrap:anywhere}
}
@media (max-width:600px){
  #masthead{flex-wrap:wrap;height:auto;row-gap:6px;padding:10px 16px}
}
#topbar{display:grid;grid-template-columns:auto 1fr;align-content:center;align-items:center;gap:8px 10px;padding:12px 18px;
  background:var(--card);border-radius:var(--r);box-shadow:var(--shadow)}
#topbar .who{grid-column:2;grid-row:1;justify-self:end;display:flex;gap:12px;align-items:center;flex-wrap:wrap;font-size:13px;color:var(--ink-2)}
#topbar .kpi{grid-column:1/-1;grid-row:2}
#poll{font-size:13.5px;display:flex;gap:8px;align-items:center;font-weight:600}
#poll::before{content:"";width:8px;height:8px;border-radius:50%;background:var(--safe);box-shadow:0 0 0 4px var(--safe-tint)}
#poll.lost{color:var(--signal)} #poll.lost::before{background:var(--signal);box-shadow:0 0 0 4px var(--signal-tint)}
#poll.wait::before{background:var(--pending);box-shadow:none}
/* KPI-04 요약 문구는 화면에서 숨긴다(스크립트는 계속 계산하고 「지연 기록 저장」으로 받는다, 2026-10-15 사용자 요청).
   둘째 줄은 보조 버튼 둘을 오른쪽 정렬로 둔다. */
#topbar .kpi{display:flex;gap:8px;align-items:center;justify-content:flex-end;font-size:12px;color:var(--ink-3);flex-wrap:wrap}
#kpi{display:none}
#show-history+.history-toggle{order:-1}
#kpi.ok{color:var(--safe)} #kpi.over{color:var(--signal);font-weight:600}
.mini{font:inherit;font-size:12.5px;font-weight:600;padding:4px 12px;border:0;background:var(--raise);color:var(--ink-2);
  border-radius:var(--pill);cursor:pointer;transition:background-color .15s}
.mini:hover{background:var(--line-2)}

/* ---- 확인할 것 ---- */
.reason.alert{color:var(--signal);font-weight:600} .reason.caution{color:#946200;background:var(--caution-tint);padding:1px 8px;border-radius:6px}
.scoped-reason{margin:6px 0 0;font-weight:600}
.cmd-auto{margin:0;padding:12px 14px;border-radius:var(--r-in);background:var(--info-tint);color:var(--info);font-weight:600}
#warnings{background:var(--caution-tint);border-radius:var(--r);padding:11px 18px;font-size:13.5px;color:#7a5200;box-shadow:inset 0 0 0 1px #fde7b0}
#warnings.empty{display:none}
#warnings h2{display:inline;margin-right:12px;font-size:13.5px;color:#7a5200}
#warnings ul{display:inline;margin:0;padding:0;list-style:none}
#warnings li{display:inline}
#warnings li+li::before{content:"·";margin:0 10px}

/* ---- 지도 ---- */
#map{display:flex;flex-direction:column;min-height:0}
#map header{display:flex;align-items:center;gap:16px;margin-bottom:12px}
#map header h2{margin:0}
.legend{display:flex;flex-wrap:wrap;gap:6px;margin:0;padding:0;list-style:none;font-size:12.5px;color:var(--ink-2)}
.legend li{display:flex;align-items:center;gap:5px;padding:3px 10px;background:var(--soft);border-radius:var(--pill)}
.plan{flex:1;min-height:0;width:100%;background:var(--map-bg);border-radius:var(--r-in)}
.plan image{image-rendering:pixelated;filter:url(#map-tone)}
.plan .frame{fill:none;stroke:var(--line-2);stroke-width:.25}
.plan .grid{stroke:var(--ink);stroke-width:.06;opacity:.06}
.plan .axis{stroke:var(--accent);stroke-width:.14;opacity:.4}
.plan .tick{font-size:2.1px;fill:var(--ink-3)}
.plan .scale line{stroke:var(--ink-2);stroke-width:.5} .plan .scale text{font-size:2.3px;fill:var(--ink-2)}
.plan .label{font-size:2.6px;font-weight:700;paint-order:stroke;stroke:#fff;stroke-width:.7}
.victim .mark,.victim .strike{fill:none;stroke-width:.85;stroke-linecap:round;stroke-linejoin:round}
.victim.unconfirmed .mark{stroke:var(--pending);stroke-dasharray:.9 .6} .victim.unconfirmed .label{fill:var(--pending)}
.victim.not_rescued .mark{stroke:var(--rescue)} .victim.not_rescued .label{fill:var(--rescue)}
.victim.supporting .mark{stroke:var(--info)} .victim.supporting .label{fill:var(--info)}
.victim.rescued .mark,.victim.rescued .strike{stroke:var(--safe)} .victim.rescued .label{fill:var(--safe)}
.robot .heading,.robot .body{fill:var(--ink);stroke:#fff;stroke-width:.35}
.exit .mark{fill:none;stroke:var(--safe);stroke-width:.75} .exit .label{fill:var(--safe)}
.passage .mark{fill:none;stroke-width:.9;stroke-linejoin:round} .passage.old{opacity:.35}
.passage.passable .mark{stroke:var(--safe)} .passage.passable .label{fill:var(--safe)}
.passage.blocked .mark{stroke:var(--signal)} .passage.blocked .label{fill:var(--signal)}
.passage.unknown .mark{stroke:var(--pending)} .passage.unknown .label{fill:var(--pending)}
.passage.approvable{cursor:pointer} .passage.approvable .mark{stroke-width:1.3}
.robot.support .heading,.robot.support .body{fill:var(--info)}
.robot .label{fill:var(--ink)} .robot.support .label{fill:var(--info)}
.robot.estimated{opacity:.55} .robot.estimated .heading,.robot.estimated .body{fill:none;stroke:var(--ink);stroke-width:.5;stroke-dasharray:1 .7}
/* 지도 설명 줄은 평소 숨기고(마우스를 올리면 지도 정보는 title), 실시간 지도가 낡았을 때만 빨간 경고로 보인다. */
.map-note{display:none;font-size:12.5px;margin-top:10px}
#map.stale .plan{opacity:.45} #map.stale .map-note{display:block;color:var(--signal);font-weight:600}
.unplotted{margin:10px 0 0;padding:10px 14px;list-style:none;background:var(--caution-tint);border-radius:var(--r-in);font-size:13px;color:#7a5200}
.unplotted .head{font-weight:700}
#map.placeholder p{margin-top:40px;text-align:center;color:var(--ink-3)}

/* ---- 작은 V 표식 ---- */
.vicon{width:16px;height:16px;flex:none;vertical-align:-3px}
.vicon path,.vicon line{fill:none;stroke-width:2.2;stroke-linecap:round;stroke-linejoin:round}
.vicon.unconfirmed path{stroke:var(--pending);stroke-dasharray:2.2 1.6}
.vicon.not_rescued path{stroke:var(--rescue)} .vicon.supporting path{stroke:var(--info)}
.vicon.rescued path,.vicon.rescued line{stroke:var(--safe)}
.vicon.robot polygon{fill:var(--ink)}

/* ---- 로봇 ---- */
.unit{background:var(--soft);border-radius:12px;padding:12px 14px}
.unit+.unit{margin-top:10px}
.unit h3{display:flex;align-items:center;gap:8px}
.unit-name{font-size:18px;font-weight:800;letter-spacing:-.02em}
.unit-role{font-size:12px;font-weight:700;padding:1px 9px;border-radius:var(--pill);background:#fff;color:var(--ink-2);box-shadow:inset 0 0 0 1px var(--line-2)}
.unit.support .unit-role{background:var(--info-tint);color:var(--info);box-shadow:none}
.unit-id{color:var(--ink-3);font-size:12px;margin-left:auto}
.unit .state{display:flex;gap:6px;margin:9px 0 10px;font-size:12.5px}
.unit .state span{padding:2px 10px;border-radius:var(--pill);background:#fff;color:var(--ink-2);font-weight:600;box-shadow:inset 0 0 0 1px var(--line-2)}
.unit .state .motion{background:var(--ink);color:#fff;box-shadow:none}
.unit .state .motion.stop_unconfirmed{background:var(--signal);color:#fff}
.battery{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.battery .bar{flex:1;height:8px;background:#e3e9f0;border-radius:4px;overflow:hidden}
.battery .bar span{display:block;height:100%;background:var(--safe);border-radius:4px}
.battery.warn .bar span{background:var(--caution)} .battery.low .bar span{background:var(--signal)}
.battery.low .value{color:var(--signal)}
.battery .value{min-width:3em;text-align:right;font-size:16px;font-weight:700} .battery .age{font-size:12px;color:var(--ink-3)}
.battery.none .value{color:var(--ink-3);text-align:left;font-size:13px;font-weight:500}
.unit dl{display:grid;grid-template-columns:auto auto auto 1fr;column-gap:10px;row-gap:3px;font-size:12.5px}
.unit dl div{display:contents} .unit dt{color:var(--ink-3)} .unit dd{margin:0;color:var(--ink-2)}
.unit .alert{margin-top:10px;padding:7px 11px;background:var(--caution-tint);border-radius:8px;font-size:12.5px;color:#7a5200}
.unit.stale,.unit.missing{box-shadow:inset 3px 0 0 var(--caution)}
.unit.stale .state,.unit.stale .battery,.unit.stale dl{opacity:.55}

/* ---- 요구조자: 색 타일 + 큰 숫자 ---- */
#targets{display:flex;flex-direction:column;min-height:240px;flex:1}
.stage-counts{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin:0 0 12px;padding:0;list-style:none}
.stage-counts li{display:grid;grid-template-columns:auto 1fr;grid-template-areas:"icon label" "num num";column-gap:5px;align-items:center;
  padding:9px 11px 10px;border-radius:12px;background:var(--pending-tint);font-size:12px;font-weight:600;color:var(--ink-2)}
.stage-counts li .vicon{grid-area:icon;width:13px;height:13px}
.stage-counts li span{grid-area:label;white-space:nowrap}
.stage-counts li b{grid-area:num;font-size:26px;line-height:1.15;font-weight:800;letter-spacing:-.02em;color:var(--ink)}
.stage-counts .not_rescued{background:var(--rescue-tint)} .stage-counts .not_rescued b{color:var(--rescue)}
.stage-counts .supporting{background:var(--info-tint)} .stage-counts .supporting b{color:var(--info)}
.stage-counts .rescued{background:var(--safe-tint)} .stage-counts .rescued b{color:var(--safe)}
.victims{margin:0;padding:0;list-style:none;overflow:auto;min-height:0}
.victim-row{display:grid;grid-template-columns:22px 1fr;column-gap:8px;padding:9px 8px;border-radius:10px}
.victim-row:hover{background:var(--soft)}
.victim-row .vicon{grid-row:span 3;margin-top:3px}
.victim-row .who{display:flex;gap:8px;align-items:center;font-weight:700}
.victim-row .stage-word{font-size:12px;font-weight:700;padding:1px 9px;border-radius:var(--pill);background:var(--pending-tint);color:var(--ink-2)}
.victim-row .rule{font-size:11.5px;color:var(--ink-3)}
.victim-row.not_rescued .stage-word{background:var(--rescue-tint);color:var(--rescue)}
.victim-row.supporting .stage-word{background:var(--info-tint);color:var(--info)}
.victim-row.rescued .stage-word{background:var(--safe-tint);color:var(--safe)}
.victim-row .what{display:flex;flex-wrap:wrap;column-gap:12px;font-size:12.5px;color:var(--ink-3);margin-top:2px}
.victim-row .support{color:var(--info);font-weight:600}
.victim-row .candidate{color:#7a5200;background:var(--caution-tint);border-radius:6px;padding:0 7px;font-weight:600}
.victim-row .notes{grid-column:2;margin:6px 0 0;padding:6px 10px;list-style:none;background:var(--caution-tint);border-radius:8px;font-size:12.5px;color:#7a5200}
.victims .none{padding:16px 0;color:var(--ink-3)}

/* ---- 사건 기록 ---- */
#events{display:flex;flex-direction:column;min-height:0}
#events .scroll{overflow:auto;min-height:0;flex:1}
#events table{border-collapse:separate;border-spacing:0;width:100%;font-size:13px}
#events th{position:sticky;top:0;background:#fff;text-align:left;font-weight:600;font-size:12px;color:var(--ink-3);
  padding:0 10px 8px;white-space:nowrap;border-bottom:1px solid var(--line)}
#events td{padding:8px 10px;vertical-align:top;color:var(--ink-2);border-bottom:1px solid var(--line)}
#events tbody tr:last-child td{border-bottom:0}
#events tbody tr:hover td{background:var(--soft)}
/* 화면 표시·표시 지연 열(KPI-04)은 숨긴다. 스크립트는 그대로 채우고 「지연 기록 저장」으로 받을 수 있다. */
#events th:nth-child(7),#events td:nth-child(7),#events th:nth-child(8),#events td:nth-child(8){display:none}
#events td.t{white-space:nowrap;color:var(--ink-3)}
#events td.kind{white-space:nowrap;font-weight:700;color:var(--ink)}
#events td.kind::before{content:"";display:inline-block;width:7px;height:7px;margin-right:8px;border-radius:50%;background:var(--pending);vertical-align:1px}
#events tr.report td.kind::before{background:var(--accent)}
#events tr.disaster_sign td.kind{color:var(--signal)} #events tr.disaster_sign td.kind::before{background:var(--signal)}
#events tr.target_disappeared_candidate td.kind{color:var(--safe)}
#events tr.target_disappeared_candidate td.kind::before,#events tr.exit_pass td.kind::before{background:var(--safe)}
#events tr.fresh td{background:var(--caution-tint)}
/* ---- 명령·오류 기록: 사건 기록 옆, 요구조자 아래. 안에서 스크롤 ---- */
#history{min-height:0;overflow:auto}
#history table{border-collapse:separate;border-spacing:0;width:100%;font-size:13px}
#history th{position:sticky;top:0;background:#fff;text-align:left;font-weight:600;font-size:12px;color:var(--ink-3);padding:0 8px 8px;border-bottom:1px solid var(--line)}
#history td{padding:8px;vertical-align:top;color:var(--ink-2);border-bottom:1px solid var(--line)}
#history td.t,#history td:nth-child(2){white-space:nowrap}
#history tr.refused .outcome,#history tr.error .outcome{color:var(--signal);font-weight:700}
#history tr.accepted .outcome{color:var(--safe);font-weight:600}
#history .missing,#history.empty p{color:var(--ink-3);font-size:13px}

/* ---- 옆 칸: 웹캠과 명령 ---- */
#cameras figure,#robot-camera figure{margin:0;position:relative;border-radius:var(--r-in);overflow:hidden;background:#0b1220}
#cameras img,#robot-camera img{display:block;width:100%;aspect-ratio:16/9;object-fit:contain;background:#0b1220;color:transparent}
#cameras .nosignal,#robot-camera .nosignal{display:none;position:absolute;left:0;right:0;top:0;aspect-ratio:16/9;align-items:center;
  justify-content:center;text-align:center;padding:0 24px;color:#cbd5e1;font-size:13.5px}
#cameras figure.lost .nosignal,#robot-camera figure.lost .nosignal{display:flex}
#cameras figure.lost img,#robot-camera figure.lost img{visibility:hidden}
/* 영상 이름표는 영상 왼쪽 아래 작은 배지(판정 화면 위쪽 상태창을 가리지 않게) */
#cameras figcaption,#robot-camera figcaption{position:absolute;left:10px;bottom:10px;display:flex;gap:8px;align-items:center;
  max-width:calc(100% - 20px);font-size:12px;font-weight:600;padding:2px 10px;background:rgba(11,18,32,.72);border-radius:var(--pill);color:#e2e8f0}
#cameras figcaption span:empty,#robot-camera figcaption span:empty{display:none}
#cameras figcaption .lost,#robot-camera figcaption .lost{color:#ff9b9e;font-weight:700}
.camera-empty p{color:var(--ink-3);font-size:13px}
.webcam-pick,.robot-cam-pick{display:flex;flex-wrap:wrap;align-items:center;gap:8px;margin-top:10px;font-size:13px;color:var(--ink-2)}
.webcam-pick button{font:inherit;font-size:12.5px;font-weight:600;padding:5px 12px;border:0;background:var(--raise);
  color:var(--ink-2);border-radius:var(--pill);cursor:pointer}
.webcam-pick button:hover{background:var(--line-2)}
.webcam-notice{width:100%;color:var(--ink-3)}
#side .alert{color:#a16207;font-size:12.5px;margin-top:8px}
#side>#commands{overflow:auto;min-height:0;flex:1 1 auto}
#commands header{display:flex;align-items:baseline;gap:10px;margin-bottom:12px}
#commands header h2{margin:0}
.cmd-access{font-size:12.5px;color:var(--ink-3)} .cmd-access.on{display:none} .cmd-access.off{color:var(--signal);font-weight:600}
.cmd-group{display:grid;gap:8px;margin-bottom:10px}
.cmd-group.approve{grid-template-columns:repeat(auto-fit,minmax(130px,1fr))}
.cmd-group.mission{grid-template-columns:repeat(3,1fr)}
.cmd{font:inherit;border:0;background:#fff;color:var(--ink);padding:10px 12px;border-radius:var(--r-in);box-shadow:inset 0 0 0 1px var(--line-2);
  cursor:pointer;display:flex;flex-direction:column;align-items:flex-start;gap:1px;text-align:left;
  transition:background-color .15s,box-shadow .15s,transform .08s}
.cmd:not(:disabled):hover{box-shadow:inset 0 0 0 1px var(--accent),0 2px 6px rgba(79,70,229,.12)}
.cmd:not(:disabled):active{transform:scale(.985)}
.cmd .cmd-label{font-weight:700}
.cmd .why{font-size:12px;line-height:1.35;color:var(--ink-3)}
.cmd.approve{padding:11px 13px;background:var(--blue);color:#fff;box-shadow:0 1px 2px rgba(37,99,235,.3)}
.cmd.approve:not(:disabled):hover{background:var(--blue-hover);box-shadow:0 4px 12px rgba(37,99,235,.3)}
.cmd.approve .cmd-label{font-size:15px} .cmd.approve .why{font-size:12px;color:rgba(255,255,255,.85)}
.cmd.stop{width:100%;padding:12px 16px;background:var(--signal);color:#fff;box-shadow:0 1px 2px rgba(229,72,77,.3)}
.cmd.stop:not(:disabled):hover{background:var(--signal-strong);box-shadow:0 4px 12px rgba(229,72,77,.3)}
.cmd.stop .cmd-label{font-size:15px} .cmd.stop .why{color:rgba(255,255,255,.88)}
/* 비활성화된 버튼도 가독성을 유지한다: 버튼 레이블 하단의 비활성화 사유 안내가 핵심 정보다. */
.cmd:disabled,.cmd.approve:disabled,.cmd.stop:disabled{cursor:not-allowed;background:var(--soft);color:#a3afbf;box-shadow:none}
.cmd:disabled .why,.cmd.approve:disabled .why,.cmd.stop:disabled .why{color:#a3afbf}
/* 2단계 승인 대화상자가 활성화되어 있는 동안 선택된 명령 버튼. */
.cmd.armed:disabled{background:var(--info-tint);box-shadow:inset 0 0 0 2px var(--info);color:var(--info)}
#cmd-confirm{border-radius:var(--r-in);background:var(--info-tint);padding:12px 14px;margin-bottom:10px;box-shadow:inset 0 0 0 1px #c9dafc}
#cmd-confirm .what{font-size:14.5px;font-weight:700;margin-bottom:8px}
#cmd-confirm label{display:flex;gap:8px;align-items:center;margin-bottom:8px;font-size:13px;color:var(--ink-2)}
#cmd-confirm label[hidden]{display:none}
#cmd-confirm select{font:inherit;flex:1}
#cmd-confirm .args{display:grid;grid-template-columns:auto 1fr;column-gap:12px;row-gap:2px;font-size:13px;margin-bottom:10px}
#cmd-confirm .args dt{color:var(--ink-3)} #cmd-confirm .args dd{margin:0}
#cmd-confirm .row{display:flex;gap:8px}
#cmd-confirm .row .cmd{align-items:center;flex:1}
.cmd.send{background:var(--blue);color:#fff;box-shadow:none}
.cmd.send:not(:disabled):hover{background:var(--blue-hover)}
#cmd-log{list-style:none;margin:0 0 10px;padding:0;font-size:12.5px}
#cmd-log li{padding:7px 12px;background:var(--soft);border-radius:8px;margin-bottom:4px;box-shadow:inset 3px 0 0 var(--pending)}
#cmd-log li time{color:var(--ink-3);margin-right:8px}
#cmd-log li.accepted{box-shadow:inset 3px 0 0 var(--safe)} #cmd-log li.rejected{box-shadow:inset 3px 0 0 var(--caution)}
#cmd-log li.failed{box-shadow:inset 3px 0 0 var(--signal);font-weight:600}
.estop{font-size:12.5px;font-weight:600;padding:9px 12px;background:var(--signal-tint);border-radius:var(--r-in);color:#b4232a}

/* ---- 폼(로그인, 계정) ---- */
.card{max-width:400px;margin:14vh auto;padding:36px 34px 34px;background:#fff;border-radius:18px;box-shadow:0 10px 30px rgba(15,23,42,.08)}
.card .sys{display:flex;align-items:center;gap:10px;font-size:13px;font-weight:600;color:var(--ink-3);margin-bottom:10px}
.card .sys::before{content:"";width:22px;height:22px;border-radius:7px;flex:none;background:var(--accent);
  box-shadow:inset 0 0 0 6px var(--accent),inset 0 0 0 9px #fff}
.card h1{font-size:25px;font-weight:800;letter-spacing:-.03em;margin-bottom:20px}
.failure{box-shadow:0 10px 30px rgba(15,23,42,.08),inset 4px 0 0 var(--signal)} .failure p{margin-top:12px}
.failure .detail{font-size:13px;color:var(--ink-3);word-break:break-all} .failure .retry{font-size:13px}
.card label{display:block;margin:14px 0 6px;font-size:13px;font-weight:600;color:var(--ink-2)}
.card input{width:100%;padding:12px 13px;font-size:14.5px;background:var(--soft);border-color:transparent}
.card input:focus{background:#fff}
.card button,form.inline button{font:inherit;font-weight:700;border:0;background:var(--accent);color:#fff;border-radius:var(--r-in);cursor:pointer;
  transition:background-color .15s}
.card button{width:100%;margin-top:24px;padding:13px 16px;font-size:15px}
form.inline button{padding:6px 14px}
.card button:hover,form.inline button:hover{background:#4338ca}
#topbar form.inline button,.admin-head form.inline button{padding:4px 12px;font-size:12.5px;font-weight:600;background:var(--raise);color:var(--ink-2);border-radius:var(--pill)}
#topbar form.inline button:hover,.admin-head form.inline button:hover{background:var(--line-2)}
.flash{padding:10px 14px;margin:10px 0;background:var(--soft);border-radius:var(--r-in);box-shadow:inset 3px 0 0 var(--ink-3)}
.flash.danger{box-shadow:inset 3px 0 0 var(--signal)}.flash.success{box-shadow:inset 3px 0 0 var(--safe)}.flash.info{box-shadow:inset 3px 0 0 var(--info)}
form.inline{display:inline}form.inline input{width:9em;padding:6px 9px}form.inline select{margin-right:4px}
.admin{max-width:1200px;margin:0 auto;padding:24px 20px}
.admin-head{display:flex;align-items:center;gap:14px;padding:4px 0 18px;margin-bottom:8px;border-bottom:1px solid var(--line)}
.admin-head h1{font-size:24px;font-weight:800;letter-spacing:-.03em}
.admin-head .sys{font-size:13px;color:var(--ink-3)}
.admin-head .who{margin-left:auto;display:flex;gap:12px;align-items:center;font-size:13px;color:var(--ink-2)}
.admin h2{margin-top:20px}
.admin table{border-collapse:separate;border-spacing:0;width:100%;background:#fff;margin-top:12px;border-radius:var(--r);box-shadow:var(--shadow);overflow:hidden}
.admin caption{text-align:left;color:var(--ink-3);padding:4px 0 8px}
.admin th,.admin td{padding:10px 14px;text-align:left;border-bottom:1px solid var(--line)}
.admin th{font-weight:600;font-size:12.5px;color:var(--ink-3)}
"""

# 사건 행마다 브라우저 최초 렌더링 시각을 기록하고(KPI-04), 명령 패널 동작을 관리한다.
# 프래그먼트는 GET 방식으로 주기적 폴링하며, 유일한 POST 요청은 /commands 경로의 운영자 명령이다.
_SCRIPT = """
// 페이지 상태: firstSeen은 사건 ref_id를 브라우저 최초 표시 시각과 매핑한다. window.kpi04는
// 사건별 표시 지연 기록을 누적한다(saveKpi를 통해 CSV 다운로드 가능). loaded는 첫 stamp() 완료 후 true가 된다.
// lastOk는 최근 폴링 성공 시각, FRESH_MS는 신규 수신 행의 강조 표시 유지 시간(ms)이다.
const firstSeen = new Map();
window.kpi04 = [];
const FRESH_MS = 3000;
let lastOk = Date.now(), loaded = false;
// render.py의 _time / _age 함수와 포맷을 일치시켜 서버 렌더링 값과 브라우저 계산 값이 동일하게 표시된다.
function clock(ms) {
  const d = new Date(ms), pad = (n, w) => String(n).padStart(w || 2, '0');
  return pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds()) + '.' + pad(d.getMilliseconds(), 3);
}
// render.py의 _age와 동일한 경과 시간 포맷. 단, 1분 미만 구간은 소수점 둘째 자리까지 표기한다.
function age(s) {
  const sign = s < 0 ? '-' : '', v = Math.abs(s);
  if (v < 60) return sign + v.toFixed(2) + 's';
  if (v < 3600) return sign + Math.floor(v / 60) + '분 ' + String(Math.floor(v % 60)).padStart(2, '0') + '초';
  if (v < 86400) return sign + Math.floor(v / 3600) + '시간 ' + Math.floor(v % 3600 / 60) + '분';
  return sign + Math.floor(v / 86400) + '일';
}
// 모든 사건 행의 화면 표시 시각 및 표시 지연(KPI-04) 셀을 계산하여 갱신한다.
// 신규 수신된 ref_id는 firstSeen에 브라우저 현재 시각을 등록하고 KPI-04 지연 샘플로 기록한다.
// 사건 표시 지연은 행이 화면에 처음 나타난 시점에 1회 측정된다. 측정값은 브라우저 시계에서 사건 발생 시각을
// 뺀 값이므로, 두 호스트 간 시계 동기화가 보장된 환경에서만 유효하다(계약 C-03).
function stamp() {
  const now = Date.now();
  for (const row of document.querySelectorAll('tr.event')) {
    const ref = row.dataset.ref, eventMs = Number(row.dataset.eventNs) / 1e6;
    if (!firstSeen.has(ref)) {
      firstSeen.set(ref, now);
      // live: 페이지가 로드되어 열려 있는 동안 실시간 수신된 행. 초기 로드 시점의 기존 행은
      // 표시 지연 측정이 아닌 과거 이력이므로 KPI-04 실시간 지연 요약 통계에서 제외한다.
      if (row.dataset.kpi !== '0') window.kpi04.push({ref_id: ref, kind: row.dataset.kind, clock_domain_id: row.dataset.clock,
                         event_ns: row.dataset.eventNs, received_at_ns: row.dataset.receivedNs || '',
                         rendered_ms: now, latency_s: (now - eventMs) / 1000, live: loaded});
    }
    const shown = firstSeen.get(ref);
    row.dataset.renderedMs = shown;
    row.querySelector('.rendered').textContent = clock(shown);
    row.querySelector('.latency').textContent = age((shown - eventMs) / 1000);
    // 페이지 접속 후 새로 도착한 행을 일정 시간 동안 시각적으로 강조한다.
    row.classList.toggle('fresh', loaded && now - shown < FRESH_MS);
  }
  loaded = true;
  kpiSummary();
}
// 상단 KPI 통계 요약: 실시간(live) 수신 사건만을 대상으로 KPI-04 예산 준수 건수 및 최대 지연 시간을 표시한다.
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
// CSV 필드 이스케이프: 따옴표, 쉼표, 개행 문자가 포함된 경우 큰따옴표로 감싸고 내부 따옴표를 이스케이프한다.
function csvCell(value) {
  const text = String(value);
  return /[",\\n]/.test(text) ? '"' + text.replace(/"/g, '""') + '"' : text;
}
// 지연 측정 기록을 브라우저 로컬에서 CSV Blob으로 다운로드한다. 서버로의 추가 네트워크 요청은 발생하지 않는다.
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
// 브라우저 탭 타이틀에 핵심 상태를 반영하여 백그라운드 탭에서도 재난 선포 및 통신 두절을 식별할 수 있도록 한다.
function title(lost) {
  const band = document.getElementById('mission');
  const tag = lost ? '[갱신 끊김] ' : band && band.dataset.mode === 'DISASTER' ? '[재난] ' : '';
  document.title = tag + '구조 관제';
}
// 폴링 통신 결과를 탭 타이틀 및 상태 표시줄(#poll)에 반영한다. 연결이 두절된 경우
// 모든 명령 버튼이 비활성화되도록 게이트 상태를 즉시 재적용한다. lastOk를 갱신하지 않아 데이터의 지연 시간이 표시된다.
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
// 단일 폴링 주기 처리: fragment 엔드포인트를 GET 조회하여 #live 영역을 교체하고
// 사건 행 타임스탬프 계산 및 게이트 상태를 갱신한다. 401 또는 리다이렉트 응답 시에는
// 폴링을 중단하고 로그인 페이지로 전환한다(AST-50-02). 그 외 일시적 통신 장애 시에는 기존 화면을 유지한 채 경고를 표시한다.
async function poll() {
  try {
    const response = await fetch('fragment', {cache: 'no-store'});
    if (response.status === 401 || response.redirected) {
      // 인증 세션 만료(외부 로그아웃, 계정 삭제 등): 응답 본문을 DOM에 삽입하지 않고
      // 폴링 타이머를 해제한 뒤 로그인 페이지로 안전하게 이동한다(AST-50-02).
      clearInterval(timer);
      location.href = response.status === 401 ? '/login' : response.url;
      return;
    }
    if (!response.ok) {
      // 서버에서 전달된 오류 사유 텍스트(render.failure_text)를 포함하여 상태를 표시한다.
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

// ---- 운영자 명령 패널 ----
// 명령 게이트 조건은 각 프래그먼트 응답(#gates)에 동기화된다. 버튼이 활성화되려면 사용자 권한(ADMIN)과
// 도메인 게이트 조건이 모두 충족되어야 하며, 화면 연결이 끊기면 즉시 비활성화된다.
// 버튼 클릭 시 2단계 확인 대화상자가 열리며, 사용자의 명시적 승인 시에만 단 1회의 POST 요청이 전송된다(자동 재시도 배제).
const panel = document.getElementById('commands');
const allowed = panel.dataset.allowed === '1';
const confirmBox = document.getElementById('cmd-confirm'), pick = document.getElementById('cmd-pick');
const sendButton = document.getElementById('cmd-send'), cancelButton = document.getElementById('cmd-cancel');
const LOG_ROWS = 6;
// gates: 최신 게이트 상태 미러. armed: 확인 대화상자가 열린 명령 객체({kind, name, args JSON 문자열, choices}).
// sending: 비동기 POST 전송 진행 중 플래그. screenLost: 최근 폴링 통신 실패 상태 플래그.
let gates = {}, armed = null, sending = false, screenLost = false;
// 프래그먼트의 hidden #gates 엘리먼트에서 게이트 JSON을 파싱한다.
// 요소가 없거나 파싱 실패 시에는 기존 상태를 보존하지 않고 모든 명령을 안전하게 비활성화한다.
function readGates() {
  const el = document.getElementById('gates');
  try { gates = el ? JSON.parse(el.dataset.gates) : {}; } catch (error) { gates = {}; }
}
// 명령 종류의 UI 레이블 반환(확인 대화상자 및 결과 로그에 표시). 매핑이 없으면 식별 코드 그대로 표기.
function nameOf(kind) {
  const button = panel.querySelector('button[data-command="' + kind + '"] .cmd-label');
  return button ? button.textContent : kind;
}
// render.py의 _short 함수와 동일한 식별자 단축 규칙.
function shortId(id) { return String(id).length > 9 ? String(id).slice(0, 8) : String(id); }
// 확인 대화상자에서 선택된 대상 반환. 선택지가 없는 명령의 경우 null.
function chosen() { return armed && armed.choices.length ? armed.choices[pick.selectedIndex] : null; }
// 현재 열려 있는 확인 대화상자의 파라미터가 최신 게이트 상태와 유효하게 일치하는지 검증한다.
// 게이트가 활성 상태이고 파라미터가 동일하며 대상이 선택지에 남아 있어야 한다. 불일치 시 취소 처리된다.
function sameArmed(gate) {
  if (!gate || !gate.enabled || JSON.stringify(gate.args) !== armed.args) return false;
  const choice = chosen();
  return !choice || gate.choices.some(c => JSON.stringify(c) === JSON.stringify(choice));
}
// 모든 명령 버튼의 활성화/비활성화 상태 및 안내 사유를 게이트 조건에 맞춰 갱신한다.
// 사용자 권한, 게이트 조건, 화면 연결 상태가 모두 정상이고 전송 중이 아닐 때만 활성화된다.
function applyGates() {
  for (const button of panel.querySelectorAll('button[data-command]')) {
    const gate = gates[button.dataset.command] || {enabled: false, reason: '관문 정보 없음'};
    // 통신 두절 중에는 게이트 정보가 지연될 수 있으므로, 비정상 화면에서의 오작동을 방지하기 위해 명령을 차단한다.
    const reason = screenLost ? '갱신 끊김 — 화면이 최신이 아니라 꺼 둠' : gate.reason;
    button.disabled = !allowed || !gate.enabled || screenLost || sending || armed !== null;
    button.classList.toggle('armed', armed !== null && armed.kind === button.dataset.command);
    button.querySelector('.why').textContent = reason;
    button.title = reason;
  }
  // 확인 대화상자가 열린 상태에서 게이트 조건이 변경되거나 화면이 stale해지면 안전하게 취소한다.
  // 단, 이미 HTTP 요청이 전송된 상태에서는 취소하지 않는다(요청 취소가 불가능하기 때문).
  if (armed && !sending && (screenLost || !sameArmed(gates[armed.kind]))) {
    disarm('상태가 바뀌어 ' + armed.name + ' 확인을 취소했습니다. 다시 누르세요.');
  }
}
// 확인 대화상자에 명령과 함께 전달될 인자 및 대상 정보를 나열한다.
// 관문 args별로 한국어 필드명을 매핑하며, 대상 선택 변경 시 재호출된다.
// textContent를 통해 안전하게 DOM에 삽입하여 XSS 위험을 원천 방지한다.
function showArgs() {
  const rows = [], args = JSON.parse(armed.args), choice = chosen();
  const labels = {target_id: '대상', target_revision: '대상 revision', route_revision: '경로 revision',
                  segment_index: '승인할 구간'};
  if (choice && armed.kind === 'DISPATCH_SUPPORT') rows.push(['대상 revision', choice.target_revision]);
  if (choice && choice.item_id) rows.push(['터널 항목', choice.item_id], ['승인한 사람', '로그인한 계정 (서버가 붙임)']);
  for (const key of Object.keys(args)) {
    // segment_index는 완료된 구간 수이자 다음에 승인할 구간의 0-based 인덱스이다.
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
// 1단계 클릭: 해당 명령의 2단계 승인 대화상자를 열고 모든 명령 버튼을 잠근다.
// 현재 게이트의 args 및 choices를 armed 객체에 복사하여 후속 폴링에 의한 파라미터 변조를 방지한다.
function arm(kind) {
  const gate = gates[kind];
  if (!allowed || screenLost || sending || !gate || !gate.enabled) return;
  armed = {kind: kind, name: nameOf(kind), args: JSON.stringify(gate.args), choices: gate.choices};
  confirmBox.querySelector('.what').textContent = gate.confirm || (armed.name + ' — 아래 내용으로 보냅니다');
  pick.replaceChildren(...gate.choices.map(c => {
    const option = document.createElement('option');
    option.textContent = c.item_id ? c.label : shortId(c.target_id) + ' ' + c.stage_label;
    option.title = c.item_id || c.target_id;
    return option;
  }));
  document.getElementById('cmd-pick-row').hidden = !gate.choices.length;
  showArgs();
  confirmBox.hidden = false;
  applyGates();
  sendButton.focus();
}
// 확인 대화상자를 닫고 버튼 상태를 원복한다. 안내 메시지가 제공된 경우 결과 로그에 기록한다.
function disarm(message) {
  armed = null;
  confirmBox.hidden = true;
  sendButton.disabled = cancelButton.disabled = false;
  sendButton.textContent = '보내기';
  if (message) log('info', message);
  applyGates();
}
// 페이지 내 명령 결과 로그에 타임스탬프가 포함된 엔트리를 추가하고 최신 LOG_ROWS건만 유지한다.
// 현재 세션에서 전송한 명령 결과만 표기되며, 전체 내역은 하단 명령/오류 이력 테이블에서 확인한다.
function log(kind, text) {
  const item = document.createElement('li'), time = document.createElement('time');
  item.className = kind;
  time.textContent = clock(Date.now()).slice(0, 8);
  item.append(time, text);
  const list = document.getElementById('cmd-log');
  list.prepend(item);
  while (list.children.length > LOG_ROWS) list.lastChild.remove();
}
// 콘솔 서버 응답 객체를 로그 표기용 [CSS클래스, 메시지문자열]로 변환한다.
// sent === true: operations 서비스가 정상 응답함(accepted는 큐 접수 상태이며 작업 완료가 아님).
// sent === false: 미전송(NOT_SENT)이거나 전달 여부가 불확실하여 command_id 조회가 필요함.
function describe(name, status, result) {
  if (result && result.sent === true) {
    const id = ' — request_id ' + result.request_id + (result.request_state ? ' (' + result.request_state + ')' : '');
    if (result.accepted) return ['accepted', name + ': 접수됨 (실행 완료 아님)' + id];
    const word = result.reason_label && result.reason_label !== result.reason ? ' (' + result.reason_label + ')' : '';
    const said = result.message ? ' — ' + result.message : '';   // 터널 승인 노드가 남긴 한 줄
    return ['rejected', name + ': 거절 ' + result.reason + word + said + id];
  }
  if (result && result.sent === false) {
    const where = result.delivery === 'NOT_SENT' ? ' · 전달 안 됨'
      : result.command_id ? ' · 전달됐는지 모름 — command_id ' + result.command_id + ' 로 명령 기록을 확인한 뒤 다시 누르세요' : '';
    return ['failed', name + ': 보내지 못함 — ' + result.error + (result.message ? ': ' + result.message : '') + where];
  }
  return ['failed', name + ': 보내지 못함 — 콘솔 응답 HTTP ' + status + ', 전달됐는지 모름'];
}
// 2단계 클릭(최종 전송): CSRF 토큰, 명령 종류, 대상 ID, 파라미터를 담아 콘솔 /commands 경로로 단 1회 POST 전송한다.
// 응답 결과를 로그에 기록하고 대화상자를 닫는다. 오류 시 자동 재시도하지 않는다(AGENTS.md 7절).
// 콘솔 서버는 POST 요청마다 새로운 command_id를 채번하므로, 자동 재전송 시 중복 명령이 발행될 위험이 있다.
async function send() {
  if (!armed || sending) return;
  const body = new URLSearchParams({csrf: panel.dataset.csrf, kind: armed.kind});
  const args = JSON.parse(armed.args), choice = chosen();
  if (choice && choice.item_id) {
    body.append('item_id', choice.item_id);
  } else if (choice) {
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
// 명령 패널 전체에 대한 클릭 이벤트 위임 리스너 등록.
// 취소 버튼 및 Escape 키는 대화상자를 닫되, HTTP 전송 중에는 Escape 키에 의한 취소를 차단한다.
panel.addEventListener('click', event => {
  const button = event.target.closest('button[data-command]');
  if (button && !button.disabled) arm(button.dataset.command);
});
// 지도의 막힌 터널 표시(data-passage)를 누르면 터널 통과 승인 확인 창을 그 항목으로 연다(#406).
// 지도는 폴링마다 다시 그려지므로 #live 에 위임한다. 확인 창은 지도 밖(명령 칸)에 있어 다시 그려도 닫히지 않는다.
document.getElementById('live').addEventListener('click', event => {
  const mark = event.target.closest('[data-passage]');
  if (!mark) return;
  const gate = gates.APPROVE_PASSAGE;
  if (!allowed || !gate || !gate.enabled) {
    log('info', '터널 통과 승인은 ' + (allowed ? (gate ? gate.reason : '관문 정보 없음') : '관리자 계정만') + ' — 보내지 않음');
    return;
  }
  if (armed === null) arm('APPROVE_PASSAGE');
  if (!armed || armed.kind !== 'APPROVE_PASSAGE') return;
  const index = armed.choices.findIndex(c => c.item_id === mark.dataset.passage);
  if (index >= 0) { pick.selectedIndex = index; showArgs(); }
  confirmBox.scrollIntoView({block: 'nearest'});
});
pick.addEventListener('change', showArgs);
sendButton.addEventListener('click', send);
cancelButton.addEventListener('click', () => disarm());
confirmBox.addEventListener('keydown', event => { if (event.key === 'Escape' && !sending) disarm(); });

// 초기화: 서버가 사전 렌더링한 초기 프래그먼트에 타임스탬프를 부여하고,
// 게이트 조건을 동기화한 후 주기적 폴링 루프를 시작한다.
stamp();
readGates();
pollStatus(true);
timer = setInterval(poll, POLL_MS);
"""


def _head(title):
    """스타일시트가 인라인 포함된 기본 HTML 문서 헤더(폐쇄망 및 오프라인 환경 대응)."""
    return ('<!doctype html><html lang="ko"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>{escape(title)}</title><style>{_STYLE}</style></head><body>')


def _flashes(messages):
    return ''.join(f'<div class="flash {escape(category)}">{escape(text)}</div>' for category, text in messages)


def _user_nav(user, csrf):
    """현재 로그인 사용자 정보 및 내비게이션 바(로그아웃 버튼은 POST 및 CSRF 검증 적용)."""
    if user is None:
        return ''
    admin = ' <a href="/admin/accounts">계정 관리</a>' if user['role'] == 'ADMIN' else ''
    return (f'<nav class="who"><span>{escape(user["display_name"])} ({escape(user["role"])})</span>{admin}'
            f' <form class="inline" method="post" action="/logout"><input type="hidden" name="csrf" value="{escape(csrf or "")}">'
            '<button type="submit">로그아웃</button></form></nav>')


def _header(user, csrf):
    """관리자 전용 계정 관리 페이지 상단 헤더."""
    return f'<header class="admin-head"><h1>구조 관제</h1><p class="sys">{SYSTEM_NAME}</p>{_user_nav(user, csrf)}</header>'


def camera_stale_ms(cameras, stale_s):
    """카메라 영상 두절 판정 대기 시간(ms). 카메라 목록 설정 시 필수 옵션이며 기본값은 제공되지 않는다."""
    if not cameras:
        return None
    if stale_s is None:
        raise ConsoleError('CONFIG_MISSING', 'cameras need --camera-stale-s (seconds without a new frame)')
    if type(stale_s) is not float or not math.isfinite(stale_s) or stale_s <= 0:
        raise ConsoleError('INVALID_INPUT', f'camera stale time must be finite seconds > 0: {stale_s!r}')
    return round(stale_s * 1000)


# 정상적으로 연결이 종료된 multipart 스트림은 오류 이벤트를 발생시키지 않고 마지막 프레임이 정지 화면으로 남는다
# (측정: 2026-10-13 Firefox). 예전에는 프레임마다 오는 load 이벤트로 살아 있음을 봤는데, Chrome 은 multipart img 에
# load 를 첫 프레임에 한 번만 낸다(측정: 2026-10-15 Chromium, 8초 40장에 load 1번). 그래서 Chrome 에서는 stale 마다
# 「영상 끊김」을 띄우고 다시 연결해 영상이 자꾸 끊겼다. 이제는 영상 중계기(mjpeg_relay·robot_view_relay)의
# /health(마지막 프레임 나이)를 stale / 2 주기로 묻는다. 낡았거나 답이 없으면 「영상 끊김」을 띄우고 캐시 방지 쿼리로
# 다시 연결한다(stale 주기당 최대 1회). 다시 새 프레임이 오면 끊김을 지우고 한 번 다시 연다. onerror는 즉시 반영된다.
_CAMERA_SCRIPT = """
function watchCameras() {
  document.querySelectorAll('#cameras figure[data-stale-ms], #robot-camera figure[data-stale-ms]').forEach(figure => {
    const img = figure.querySelector('img'), note = figure.querySelector('figcaption').lastChild;
    const stale = Number(figure.dataset.staleMs), base = img.getAttribute('src');
    const health = new URL('/health', base).href;
    let retried = Date.now();
    const reopen = now => { retried = now; img.src = base + (base.includes('?') ? '&' : '?') + '_r=' + now; };
    const lost = () => {
      figure.classList.add('lost'); note.textContent = figure.dataset.lostText || '영상 끊김'; note.className = 'lost';
      const now = Date.now();
      if (now - retried >= stale) reopen(now);
    };
    setInterval(() => {
      fetch(health, {cache: 'no-store'}).then(r => r.ok ? r.json() : null).then(h => {
        if (!h || typeof h.age_s !== 'number' || h.age_s * 1000 >= stale) { lost(); return; }
        if (figure.classList.contains('lost')) {   // 중계기에 새 프레임: 끊긴 스트림을 한 번 다시 연다
          figure.classList.remove('lost'); note.textContent = ''; note.className = '';
          reopen(Date.now());
        }
      }).catch(lost);
    }, Math.max(100, Math.min(500, stale / 2)));
  });
}
watchCameras();
"""


def _cameras(cameras, stale_ms=None, webcam=None, csrf=None, mode=None):
    """카메라 PC의 MJPEG 게이트웨이(IF-082) 스트림을 직접 임베드한다.

    폴링 프래그먼트 외부에 배치되어 브라우저 갱신 시에도 영상 스트림이 재연결되지 않는다.
    콘솔 서버는 영상을 중계하지 않고 클라이언트가 직접 스트림을 수신한다. 영상 두절 시 자동으로 감지 및 재시도된다.
    """
    if not cameras:
        return ('<section class="camera-empty"><h2>현장 웹캠</h2><p>웹캠 주소가 없습니다. 실행할 때 '
                '<code>--camera cam_a=http://…</code> 로 카메라 PC 의 영상 주소를 주세요.</p></section>')
    figures = ''.join(
        f'<figure data-stale-ms="{stale_ms}"><img src="{escape(url)}" '
        f'alt="{escape(camera_id)} 영상" '
        'onerror="var f=this.parentNode,s=f.querySelector(\'figcaption\').lastChild;'
        'f.classList.add(\'lost\');s.textContent=\'영상 끊김\';s.className=\'lost\'">'
        '<p class="nosignal">영상이 들어오지 않습니다. 카메라 PC 의 영상 서버와 주소를 확인하세요.</p>'
        f'<figcaption><span>{escape(camera_id)}</span><span></span></figcaption></figure>'
        for camera_id, url in cameras)
    return f'<section id="cameras"><h2>현장 웹캠</h2>{figures}{_webcam_picker(webcam, csrf, mode)}</section>'


def _webcam_picker(webcam, csrf, mode):
    """관리자의 카메라 고르기(OPS-183). 바꾸면 지진 판정이 처음부터 다시 돌고 출구 감시가 풀리므로 늘 확인을 받고,
    재난 모드(임무 진행 중)에는 그 말을 더 크게 한다."""
    if webcam is None:
        return ''
    notices = ''.join(f'<p class="webcam-notice">{escape(text)}</p>' for text in webcam['notices'])
    if webcam['problem']:
        return f'<div class="webcam-pick">{notices}<p class="alert">{escape(webcam["problem"])}</p></div>'
    options = ''.join(f'<option value="{escape(name)}"{" selected" if name == webcam["current"] else ""}>'
                      f'{escape(name)}</option>' for name in webcam['cameras'])
    warn = ('임무 진행 중(재난 모드)입니다. 카메라를 바꾸면 지진 판정이 처음부터 다시 돌고 출구 감시가 풀립니다. 바꿀까요?'
            if mode == 'DISASTER' else '카메라를 바꾸면 지진 판정이 처음부터 다시 돌고 출구 감시가 풀립니다. 바꿀까요?')
    return (f'<form class="webcam-pick" method="post" action="/webcam" onsubmit="return confirm({escape(json.dumps(warn, ensure_ascii=False))})">'
            f'{notices}<input type="hidden" name="csrf" value="{escape(csrf or "")}">'
            f'<label>카메라 <select name="device">{options}</select></label>'
            '<button type="submit">카메라 바꾸기</button></form>')


def _robot_camera(camera, csrf, stale_ms):
    """로봇 카메라 칸(OPS-191): 고른 로봇 하나의 판정 영상과 TB-6/TB-5 고르기. 보기 설정이라 누구나, 확인 창 없음.
    끊기면 「카메라 영상 낡음」이고 마지막 화면을 감춘다(웹캠과 같은 감시 스크립트)."""
    if camera is None:
        return ''
    notices = ''.join(f'<p class="alert">{escape(text)}</p>' for text in camera['notices'])
    names = dict(camera['robots'])
    title = f'{names.get(camera["current"], camera["current"] or "로봇")} 카메라'
    if camera['problem']:
        pick = f'<p class="alert">{escape(camera["problem"])}</p>'
    else:
        options = ''.join(f'<option value="{escape(rid)}"{" selected" if rid == camera["current"] else ""}>'
                          f'{escape(name)}</option>' for rid, name in camera['robots'])
        pick = (f'<form class="robot-cam-pick" method="post" action="/robot-camera">'
                f'<input type="hidden" name="csrf" value="{escape(csrf or "")}">'
                f'<label>로봇 <select name="robot" onchange="this.form.submit()">{options}</select></label>'
                '<noscript><button type="submit">보기</button></noscript></form>')
    return (f'<section id="robot-camera"><h2>로봇 카메라</h2>{notices}'
            f'<figure data-stale-ms="{stale_ms}" data-lost-text="카메라 영상 낡음"><img src="{escape(camera["url"])}" '
            f'alt="{escape(title)} 판정 영상" '
            'onerror="var f=this.parentNode,s=f.querySelector(\'figcaption\').lastChild;'
            'f.classList.add(\'lost\');s.textContent=\'카메라 영상 낡음\';s.className=\'lost\'">'
            '<p class="nosignal">로봇 판정 영상이 들어오지 않습니다. 로봇 PC 의 판정 영상과 운영 PC 중계를 확인하세요.</p>'
            f'<figcaption><span>{escape(title)}</span><span></span></figcaption></figure>{pick}</section>')


def page_title(view):
    """초기 로드 시 탭 타이틀. 이후에는 프론트엔드 title() 함수가 상태에 따라 동적으로 갱신한다."""
    return ('[재난] ' if view['mission']['mode'] == 'DISASTER' else '') + '구조 관제'


def failure_text(code, message):
    """갱신 실패 시 상태 표기용 한 줄 텍스트(오류 코드, 조치 안내, 상세 메시지)."""
    return f'{code}: {FAILURE_HINTS.get(code, FAILURE_HINT_OTHER)} ({message})'


def render_failure(code, message):
    """페이지 초기 로드 전체 실패(503): 오류 원인 및 점검 사항을 안내하며 FAILURE_RETRY_S 주기로 자동 재시도한다.

    정상적인 데이터를 조회할 수 없는 상태에서 빈 정상 화면을 표시하지 않는다(AGENTS.md 7절: 실패는 명확히 드러나야 함).
    """
    return (_head('[화면 오류] 구조 관제').replace(
                '<meta charset="utf-8">', f'<meta charset="utf-8"><meta http-equiv="refresh" content="{FAILURE_RETRY_S}">')
            + '<main class="card failure"><h1>화면을 그릴 수 없습니다</h1>'
            f'<p class="hint">{escape(FAILURE_HINTS.get(code, FAILURE_HINT_OTHER))}</p>'
            f'<p class="detail"><code>{escape(code)}</code> {escape(message)}</p>'
            f'<p class="retry">{FAILURE_RETRY_S}초마다 다시 시도합니다. 로봇은 이 화면과 상관없이 동작하므로, '
            '위험하면 현장 비상 정지 장치를 쓰세요.</p></main></body></html>')


def render_page(view, user=None, csrf=None, cameras=(), command_access=None, camera_stale_s=None, webcam=None,
                robot_camera=None):
    """`cameras`: CLI 인자로 전달된 ((camera_id, URL), ...). 설정 시 `camera_stale_s` 옵션도 필수이다
    (영상 두절 판정 대기 시간, 기본값 없음). `command_access`:
    commands.command_access의 (allowed, reason) 튜플이며, 미지정 시 명령 패널이 비활성화된다."""
    access = command_access or (False, '명령 연결 없음')
    stale_ms = camera_stale_ms(list(cameras) + ([robot_camera] if robot_camera else []), camera_stale_s)
    script = _SCRIPT.replace('POLL_MS', str(POLL_INTERVAL_MS)).replace('KPI_BUDGET_S', f'{KPI04_BUDGET_S:g}')
    if cameras or robot_camera:
        script += _CAMERA_SCRIPT
    return (_head(page_title(view)) + '<main class="console">'
            + f'<header id="masthead"><p class="sys">{SYSTEM_NAME}</p><p class="screen">구조 관제</p>'
            + '<a class="replay-link" href="/replay">지난 회차 다시 보기</a>'
            + ('<a class="replay-link" href="/reset">회차 초기화</a>' if user and user['role'] == 'ADMIN' else '')
            + '</header>'
            + f'<section id="topbar"><p id="poll" class="wait">연결 중</p><div id="poll-error" hidden></div>'
            + '<div class="kpi"><p id="kpi" title="발생 시각과 이 화면에 처음 그린 시각의 차이. 두 PC 의 시계가 맞을 때만 의미가 있습니다 (C-03)">'
            'KPI-04: 페이지를 연 뒤 들어온 사건 없음</p>'
            '<button type="button" id="kpi-save" class="mini" title="사건별 화면 표시 지연을 파일로 저장">지연 기록 저장</button>'
            '<input type="checkbox" id="show-history"><label for="show-history" class="history-toggle" '
            'title="운영자가 보낸 명령의 접수·거절과 operations 오류 기록을 펴거나 접습니다">보낸 명령 · 오류</label></div>'
            + _user_nav(user, csrf) + '</section>'
            + f'<aside id="side">{_cameras(cameras, stale_ms, webcam, csrf, view["mission"]["mode"])}'
            + f'{_robot_camera(robot_camera, csrf, stale_ms)}{_commands(view, access, csrf)}</aside>'
            + f'<div id="live">{render_fragment(view)}</div></main>'
            f'<script>{script}</script></body></html>')


def render_login(csrf, messages=()):
    """로그인 폼 렌더링(SQLite 사용자 계정 데이터베이스 인증 연동)."""
    return (_head('구조 관제 로그인') + f'<div class="card"><p class="sys">{SYSTEM_NAME}</p><h1>구조 관제 로그인</h1>' + _flashes(messages)
            + '<form method="post" action="/login">'
            f'<input type="hidden" name="csrf" value="{escape(csrf)}">'
            '<label for="username">아이디</label><input id="username" name="username" autocomplete="username" required>'
            '<label for="password">비밀번호</label>'
            '<input id="password" name="password" type="password" autocomplete="current-password" required>'
            '<p><button type="submit">로그인</button></p></form></div></body></html>')


def render_accounts(accounts, user, csrf, messages=()):
    """관리자(ADMIN) 전용 계정 관리 화면: 계정 목록 조회, 신규 계정 등록, 정보 수정 및 삭제."""
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
