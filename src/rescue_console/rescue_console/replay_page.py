"""리플레이 페이지 HTML. 표준 라이브러리만 쓰고 Flask 없이 시험한다.

배치는 사용자 결정(2026-10-15, 좌우 두 칸): 위에 회차·재생 버튼·시간 막대, 아래 왼쪽에 지도 칸, 오른쪽에 기록 표.
재생 시계는 브라우저가 돌리고, 시점이 바뀌면 /replay/fragment?offset_ns= 로 그 시점의 조각만 받아 바꿔 끼운다.
JavaScript 수는 2^53 을 넘는 epoch ns 를 정확히 다루지 못하므로 브라우저는 회차 시작부터의 offset 만 쓴다.
이 페이지에는 명령 버튼과 로봇에 닿는 경로가 없다.
"""

from html import escape

from .render import _STYLE, SYSTEM_NAME, _reply, _short, _time, _user_nav

# 재생 중 조각을 다시 받는 최소 간격([제안] 관제·화면, 2026-10-15). 받는 중이면 다음 틱은 건너뛴다.
REPLAY_FETCH_MS = 250
KIND_WORDS = {'EVENT': '사건', 'COMMAND': '명령', 'ERROR': '오류', 'EXIT': '출구'}
SECTION_WORDS = {'site_events': '사건', 'operator_commands': '명령', 'error_log': '오류', 'exit_locations': '출구'}

_REPLAY_STYLE = """
.replay{display:grid;gap:var(--gap);padding:0 16px 16px;grid-template-columns:minmax(0,1fr) minmax(0,1fr);
  grid-template-areas:"head head" "bar bar" "map log"}
.replay #masthead{grid-area:head}
.replay #masthead .back{order:3;margin-left:auto;font-size:13px}
#replay-bar{grid-area:bar} #replay-map{grid-area:map} #replay-log{grid-area:log}
#replay-live{display:contents}
#replay-bar,#replay-map,#replay-log,.replay .empty-run{background:var(--card);border-radius:var(--r);padding:16px 18px;box-shadow:var(--shadow)}
#replay-bar .row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
#replay-bar button{font-size:13px;font-weight:600;padding:5px 12px;border-radius:var(--pill);border:1px solid var(--line-2);
  background:#fff;color:var(--ink-2);cursor:pointer}
#replay-bar button[aria-pressed=true]{background:var(--accent);border-color:var(--accent);color:#fff}
#replay-clock{margin-left:auto;font-weight:700;font-size:15px}
#replay-track{position:relative;margin-top:12px}
#replay-slider{width:100%;accent-color:var(--accent)}
#replay-marks{position:relative;height:14px;margin:2px 7px 0}
#replay-marks button{position:absolute;top:2px;width:8px;height:8px;margin-left:-4px;padding:0;border:0;border-radius:50%;cursor:pointer}
.mark-EVENT{background:var(--signal)!important} .mark-COMMAND{background:var(--info)!important}
.mark-ERROR{background:var(--caution)!important} .mark-EXIT{background:var(--safe)!important}
.legend{display:flex;gap:14px;font-size:12.5px;color:var(--ink-3);margin-top:6px}
.legend i{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px}
.notice{margin-top:8px;padding:6px 10px;border-radius:var(--r-in);background:var(--caution-tint);font-size:13px}
#replay-map .nomap{padding:28px 12px;text-align:center;border-radius:var(--r-in);background:var(--map-bg);color:var(--ink-2)}
#replay-map dl{display:grid;grid-template-columns:auto 1fr;gap:6px 14px;margin-top:14px}
#replay-map dt{color:var(--ink-3)} #replay-map dd{margin:0}
#replay-log table{width:100%;border-collapse:collapse;font-size:13px}
#replay-log th,#replay-log td{text-align:left;padding:6px 8px;border-bottom:1px solid var(--line);vertical-align:top}
#replay-log tr.refused .outcome,#replay-log tr.ERROR .outcome{color:var(--signal-strong)}
#replay-log .count{color:var(--ink-3);font-size:12.5px;margin-bottom:6px}
@media (max-width:1100px){.replay{grid-template-columns:minmax(0,1fr);grid-template-areas:"head" "bar" "map" "log"}}
"""

_REPLAY_SCRIPT = """
(function(){
  var DUR = DURATION_NS, FETCH_MS = FETCH_EVERY_MS;
  var slider = document.getElementById('replay-slider'), clock = document.getElementById('replay-clock');
  var live = document.getElementById('replay-live'), playBtn = document.getElementById('replay-play');
  var speed = 1, playing = false, offset = DUR, lastTick = null, lastFetch = 0, busy = false, wanted = null;
  function fmt(ns){ var s = Math.floor(ns / 1e9), m = Math.floor(s / 60);
    return m + '분 ' + String(s % 60).padStart(2, '0') + '초 / ' + Math.floor(DUR / 6e10) + '분 '
      + String(Math.floor(DUR / 1e9) % 60).padStart(2, '0') + '초'; }
  function fetchAt(ns){
    wanted = ns; if (busy) return; busy = true; lastFetch = performance.now();
    fetch('/replay/fragment?offset_ns=' + Math.round(ns), {credentials: 'same-origin'})
      .then(function(r){ return r.text().then(function(t){ if (!r.ok) throw new Error(t); return t; }); })
      .then(function(html){ live.innerHTML = html; })
      .catch(function(e){ live.innerHTML = '<p class="notice">불러오기 실패: ' + String(e.message).replace(/</g, '&lt;') + '</p>'; })
      .finally(function(){ busy = false; if (wanted !== null && wanted !== ns) fetchAt(wanted); else wanted = null; });
  }
  function seek(ns, now){ offset = Math.max(0, Math.min(DUR, ns)); slider.value = offset; clock.textContent = fmt(offset);
    if (now || performance.now() - lastFetch >= FETCH_MS) fetchAt(offset); }
  function setPlaying(on){ playing = on && DUR > 0; playBtn.setAttribute('aria-pressed', playing ? 'true' : 'false');
    playBtn.textContent = playing ? '멈춤' : '재생'; lastTick = null; if (playing) requestAnimationFrame(tick); }
  function tick(t){ if (!playing) return; if (lastTick !== null) seek(offset + (t - lastTick) * 1e6 * speed, false);
    lastTick = t; if (offset >= DUR) { seek(DUR, true); setPlaying(false); return; } requestAnimationFrame(tick); }
  playBtn.addEventListener('click', function(){ if (!playing && offset >= DUR) seek(0, true); setPlaying(!playing); });
  document.querySelectorAll('[data-speed]').forEach(function(b){ b.addEventListener('click', function(){
    speed = Number(b.dataset.speed);
    document.querySelectorAll('[data-speed]').forEach(function(o){ o.setAttribute('aria-pressed', o === b ? 'true' : 'false'); });
  }); });
  slider.addEventListener('input', function(){ setPlaying(false); seek(Number(slider.value), false); });
  slider.addEventListener('change', function(){ seek(Number(slider.value), true); });
  document.querySelectorAll('#replay-marks button').forEach(function(b){ b.addEventListener('click', function(){
    setPlaying(false); seek(Number(b.dataset.offset), true); }); });
  clock.textContent = fmt(offset);
})();
"""


def _row(item, ref_ns):
    """기록 표 한 줄. 명령은 접수·거절과 사유, 오류는 코드와 내용, 사건은 카메라와 대상, 출구는 좌표."""
    kind = item['kind']
    if kind == 'COMMAND':
        outcome, css = ('접수', 'accepted') if item['accepted'] else ('거절', 'refused')
        result = f'<b class="outcome">{outcome}</b> {_reply(item["reason"])}'
    elif kind == 'ERROR':
        css, result = 'ERROR', f'<b class="outcome">{escape(item["what"])}</b> {escape(item["detail"] or "")}'
    elif kind == 'EVENT':
        target = '' if item['subject'] is None else f' 대상 {_short(item["subject"])}'
        css, result = 'EVENT', f'{escape(item["source"] or "")}{target} {escape(item["detail"] or "")}'
    else:
        css, result = 'EXIT', escape(item['detail'] or '')
    return (f'<tr class="{css}" data-ref="{escape(item["ref_id"])}"><td class="t">{_time(item["at_ns"], ref_ns)}</td>'
            f'<td>{KIND_WORDS[kind]}</td><td>{escape(item["label"])}</td><td>{result}</td></tr>')


def render_replay_fragment(state, timeline):
    """시점 조각: 왼쪽 지도 칸(지금은 지도 이력 없음, 출구 관측·통과 수, 이력 없는 칸 이름)과 오른쪽 기록 표."""
    exit_ = state['exit']
    exit_text = ('관측 없음' if exit_ is None else
                 f'({exit_["x"]:.2f}, {exit_["y"]:.2f}) {escape(exit_["frame_id"])} · {_time(exit_["at_ns"], state["t_ns"])}')
    no_history = ''.join(f'<dt>{escape(row["label"])}</dt><dd>이력 없음</dd>' for row in timeline['no_history']
                         if row['key'] != 'map')
    left = ('<section id="replay-map"><h2>지도</h2>'
            '<p class="nomap">지도 이력 없음 — 지도 그림이 아직 시각별로 남지 않습니다</p>'
            f'<dl><dt>출구 관측</dt><dd>{exit_text}</dd><dt>출구 통과</dt><dd>{state["exit_pass_count"]}</dd>'
            f'{no_history}</dl></section>')
    rows = ''.join(_row(item, state['t_ns']) for item in state['rows'])
    shown = len(state['rows'])
    count = (f'<p class="count">이 시점까지 {state["seen_count"]}건'
             + (f' 중 최근 {shown}건' if shown < state['seen_count'] else '') + '</p>')
    body = ('<table><thead><tr><th>시각</th><th>종류</th><th>내용</th><th>결과 · 이유</th></tr></thead>'
            f'<tbody>{rows}</tbody></table>') if rows else '<p>이 시점까지 기록 없음</p>'
    return left + f'<section id="replay-log"><h2>기록</h2>{count}{body}</section>'


def _marks(timeline):
    start, duration = timeline['start_ns'], timeline['end_ns'] - timeline['start_ns']
    marks = []
    for item in timeline['items']:
        offset = item['at_ns'] - start
        left = 0.0 if duration == 0 else offset * 100.0 / duration
        marks.append(f'<button type="button" class="mark-{item["kind"]}" style="left:{left:.3f}%" '
                     f'data-offset="{offset}" title="{KIND_WORDS[item["kind"]]} {escape(item["label"])}" '
                     f'aria-label="{KIND_WORDS[item["kind"]]} {escape(item["label"])}"></button>')
    return ''.join(marks)


def _notices(timeline):
    notes = []
    if timeline['truncated']:
        names = ', '.join(SECTION_WORDS[key] for key in timeline['truncated'])
        notes.append(f'<p class="notice">{names} 기록이 읽기 상한에 닿아 일부만 보입니다</p>')
    if timeline['missing']:
        names = ', '.join(SECTION_WORDS[key] for key in timeline['missing'])
        notes.append(f'<p class="notice">{names} 기록 입력 없음</p>')
    return ''.join(notes)


def render_replay_page(timeline, state, user=None, csrf=None):
    """리플레이 전체 페이지. 처음에는 회차 끝 시점(state)을 그려 둔다. 기록이 없으면 state 는 None 이다."""
    head = ('<!doctype html><html lang="ko"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>지난 회차 다시 보기</title><style>{_STYLE}{_REPLAY_STYLE}</style></head><body><main class="replay">'
            f'<header id="masthead"><p class="sys">{SYSTEM_NAME}</p><p class="screen">지난 회차 다시 보기</p>'
            f'<a class="back" href="/">관제 화면으로</a>{_user_nav(user, csrf)}</header>')
    if timeline['start_ns'] is None:
        return head + ('<section class="empty-run"><h2>기록 없음</h2><p>이 기록 DB 에는 아직 사건·명령·오류·출구 기록이 없습니다.</p>'
                       f'{_notices(timeline)}</section></main></body></html>')
    duration = timeline['end_ns'] - timeline['start_ns']
    speeds = ''.join(f'<button type="button" data-speed="{n}" aria-pressed="{"true" if n == 1 else "false"}">{n}×</button>'
                     for n in timeline['speeds'])
    legend = ''.join(f'<span><i class="mark-{kind}"></i>{word}</span>' for kind, word in KIND_WORDS.items())
    bar = ('<section id="replay-bar"><div class="row">'
           '<label for="replay-run">회차</label><select id="replay-run" disabled>'
           f'<option>이 기록 DB 하나 · {_time(timeline["start_ns"])} 시작</option></select>'
           '<button type="button" id="replay-play" aria-pressed="false">재생</button>'
           f'{speeds}<span id="replay-clock"></span></div>'
           f'<div id="replay-track"><input type="range" id="replay-slider" min="0" max="{duration}" step="1" '
           f'value="{duration}" aria-label="시간 막대"><div id="replay-marks">{_marks(timeline)}</div></div>'
           f'<div class="legend">{legend}</div>{_notices(timeline)}</section>')
    script = (_REPLAY_SCRIPT.replace('DURATION_NS', str(duration)).replace('FETCH_EVERY_MS', str(REPLAY_FETCH_MS)))
    return (head + bar + f'<div id="replay-live">{render_replay_fragment(state, timeline)}</div></main>'
            f'<script>{script}</script></body></html>')
