"""회차 초기화 페이지 HTML. 표준 라이브러리만 쓰고 Flask 없이 시험한다.

배치는 사용자 결정(2026-10-15, 따로 페이지 /reset): 위 칸에 지금 회차·로봇 미리 보기·초기화 보내기, 아래 칸에
PC 별 다시 띄우기 체크 목록. 보내기 칸은 고정이고, 미리 보기와 체크 목록은 1초마다 /reset/fragment 로 바꿔 끼운다.
보내기는 ADMIN 만, 사람 확인 칸 체크도 ADMIN 만 한다. 다른 계정은 보기만 한다.
"""

from datetime import datetime
from html import escape

from .labels import label
from .render import _STYLE, SYSTEM_NAME, _age, _flashes, _user_nav
from .reset import REFUSAL_HINT

RESET_POLL_MS = 1000   # 관제 화면 폴링과 같은 주기([제안] 관제·화면, 2026-10-15)

_RESET_STYLE = """
.reset{max-width:1100px;margin:0 auto;padding:0 16px 24px;display:flex;flex-direction:column;gap:var(--gap)}
.reset #masthead{margin:0 -16px}
.reset #masthead .back{order:3;margin-left:auto;font-size:13px}
.reset section{background:var(--card);border-radius:var(--r);padding:16px 18px;box-shadow:var(--shadow)}
.reset table{width:100%;border-collapse:collapse;font-size:13.5px}
.reset th,.reset td{text-align:left;padding:7px 8px;border-bottom:1px solid var(--line);vertical-align:top}
.reset .ok{color:var(--safe);font-weight:700} .reset .bad{color:var(--signal-strong);font-weight:700}
.reset .wait{color:var(--ink-3)}
.reset .hint{margin-top:8px;font-size:13px;color:var(--ink-2)}
.reset .refuse{margin-top:10px;padding:8px 12px;border-radius:var(--r-in);background:var(--signal-tint);color:var(--signal-strong);font-weight:600}
#reset-send form{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-top:12px}
#reset-send input[name=note]{padding:6px 10px;min-width:260px}
#reset-send button{font-weight:700;padding:8px 18px;border-radius:var(--pill);border:0;background:var(--signal);color:#fff;cursor:pointer}
#reset-send button:disabled{background:var(--pending);cursor:not-allowed}
#reset-send .confirm{display:flex;gap:6px;align-items:center;font-size:13.5px}
.flash{padding:8px 12px;border-radius:var(--r-in);font-weight:600}
.flash.reset-success{background:var(--safe-tint);color:var(--safe)}
.flash.reset-danger{background:var(--signal-tint);color:var(--signal-strong)} .flash.reset-warning{background:var(--caution-tint);color:#8a6100}
.step td:first-child{white-space:nowrap;font-weight:700}
.items{list-style:none;margin:0;padding:0;display:flex;flex-direction:column;gap:4px}
.items li{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.items .tag{font-size:11.5px;padding:1px 7px;border-radius:var(--pill);background:var(--raise);color:var(--ink-3)}
.items form{display:inline} .items button{font-size:12px;padding:2px 10px;border-radius:var(--pill);border:1px solid var(--line-2);background:#fff;cursor:pointer}
"""

_RESET_SCRIPT = """
(function(){
  var live = document.getElementById('reset-live');
  function poll(){
    fetch('/reset/fragment', {credentials: 'same-origin'})
      .then(function(r){ return r.text().then(function(t){ if (!r.ok) throw new Error(t); return t; }); })
      .then(function(html){ live.innerHTML = html; })
      .catch(function(e){ live.innerHTML = '<section><p class="bad">갱신 끊김: ' + String(e.message).replace(/</g, '&lt;') + '</p></section>'; });
  }
  setInterval(poll, POLL_MS);
})();
"""


def _run_text(run):
    if run is None:
        return '열린 회차 없음'
    started = datetime.fromtimestamp(run['started_at_ns'] / 1e9).strftime('%m-%d %H:%M')
    name = escape(run['note']) if run['number'] is None else f'회차 {run["number"]}'
    return f'{name} · {started} 시작'


def _precheck(rows, first, names):
    lines = []
    for row in rows:
        name = escape(names.get(row['robot_id'], row['robot_id']))
        if row['reason'] is None:
            state = '<span class="ok">초기화 가능</span>'
        else:
            state = f'<span class="bad">{escape(row["reason_label"])}</span> <code>{escape(row["reason"])}</code>'
        motion = label('motion', row['motion_state']) if row['motion_state'] else '—'
        dock = escape(row['dock_state'] or '—')
        age = '—' if row['age_ns'] is None else f'{_age(row["age_ns"] / 1e9)} 전'
        lines.append(f'<tr data-robot="{escape(row["robot_id"])}"><td>{name}</td><td>{escape(motion)}</td>'
                     f'<td>{dock}</td><td>{age}</td><td>{state}</td></tr>')
    warn = ('' if first is None else
            f'<p class="refuse">지금 보내면 거절됩니다: {escape(label("reply", first))} ({escape(first)}). {REFUSAL_HINT}</p>')
    return ('<section id="reset-precheck"><h2>로봇 미리 보기</h2>'
            '<table><thead><tr><th>로봇</th><th>움직임</th><th>도킹</th><th>상태 받은 때</th><th>초기화</th></tr></thead>'
            f'<tbody>{"".join(lines)}</tbody></table>{warn}'
            '<p class="hint">미리 보기일 뿐입니다. operations 가 받을 때 같은 검사를 다시 합니다.</p></section>')


def _item(row, can_check, csrf):
    if row['kind'] == 'person':
        tag = '사람'
        who = '' if row['by_username'] is None else f' <span class="wait">({escape(row["by_username"])})</span>'
        button = ''
        if can_check:
            value = '0' if row['done'] else '1'
            words = '체크 풀기' if row['done'] else '확인함'
            button = (f'<form method="post" action="/reset/check"><input type="hidden" name="csrf" value="{escape(csrf or "")}">'
                      f'<input type="hidden" name="item" value="{escape(row["key"])}"><input type="hidden" name="checked" value="{value}">'
                      f'<button type="submit">{words}</button></form>')
    else:
        tag, who, button = '자동', '', ''
    if row.get('unknown'):
        mark = '<span class="wait">? 이름의 로봇을 모름</span>'
    else:
        mark = '<span class="ok">✔</span>' if row['done'] else '<span class="wait">☐</span>'
    return (f'<li data-item="{escape(row["key"])}" class="{"done" if row["done"] else "todo"}">{mark} '
            f'{escape(row["label"])} <span class="tag">{tag}</span>{who}{button}</li>')


def _checklist(model, can_check, csrf):
    if model is None:
        return ('<section id="reset-checklist"><h2>다시 띄우기 체크 목록</h2>'
                '<p class="wait">아직 초기화한 적이 없습니다. 초기화를 보내면 PC 별 순서가 여기에 나옵니다.</p></section>')
    rows = ''.join(f'<tr class="step"><td>{step["number"]}</td><td>{escape(step["pc"])}</td><td>{escape(step["text"])}'
                   f'<ul class="items">{"".join(_item(row, can_check, csrf) for row in step["items"])}</ul></td></tr>'
                   for step in model['steps'])
    return (f'<section id="reset-checklist"><h2>다시 띄우기 체크 목록 ({model["done"]}/{model["total"]})</h2>'
            '<table><thead><tr><th>순서</th><th>PC</th><th>할 일 · 확인</th></tr></thead>'
            f'<tbody>{rows}</tbody></table>'
            '<p class="hint">자동 칸은 operations 기록으로 채워집니다. SLAM·지도 고정·relay·AMCL·출발 정렬은 기록에 '
            '흔적이 없어 사람이 체크합니다. 체크는 이번 회차에만 묶입니다.</p></section>')


def render_reset_fragment(model):
    """1초마다 바꿔 끼우는 부분: 로봇 미리 보기와 체크 목록."""
    return (_precheck(model['precheck'], model['first_refusal'], model['names'])
            + _checklist(model['checklist'], model['can_check'], model['csrf']))


def render_reset_page(model, user=None, csrf=None, messages=()):
    """회차 초기화 전체 페이지. model 은 app 이 만든 dict(current_run, next_number, can_send, send_off, …)."""
    head = ('<!doctype html><html lang="ko"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>회차 초기화</title><style>{_STYLE}{_RESET_STYLE}</style></head><body><main class="reset">'
            f'<header id="masthead"><p class="sys">{SYSTEM_NAME}</p><p class="screen">회차 초기화</p>'
            f'<a class="back" href="/">관제 화면으로</a>{_user_nav(user, csrf)}</header>')
    if model['can_send']:
        form = (f'<form method="post" action="/reset/send"><input type="hidden" name="csrf" value="{escape(csrf or "")}">'
                '<input name="note" maxlength="200" placeholder="메모(선택)">'
                '<label class="confirm"><input type="checkbox" name="confirm" value="yes" required> '
                f'지도·임무가 처음으로 돌아가고, 지금 기록은 {escape(model["current_word"])}(으)로 보관됨을 확인했습니다</label>'
                '<button type="submit">초기화 보내기</button></form>')
    else:
        form = f'<p class="wait">초기화 보내기 꺼짐: {escape(model["send_off"])}</p>'
    send = ('<section id="reset-send"><h2>초기화</h2>'
            f'<p>지금: <b>{_run_text(model["current_run"])}</b> → 보내면 새 회차 {model["next_number"]} 이 열립니다.</p>'
            '<p class="hint">TB-6 SLAM 지도와 지도 고정까지 처음으로 되돌립니다. 지난 기록은 지우지 않고 회차별로 남아 '
            '<a href="/replay">지난 회차 다시 보기</a>에서 볼 수 있습니다. 요청은 한 번만 보내고 다시 보내지 않습니다.</p>'
            f'{form}</section>')
    script = _RESET_SCRIPT.replace('POLL_MS', str(RESET_POLL_MS))
    return (head + _flashes(messages) + send + f'<div id="reset-live">{render_reset_fragment(model)}</div></main>'
            f'<script>{script}</script></body></html>')
