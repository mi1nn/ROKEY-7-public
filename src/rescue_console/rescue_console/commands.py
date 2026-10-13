"""운영자 명령: 관제 웹이 operations 로 보내는 것과 그 답을 알리는 방식.

관제 웹은 operations 의 `POST /api/operator-commands` 를 부르는 클라이언트다(console-and-db-plan 4절).
버튼을 한 번 누를 때마다 요청 하나를 만들어 한 번 보내고, 답은 온 그대로 넘긴다.
실패는 실패로 알린다. 다시 보내지 않고 추측한 값으로 바꾸지도 않는다(AGENTS.md 7절).
명령을 받거나 거절하는 관문은 조정기의 것이다. 관제 웹은 아무것도 쓰지 않는다.
표준 라이브러리만 쓴다.
"""

import json
import socket
import urllib.error
import urllib.parse
import urllib.request

from .labels import label
from .view import ConsoleError

COMMAND_PATH = '/api/operator-commands'
# 관제 웹이 operations 의 답을 기다리는 시간. operations 가 조정기 서비스를 부르는 시간까지
# 덮어야 한다([제안] 관제·화면, 2026-10-12; PR 본문 「열린 것」).
COMMAND_TIMEOUT_S = 5.0
# 종류마다 싣는 필드. 조정기가 읽는 것과 같다(rescue_coordination.mission.COMMAND_FIELDS;
# tests/test_console_commands.py 가 둘을 비교한다). PAUSE·RESUME·CANCEL 은 Q-22 재개 관문을 기다리고,
# STOP_ROBOT 은 자기 RequestStop 경로가 따로 있어서, 여기서는 보낼 수 없다.
COMMAND_FIELDS = {
    'START_SEARCH': (),
    'DISPATCH_SUPPORT': ('target_id', 'target_revision'),
    'CONTINUE_SEGMENT': ('target_id', 'target_revision', 'route_revision', 'segment_index'),
    'REQUEST_RECHECK': ('target_id',),
    'RETURN_HOME': (),
}
SENDABLE = tuple(COMMAND_FIELDS)
# revision 들과 segment 는 운영자가 화면에서 본 값이다. 그래서 승인이 그 뒤에 바뀐 대상이나 경로에
# 붙지 못한다(SubmitOperatorCommand 의 expected_* 필드). 값을 모르면 None 을 보내지 않고
# 버튼을 꺼 둔다(작전 #115, 2026-10-12).
INT_FIELDS = frozenset({'target_revision', 'route_revision', 'segment_index'})
# 모든 종류가 자기 필드 말고도 실을 수 있는 폼 키. 그 밖의 키는 거부한다(build_request).
FORM_KEYS = frozenset({'csrf', 'kind'})
# 폼에서 받는 target_id 의 최대 길이. 상한일 뿐이다: 아는 대상 ID 인지는 operations 가 판단한다.
MAX_ID = 128
# 실패한 전송의 전달 여부: operations 에 아무것도 닿지 않았으면 NOT_SENT, 닿았을 수도 있으면 UNKNOWN.
DELIVERY = frozenset({'NOT_SENT', 'UNKNOWN'})


class CommandSendError(Exception):
    """전송 실패. `delivery` 는 operations 가 명령을 받았을 수도 있는지를 말한다."""

    def __init__(self, code, message, *, delivery='UNKNOWN'):
        if delivery not in DELIVERY:
            raise ValueError(f'delivery must be one of {sorted(DELIVERY)}')
        self.code, self.message, self.delivery = code, message, delivery
        super().__init__(f'{code}: {message}')


def _invalid(message):
    return ConsoleError('INVALID_INPUT', message)


def parse_operations_url(text):
    """'http://host:port[/base]' -> 끝 슬래시를 뗀 기본 URL. 기본값은 없고 아무것도 추측하지 않는다."""
    parts = urllib.parse.urlsplit(text or '')
    if parts.scheme not in ('http', 'https') or not parts.hostname or parts.query or parts.fragment:
        raise _invalid(f'operations must be http(s)://host:port : {text!r}')
    return urllib.parse.urlunsplit((parts.scheme, parts.netloc, parts.path.rstrip('/'), '', ''))


def command_access(role, connected):
    """로그인한 역할의 (allowed, reason). 누가 누를 수 있는지이지, 명령이 받아들여질지가 아니다."""
    if role != 'ADMIN':
        return False, '보기 전용 계정 — 명령은 ADMIN 만 보냅니다'
    if not connected:
        return False, 'operations 주소 없음 — 실행할 때 --operations 로 주면 켜집니다'
    return True, '누르면 operations 로 한 번 보냅니다. 받을지는 조정기가 정합니다'


def _target_id(text):
    if not text or len(text) > MAX_ID or not text.isprintable() or any(ch.isspace() for ch in text):
        raise _invalid('target_id must be 1..128 printable characters without spaces')
    return text


def _count(name, text):
    # isdigit 만으로는 '²' 같은 비 ASCII 숫자도 통과하는데 int() 는 이를 거부한다. isascii 로 0-9 만 남긴다.
    # 부호도 소수점도 받지 않으므로 값은 음수가 되거나 반올림되지 않는다.
    if not (isinstance(text, str) and text.isascii() and text.isdigit()):
        raise _invalid(f'{name} must be a non-negative integer')
    return int(text)


def build_request(form, *, operator_id, command_id):
    """브라우저 폼 -> operations 로 보낼 JSON 본문. `operator_id` 는 로그인 세션에서 온다.

    그 종류의 필드만 받고 폼의 다른 것은 모두 거부한다. 그래서 페이지나 손으로 만든 요청이
    로봇, 다른 운영자, 보류된 명령을 끼워 넣을 수 없다.
    """
    kind = form.get('kind', '')
    if kind not in COMMAND_FIELDS:
        raise _invalid(f'command {kind!r} is not sendable from the console')
    fields = COMMAND_FIELDS[kind]
    extra = sorted(key for key in form if key not in FORM_KEYS | set(fields))
    if extra:
        raise _invalid(f'{kind} does not take {", ".join(extra)}')
    # command_id 와 operator_id 는 서버(app.py)가 정하고, 폼에서 오지 않는다.
    payload = {'command_id': command_id, 'kind': kind, 'operator_id': operator_id}
    for name in fields:
        text = form.get(name, '')
        if name in INT_FIELDS:
            payload[name] = _count(name, text)
        else:
            payload[name] = _target_id(text)
    return payload


def post_json(url, payload, timeout):
    """POST 한 번, 재시도 없음. HTTP 답이 오면 무엇이든 (status, body) 를 돌려주고, 아니면 CommandSendError 를 낸다."""
    request = urllib.request.Request(url, data=json.dumps(payload).encode(), method='POST',
                                     headers={'Content-Type': 'application/json', 'Accept': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:          # 오류 상태의 답: 그래도 본문에 이유가 있을 수 있다
        with exc:                                  # 이 오류는 응답이기도 하다: 본문을 읽은 뒤 닫는다
            return exc.code, exc.read()
    # HTTP 답이 없음. 연결 자체가 안 됐을 때(거부됐거나 호스트 이름을 풀지 못함)만 전달 여부가 NOT_SENT 이다.
    # 그 밖의 모든 실패에서는 operations 가 요청을 받았을 수도 있다.
    except urllib.error.URLError as exc:
        if isinstance(exc.reason, (TimeoutError, socket.timeout)):
            raise CommandSendError('TIMEOUT', f'no answer within {timeout:g}s') from exc
        if isinstance(exc.reason, (ConnectionRefusedError, socket.gaierror)):
            raise CommandSendError('UNREACHABLE', str(exc.reason), delivery='NOT_SENT') from exc
        raise CommandSendError('UNREACHABLE', str(exc.reason)) from exc
    except (TimeoutError, socket.timeout) as exc:  # 연결한 뒤 답을 기다리다 시간 초과
        raise CommandSendError('TIMEOUT', f'no answer within {timeout:g}s') from exc
    except OSError as exc:
        raise CommandSendError('SEND_FAILED', str(exc)) from exc


def _optional_text(value):
    return value is None or (isinstance(value, str) and value != '')


def read_reply(status, body):
    """operations 의 답 -> {accepted, request_id, reason, request_state, created_task_id}.

    상태 코드와 상관없이 읽는다. 오류 상태여도 답을 싣고 있으면 답이다.
    그 밖의 것은 전달 여부를 모르는 실패다: 요청은 operations 에 닿았다.
    """
    try:
        data = json.loads(body)
    except (TypeError, ValueError):
        data = None
    # 답의 조건: `accepted` 는 진짜 bool(1/0 이 아님), 다른 필드는 없거나 null 이거나 빈 문자열이 아닌 텍스트,
    # 거절은 이유를 밝히고, 2xx 가 아닌 상태로는 어떤 것도 accepted 가 아니다.
    ok = (isinstance(data, dict) and type(data.get('accepted')) is bool
          and all(_optional_text(data.get(key)) for key in ('request_id', 'reason', 'request_state', 'created_task_id'))
          and (data['accepted'] or data.get('reason') is not None)
          and not (data['accepted'] and not 200 <= status < 300))
    if not ok:
        snippet = (body or b'')[:200].decode('utf-8', 'replace')   # 앞부분만 써서 메시지를 짧게 둔다
        # 2xx 이거나 JSON 객체인데 올바른 답이 아니면 BAD_RESPONSE, 그 밖은 그냥 HTTP 오류다.
        if 200 <= status < 300 or isinstance(data, dict):
            raise CommandSendError('BAD_RESPONSE', f'HTTP {status}: not an operator-command answer: {snippet}')
        raise CommandSendError('HTTP_ERROR', f'HTTP {status}: {snippet}')
    return {key: data.get(key) for key in ('accepted', 'request_id', 'reason', 'request_state', 'created_task_id')}


def send_command(base_url, payload, *, transport=post_json, timeout=COMMAND_TIMEOUT_S):
    """한 번 보내고 무슨 일이 있었는지 알린다. 다시 보내지 않는다: 두 번째 누름은 운영자의 결정이다."""
    head = {'command_id': payload['command_id'], 'kind': payload['kind']}
    try:
        reply = read_reply(*transport(base_url + COMMAND_PATH, payload, timeout))
    except CommandSendError as exc:
        return {'sent': False, **head, 'error': exc.code, 'message': exc.message, 'delivery': exc.delivery}
    return {'sent': True, **head, **reply, 'reason_label': label('reply', reply['reason'])}
