"""운영자 명령: 관제 웹에서 operations 서비스로의 명령 전송 및 응답 처리 모듈.

관제 웹은 operations의 `POST /api/operator-commands` 엔드포인트를 호출하는 HTTP 클라이언트다(console-and-db-plan 4절).
버튼 클릭 1회당 1개의 요청을 생성해 1회 전송하며, 응답은 가공 없이 그대로 전달한다.
전송 실패는 명시적으로 실패로 보고하며, 자동 재전송이나 임의 대체 값을 사용하지 않는다(AGENTS.md 7절).
명령 수락 또는 거절 판정 권한은 coordinator(조정기)에 있으며, 관제 웹은 DB에 직접 쓰지 않는다.
외부 종속성 없이 Python 표준 라이브러리만 사용한다.
"""

import json
import socket
import uuid
import urllib.error
import urllib.parse
import urllib.request

from .labels import label
from .view import ConsoleError

COMMAND_PATH = '/api/operator-commands'
# 터널 통과 승인(#406 /rescue/passage_keepout/approve). 웹은 ROS 노드가 아니라 운영 API 에 보내고 운영 노드가 서비스를
# 부른다(작전 2026-10-16). 경로와 답 모양은 [제안] 관제·화면이고 운영 쪽 구현을 기다린다(console README 「후속」).
PASSAGE_PATH = '/api/passages/approve'
WEBCAM_PATH = '/api/webcam'   # operations: GET 꽂힌 카메라 목록, POST {device, operator_id} 바꾸기(OPS-183)
# 관제 웹이 operations의 응답을 대기하는 타임아웃. operations가 coordinator 서비스를 호출하는 시간까지
# 감안하여 설정한다([제안] 관제·화면, 2026-10-12; PR 본문 「열린 것」).
COMMAND_TIMEOUT_S = 5.0
# 명령 종류별 페이로드 필드 정의. coordinator 요구 규격과 일치한다(rescue_coordination.mission.COMMAND_FIELDS;
# tests/test_console_commands.py에서 상호 검증). PAUSE·RESUME·CANCEL은 Q-22 재개 관문 결정을 대기하며,
# STOP_ROBOT은 전용 RequestStop 경로를 사용하므로 본 모듈에서는 전송하지 않는다.
COMMAND_FIELDS = {
    'START_SEARCH': (),
    'DISPATCH_SUPPORT': ('target_id', 'target_revision'),
    'CONTINUE_SEGMENT': ('target_id', 'target_revision', 'route_revision', 'segment_index'),
    'REQUEST_RECHECK': ('target_id',),
    'RETURN_HOME': (),
}
# 재난 선포: 평시에 운영자가 재난 모드로 넣는다. 조정기는 COMMAND_FIELDS 와 따로 둔다
# (rescue_coordination.mission.MODE_COMMAND_FIELDS, 작전 2026-10-15). 이미 재난이면 TASK_BUSY(사유 ALREADY_DISASTER)로 거절한다.
MODE_COMMAND_FIELDS = {'DECLARE_DISASTER': ()}
# 명령별 거절 풀이. 재난 선포의 TASK_BUSY 는 이미 재난 모드라는 뜻 하나뿐이다(작전 2026-10-15, #330).
KIND_REPLY_WORDS = {('DECLARE_DISASTER', 'TASK_BUSY'): '이미 재난 모드입니다'}
# 조정기를 거치지 않는 명령. 운영자 이름(approved_by)은 서버가 로그인 세션에서 붙이고 폼에서 받지 않는다.
PASSAGE_COMMAND_FIELDS = {'APPROVE_PASSAGE': ('item_id',)}
SENDABLE = tuple(COMMAND_FIELDS)   # 재난 모드에서 켜지는 명령. 재난 선포는 MODE_COMMAND_FIELDS 에 따로 둔다
_ALL_FIELDS = COMMAND_FIELDS | MODE_COMMAND_FIELDS | PASSAGE_COMMAND_FIELDS
# revision 및 segment 값은 운영자가 화면에서 확인한 당시 기준값이다. 따라서 승인 직후 변경된 대상이나 경로에
# 오적용되지 않도록 방지한다(SubmitOperatorCommand의 expected_* 필드). 값을 알 수 없으면 None을 전송하지 않고
# 버튼 자체를 비활성화한다(작전 #115, 2026-10-12).
INT_FIELDS = frozenset({'target_revision', 'route_revision', 'segment_index'})
# 모든 명령 종류에서 공통으로 허용되는 폼 필드 키. 정의되지 않은 키는 거부된다(build_request).
FORM_KEYS = frozenset({'csrf', 'kind'})
# 폼에서 수신하는 target_id의 최대 길이 제한(상한 검증용이며 유효 대상 여부는 operations가 판정).
MAX_ID = 128
# 전송 실패 시 요청 전달 상태: operations에 도달하지 않았음이 확실하면 NOT_SENT, 전달 여부 불확실 시 UNKNOWN.
DELIVERY = frozenset({'NOT_SENT', 'UNKNOWN'})


class CommandSendError(Exception):
    """명령 전송 실패 예외. `delivery`는 operations에 요청이 도달했을 가능성 여부를 나타낸다."""

    def __init__(self, code, message, *, delivery='UNKNOWN'):
        if delivery not in DELIVERY:
            raise ValueError(f'delivery must be one of {sorted(DELIVERY)}')
        self.code, self.message, self.delivery = code, message, delivery
        super().__init__(f'{code}: {message}')


def _invalid(message):
    return ConsoleError('INVALID_INPUT', message)


def parse_operations_url(text):
    """'http://host:port[/base]' URL에서 말미 슬래시를 제거한 기본 URL을 반환한다(추정 기본값 없음)."""
    parts = urllib.parse.urlsplit(text or '')
    if parts.scheme not in ('http', 'https') or not parts.hostname or parts.query or parts.fragment:
        raise _invalid(f'operations must be http(s)://host:port : {text!r}')
    return urllib.parse.urlunsplit((parts.scheme, parts.netloc, parts.path.rstrip('/'), '', ''))


def command_access(role, connected):
    """로그인 계정 역할에 따른 명령 권한 (allowed, reason)을 반환한다(버튼 클릭 권한이며 수락 여부와 무관)."""
    if role != 'ADMIN':
        return False, '보기 전용 계정 — 명령은 ADMIN 만 보냅니다'
    if not connected:
        return False, 'operations 주소 없음 — 실행할 때 --operations 로 주면 켜집니다'
    return True, '누르면 operations 로 한 번 보냅니다. 받을지는 조정기가 정합니다'


def _target_id(text):
    if not text or len(text) > MAX_ID or not text.isprintable() or any(ch.isspace() for ch in text):
        raise _invalid('target_id must be 1..128 printable characters without spaces')
    return text


def _item_id(text):
    """터널 항목 id 는 계약상 UUID 다(rescue_contracts.overhead). 지금 목록에 있는지는 승인 노드가 본다(NO_SUCH_ITEM)."""
    try:
        valid = str(uuid.UUID(text)) == text
    except (ValueError, AttributeError, TypeError):
        valid = False
    if not valid:
        raise _invalid('item_id must be a canonical UUID')
    return text


def _count(name, text):
    # isdigit만으로는 '²' 등의 비-ASCII 특수 숫자도 통과하지만 int()는 실패하므로 isascii로 0-9만 허용한다.
    # 부호 및 소수점 기호를 불허하여 음수나 부동소수점 오차 입력을 원천 차단한다.
    if not (isinstance(text, str) and text.isascii() and text.isdigit()):
        raise _invalid(f'{name} must be a non-negative integer')
    return int(text)


def build_request(form, *, operator_id, command_id):
    """브라우저 폼 데이터를 operations 전송용 JSON 페이로드로 변환한다(`operator_id`는 세션 기반).

    명령 종류에 정의된 필드만 추출하고 기타 폼 데이터는 엄격히 거부한다. 이를 통해 변조된 요청에서
    로봇 ID, 타 운영자 ID, 임의 명령 파라미터가 비정상 주입되는 것을 차단한다.
    """
    kind = form.get('kind', '')
    if kind not in _ALL_FIELDS:
        raise _invalid(f'command {kind!r} is not sendable from the console')
    fields = _ALL_FIELDS[kind]
    extra = sorted(key for key in form if key not in FORM_KEYS | set(fields))
    if extra:
        raise _invalid(f'{kind} does not take {", ".join(extra)}')
    # command_id 및 operator_id는 서버(app.py)에서 직접 생성/부여하며 클라이언트 폼에서 주입받지 않는다.
    payload = {'command_id': command_id, 'kind': kind, 'operator_id': operator_id}
    for name in fields:
        text = form.get(name, '')
        if name in INT_FIELDS:
            payload[name] = _count(name, text)
        elif name == 'item_id':
            payload[name] = _item_id(text)
        else:
            payload[name] = _target_id(text)
    return payload


def post_json(url, payload, timeout):
    """재시도 없이 단일 HTTP POST(payload None 이면 GET)를 수행한다. 응답 수신 시 (status, body)를 반환하고, 실패 시
    CommandSendError를 발생시킨다."""
    request = urllib.request.Request(url, data=None if payload is None else json.dumps(payload).encode(),
                                     method='GET' if payload is None else 'POST',
                                     headers={'Content-Type': 'application/json', 'Accept': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:          # HTTP 오류 응답: 본문에 세부 오류 사유가 포함될 수 있음
        with exc:                                  # HTTPError는 응답 스트림이므로 본문 판독 후 컨텍스트를 종료한다
            return exc.code, exc.read()
    # HTTP 응답 수신 실패: 연결 자체가 성립되지 않은 경우(접속 거부, 호스트 DNS 해석 실패)만 NOT_SENT로 확정한다.
    # 그 외 모든 전송 실패 상황에서는 operations가 이미 요청을 수신했을 가능성이 존재하므로 UNKNOWN으로 처리한다.
    except urllib.error.URLError as exc:
        if isinstance(exc.reason, (TimeoutError, socket.timeout)):
            raise CommandSendError('TIMEOUT', f'no answer within {timeout:g}s') from exc
        if isinstance(exc.reason, (ConnectionRefusedError, socket.gaierror)):
            raise CommandSendError('UNREACHABLE', str(exc.reason), delivery='NOT_SENT') from exc
        raise CommandSendError('UNREACHABLE', str(exc.reason)) from exc
    except (TimeoutError, socket.timeout) as exc:  # 연결 수립 후 응답 수신 대기 중 타임아웃 발생
        raise CommandSendError('TIMEOUT', f'no answer within {timeout:g}s') from exc
    except OSError as exc:
        raise CommandSendError('SEND_FAILED', str(exc)) from exc


def _optional_text(value):
    return value is None or (isinstance(value, str) and value != '')


def read_reply(status, body):
    """operations 응답 본문을 파싱하여 표준 딕셔너리로 반환한다.

    HTTP 상태 코드와 무관하게 본문 구조를 검증한다. 에러 상태 코드여도 유효한 JSON 사유가 포함되면 정상 파싱한다.
    규격에 맞지 않는 응답은 전송 후 결과 판독 불가 실패(요청은 이미 전달됨)로 간주한다.
    """
    try:
        data = json.loads(body)
    except (TypeError, ValueError):
        data = None
    # 응답 유효성 조건: accepted는 엄격한 bool(1/0 불허), 기타 필드는 생략/null/비공백 문자열이어야 함.
    # 거절 시에는 reason 필드가 필수이며, 2xx 상태 코드가 아니면 accepted=True는 허용되지 않는다.
    ok = (isinstance(data, dict) and type(data.get('accepted')) is bool
          and all(_optional_text(data.get(key)) for key in ('request_id', 'reason', 'request_state', 'created_task_id'))
          and (data['accepted'] or data.get('reason') is not None)
          and not (data['accepted'] and not 200 <= status < 300))
    if not ok:
        snippet = (body or b'')[:200].decode('utf-8', 'replace')   # 응답 앞부분 200바이트만 발췌하여 로그 간소화
        # 2xx 응답이거나 JSON 객체임에도 규격 미달인 경우 BAD_RESPONSE, 그 외는 일반 HTTP_ERROR로 분류한다.
        if 200 <= status < 300 or isinstance(data, dict):
            raise CommandSendError('BAD_RESPONSE', f'HTTP {status}: not an operator-command answer: {snippet}')
        raise CommandSendError('HTTP_ERROR', f'HTTP {status}: {snippet}')
    return {key: data.get(key) for key in ('accepted', 'request_id', 'reason', 'request_state', 'created_task_id')}


def send_command(base_url, payload, *, transport=post_json, timeout=COMMAND_TIMEOUT_S):
    """명령을 1회 전송하고 결과를 반환한다. 자동 재전송은 하지 않으며, 재시도 여부는 운영자가 직접 판단한다."""
    head = {'command_id': payload['command_id'], 'kind': payload['kind']}
    passage = payload['kind'] in PASSAGE_COMMAND_FIELDS
    try:
        status, body = transport(base_url + (PASSAGE_PATH if passage else COMMAND_PATH), payload, timeout)
        reply = read_reply(status, body)
    except CommandSendError as exc:
        return {'sent': False, **head, 'error': exc.code, 'message': exc.message, 'delivery': exc.delivery}
    words = KIND_REPLY_WORDS.get((payload['kind'], reply['reason'])) or label('reply', reply['reason'])
    result = {'sent': True, **head, **reply, 'reason_label': words}
    if passage:   # 승인 서비스 응답의 message(노드가 남긴 한 줄)는 글자일 때만 그대로 보인다
        message = json.loads(body).get('message')
        result['message'] = message if isinstance(message, str) and message else None
    return result


WEBCAM_LIST_TIMEOUT_S = 1.0   # 화면을 그릴 때 한 번 묻는다: 오래 막지 않는다


def webcam_state(base_url, *, transport=post_json):
    """(cameras, current, problem): operations 가 본 꽂힌 카메라(by-id 이름)와 지금 장치. 못 받으면 problem 만."""
    try:
        status, body = transport(base_url + WEBCAM_PATH, None, WEBCAM_LIST_TIMEOUT_S)
        data = json.loads(body)
        if status != 200 or not isinstance(data.get('cameras'), list):
            raise ValueError(f'HTTP {status}')
    except (CommandSendError, ValueError, TypeError, AttributeError) as exc:
        return [], None, f'카메라 목록을 못 받음: {getattr(exc, "message", exc)}'
    return [str(name) for name in data['cameras']], data.get('current'), None


def robot_camera_state(select_url, *, transport=post_json):
    """(robots, current, problem): robot_view_relay 의 고를 수 있는 로봇과 지금 로봇(OPS-191). 못 받으면 problem 만."""
    try:
        status, body = transport(select_url, None, WEBCAM_LIST_TIMEOUT_S)
        data = json.loads(body)
        if status != 200 or not isinstance(data.get('robots'), list):
            raise ValueError(f'HTTP {status}')
    except (CommandSendError, ValueError, TypeError, AttributeError) as exc:
        return [], None, f'로봇 카메라 선택 상태를 못 받음: {getattr(exc, "message", exc)}'
    return [str(robot) for robot in data['robots']], data.get('robot'), None


def select_robot_camera(select_url, robot, *, transport=post_json):
    """(ok, 문구): 로봇 카메라를 고른다(보기 설정, 재시도 없음)."""
    try:
        status, body = transport(select_url, {'robot': robot}, WEBCAM_LIST_TIMEOUT_S)
        data = json.loads(body)
    except CommandSendError as exc:
        return False, f'로봇 카메라 바꾸기 실패({exc.code}): {exc.message}'
    except (TypeError, ValueError):
        return False, f'로봇 카메라 바꾸기 답을 못 읽음(HTTP {status})'
    if status == 200 and isinstance(data, dict) and data.get('robot') == robot:
        return True, None
    return False, f'로봇 카메라 바꾸기 거절: {data.get("error") if isinstance(data, dict) else status}'


def switch_webcam(base_url, device, *, operator_id, transport=post_json, timeout=COMMAND_TIMEOUT_S):
    """(ok, 문구): 고른 카메라를 operations 로 한 번 보낸다. 재시도하지 않고 답을 그대로 문구로 만든다."""
    try:
        status, body = transport(base_url + WEBCAM_PATH, {'device': device, 'operator_id': operator_id}, timeout)
        data = json.loads(body)
    except CommandSendError as exc:
        return False, f'카메라 전환 요청 실패({exc.code}): {exc.message} — 바뀌었는지 모름' if exc.delivery == 'UNKNOWN' \
            else f'카메라 전환 요청 실패({exc.code}): {exc.message}'
    except (TypeError, ValueError):
        return False, f'카메라 전환 답을 못 읽음(HTTP {status}) — 바뀌었는지 모름'
    if isinstance(data, dict) and data.get('switched') is True:
        return True, f'카메라 전환 — 지진 판정 다시 시작, 출구 감시 풀림 ({data.get("current")})'
    reason = data.get('reason') if isinstance(data, dict) else None
    return False, f'카메라 전환 거절: {reason or f"HTTP {status}"}'
