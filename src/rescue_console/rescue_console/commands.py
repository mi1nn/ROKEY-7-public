"""Operator commands: what the console sends to operations and how it reports the answer.

The console is a client of operations' `POST /api/operator-commands` (console-and-db-plan 4절).
It builds one request per button press, sends it once and passes the answer on as it came.
A failure is reported as a failure, never retried and never replaced by a guess (AGENTS.md 7절).
The gate that accepts or rejects a command is the coordinator's; the console writes nothing.
Standard library only.
"""

import json
import socket
import urllib.error
import urllib.parse
import urllib.request

from .labels import label
from .view import ConsoleError

COMMAND_PATH = '/api/operator-commands'
# How long the console waits for operations' answer. It must cover operations calling the
# coordinator service ([제안] 관제·화면, 2026-10-12; PR body 「열린 것」).
COMMAND_TIMEOUT_S = 5.0
# Fields each kind carries, as the coordinator reads them (rescue_coordination.mission.COMMAND_FIELDS;
# tests/test_console_commands.py compares the two). PAUSE·RESUME·CANCEL wait for the Q-22 resume gate and
# STOP_ROBOT has its own RequestStop path, so they are not sendable from here.
COMMAND_FIELDS = {
    'START_SEARCH': (),
    'DISPATCH_SUPPORT': ('target_id', 'target_revision'),
    'CONTINUE_SEGMENT': ('target_id', 'target_revision', 'route_revision', 'segment_index'),
    'REQUEST_RECHECK': ('target_id',),
    'RETURN_HOME': (),
}
SENDABLE = tuple(COMMAND_FIELDS)
# The revisions and the segment are the values the operator saw on the screen, so an approval cannot
# attach to a target or route that changed since (SubmitOperatorCommand expected_* fields). None is sent
# as unknown: the button is off instead (작전 #115, 2026-10-12).
INT_FIELDS = frozenset({'target_revision', 'route_revision', 'segment_index'})
# Form keys every kind may carry besides its own fields; anything else is refused (build_request).
FORM_KEYS = frozenset({'csrf', 'kind'})
# Longest target_id taken from a form. Only a bound: whether it is a known target ID is for operations.
MAX_ID = 128
# Delivery of a failed send: NOT_SENT when nothing reached operations, UNKNOWN when it may have.
DELIVERY = frozenset({'NOT_SENT', 'UNKNOWN'})


class CommandSendError(Exception):
    """Sending failed. `delivery` says whether operations may have received the command."""

    def __init__(self, code, message, *, delivery='UNKNOWN'):
        if delivery not in DELIVERY:
            raise ValueError(f'delivery must be one of {sorted(DELIVERY)}')
        self.code, self.message, self.delivery = code, message, delivery
        super().__init__(f'{code}: {message}')


def _invalid(message):
    return ConsoleError('INVALID_INPUT', message)


def parse_operations_url(text):
    """'http://host:port[/base]' -> the base URL without a trailing slash. No default, nothing guessed."""
    parts = urllib.parse.urlsplit(text or '')
    if parts.scheme not in ('http', 'https') or not parts.hostname or parts.query or parts.fragment:
        raise _invalid(f'operations must be http(s)://host:port : {text!r}')
    return urllib.parse.urlunsplit((parts.scheme, parts.netloc, parts.path.rstrip('/'), '', ''))


def command_access(role, connected):
    """(allowed, reason) for the logged-in role. Who may press, not whether the command is accepted."""
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
    # isdigit alone also accepts non-ASCII digits such as '²', which int() rejects; isascii keeps it to 0-9.
    # No sign and no decimal point, so a value is never negative or rounded.
    if not (isinstance(text, str) and text.isascii() and text.isdigit()):
        raise _invalid(f'{name} must be a non-negative integer')
    return int(text)


def build_request(form, *, operator_id, command_id):
    """Browser form -> the JSON body for operations. `operator_id` comes from the login session.

    Only the fields of the kind are accepted; anything else in the form is refused, so a page
    or a hand-made request cannot add a robot, a different operator or a held command.
    """
    kind = form.get('kind', '')
    if kind not in COMMAND_FIELDS:
        raise _invalid(f'command {kind!r} is not sendable from the console')
    fields = COMMAND_FIELDS[kind]
    extra = sorted(key for key in form if key not in FORM_KEYS | set(fields))
    if extra:
        raise _invalid(f'{kind} does not take {", ".join(extra)}')
    # command_id and operator_id come from the server (app.py), never from the form.
    payload = {'command_id': command_id, 'kind': kind, 'operator_id': operator_id}
    for name in fields:
        text = form.get(name, '')
        if name in INT_FIELDS:
            payload[name] = _count(name, text)
        else:
            payload[name] = _target_id(text)
    return payload


def post_json(url, payload, timeout):
    """One POST, no retry. Returns (status, body) for any HTTP answer; raises CommandSendError otherwise."""
    request = urllib.request.Request(url, data=json.dumps(payload).encode(), method='POST',
                                     headers={'Content-Type': 'application/json', 'Accept': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:          # an answer with an error status: its body may still say why
        with exc:                                  # the error is also the response: read its body, then close it
            return exc.code, exc.read()
    # No HTTP answer. Delivery is NOT_SENT only when the connection was never made (refused, or the host
    # name did not resolve); for every other failure operations may have received the request.
    except urllib.error.URLError as exc:
        if isinstance(exc.reason, (TimeoutError, socket.timeout)):
            raise CommandSendError('TIMEOUT', f'no answer within {timeout:g}s') from exc
        if isinstance(exc.reason, (ConnectionRefusedError, socket.gaierror)):
            raise CommandSendError('UNREACHABLE', str(exc.reason), delivery='NOT_SENT') from exc
        raise CommandSendError('UNREACHABLE', str(exc.reason)) from exc
    except (TimeoutError, socket.timeout) as exc:  # timed out after connecting, while waiting for the answer
        raise CommandSendError('TIMEOUT', f'no answer within {timeout:g}s') from exc
    except OSError as exc:
        raise CommandSendError('SEND_FAILED', str(exc)) from exc


def _optional_text(value):
    return value is None or (isinstance(value, str) and value != '')


def read_reply(status, body):
    """operations' answer -> {accepted, request_id, reason, request_state, created_task_id}.

    Read whatever the status code; an error status that carries an answer is still an answer.
    Anything else is a failure with unknown delivery: the request reached operations.
    """
    try:
        data = json.loads(body)
    except (TypeError, ValueError):
        data = None
    # An answer: `accepted` is a real bool (not 1/0), the other fields are absent, null or non-empty text,
    # a rejection names its reason, and nothing is accepted with a non-2xx status.
    ok = (isinstance(data, dict) and type(data.get('accepted')) is bool
          and all(_optional_text(data.get(key)) for key in ('request_id', 'reason', 'request_state', 'created_task_id'))
          and (data['accepted'] or data.get('reason') is not None)
          and not (data['accepted'] and not 200 <= status < 300))
    if not ok:
        snippet = (body or b'')[:200].decode('utf-8', 'replace')   # only the start, so the message stays short
        # A 2xx or a JSON object that is not a valid answer is BAD_RESPONSE; anything else is a plain HTTP error.
        if 200 <= status < 300 or isinstance(data, dict):
            raise CommandSendError('BAD_RESPONSE', f'HTTP {status}: not an operator-command answer: {snippet}')
        raise CommandSendError('HTTP_ERROR', f'HTTP {status}: {snippet}')
    return {key: data.get(key) for key in ('accepted', 'request_id', 'reason', 'request_state', 'created_task_id')}


def send_command(base_url, payload, *, transport=post_json, timeout=COMMAND_TIMEOUT_S):
    """Send once and say what happened. Never retries: a second press is the operator's decision."""
    head = {'command_id': payload['command_id'], 'kind': payload['kind']}
    try:
        reply = read_reply(*transport(base_url + COMMAND_PATH, payload, timeout))
    except CommandSendError as exc:
        return {'sent': False, **head, 'error': exc.code, 'message': exc.message, 'delivery': exc.delivery}
    return {'sent': True, **head, **reply, 'reason_label': label('reply', reply['reason'])}
