#!/usr/bin/env python3
"""The console's command buttons from a terminal, for runs without the web console (웹 없는 트랙, 2026-10-15).

Posts to the operations API on this PC, the same one the console uses:

    operator_command.py --port <api_port> --operator <id> DECLARE_DISASTER
    operator_command.py --port <api_port> --operator <id> REQUEST_RECHECK --target-id <uuid>
    operator_command.py --port <api_port> --operator <id> DISPATCH_SUPPORT --target-id <uuid> --db <site_state.sqlite3>
    operator_command.py --port <api_port> --operator <id> CONTINUE_SEGMENT --db <site_state.sqlite3>
    operator_command.py --port <api_port> --operator <id> RESET_RUN [--note <text>]

The kinds and their fields are command_api.COMMAND_FIELDS, plus RESET_RUN (POST /api/runs/reset). Each run
makes a new command_id, so running it again is a new command, not a resend. The answer JSON is printed as it
came. Exit code: 0 accepted, 1 refused, 2 not sent (bad arguments or a value the DB does not have), 3 no
answer (operations or the coordinator did not answer; the command may or may not have been taken).

Revisions are never typed by hand (a wrong one is easy to type and the coordinator would refuse it or, worse,
take an old one). They are read from the operations DB, read-only: DISPATCH_SUPPORT's target_revision from the
target's row, CONTINUE_SEGMENT's target, revisions and segment from the mission's dispatch. The coordinator
still judges them (TARGET_REVISION_STALE and the like).
Standard library and this checkout's rescue_operations only; no ROS.
"""
import argparse
import json
from pathlib import Path
import sys
import urllib.error
import urllib.request
import uuid

REPO = Path(__file__).resolve().parents[3]
try:
    import rescue_operations  # noqa: F401  (installed or on PYTHONPATH)
except ImportError:  # run straight from the checkout
    for package in ('rescue_contracts', 'rescue_operations'):
        sys.path.insert(0, str(REPO / 'src' / package))

from rescue_operations.command_api import COMMAND_FIELDS, COMMAND_PATH, RESET_PATH  # noqa: E402
from rescue_operations.schema import OperationsError, uid  # noqa: E402
from rescue_operations.site_state import read_mission_state, read_target_states  # noqa: E402

KINDS = (*COMMAND_FIELDS, 'RESET_RUN')
FROM_DB = ('target_revision', 'route_revision', 'segment_index')  # fields this tool reads, never takes
TIMEOUT_S = 10.0  # longer than the API's own coordinator wait (operations.launch command_timeout_s)
ACCEPTED, REFUSED, NOT_SENT, NO_ANSWER = 0, 1, 2, 3


class NotSent(Exception):
    """The command was not sent; the message says why."""


def _read(reader, db):
    try:
        return reader(db)
    except OperationsError as exc:
        raise NotSent(f'운영 DB 를 읽지 못함 ({db}): {exc.detail}') from exc


def _from_db(kind, target_id, db):
    """The revision fields of ``kind`` as the DB has them now."""
    if db is None:
        raise NotSent(f'{kind} 는 revision 을 운영 DB 에서 읽는다: --db <site_state.sqlite3> 가 필요하다')
    if kind == 'DISPATCH_SUPPORT':
        rows = [t for t in _read(read_target_states, db)['targets'] if t['target_id'] == target_id]
        if not rows or rows[0]['target_revision'] is None:
            raise NotSent(f'대상 {target_id} 의 target_revision 이 DB 에 없다')
        return {'target_revision': rows[0]['target_revision']}
    if kind == 'CONTINUE_SEGMENT':
        mission = _read(read_mission_state, db)['mission'] or {}
        values = {key: mission.get(key) for key in ('target_id', *FROM_DB)}
        if None in values.values():
            raise NotSent(f'임무에 지원 출동 구간이 없다: {values}')
        return values
    raise NotSent(f'{kind} 의 revision 을 읽는 법을 모른다')


def build(kind, *, operator_id, target_id=None, db=None, note=None):
    """(path, body) for one command. Raises NotSent."""
    body = {'command_id': str(uuid.uuid4()), 'operator_id': operator_id}
    if kind == 'RESET_RUN':
        if target_id is not None:
            raise NotSent('RESET_RUN 은 --target-id 를 받지 않는다')
        return RESET_PATH, body | ({} if note is None else {'note': note})
    if note is not None:
        raise NotSent('--note 는 RESET_RUN 에만 쓴다')
    fields = COMMAND_FIELDS[kind]
    body['kind'] = kind
    if any(name in FROM_DB for name in fields):
        filled = _from_db(kind, target_id, db)
        if 'target_id' in filled and target_id is not None:
            raise NotSent(f'{kind} 의 대상은 임무에서 읽는다: --target-id 를 빼라')
        target_id = filled.pop('target_id', target_id)
        body |= filled
    if ('target_id' in fields) != (target_id is not None):
        raise NotSent(f'{kind} 는 --target-id 를 ' + ('받아야 한다' if 'target_id' in fields else '받지 않는다'))
    if target_id is not None:
        try:
            body['target_id'] = uid(target_id, 'target_id')
        except OperationsError as exc:
            raise NotSent(exc.detail) from exc
    return path_for(kind), body


def path_for(kind):
    return RESET_PATH if kind == 'RESET_RUN' else COMMAND_PATH


def post(host, port, path, body):
    """(HTTP status, answer JSON). Raises OSError when nothing answered."""
    request = urllib.request.Request(f'http://{host}:{port}{path}', data=json.dumps(body).encode(), method='POST',
                                     headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=TIMEOUT_S) as answer:
            return answer.status, json.loads(answer.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read() or b'null')


def main(argv=None):
    parser = argparse.ArgumentParser(description='관제 웹 버튼 대신 운영 API 로 운영자 명령을 보낸다')
    parser.add_argument('kind', choices=KINDS)
    parser.add_argument('--port', type=int, required=True, help='운영 API 포트 (operations.launch api_port)')
    parser.add_argument('--host', default='127.0.0.1', help='운영 API 주소 (기본 127.0.0.1)')
    parser.add_argument('--operator', required=True, help='operator_id')
    parser.add_argument('--target-id', help='대상 UUID (DISPATCH_SUPPORT, REQUEST_RECHECK, SET_CAMERA_WATCH)')
    parser.add_argument('--db', type=Path, help='운영 DB site_state.sqlite3 (DISPATCH_SUPPORT, CONTINUE_SEGMENT)')
    parser.add_argument('--note', help='RESET_RUN 의 메모')
    args = parser.parse_args(argv)
    try:
        path, body = build(args.kind, operator_id=args.operator, target_id=args.target_id, db=args.db,
                           note=args.note)
    except NotSent as exc:
        print(f'보내지 않음: {exc}', file=sys.stderr)
        return NOT_SENT
    print(f'보냄 {path}: {json.dumps(body, ensure_ascii=False)}', file=sys.stderr)
    try:
        status, answer = post(args.host, args.port, path, body)
    except (OSError, ValueError) as exc:
        print(f'답 없음 (명령이 들어갔는지 모름): {exc}', file=sys.stderr)
        return NO_ANSWER
    print(json.dumps(answer, ensure_ascii=False))
    if status in (503, 504) or not isinstance(answer, dict) or 'accepted' not in answer:
        return NO_ANSWER
    return ACCEPTED if status == 200 and answer['accepted'] is True else REFUSED


if __name__ == '__main__':
    raise SystemExit(main())
