"""관제 웹 로그인 정보: 누가 관제 웹을 열 수 있고 그 안에서 무엇을 할 수 있는지를 다룬다.

역할은 둘이다. ADMIN 은 모든 것을 보고, 운영자 명령을 맡게 되며, CUSTOMER 계정을 만들고·고치고·
지운다. CUSTOMER 는 관제 웹을 보기만 한다. 계정 저장소는 관제 웹이 소유한 작은
SQLite 파일이다(운영 장부가 *아니다*: 현장 상태는 한 곳만 쓴다는 단일 기록자 규칙을
건드리지 않는다). 비밀번호는 소금을 친 PBKDF2-HMAC-SHA256 해시로만 두고,
평문 비밀번호는 저장하지도 로그에 남기지도 않는다.

처음 만들기(활성 ADMIN 한 명, 비밀번호는 환경 변수나 입력 프롬프트에서 받는다):

    RESCUE_CONSOLE_ADMIN_PASSWORD=... python3 -m rescue_console.accounts init <ACCOUNTS.sqlite3> --admin <name>
"""
import argparse
import getpass
import hashlib
import hmac
import os
import re
import secrets
import sqlite3
import sys
import time
import uuid
from pathlib import Path

from .view import ConsoleError

ACCOUNT_ROLES = ('ADMIN', 'CUSTOMER')
ACCOUNT_STATUSES = ('ACTIVE', 'INACTIVE')
USERNAME = re.compile(r'[a-z][a-z0-9_.-]{2,31}')
# 모양만 본다('@' 가 하나, 호스트 부분에 점이 있는지). 이 주소로 메일을 보내는 일은 없다.
EMAIL = re.compile(r'[^@\s]+@[^@\s]+\.[^@\s]+')
MIN_PASSWORD = 8
# `metadata` 에 써 둔다. open() 은 이 쌍이 없는 SQLite 파일(예: 운영 장부)을 모두 거절한다.
_ROLE_KEY, _ROLE = 'role', 'console-accounts'
_SCHEMA = '1.0'
# 새 해시의 PBKDF2 반복 수(src/rescue_console/README.md: 200,000). 저장된 해시마다 자기 반복 수를
# 담으므로 나중에 이 값을 올려도 예전 비밀번호는 그대로 맞는다. 한 번 대입하는 비용이 커서 비밀번호 추측도 느려진다.
_ITERATIONS = 200_000
# 저장소 밖으로 나가는 열은 이것뿐이다. password_hash 는 호출한 쪽에 절대 돌려주지 않는다.
_PUBLIC = ('account_id', 'username', 'display_name', 'email', 'role', 'status', 'auth_version',
           'created_at_ns', 'updated_at_ns')
# update() 용 표지값: 「인자를 주지 않음」을 뜻하며, '' 를 포함한 어떤 실제 값과도 다르다.
_KEEP = object()


def _require(ok, message, code='INVALID_INPUT'):
    if not ok:
        raise ConsoleError(code, message)


# _dummy_hash() 의 캐시: import 때가 아니라 처음 쓸 때 채우므로, 이 모듈을 import 할 때 해시 비용이 들지 않는다.
_DUMMY = []


def _dummy_hash():
    """무작위 비밀의 진짜 해시. 한 번만 만들고, 아이디가 없을 때 이것과 대조해 확인한다."""
    if not _DUMMY:
        _DUMMY.append(hash_password(secrets.token_urlsafe(16)))
    return _DUMMY[0]


def hash_password(password):
    """`password` 의 소금 친 PBKDF2-HMAC-SHA256 을 `pbkdf2_sha256$<iterations>$<salt hex>$<digest hex>` 로 돌려준다.

    MIN_PASSWORD 자 이상의 문자열이 아니면 ConsoleError INVALID_INPUT 을 낸다.
    호출마다 16 byte 무작위 소금을 새로 만들므로, 같은 비밀번호를 쓰는 두 계정도 저장값이 같지 않다.
    """
    _require(isinstance(password, str) and len(password) >= MIN_PASSWORD,
             f'password must be at least {MIN_PASSWORD} characters')
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, _ITERATIONS)
    return f'pbkdf2_sha256${_ITERATIONS}${salt.hex()}${digest.hex()}'


def verify_password(stored, password):
    """`password` 의 해시가 `stored`(hash_password() 문자열)의 digest 와 같으면 True, 아니면 False.

    소금과 반복 수는 지금 상수가 아니라 `stored` 에서 가져온다. 저장값이 형식이 틀렸거나
    다른 방식(scheme)이거나 문자열이 아니면 예외를 내지 않고 False 를 돌려준다.
    """
    try:
        scheme, iterations, salt_hex, digest_hex = stored.split('$')
        if scheme != 'pbkdf2_sha256' or not isinstance(password, str):
            return False
        digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), bytes.fromhex(salt_hex), int(iterations))
        # 상수 시간 비교: 응답 시간으로 digest 가 얼마나 맞았는지 드러나지 않는다.
        return hmac.compare_digest(digest.hex(), digest_hex)
    except (ValueError, AttributeError):
        return False


def _username(value):
    _require(isinstance(value, str) and USERNAME.fullmatch(value) is not None,
             'username: 3-32 chars, lowercase letter first, then [a-z0-9_.-]')
    return value


def _text(value, label, maximum=128):
    _require(isinstance(value, str) and len(value) <= maximum and '\n' not in value, f'{label}: text up to {maximum}')
    return value


def _email(value):
    _text(value, 'email')
    _require(value == '' or EMAIL.fullmatch(value) is not None, 'email: name@host.tld or empty')
    return value


def _choice(value, choices, label):
    _require(isinstance(value, str) and value in choices, f'{label} must be one of {list(choices)}')
    return value


def _public(row):
    return {key: row[key] for key in _PUBLIC}


class AccountStore:
    """SQLite 파일 하나. 모든 공개 메서드는 먼저 검사하고 한 트랜잭션 안에서 쓴다."""

    def __init__(self, path):
        self.path = Path(path)

    # ---- 만들기·열기 ---------------------------------------------------------------------
    @classmethod
    def create(cls, path, admin_username, admin_password):
        """표 두 개와 ACTIVE ADMIN 한 명이 든 새 계정 파일을 만들고 저장소 객체를 돌려준다.

        이미 있는 경로는 거절한다(ALREADY_EXISTS). 그래서 `init` 을 두 번 돌려도 쓰고 있는 계정을 덮어쓰지 않는다.
        아이디나 비밀번호가 틀리면 파일을 만들기 전에 INVALID_INPUT 을 내고, sqlite 오류는 STORAGE_ERROR 를 낸다.
        """
        path = Path(path)
        _require(not path.exists(), f'{path.name} already exists; refusing to overwrite', 'ALREADY_EXISTS')
        admin_username = _username(admin_username)
        password_hash = hash_password(admin_password)
        try:
            db = sqlite3.connect(path)
            with db:
                db.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
                # account_id: 어디에도 이어지지 않는다 — 지웠다가 다시 만든 같은 이름은 새 id 를 받는다.
                # auth_version: 비밀번호·역할·상태가 바뀔 때마다 올린다. 그래서 바뀌기 전에 발급한
                # 세션은 모두 더는 맞지 않는다(세션이 두 값을 모두 들고 다닌다).
                db.execute('''CREATE TABLE accounts (
                    username TEXT PRIMARY KEY, account_id TEXT NOT NULL UNIQUE,
                    display_name TEXT NOT NULL, email TEXT NOT NULL,
                    role TEXT NOT NULL, status TEXT NOT NULL, password_hash TEXT NOT NULL,
                    auth_version INTEGER NOT NULL,
                    created_at_ns INTEGER NOT NULL, updated_at_ns INTEGER NOT NULL)''')
                db.executemany('INSERT INTO metadata VALUES (?, ?)', [(_ROLE_KEY, _ROLE), ('schema_version', _SCHEMA)])
                now = time.time_ns()
                db.execute('INSERT INTO accounts VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)',
                           (admin_username, str(uuid.uuid4()), admin_username, '', 'ADMIN', 'ACTIVE', password_hash,
                            1, now, now))
            db.close()
        except sqlite3.Error as exc:
            raise ConsoleError('STORAGE_ERROR', f'cannot create accounts database: {exc}') from exc
        return cls(path)

    @classmethod
    def open(cls, path):
        """이미 있는 계정 파일을, 계정 파일이 맞는지 확인한 뒤 연다. 파일을 새로 만들지 않는다.

        파일이 없으면 STORAGE_ERROR. SQLite 가 아예 아닌 파일은 BEGIN 에서 실패하며 이것도 STORAGE_ERROR 다.
        `metadata` 를 읽을 수 없거나 role / schema_version 이 이 모듈의 값과 다르면 POLICY_MISMATCH.
        """
        store = cls(path)
        # 먼저 확인한다: sqlite3.connect 는 잘못 친 경로에 빈 파일을 조용히 만들어 버린다.
        _require(store.path.exists(), f'accounts database not found: {store.path.name}', 'STORAGE_ERROR')
        with store._connect() as db:
            try:
                rows = dict(db.execute('SELECT key, value FROM metadata').fetchall())
            except sqlite3.Error as exc:
                raise ConsoleError('POLICY_MISMATCH', 'not a console accounts database') from exc
            _require(rows.get(_ROLE_KEY) == _ROLE and rows.get('schema_version') == _SCHEMA,
                     'not a console accounts database (role/schema mismatch)', 'POLICY_MISMATCH')
        return store

    def _connect(self):
        """_Transaction 으로 감싼 새 연결. `with` 블록 하나에 쓴다. 열 수 없으면 STORAGE_ERROR.

        호출마다 새 연결이라 요청 스레드끼리 연결을 나눠 쓰지 않는다. isolation_level=None 은 sqlite3 모듈의
        암묵적 트랜잭션을 끄므로, 트랜잭션은 _Transaction 의 BEGIN IMMEDIATE / COMMIT 뿐이다.
        행은 sqlite3.Row 로 돌아오고 열 이름으로 읽는다.
        """
        try:
            db = sqlite3.connect(self.path, isolation_level=None)
        except sqlite3.Error as exc:
            raise ConsoleError('STORAGE_ERROR', f'cannot open accounts database: {exc}') from exc
        db.row_factory = sqlite3.Row
        return _Transaction(db)

    # ---- 읽기 ---------------------------------------------------------------------------
    def list_accounts(self):
        with self._connect() as db:
            return [_public(row) for row in db.execute('SELECT * FROM accounts ORDER BY username')]

    def get(self, username):
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE username = ?', (username,)).fetchone()
        return _public(row) if row else None

    def get_by_id(self, account_id):
        """세션이 가리키는 행. 다시 쓴 아이디는 id 가 달라서 맞는 일이 없다."""
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE account_id = ?', (account_id,)).fetchone()
        return _public(row) if row else None

    def authenticate(self, username, password):
        """ACTIVE 계정에 비밀번호가 맞으면 그 계정의 공개 행, 아니면 None.

        없는 아이디, 틀린 비밀번호, 비활성 계정은 호출한 쪽이 일부러 구별할 수 없게 한다
        (로그인 화면은 같은 말 하나만 한다). 사유는 로그에도 남기지 않는다.
        """
        if not isinstance(username, str) or not isinstance(password, str):
            return None
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE username = ?', (username,)).fetchone()
        # 어느 아이디든 비밀번호 확인을 한 번 하므로, 응답 시간으로 어떤 아이디가 있는지 알 수 없다.
        matches = verify_password(_dummy_hash() if row is None else row['password_hash'], password)
        if row is None or row['status'] != 'ACTIVE' or not matches:
            return None
        return _public(row)

    # ---- 쓰기(ADMIN 만. 누가 부르는지는 웹 계층이 막는다) --------------------------------
    def add(self, username, password, *, role, display_name='', email='', status='ACTIVE'):
        """새 계정을 넣고 그 공개 행을 돌려준다.

        모든 필드 검사와 느린 비밀번호 해시를 쓰기 트랜잭션을 열기 전에 끝내므로,
        잠금은 중복 확인과 삽입 동안만 잡는다. 필드가 틀리면 INVALID_INPUT, 이미 쓰는 아이디면 ALREADY_EXISTS 를
        낸다. 빈 display_name 은 아이디로 채우고, 새 계정은 auth_version 1 에서 시작한다.
        """
        username = _username(username)
        record = (username, str(uuid.uuid4()), _text(display_name, 'display_name') or username, _email(email),
                  _choice(role, ACCOUNT_ROLES, 'role'), _choice(status, ACCOUNT_STATUSES, 'status'),
                  hash_password(password), 1)
        with self._connect() as db:
            # BEGIN IMMEDIATE 안에서 확인한다: 확인과 삽입 사이에 다른 쓰기 쪽이 같은 아이디를 넣을 수 없다.
            _require(db.execute('SELECT 1 FROM accounts WHERE username = ?', (username,)).fetchone() is None,
                     f'account {username} already exists', 'ALREADY_EXISTS')
            now = time.time_ns()
            db.execute('INSERT INTO accounts VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)', record + (now, now))
        return self.get(username)

    def update(self, username, *, display_name=_KEEP, email=_KEEP, role=_KEEP, status=_KEEP, password=None):
        """계정 하나에서 주어진 필드를 바꾸고 그 공개 행을 돌려준다.

        실제로 넘긴 인자만 바꾼다(_KEEP = 주지 않음, password=None 은 비밀번호를 그대로 둔다).
        INVALID_INPUT(틀린 값, 또는 바꿀 것이 없음), NOT_FOUND, 또는 바꾼 뒤 ACTIVE ADMIN 이 하나도
        남지 않으면 LAST_ADMIN 을 낸다. 비밀번호·역할·상태가 실제로 바뀌면 auth_version 을 하나 올리고,
        그 전에 발급한 세션은 모두 끝난다(세션은 account_id 와 auth_version 을 들고 다닌다).
        """
        changes = {}
        if display_name is not _KEEP:
            changes['display_name'] = _text(display_name, 'display_name')
        if email is not _KEEP:
            changes['email'] = _email(email)
        if role is not _KEEP:
            changes['role'] = _choice(role, ACCOUNT_ROLES, 'role')
        if status is not _KEEP:
            changes['status'] = _choice(status, ACCOUNT_STATUSES, 'status')
        if password is not None:
            changes['password_hash'] = hash_password(password)
        _require(changes, 'nothing to update')
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE username = ?', (username,)).fetchone()
            _require(row is not None, f'account {username!r} not found', 'NOT_FOUND')
            after_role = changes.get('role', row['role'])
            after_status = changes.get('status', row['status'])
            # 마지막 ADMIN 을 없앨 수 있는 것은 ACTIVE ADMIN 을 강등하거나 비활성으로 바꾸는 경우뿐이다.
            if (row['role'], row['status']) == ('ADMIN', 'ACTIVE') and (after_role, after_status) != ('ADMIN', 'ACTIVE'):
                self._guard_last_admin(db, username)
            if 'display_name' in changes and changes['display_name'] == '':
                changes['display_name'] = username
            # 저장된 행과 비교한다: 같은 역할이나 상태를 다시 보내도 어떤 세션도 끝나지 않는다.
            credential_changed = ('password_hash' in changes or after_role != row['role'] or after_status != row['status'])
            if credential_changed:
                changes['auth_version'] = row['auth_version'] + 1   # 이전 세션은 모두 더는 맞지 않는다
            changes['updated_at_ns'] = time.time_ns()
            # 열 이름은 위에서 정한 고정 키에서만 나오고, 값은 모두 바인딩 매개변수로 넘긴다.
            assignments = ', '.join(f'{key} = ?' for key in changes)
            db.execute(f'UPDATE accounts SET {assignments} WHERE username = ?', (*changes.values(), username))
        return self.get(username)

    def delete(self, username, *, by):
        """계정 하나를 지운다. `by` 는 요청을 보낸, 로그인한 ADMIN 의 아이디다.

        `by` 가 지우려는 계정 자신이면 SELF_DELETE, 없으면 NOT_FOUND, 마지막 ACTIVE ADMIN 이면
        LAST_ADMIN 을 낸다. 나중에 같은 이름으로 만든 계정은 새 account_id 를 받아 이 계정의 세션과 맞지 않는다.
        """
        _require(username != by, 'an admin cannot delete the account it is logged in with', 'SELF_DELETE')
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE username = ?', (username,)).fetchone()
            _require(row is not None, f'account {username!r} not found', 'NOT_FOUND')
            if (row['role'], row['status']) == ('ADMIN', 'ACTIVE'):
                self._guard_last_admin(db, username)
            db.execute('DELETE FROM accounts WHERE username = ?', (username,))

    @staticmethod
    def _guard_last_admin(db, excluding):
        # 호출한 쪽의 연결에서 그 BEGIN IMMEDIATE 안에 돌므로, 센 수가 그 쓰기까지 그대로 유지된다.
        others = db.execute("SELECT COUNT(*) FROM accounts WHERE role = 'ADMIN' AND status = 'ACTIVE' AND username != ?",
                            (excluding,)).fetchone()[0]
        _require(others > 0, 'at least one ACTIVE ADMIN must remain', 'LAST_ADMIN')


class _Transaction:
    """블록을 BEGIN IMMEDIATE ... COMMIT/ROLLBACK 으로 감싸고, 끝나면 연결을 닫는다.

    BEGIN 이나 COMMIT 이 실패하면 STORAGE_ERROR 가 된다. 블록 안에서 난 오류는 `__exit__` 를 참고한다.
    """

    def __init__(self, db):
        self.db = db

    def __enter__(self):
        """트랜잭션을 시작하고 연결을 블록에 넘긴다.

        IMMEDIATE 는 BEGIN 에서 쓰기 잠금을 잡는다(읽기 블록도). 그래서 블록 안의 확인 뒤 쓰기(아이디 중복,
        마지막 ADMIN)가 다른 쓰기 쪽과 섞이지 않는다. 두 번째 호출의 BEGIN 은 sqlite3 의 busy
        timeout 까지 기다린다. BEGIN 이 실패하면(busy, 또는 데이터베이스가 아님) 연결을 닫고 STORAGE_ERROR 를 낸다.
        """
        try:
            self.db.execute('BEGIN IMMEDIATE')
        except sqlite3.Error as exc:
            self.db.close()
            raise ConsoleError('STORAGE_ERROR', f'accounts database busy or broken: {exc}') from exc
        return self.db

    def __exit__(self, exc_type, exc, tb):
        """블록이 정상으로 끝나면 COMMIT, 예외가 나면 ROLLBACK 하고, 연결은 늘 닫는다.

        COMMIT 이 실패하면 STORAGE_ERROR 를 낸다. ROLLBACK 실패는 버리므로 블록 자신의 예외가
        그대로 보고된다. 타입이 정확히 sqlite3.Error 인 예외(`is` 비교)만 STORAGE_ERROR 로 바꿔
        다시 낸다. 그 밖의 예외는 sqlite3.OperationalError 같은 sqlite3 하위 클래스와
        블록 검사에서 난 ConsoleError 까지 포함해 그대로 전파된다(False 를 돌려준다).
        """
        try:
            if exc_type is None:
                self.db.execute('COMMIT')
            else:
                self.db.execute('ROLLBACK')
        except sqlite3.Error as error:
            if exc_type is None:
                raise ConsoleError('STORAGE_ERROR', f'accounts commit failed: {error}') from error
        finally:
            self.db.close()
        if exc_type is sqlite3.Error:
            raise ConsoleError('STORAGE_ERROR', f'accounts database error: {exc}') from exc
        return False


def main(argv=None):
    """`init` 명령: ACTIVE ADMIN 한 명이 든 계정 파일을 만들고 프로세스 종료코드를 돌려준다.

    비밀번호는 --password-env 가 가리키는 환경 변수에서 받고, 그 변수가 없거나 비었으면
    프롬프트에서 받으므로 명령행에 치는 일이 없다. 0 을 돌려주거나, `ACCOUNTS FAILED: ...` 를 찍은 뒤 2 를 돌려준다.
    """
    parser = argparse.ArgumentParser(description='Create the console accounts database with one ADMIN.')
    sub = parser.add_subparsers(dest='command', required=True)
    init = sub.add_parser('init', help='create ACCOUNTS with one active admin')
    init.add_argument('accounts', help='path of the new accounts SQLite file (must not exist)')
    init.add_argument('--admin', required=True, help='admin username')
    init.add_argument('--password-env', default='RESCUE_CONSOLE_ADMIN_PASSWORD',
                      help='environment variable holding the admin password; prompted if unset')
    args = parser.parse_args(argv)
    password = os.environ.get(args.password_env) or getpass.getpass(f'password for {args.admin}: ')
    try:
        AccountStore.create(args.accounts, args.admin, password)
    except ConsoleError as exc:
        print(f'ACCOUNTS FAILED: {exc}', file=sys.stderr)
        return 2
    print(f'created {args.accounts} with ADMIN {args.admin}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
