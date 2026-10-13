"""관제 웹 로그인 계정 관리: 웹 콘솔 접근 권한 및 역할별 작업 범위를 정의한다.

역할은 2단계다. ADMIN은 전체 데이터를 열람하고 운영자 명령 권한을 가지며, CUSTOMER 계정을 생성·수정·
삭제한다. CUSTOMER는 관제 웹 화면 열람만 가능하다. 계정 저장소는 관제 웹 전용 독립
SQLite 파일이다(운영 원장이 *아님*: 현장 상태 단일 기록자 원칙을
침해하지 않는다). 비밀번호는 솔트가 적용된 PBKDF2-HMAC-SHA256 해시로만 보관하며,
평문 비밀번호는 저장하거나 로그에 기록하지 않는다.

초기 계정 생성(활성 ADMIN 1명, 비밀번호는 환경 변수 또는 대화형 프롬프트로 입력):

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
# 기본 형식만 검증한다('@' 1개 포함 및 호스트 도메인 점 여부). 실제로 메일을 발송하지는 않는다.
EMAIL = re.compile(r'[^@\s]+@[^@\s]+\.[^@\s]+')
MIN_PASSWORD = 8
# `metadata`에 기록할 식별 값. open()은 이 쌍이 없는 SQLite 파일(예: 운영 원장)을 모두 거부한다.
_ROLE_KEY, _ROLE = 'role', 'console-accounts'
_SCHEMA = '1.0'
# 신규 해시 생성 시 적용할 PBKDF2 반복 횟수(src/rescue_console/README.md: 200,000). 저장된 해시 문자열에 반복 횟수가
# 포함되므로 향후 기본값을 올려도 기존 비밀번호 검증이 호환된다. 1회 연산 비용이 높아 무차별 대입 공격을 지연시킨다.
_ITERATIONS = 200_000
# 외부로 노출 가능한 공개 필드 목록. password_hash는 외부에 절대 반환하지 않는다.
_PUBLIC = ('account_id', 'username', 'display_name', 'email', 'role', 'status', 'auth_version',
           'created_at_ns', 'updated_at_ns')
# update() 호출 시 필드 유지 표식: '인자 미제공'을 의미하며 빈 문자열('')을 포함한 실제 값과 구별된다.
_KEEP = object()


def _require(ok, message, code='INVALID_INPUT'):
    if not ok:
        raise ConsoleError(code, message)


# _dummy_hash() 캐시: import 시점이 아닌 최초 사용 시 생성하므로 모듈 import 부하를 유발하지 않는다.
_DUMMY = []


def _dummy_hash():
    """무작위 비밀번호의 사전 계산 해시. 1회만 생성하며 미등록 계정 로그인 시 타이밍 공격 방지용으로 대조한다."""
    if not _DUMMY:
        _DUMMY.append(hash_password(secrets.token_urlsafe(16)))
    return _DUMMY[0]


def hash_password(password):
    """`password`를 솔트 기반 PBKDF2-HMAC-SHA256(`pbkdf2_sha256$<iterations>$<salt hex>$<digest hex>`)로 해시한다.

    MIN_PASSWORD 자 이상의 문자열이 아니면 ConsoleError INVALID_INPUT 예외를 발생시킨다.
    호출마다 16바이트 무작위 솔트를 새로 생성하므로 동일한 비밀번호라도 계정마다 해시 결과가 다르다.
    """
    _require(isinstance(password, str) and len(password) >= MIN_PASSWORD,
             f'password must be at least {MIN_PASSWORD} characters')
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, _ITERATIONS)
    return f'pbkdf2_sha256${_ITERATIONS}${salt.hex()}${digest.hex()}'


def verify_password(stored, password):
    """`password` 해시가 `stored`(hash_password 형식)의 다이제스트와 일치하면 True, 아니면 False 반환.

    솔트 및 반복 횟수는 현재 상수가 아니라 `stored` 문자열에 기록된 값을 파싱해 사용한다. 포맷이 비정상이거나
    지원하지 않는 방식(scheme)이거나 문자열이 아닌 경우 예외 없이 False를 반환한다.
    """
    try:
        scheme, iterations, salt_hex, digest_hex = stored.split('$')
        if scheme != 'pbkdf2_sha256' or not isinstance(password, str):
            return False
        digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), bytes.fromhex(salt_hex), int(iterations))
        # 상수 시간 비교: 다이제스트 일치 길이에 따른 응답 시간 차이로 정보가 누출되는 타이밍 공격을 방어한다.
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
    """단일 SQLite 계정 저장소. 모든 공개 메서드는 사전 유효성 검증 후 단일 트랜잭션 내에서 실행된다."""

    def __init__(self, path):
        self.path = Path(path)

    # ---- 만들기·열기 ---------------------------------------------------------------------
    @classmethod
    def create(cls, path, admin_username, admin_password):
        """메타데이터 및 ACTIVE ADMIN 1명을 포함하는 신규 계정 DB를 생성하고 저장소 인스턴스를 반환한다.

        이미 파일이 존재하면 ALREADY_EXISTS 예외를 낸다. 따라서 `init`을 재실행해도 기존 계정을 덮어쓰지 않는다.
        계정명이나 비밀번호가 유효하지 않으면 파일 생성 전에 INVALID_INPUT, sqlite 오류는 STORAGE_ERROR를 낸다.
        """
        path = Path(path)
        _require(not path.exists(), f'{path.name} already exists; refusing to overwrite', 'ALREADY_EXISTS')
        admin_username = _username(admin_username)
        password_hash = hash_password(admin_password)
        try:
            db = sqlite3.connect(path)
            with db:
                db.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
                # account_id: 계정을 지우면 함께 사라진다 — 같은 이름으로 다시 만들면 새 UUID 를 받는다.
                # auth_version: 비밀번호·역할·상태가 변경될 때마다 1씩 증가한다. 따라서 변경 이전에 발급된
                # 기존 세션은 즉시 무효화된다(세션 쿠키가 account_id와 auth_version을 함께 검증함).
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
        """기존 계정 데이터베이스 파일을 검증한 후 연다. 신규 파일을 자동 생성하지 않는다.

        파일이 없으면 STORAGE_ERROR. 유효한 SQLite 파일이 아니면 트랜잭션 시작 시 STORAGE_ERROR가 발생한다.
        `metadata` 테이블을 읽을 수 없거나 role/schema_version이 불일치하면 POLICY_MISMATCH 예외를 낸다.
        """
        store = cls(path)
        # 사전 확인 필수: sqlite3.connect는 경로 오타 시 빈 데이터베이스 파일을 자동으로 생성해 버린다.
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
        """_Transaction으로 래핑된 새 DB 연결을 생성한다(`with` 블록 전용). 연결 실패 시 STORAGE_ERROR.

        호출마다 독립된 새 연결을 사용하므로 요청 스레드 간 연결 경합이 없다. isolation_level=None으로 설정해
        sqlite3 기본 암묵적 트랜잭션을 끄고, _Transaction의 명시적 BEGIN IMMEDIATE / COMMIT만 동작하게 한다.
        조회 행은 열 이름 접근이 가능한 sqlite3.Row 객체로 반환된다.
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
        """세션 쿠키가 지시하는 행을 조회한다. 동일 사용자명으로 재생성되어도 UUID가 달라 불일치 처리된다."""
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE account_id = ?', (account_id,)).fetchone()
        return _public(row) if row else None

    def authenticate(self, username, password):
        """ACTIVE 계정의 비밀번호가 일치하면 해당 계정의 공개 행 딕셔너리를 반환하고 불일치 시 None을 반환한다.

        존재하지 않는 계정, 비밀번호 불일치, 비활성화 계정 모두 동일한 None을 반환해 계정 존재 여부 유출을 차단한다.
        실패 사유는 관제 웹 로그인 화면에 일괄 처리되며 서버 로그에도 구분하여 남기지 않는다.
        """
        if not isinstance(username, str) or not isinstance(password, str):
            return None
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE username = ?', (username,)).fetchone()
        # 계정 존재 여부와 무관하게 항상 해시 검증을 1회 수행하여 응답 타이밍 차이로 계정을 추측하지 못하게 한다.
        matches = verify_password(_dummy_hash() if row is None else row['password_hash'], password)
        if row is None or row['status'] != 'ACTIVE' or not matches:
            return None
        return _public(row)

    # ---- 쓰기(ADMIN 전용. 호출 권한 인가는 상위 웹 애플리케이션 계층에서 통제함) -----------------
    def add(self, username, password, *, role, display_name='', email='', status='ACTIVE'):
        """새 계정을 등록하고 생성된 공개 행 딕셔너리를 반환한다.

        모든 필드 유효성 검증과 PBKDF2 해시 연산을 트랜잭션 개시 전에 완료하여 쓰기 락 점유 시간을 최소화한다.
        중복 검사와 삽입 동안만 잠금을 유지한다. 필드 오류 시 INVALID_INPUT, 중복 시 ALREADY_EXISTS를 낸다.
        display_name이 비어 있으면 username으로 대체되며 신규 계정은 auth_version 1로 시작한다.
        """
        username = _username(username)
        record = (username, str(uuid.uuid4()), _text(display_name, 'display_name') or username, _email(email),
                  _choice(role, ACCOUNT_ROLES, 'role'), _choice(status, ACCOUNT_STATUSES, 'status'),
                  hash_password(password), 1)
        with self._connect() as db:
            # BEGIN IMMEDIATE 내에서 검증하여 검증과 INSERT 사이의 경쟁 상태(Race Condition)를 방지한다.
            _require(db.execute('SELECT 1 FROM accounts WHERE username = ?', (username,)).fetchone() is None,
                     f'account {username} already exists', 'ALREADY_EXISTS')
            now = time.time_ns()
            db.execute('INSERT INTO accounts VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)', record + (now, now))
        return self.get(username)

    def update(self, username, *, display_name=_KEEP, email=_KEEP, role=_KEEP, status=_KEEP, password=None):
        """계정의 지정된 필드를 수정하고 갱신된 공개 행 딕셔너리를 반환한다.

        명시적으로 전달된 인자만 수정한다(_KEEP은 유지, password=None은 기존 비밀번호 유지).
        INVALID_INPUT(유효하지 않은 값 또는 변경 사항 없음), NOT_FOUND, 또는 수정 후 ACTIVE ADMIN이
        남지 않게 되면 LAST_ADMIN 예외를 낸다. 비밀번호·역할·상태 변경 시 auth_version을 1 증가시켜
        기존 세션을 모두 만료시킨다(세션은 account_id와 auth_version을 동시에 검증한다).
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
            # 유일한 ACTIVE ADMIN을 강등하거나 비활성화하는 경우에만 마지막 관리자 박탈 위험이 발생한다.
            if (row['role'], row['status']) == ('ADMIN', 'ACTIVE') and (after_role, after_status) != ('ADMIN', 'ACTIVE'):
                self._guard_last_admin(db, username)
            if 'display_name' in changes and changes['display_name'] == '':
                changes['display_name'] = username
            # 저장된 기존 값과 비교한다: 동일한 역할이나 상태를 재지정해도 불필요하게 세션을 무효화하지 않는다.
            credential_changed = ('password_hash' in changes or after_role != row['role'] or after_status != row['status'])
            if credential_changed:
                changes['auth_version'] = row['auth_version'] + 1   # 변경 전 발급된 모든 세션을 무효화한다
            changes['updated_at_ns'] = time.time_ns()
            # 열 이름은 내부에서 정의한 안전한 키만 허용하며, 모든 값은 파라미터 바인딩으로 바인딩한다.
            assignments = ', '.join(f'{key} = ?' for key in changes)
            db.execute(f'UPDATE accounts SET {assignments} WHERE username = ?', (*changes.values(), username))
        return self.get(username)

    def delete(self, username, *, by):
        """계정을 삭제한다. `by`는 현재 삭제 요청을 수행 중인 로그인된 ADMIN 계정의 username이다.

        `by`가 삭제 대상 계정 본인이면 SELF_DELETE, 대상이 없으면 NOT_FOUND, 유일한 ACTIVE ADMIN이면
        LAST_ADMIN 예외를 낸다. 향후 동일 이름으로 계정이 재생성되어도 UUID가 달라 이전 세션과 결합할 수 없다.
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
        # 호출 측의 동일한 BEGIN IMMEDIATE 트랜잭션 내에서 실행되므로 카운트 시점과 쓰기 시점의 일관성이 보장된다.
        others = db.execute("SELECT COUNT(*) FROM accounts WHERE role = 'ADMIN' AND status = 'ACTIVE' AND username != ?",
                            (excluding,)).fetchone()[0]
        _require(others > 0, 'at least one ACTIVE ADMIN must remain', 'LAST_ADMIN')


class _Transaction:
    """코드 블록을 BEGIN IMMEDIATE ... COMMIT/ROLLBACK으로 감싸고 종료 시 연결을 해제한다.

    BEGIN 또는 COMMIT 실패 시 STORAGE_ERROR를 발생시킨다. 블록 내부 발생 예외 처리는 `__exit__`를 참조한다.
    """

    def __init__(self, db):
        self.db = db

    def __enter__(self):
        """트랜잭션을 시작하고 열린 DB 연결 객체를 컨텍스트 블록에 전달한다.

        IMMEDIATE 모드는 BEGIN 시점에 배타적 쓰기 락을 확보하므로 블록 내부의 검증-후-쓰기(중복 계정,
        마지막 관리자 검사)가 동시 쓰기 요청과 충돌하지 않는다. 두 번째 접근은 sqlite3 busy
        timeout까지 대기한다. BEGIN 실패 시(busy 타임아웃 또는 비정상 DB 파일) 연결을 닫고 STORAGE_ERROR를 낸다.
        """
        try:
            self.db.execute('BEGIN IMMEDIATE')
        except sqlite3.Error as exc:
            self.db.close()
            raise ConsoleError('STORAGE_ERROR', f'accounts database busy or broken: {exc}') from exc
        return self.db

    def __exit__(self, exc_type, exc, tb):
        """블록이 정상 종료되면 COMMIT, 예외 발생 시 ROLLBACK을 수행하며 항상 연결을 닫는다.

        COMMIT 실패 시 STORAGE_ERROR를 발생시킨다. ROLLBACK 실패는 무시하여 블록 본래의 원인 예외를 보존한다.
        타입이 정확히 sqlite3.Error인 예외(`is` 비교)만 STORAGE_ERROR로 변환하여
        재발생시킨다. 그 외의 예외(sqlite3.OperationalError 등 sqlite3 하위 클래스 및
        블록 내부의 ConsoleError 포함)는 변환하지 않고 그대로 전파한다(False 반환).
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
    """`init` CLI 명령: ACTIVE ADMIN 1명이 포함된 계정 DB를 생성하고 프로세스 종료 코드를 반환한다.

    비밀번호는 --password-env로 지정된 환경 변수에서 읽고, 미설정 시 대화형 프롬프트로 입력받아
    명령행 인자에 평문 비밀번호가 노출되지 않도록 보호한다. 정상 시 0, 오류 시 `ACCOUNTS FAILED: ...` 출력 후 2 반환.
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
