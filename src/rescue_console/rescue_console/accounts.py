"""Console login data: who may open the console and what they may do.

Two roles. ADMIN sees everything, will own operator commands, and creates / updates /
deletes CUSTOMER accounts. CUSTOMER only views the console. The store is a small
SQLite file owned by the console (it is *not* the operations ledger: the single-writer
rule for site state is untouched). Passwords are salted PBKDF2-HMAC-SHA256 hashes;
plain passwords are never stored or logged.

Bootstrap (one active admin, password from the environment or a prompt):

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
EMAIL = re.compile(r'[^@\s]+@[^@\s]+\.[^@\s]+')
MIN_PASSWORD = 8
_ROLE_KEY, _ROLE = 'role', 'console-accounts'
_SCHEMA = '1.0'
_ITERATIONS = 200_000
_PUBLIC = ('account_id', 'username', 'display_name', 'email', 'role', 'status', 'auth_version',
           'created_at_ns', 'updated_at_ns')
_KEEP = object()


def _require(ok, message, code='INVALID_INPUT'):
    if not ok:
        raise ConsoleError(code, message)


_DUMMY = []


def _dummy_hash():
    """A real hash of a random secret, made once, checked against when the name does not exist."""
    if not _DUMMY:
        _DUMMY.append(hash_password(secrets.token_urlsafe(16)))
    return _DUMMY[0]


def hash_password(password):
    _require(isinstance(password, str) and len(password) >= MIN_PASSWORD,
             f'password must be at least {MIN_PASSWORD} characters')
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, _ITERATIONS)
    return f'pbkdf2_sha256${_ITERATIONS}${salt.hex()}${digest.hex()}'


def verify_password(stored, password):
    try:
        scheme, iterations, salt_hex, digest_hex = stored.split('$')
        if scheme != 'pbkdf2_sha256' or not isinstance(password, str):
            return False
        digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), bytes.fromhex(salt_hex), int(iterations))
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
    """One SQLite file. Every public method validates first and writes in one transaction."""

    def __init__(self, path):
        self.path = Path(path)

    # ---- lifecycle -------------------------------------------------------------------
    @classmethod
    def create(cls, path, admin_username, admin_password):
        path = Path(path)
        _require(not path.exists(), f'{path.name} already exists; refusing to overwrite', 'ALREADY_EXISTS')
        admin_username = _username(admin_username)
        password_hash = hash_password(admin_password)
        try:
            db = sqlite3.connect(path)
            with db:
                db.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
                # account_id: survives nothing — a deleted name that is created again gets a new id.
                # auth_version: bumped whenever password / role / status change, so every session
                # issued before the change stops matching (sessions carry both values).
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
        store = cls(path)
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
        try:
            db = sqlite3.connect(self.path, isolation_level=None)
        except sqlite3.Error as exc:
            raise ConsoleError('STORAGE_ERROR', f'cannot open accounts database: {exc}') from exc
        db.row_factory = sqlite3.Row
        return _Transaction(db)

    # ---- reads -----------------------------------------------------------------------
    def list_accounts(self):
        with self._connect() as db:
            return [_public(row) for row in db.execute('SELECT * FROM accounts ORDER BY username')]

    def get(self, username):
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE username = ?', (username,)).fetchone()
        return _public(row) if row else None

    def get_by_id(self, account_id):
        """The row behind a session. A reused username has a different id, so it never matches."""
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE account_id = ?', (account_id,)).fetchone()
        return _public(row) if row else None

    def authenticate(self, username, password):
        """The account's public row for a correct password on an ACTIVE account, else None.

        Unknown user, wrong password and inactive account are deliberately indistinguishable
        to the caller (the login page says one thing); the reason is not logged either.
        """
        if not isinstance(username, str) or not isinstance(password, str):
            return None
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE username = ?', (username,)).fetchone()
        # Every name costs one password check, so the response time does not tell which names exist.
        matches = verify_password(_dummy_hash() if row is None else row['password_hash'], password)
        if row is None or row['status'] != 'ACTIVE' or not matches:
            return None
        return _public(row)

    # ---- writes (ADMIN only; the web layer enforces who calls these) ------------------
    def add(self, username, password, *, role, display_name='', email='', status='ACTIVE'):
        username = _username(username)
        record = (username, str(uuid.uuid4()), _text(display_name, 'display_name') or username, _email(email),
                  _choice(role, ACCOUNT_ROLES, 'role'), _choice(status, ACCOUNT_STATUSES, 'status'),
                  hash_password(password), 1)
        with self._connect() as db:
            _require(db.execute('SELECT 1 FROM accounts WHERE username = ?', (username,)).fetchone() is None,
                     f'account {username} already exists', 'ALREADY_EXISTS')
            now = time.time_ns()
            db.execute('INSERT INTO accounts VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)', record + (now, now))
        return self.get(username)

    def update(self, username, *, display_name=_KEEP, email=_KEEP, role=_KEEP, status=_KEEP, password=None):
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
            if (row['role'], row['status']) == ('ADMIN', 'ACTIVE') and (after_role, after_status) != ('ADMIN', 'ACTIVE'):
                self._guard_last_admin(db, username)
            if 'display_name' in changes and changes['display_name'] == '':
                changes['display_name'] = username
            credential_changed = ('password_hash' in changes or after_role != row['role'] or after_status != row['status'])
            if credential_changed:
                changes['auth_version'] = row['auth_version'] + 1   # every earlier session stops matching
            changes['updated_at_ns'] = time.time_ns()
            assignments = ', '.join(f'{key} = ?' for key in changes)
            db.execute(f'UPDATE accounts SET {assignments} WHERE username = ?', (*changes.values(), username))
        return self.get(username)

    def delete(self, username, *, by):
        _require(username != by, 'an admin cannot delete the account it is logged in with', 'SELF_DELETE')
        with self._connect() as db:
            row = db.execute('SELECT * FROM accounts WHERE username = ?', (username,)).fetchone()
            _require(row is not None, f'account {username!r} not found', 'NOT_FOUND')
            if (row['role'], row['status']) == ('ADMIN', 'ACTIVE'):
                self._guard_last_admin(db, username)
            db.execute('DELETE FROM accounts WHERE username = ?', (username,))

    @staticmethod
    def _guard_last_admin(db, excluding):
        others = db.execute("SELECT COUNT(*) FROM accounts WHERE role = 'ADMIN' AND status = 'ACTIVE' AND username != ?",
                            (excluding,)).fetchone()[0]
        _require(others > 0, 'at least one ACTIVE ADMIN must remain', 'LAST_ADMIN')


class _Transaction:
    """BEGIN IMMEDIATE ... COMMIT/ROLLBACK around a block; sqlite errors become STORAGE_ERROR."""

    def __init__(self, db):
        self.db = db

    def __enter__(self):
        try:
            self.db.execute('BEGIN IMMEDIATE')
        except sqlite3.Error as exc:
            self.db.close()
            raise ConsoleError('STORAGE_ERROR', f'accounts database busy or broken: {exc}') from exc
        return self.db

    def __exit__(self, exc_type, exc, tb):
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
