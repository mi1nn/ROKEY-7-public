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
# Shape check only (one '@', a dot in the host part); no mail is ever sent to it.
EMAIL = re.compile(r'[^@\s]+@[^@\s]+\.[^@\s]+')
MIN_PASSWORD = 8
# Written into `metadata`; open() refuses any SQLite file without this pair (e.g. the operations ledger).
_ROLE_KEY, _ROLE = 'role', 'console-accounts'
_SCHEMA = '1.0'
# PBKDF2 rounds for new hashes (src/rescue_console/README.md: 200,000). Each stored hash carries its own
# count, so raising this later keeps old passwords valid. The cost per guess also slows password guessing.
_ITERATIONS = 200_000
# The only columns that leave the store: password_hash is never returned to a caller.
_PUBLIC = ('account_id', 'username', 'display_name', 'email', 'role', 'status', 'auth_version',
           'created_at_ns', 'updated_at_ns')
# Sentinel for update(): "argument not given", distinct from every real value including ''.
_KEEP = object()


def _require(ok, message, code='INVALID_INPUT'):
    if not ok:
        raise ConsoleError(code, message)


# Cache for _dummy_hash(): filled on first use, not at import, so importing this module costs no hash.
_DUMMY = []


def _dummy_hash():
    """A real hash of a random secret, made once, checked against when the name does not exist."""
    if not _DUMMY:
        _DUMMY.append(hash_password(secrets.token_urlsafe(16)))
    return _DUMMY[0]


def hash_password(password):
    """Salted PBKDF2-HMAC-SHA256 of `password` as `pbkdf2_sha256$<iterations>$<salt hex>$<digest hex>`.

    Raises ConsoleError INVALID_INPUT unless it is a string of at least MIN_PASSWORD characters.
    A new random 16-byte salt per call means two accounts with one password do not share a stored value.
    """
    _require(isinstance(password, str) and len(password) >= MIN_PASSWORD,
             f'password must be at least {MIN_PASSWORD} characters')
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, _ITERATIONS)
    return f'pbkdf2_sha256${_ITERATIONS}${salt.hex()}${digest.hex()}'


def verify_password(stored, password):
    """True when `password` hashes to the digest in `stored` (a hash_password() string), else False.

    Salt and iteration count are taken from `stored`, not from the current constants. A stored value that
    is malformed, of another scheme or not a string gives False instead of raising.
    """
    try:
        scheme, iterations, salt_hex, digest_hex = stored.split('$')
        if scheme != 'pbkdf2_sha256' or not isinstance(password, str):
            return False
        digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), bytes.fromhex(salt_hex), int(iterations))
        # Constant-time compare: the response time does not reveal how much of the digest matched.
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
        """Make a new accounts file with its two tables and one ACTIVE ADMIN; return the store.

        An existing path is refused (ALREADY_EXISTS), so a second `init` never overwrites live accounts.
        A bad name or password raises INVALID_INPUT before any file is made; sqlite errors raise STORAGE_ERROR.
        """
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
        """Open an existing accounts file after checking that it is one; never creates a file.

        Missing file -> STORAGE_ERROR; a file that is not SQLite at all fails at BEGIN, also STORAGE_ERROR.
        No readable `metadata`, or a role / schema_version other than this module's -> POLICY_MISMATCH.
        """
        store = cls(path)
        # Checked first: sqlite3.connect would silently create an empty file at a mistyped path.
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
        """A new connection wrapped in _Transaction, used for one `with` block; STORAGE_ERROR if it cannot open.

        A new connection per call means request threads never share one. isolation_level=None turns off the
        sqlite3 module's implicit transactions, so _Transaction's BEGIN IMMEDIATE / COMMIT are the only ones.
        Rows come back as sqlite3.Row, read by column name.
        """
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
        """Insert a new account and return its public row.

        Every field is checked, and the slow password hash made, before the write transaction opens, so the
        lock is held only for the check and the insert. Raises INVALID_INPUT for a bad field, ALREADY_EXISTS for a
        taken name. An empty display_name becomes the username; a new account starts at auth_version 1.
        """
        username = _username(username)
        record = (username, str(uuid.uuid4()), _text(display_name, 'display_name') or username, _email(email),
                  _choice(role, ACCOUNT_ROLES, 'role'), _choice(status, ACCOUNT_STATUSES, 'status'),
                  hash_password(password), 1)
        with self._connect() as db:
            # Checked inside BEGIN IMMEDIATE: no other writer can add the same name between check and insert.
            _require(db.execute('SELECT 1 FROM accounts WHERE username = ?', (username,)).fetchone() is None,
                     f'account {username} already exists', 'ALREADY_EXISTS')
            now = time.time_ns()
            db.execute('INSERT INTO accounts VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)', record + (now, now))
        return self.get(username)

    def update(self, username, *, display_name=_KEEP, email=_KEEP, role=_KEEP, status=_KEEP, password=None):
        """Change the given fields of one account and return its public row.

        Only arguments actually passed change (_KEEP = not given; password=None keeps the password).
        Raises INVALID_INPUT (bad value, or nothing to change), NOT_FOUND, or LAST_ADMIN when the change
        would leave no ACTIVE ADMIN. A real change of password, role or status raises auth_version by one,
        which ends every session issued before it (sessions carry account_id and auth_version).
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
            # Only demoting or deactivating an ACTIVE ADMIN can remove the last one.
            if (row['role'], row['status']) == ('ADMIN', 'ACTIVE') and (after_role, after_status) != ('ADMIN', 'ACTIVE'):
                self._guard_last_admin(db, username)
            if 'display_name' in changes and changes['display_name'] == '':
                changes['display_name'] = username
            # Compared with the stored row: re-sending the same role or status does not end any session.
            credential_changed = ('password_hash' in changes or after_role != row['role'] or after_status != row['status'])
            if credential_changed:
                changes['auth_version'] = row['auth_version'] + 1   # every earlier session stops matching
            changes['updated_at_ns'] = time.time_ns()
            # Column names come only from the fixed keys set above; every value is a bound parameter.
            assignments = ', '.join(f'{key} = ?' for key in changes)
            db.execute(f'UPDATE accounts SET {assignments} WHERE username = ?', (*changes.values(), username))
        return self.get(username)

    def delete(self, username, *, by):
        """Remove one account. `by` is the username of the logged-in ADMIN making the request.

        Raises SELF_DELETE when `by` is that same account, NOT_FOUND, or LAST_ADMIN for the last ACTIVE
        ADMIN. A later account with the same name gets a new account_id, so this one's sessions never match it.
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
        # Runs on the caller's connection inside its BEGIN IMMEDIATE, so the count holds until its write.
        others = db.execute("SELECT COUNT(*) FROM accounts WHERE role = 'ADMIN' AND status = 'ACTIVE' AND username != ?",
                            (excluding,)).fetchone()[0]
        _require(others > 0, 'at least one ACTIVE ADMIN must remain', 'LAST_ADMIN')


class _Transaction:
    """BEGIN IMMEDIATE ... COMMIT/ROLLBACK around a block, closing the connection after.

    A failed BEGIN or COMMIT becomes STORAGE_ERROR; for errors raised inside the block see `__exit__`.
    """

    def __init__(self, db):
        self.db = db

    def __enter__(self):
        """Start the transaction and hand the connection to the block.

        IMMEDIATE takes the write lock at BEGIN (reads too), so a check-then-write in the block (name taken,
        last ADMIN) cannot interleave with another writer; a second caller's BEGIN waits up to sqlite3's busy
        timeout. If BEGIN fails (busy, or not a database) the connection is closed and STORAGE_ERROR raised.
        """
        try:
            self.db.execute('BEGIN IMMEDIATE')
        except sqlite3.Error as exc:
            self.db.close()
            raise ConsoleError('STORAGE_ERROR', f'accounts database busy or broken: {exc}') from exc
        return self.db

    def __exit__(self, exc_type, exc, tb):
        """COMMIT when the block ended normally, ROLLBACK when it raised; always close the connection.

        A failed COMMIT raises STORAGE_ERROR. A failed ROLLBACK is dropped, so the block's own exception is
        the one reported. Only an exception whose type is exactly sqlite3.Error (an `is` test) is re-raised as
        STORAGE_ERROR; any other exception, including sqlite3 subclasses such as sqlite3.OperationalError and
        ConsoleError from the block's checks, propagates unchanged (returns False).
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
    """`init` command: create the accounts file with one ACTIVE ADMIN; return the process exit code.

    The password comes from the environment variable named by --password-env, or from a prompt when that is
    unset or empty, so it is never typed on the command line. Returns 0, or 2 after `ACCOUNTS FAILED: ...`.
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
