"""Console login data (tutor day5 "Login Data" DB): admin/customer accounts, admin manages customers.

Pure Python: SQLite + hashlib PBKDF2. No Flask needed here.
"""
import sqlite3
import tempfile
import unittest
from pathlib import Path

from rescue_console.accounts import ACCOUNT_ROLES, ACCOUNT_STATUSES, AccountStore
from rescue_console.view import ConsoleError


class AccountStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'accounts.sqlite3'
        self.store = AccountStore.create(self.path, 'admin', 'admin-pass-1')

    def test_vocabularies(self):
        self.assertEqual(('ADMIN', 'CUSTOMER'), ACCOUNT_ROLES)
        self.assertEqual(('ACTIVE', 'INACTIVE'), ACCOUNT_STATUSES)

    def test_create_bootstraps_one_active_admin_and_refuses_to_overwrite(self):
        [admin] = self.store.list_accounts()
        self.assertEqual(('admin', 'ADMIN', 'ACTIVE'), (admin['username'], admin['role'], admin['status']))
        self.assertNotIn('password_hash', admin)
        with self.assertRaises(ConsoleError) as caught:
            AccountStore.create(self.path, 'admin2', 'another-pass')
        self.assertEqual('ALREADY_EXISTS', caught.exception.detail_code)

    def test_open_refuses_missing_or_foreign_database(self):
        with self.assertRaises(ConsoleError) as caught:
            AccountStore.open(self.path.with_name('absent.sqlite3'))
        self.assertEqual('STORAGE_ERROR', caught.exception.detail_code)
        self.assertFalse(self.path.with_name('absent.sqlite3').exists())
        foreign = self.path.with_name('foreign.sqlite3')
        sqlite3.connect(foreign).execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT)').connection.commit()
        with self.assertRaises(ConsoleError) as caught:
            AccountStore.open(foreign)
        self.assertEqual('POLICY_MISMATCH', caught.exception.detail_code)

    def test_passwords_are_salted_hashes_and_authenticate_is_strict(self):
        raw = sqlite3.connect(self.path).execute('SELECT password_hash FROM accounts').fetchone()[0]
        self.assertTrue(raw.startswith('pbkdf2_sha256$'))
        self.assertNotIn('admin-pass-1', raw)
        self.assertEqual('ADMIN', self.store.authenticate('admin', 'admin-pass-1')['role'])
        for username, password in (('admin', 'wrong'), ('nobody', 'admin-pass-1'), ('ADMIN', 'admin-pass-1'),
                                   ('admin', ''), ('', ''), (None, 'x'), ('admin', None)):
            with self.subTest(username=username):
                self.assertIsNone(self.store.authenticate(username, password))

    def test_admin_creates_updates_and_deletes_customers(self):
        self.store.add('jane', 'jane-pass-1', role='CUSTOMER', display_name='Jane', email='jane@example.com')
        self.assertEqual('CUSTOMER', self.store.authenticate('jane', 'jane-pass-1')['role'])
        self.store.update('jane', display_name='Jane S.', email='js@example.com')
        self.assertEqual(('Jane S.', 'js@example.com'), tuple(self.store.get('jane')[k] for k in ('display_name', 'email')))
        self.store.update('jane', status='INACTIVE')
        self.assertIsNone(self.store.authenticate('jane', 'jane-pass-1'))  # inactive accounts cannot log in
        self.store.update('jane', status='ACTIVE', password='jane-pass-2')
        self.assertIsNone(self.store.authenticate('jane', 'jane-pass-1'))
        self.assertEqual('jane', self.store.authenticate('jane', 'jane-pass-2')['username'])
        self.store.delete('jane', by='admin')
        self.assertIsNone(self.store.get('jane'))
        self.assertEqual(['admin'], [a['username'] for a in self.store.list_accounts()])

    def test_invalid_inputs_are_rejected_with_the_contract_error(self):
        for args, kwargs in ((('Jane', 'jane-pass-1'), dict(role='CUSTOMER')),        # uppercase
                             (('jo', 'jane-pass-1'), dict(role='CUSTOMER')),          # too short
                             (('jane', 'short'), dict(role='CUSTOMER')),              # weak password
                             (('jane', 'jane-pass-1'), dict(role='ROOT')),            # unknown role
                             (('jane', 'jane-pass-1'), dict(role='CUSTOMER', status='PENDING')),
                             (('jane', 'jane-pass-1'), dict(role='CUSTOMER', email='not-an-email')),
                             (('admin', 'jane-pass-1'), dict(role='CUSTOMER'))):      # duplicate
            with self.subTest(args=args, kwargs=kwargs), self.assertRaises(ConsoleError) as caught:
                self.store.add(*args, **kwargs)
            self.assertIn(caught.exception.detail_code, ('INVALID_INPUT', 'ALREADY_EXISTS'))
        with self.assertRaises(ConsoleError) as caught:
            self.store.update('ghost', status='INACTIVE')
        self.assertEqual('NOT_FOUND', caught.exception.detail_code)

    def test_last_active_admin_is_protected(self):
        for kwargs in (dict(status='INACTIVE'), dict(role='CUSTOMER')):
            with self.subTest(kwargs=kwargs), self.assertRaises(ConsoleError) as caught:
                self.store.update('admin', **kwargs)
            self.assertEqual('LAST_ADMIN', caught.exception.detail_code)
        with self.assertRaises(ConsoleError) as caught:
            self.store.delete('admin', by='admin')
        self.assertIn(caught.exception.detail_code, ('LAST_ADMIN', 'SELF_DELETE'))
        self.store.add('boss', 'boss-pass-1', role='ADMIN')
        self.store.update('admin', status='INACTIVE')  # now allowed: boss is still an active admin
        with self.assertRaises(ConsoleError) as caught:
            self.store.delete('boss', by='boss')
        self.assertEqual('SELF_DELETE', caught.exception.detail_code)
        with self.assertRaises(ConsoleError) as caught:
            self.store.update('boss', status='INACTIVE')
        self.assertEqual('LAST_ADMIN', caught.exception.detail_code)

    def test_auth_nonce_rotates_for_credentials_role_and_status_but_not_profile(self):
        self.store.add('jane', 'jane-pass-1', role='CUSTOMER')
        nonce = self.store.get('jane')['auth_nonce']
        self.assertNotEqual(nonce, self.store.get('admin')['auth_nonce'])
        self.store.update('jane', display_name='Jane', email='jane@example.com', role='CUSTOMER', status='ACTIVE')
        self.assertEqual(nonce, self.store.get('jane')['auth_nonce'])
        for change in (dict(password='jane-pass-2'), dict(role='ADMIN'), dict(status='INACTIVE'), dict(status='ACTIVE')):
            with self.subTest(change=list(change)):
                self.store.update('jane', **change)
                newer = self.store.get('jane')['auth_nonce']
                self.assertNotEqual(nonce, newer)
                nonce = newer

    def test_open_migrates_legacy_accounts_without_resetting_passwords_or_nonce(self):
        legacy = self.path.with_name('legacy.sqlite3')
        with sqlite3.connect(self.path) as source:
            row = source.execute('SELECT username, display_name, email, role, status, password_hash, '
                                 'created_at_ns, updated_at_ns FROM accounts').fetchone()
        with sqlite3.connect(legacy) as db:
            db.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
            db.executemany('INSERT INTO metadata VALUES (?, ?)', [('role', 'console-accounts'), ('schema_version', '1.0')])
            db.execute('CREATE TABLE accounts (username TEXT PRIMARY KEY, display_name TEXT NOT NULL, '
                       'email TEXT NOT NULL, role TEXT NOT NULL, status TEXT NOT NULL, password_hash TEXT NOT NULL, '
                       'created_at_ns INTEGER NOT NULL, updated_at_ns INTEGER NOT NULL)')
            db.execute('INSERT INTO accounts VALUES (?, ?, ?, ?, ?, ?, ?, ?)', row)
        migrated = AccountStore.open(legacy)
        account = migrated.authenticate('admin', 'admin-pass-1')
        self.assertEqual('ADMIN', account['role'])
        self.assertTrue(account['auth_nonce'])
        self.assertEqual(account['auth_nonce'], AccountStore.open(legacy).get('admin')['auth_nonce'])
        with sqlite3.connect(legacy) as db:
            self.assertEqual('1.1', db.execute("SELECT value FROM metadata WHERE key='schema_version'").fetchone()[0])


if __name__ == '__main__':
    unittest.main()
