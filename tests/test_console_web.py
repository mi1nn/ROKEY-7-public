"""Console page and web boundary: login required, ADMIN manages accounts, data routes stay read-only."""
import importlib.util
import re
import sys
import unittest
from unittest import mock

from rescue_console.accounts import AccountStore
from rescue_console.render import render_accounts, render_fragment, render_login, render_page
from rescue_console.view import COMMANDS, ConsoleError, request_command
from test_console_view import LedgerFixture, digest_dir, site_event

HAS_FLASK = importlib.util.find_spec('flask') is not None
SECRET = 'unit-test-secret-key-0123456789'
ADMIN, CUSTOMER = ('admin', 'admin-pass-1'), ('jane', 'jane-pass-1')


class AccountsFixture(LedgerFixture):
    def setUp(self):
        super().setUp()
        self.accounts = self.dir / 'accounts.sqlite3'
        store = AccountStore.create(self.accounts, *ADMIN)
        store.add(CUSTOMER[0], CUSTOMER[1], role='CUSTOMER', display_name='Jane', email='jane@example.com')

    def app(self, ledger=None):
        from rescue_console.app import create_app
        return create_app(ledger or self.db, accounts_path=self.accounts, secret_key=SECRET)

    @staticmethod
    def login(client, username, password):
        page = client.get('/login').get_data(as_text=True)
        csrf = re.search(r'name="csrf" value="([^"]+)"', page).group(1)
        return client.post('/login', data={'csrf': csrf, 'username': username, 'password': password})

    @staticmethod
    def csrf(client, path='/admin/accounts'):
        return re.search(r'name="csrf" value="([^"]+)"', client.get(path).get_data(as_text=True)).group(1)


class CommandPlaceholderTests(LedgerFixture):
    def test_every_command_is_disabled_in_the_model(self):
        commands = self.view()['commands']
        self.assertEqual([c['name'] for c in commands], [name for name, _ in COMMANDS])
        self.assertEqual({c['enabled'] for c in commands}, {False})

    def test_requesting_a_command_sends_nothing_and_changes_nothing(self):
        before = digest_dir(self.dir)
        for name, _ in COMMANDS:
            with self.subTest(command=name), self.assertRaises(ConsoleError) as caught:
                request_command(name)
            self.assertEqual(caught.exception.detail_code, 'NOT_CONNECTED')
        with self.assertRaises(ConsoleError) as caught:
            request_command('EMERGENCY_STOP')
        self.assertEqual(caught.exception.detail_code, 'INVALID_INPUT')
        self.assertEqual(digest_dir(self.dir), before)

    def test_page_buttons_are_disabled_and_page_has_no_write_path(self):
        html = render_page(self.view())
        buttons = re.findall(r'<button[^>]*>', html)
        self.assertEqual(len(buttons), len(COMMANDS))
        for button in buttons:
            self.assertIn(' disabled', button)
            self.assertNotIn('onclick', button)
        self.assertNotIn('<form', html)
        self.assertNotRegex(html, r"(?i)method\s*[:=]\s*['\"]?(post|put|delete)")
        self.assertEqual(re.findall(r"fetch\('([^']*)'", html), ['fragment'])

    def test_dashboard_header_names_the_user_and_hides_admin_link_from_customers(self):
        view = self.view()
        admin_html = render_page(view, {'username': 'admin', 'display_name': 'Boss', 'role': 'ADMIN'})
        self.assertIn('Boss (ADMIN)', admin_html)
        self.assertIn('href="/admin/accounts"', admin_html)
        self.assertIn('href="/logout"', admin_html)
        customer_html = render_page(view, {'username': 'jane', 'display_name': '<b>', 'role': 'CUSTOMER'})
        self.assertNotIn('href="/admin/accounts"', customer_html)
        self.assertIn('&lt;b&gt; (CUSTOMER)', customer_html)
        self.assertNotIn('<form', customer_html)

    def test_login_and_accounts_pages_carry_csrf_and_escape_values(self):
        login = render_login('tok&en', [('danger', '<x>')])
        self.assertIn('value="tok&amp;en"', login)
        self.assertIn('&lt;x&gt;', login)
        self.assertIn('type="password"', login)
        me = {'username': 'admin', 'display_name': 'admin', 'role': 'ADMIN'}
        accounts = [dict(me, email='', status='ACTIVE', created_at_ns=1, updated_at_ns=1),
                    {'username': 'jane', 'display_name': '"Jane"', 'email': 'j@x.io', 'role': 'CUSTOMER',
                     'status': 'INACTIVE', 'created_at_ns': 1, 'updated_at_ns': 1}]
        html = render_accounts(accounts, me, 'tok', [])
        self.assertIn('&quot;Jane&quot;', html)
        self.assertIn('<em>본인</em>', html)                       # no delete form for the logged-in admin
        self.assertEqual(html.count('name="_action" value="delete"'), 1)
        self.assertIn('<option value="INACTIVE" selected>', html)


class PageTests(LedgerFixture):
    def test_event_rows_carry_event_time_for_render_latency(self):
        view = self.view()
        html = render_fragment(view)
        stamped = [int(ns) for ns in re.findall(r'data-event-ns="(\d+)"', html)]
        self.assertEqual(stamped, [row['event_ns'] for row in view['events']])
        self.assertIn(f'data-built-ns="{view["built_at_ns"]}"', html)

    def test_one_screen_has_map_slot_robots_targets_exit_count_and_events(self):
        html = render_page(self.view())
        for marker in ('id="map"', 'id="robots"', 'id="targets"', 'id="exit-count"', 'id="events"', 'id="commands"'):
            self.assertIn(marker, html)
        for label in ('미확정', '미구조', '지원 중', '구출 완료'):
            self.assertIn(label, html)

    def test_untrusted_text_is_escaped(self):
        status = dict(self.status, site_events={'schema_version': '1.1', 'exit_pass_count': 1,
                                                'site_events': [site_event('x', detail='<script>alert(1)</script>')]})
        html = render_fragment(self.view(status))
        self.assertNotIn('<script>alert', html)
        self.assertIn('&lt;script&gt;', html)


class FlaskBoundaryTests(AccountsFixture):
    def test_missing_flask_is_a_clear_error_not_a_fallback(self):
        from rescue_console.app import create_app
        with mock.patch.dict(sys.modules, {'flask': None}), self.assertRaises(ConsoleError) as caught:
            create_app(self.db, accounts_path=self.accounts, secret_key=SECRET)
        self.assertEqual(caught.exception.detail_code, 'DEPENDENCY_MISSING')
        self.assertIn('pip install flask', str(caught.exception))

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_missing_secret_or_accounts_fail_at_startup(self):
        from rescue_console.app import create_app
        with self.assertRaises(ConsoleError) as caught:
            create_app(self.db, accounts_path=self.accounts, secret_key='short')
        self.assertEqual(caught.exception.detail_code, 'CONFIG_MISSING')
        with self.assertRaises(ConsoleError) as caught:
            create_app(self.db, accounts_path=self.dir / 'none.sqlite3', secret_key=SECRET)
        self.assertEqual(caught.exception.detail_code, 'STORAGE_ERROR')

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_everything_needs_login_and_data_routes_stay_get_only(self):
        app = self.app()
        data_rules = {rule.rule: rule.methods - {'HEAD', 'OPTIONS'} for rule in app.url_map.iter_rules()
                      if not rule.rule.startswith(('/login', '/logout', '/admin', '/static'))}
        self.assertEqual({frozenset(methods) for methods in data_rules.values()}, {frozenset({'GET'})})
        client = app.test_client()
        self.assertEqual(client.get('/').status_code, 302)
        self.assertEqual(client.get('/').headers['Location'].split('?')[0], '/login')
        self.assertEqual(client.get('/fragment').status_code, 302)
        self.assertEqual(client.get('/api/view').status_code, 401)
        self.assertEqual(client.get('/admin/accounts').status_code, 302)
        for name, _ in COMMANDS:
            self.assertIn(client.post(f'/api/commands/{name}').status_code, (404, 405))

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_login_logout_and_wrong_password(self):
        client = self.app().test_client()
        before = digest_dir(self.dir)
        response = self.login(client, 'admin', 'nope')
        self.assertEqual(response.status_code, 302)
        self.assertIn('아이디 또는 비밀번호', client.get('/login').get_data(as_text=True))
        self.assertEqual(client.get('/').status_code, 302)
        self.assertEqual(client.post('/login', data={'username': 'admin', 'password': ADMIN[1]}).status_code, 400)  # no csrf
        self.assertEqual(self.login(client, *ADMIN).headers['Location'], '/')
        page = client.get('/').get_data(as_text=True)
        self.assertIn('admin (ADMIN)', page)
        self.assertEqual(client.get('/api/view').get_json()['exit_count'], None)
        self.assertEqual(client.get('/login').headers['Location'], '/')   # already logged in
        self.assertEqual(client.get('/logout').status_code, 302)
        self.assertEqual(client.get('/api/view').status_code, 401)
        self.assertEqual(digest_dir(self.dir), before)                      # ledgers untouched

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_customer_views_only_and_admin_manages_accounts(self):
        app = self.app()
        customer = app.test_client()
        self.login(customer, *CUSTOMER)
        self.assertIn('Jane (CUSTOMER)', customer.get('/').get_data(as_text=True))
        self.assertEqual(customer.get('/admin/accounts').status_code, 403)
        self.assertEqual(customer.post('/admin/accounts', data={'username': 'x'}).status_code, 403)
        admin = app.test_client()
        self.login(admin, *ADMIN)
        csrf = self.csrf(admin)
        self.assertEqual(admin.post('/admin/accounts', data={'username': 'sam', 'password': 'sam-pass-1'}).status_code, 400)
        response = admin.post('/admin/accounts', data={'csrf': csrf, 'username': 'sam', 'password': 'sam-pass-1',
                                                       'role': 'CUSTOMER', 'display_name': 'Sam', 'email': 's@x.io'})
        self.assertEqual(response.status_code, 303)
        self.assertIn('sam', [a['username'] for a in AccountStore.open(self.accounts).list_accounts()])
        admin.post('/admin/accounts/jane', data={'csrf': csrf, '_action': 'update', 'display_name': 'Jane',
                                                 'email': 'jane@example.com', 'role': 'CUSTOMER', 'status': 'INACTIVE'})
        self.assertEqual(self.login(app.test_client(), *CUSTOMER).headers['Location'], '/login')   # deactivated
        self.assertEqual(customer.get('/').status_code, 302)                                        # session dropped too
        admin.post('/admin/accounts/sam', data={'csrf': csrf, '_action': 'delete'})
        self.assertIsNone(AccountStore.open(self.accounts).get('sam'))
        admin.post('/admin/accounts/admin', data={'csrf': csrf, '_action': 'delete'})
        self.assertIsNotNone(AccountStore.open(self.accounts).get('admin'))                        # self / last admin
        self.assertIn('변경 실패', admin.get('/admin/accounts').get_data(as_text=True))

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_broken_ledger_is_a_visible_failure(self):
        client = self.app(self.dir / 'absent.sqlite3').test_client()
        self.login(client, *ADMIN)
        self.assertEqual(client.get('/api/view').status_code, 503)
        self.assertFalse((self.dir / 'absent.sqlite3').exists())


if __name__ == '__main__':
    unittest.main()
