"""Console page and web boundary: login required, ADMIN manages accounts, data routes stay read-only."""
import importlib.util
import re
import sys
import unittest
from unittest import mock

from rescue_console.accounts import AccountStore
from rescue_console.render import (_age, _elapsed, failure_text, render_accounts, render_failure, render_fragment,
                                   render_login, render_page)
from rescue_console.view import COMMANDS, ConsoleError, request_command
from test_console_view import LedgerFixture, digest_dir, site_event, write_site_state

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
        commands = [b for b in buttons if 'data-command=' in b]
        self.assertEqual(len(commands), len(COMMANDS))
        for button in commands:
            self.assertIn(' disabled', button)
            self.assertNotIn('onclick', button)
        # The only other button saves the KPI-04 records in the browser.
        self.assertEqual([b for b in buttons if 'data-command=' not in b],
                         ['<button type="button" id="kpi-save" class="mini">'])
        self.assertNotIn('<form', html)
        self.assertNotRegex(html, r"(?i)method\s*[:=]\s*['\"]?(post|put|delete)")
        self.assertEqual(re.findall(r"fetch\('([^']*)'", html), ['fragment'])

    def test_logged_in_page_has_only_the_logout_form(self):
        html = render_page(self.view(), {'username': 'jane', 'display_name': 'Jane', 'role': 'CUSTOMER'}, csrf='tok')
        forms = re.findall(r'<form[^>]*>', html)
        self.assertEqual(len(forms), 1)
        self.assertIn('action="/logout"', forms[0])
        self.assertIn('name="csrf"', html)

    def test_dashboard_header_names_the_user_and_hides_admin_link_from_customers(self):
        view = self.view()
        admin_html = render_page(view, {'username': 'admin', 'display_name': 'Boss', 'role': 'ADMIN'})
        self.assertIn('Boss (ADMIN)', admin_html)
        self.assertIn('href="/admin/accounts"', admin_html)
        self.assertIn('action="/logout"', admin_html)
        customer_html = render_page(view, {'username': 'jane', 'display_name': '<b>', 'role': 'CUSTOMER'})
        self.assertNotIn('href="/admin/accounts"', customer_html)
        self.assertIn('&lt;b&gt; (CUSTOMER)', customer_html)
        self.assertNotIn('action="/admin', customer_html)

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

    def test_one_screen_has_mode_map_slot_robots_targets_and_events(self):
        html = render_page(self.view())
        for marker in ('id="mission"', 'id="map"', 'id="robots"', 'id="targets"', 'id="events"', 'id="commands"'):
            self.assertIn(marker, html)
        for label in ('미확정', '미구조', '지원 중', '구출 완료'):
            self.assertIn(label, html)
        # One fixed camera: no exit counter on screen (db-schema 2절).
        self.assertNotIn('exit-count', html)
        self.assertNotIn('출구 누적', html)

    def test_mode_badge_shows_disaster_with_trigger_time(self):
        html = render_fragment(self.view())
        self.assertIn('data-mode="DISASTER"', html)
        self.assertIn('class="mode-word">재난</h1>', html)
        self.assertIn(f'title="{self.status["site_events"]["site_events"][1]["observed_at_ns"]}"', html)
        self.assertIn('data-mode=""', render_fragment(self.view({})))

    def test_stale_robot_rows_are_marked(self):
        html = render_fragment(self.view())   # received 10 s before the build time
        self.assertEqual(len(re.findall(r'<section class="unit [a-z]+ stale"', html)), 2)
        self.assertIn('낡음', html)
        fresh = render_fragment(self.view(built_at_ns=self.status['robot_status']['robots'][0]['received_at_ns']))
        self.assertNotIn(' stale"', fresh)

    def test_page_loads_nothing_from_outside(self):
        # The field network may have no internet: no CDN, web font, external stylesheet or script.
        html = render_page(self.view(), cameras=(('cam_a', 'http://camera.invalid:8081/stream'),))
        self.assertNotRegex(html, r'<link|@import|url\(|<script[^>]+src=')
        external = re.findall(r'(?:src|href)="(https?://[^"]+)"', html)
        self.assertEqual(external, ['http://camera.invalid:8081/stream'])   # only the camera the operator gave

    def test_codes_are_shown_as_korean_words_and_unknown_codes_stay_visible(self):
        html = render_fragment(self.view())
        robots = html[html.index('id="robots"'):html.index('id="targets"')]
        self.assertIn('이동 중', robots)
        self.assertIn('준비됨', robots)
        self.assertNotIn('>MOVING<', robots)
        self.assertIn('정찰', robots)
        status = dict(self.status, robot_status={'schema_version': '1.0', 'robots': [
            dict(self.status['robot_status']['robots'][0], motion_state='DOCKING_SOMEHOW')]})
        self.assertIn('DOCKING_SOMEHOW', render_fragment(self.view(status)))

    def test_command_panel_puts_approvals_first_and_stop_apart(self):
        html = render_page(self.view())
        panel = html[html.index('id="commands"'):]
        approve = panel[panel.index('cmd-group approve'):panel.index('cmd-group mission')]
        self.assertIn('data-command="DISPATCH_SUPPORT"', approve)
        self.assertIn('data-command="CONTINUE_SEGMENT"', approve)
        stop = panel[panel.index('cmd-group stop'):]
        self.assertIn('data-command="STOP_ROBOT"', stop)
        # AGENTS.md 7절: the field stop does not depend on this page.
        self.assertIn('현장 비상 정지 장치', panel)

    def test_rescued_victim_is_struck_through_on_the_map_legend(self):
        html = render_fragment(self.view())
        rescued_icon = re.search(r'<svg class="vicon rescued".*?</svg>', html).group(0)
        self.assertIn('<line', rescued_icon)
        self.assertNotIn('<line', re.search(r'<svg class="vicon not_rescued".*?</svg>', html).group(0))

    def test_kpi04_recorder_keeps_live_rows_apart_and_sends_nothing(self):
        html = render_page(self.view())
        self.assertIn('id="kpi"', html)
        self.assertIn('live: loaded', html)                       # rows present at load are history
        self.assertIn('r.latency_s <= 3)', html)                  # KPI-04 budget substituted
        self.assertNotIn('KPI_BUDGET_S', html)
        self.assertIn('new Blob(', html)                          # saved in the browser
        self.assertEqual(re.findall(r"fetch\('([^']*)'", html), ['fragment'])

    def test_event_rows_carry_what_the_kpi04_file_needs(self):
        view = self.view()
        html = render_fragment(view)
        for event in view['events']:
            row = re.search(r'<tr class="event[^>]*data-ref="%s"[^>]*>' % re.escape(event['ref_id']), html).group(0)
            self.assertIn(f'data-kind="{event["kind"]}"', row)
            self.assertIn(f'data-clock="{event["clock_domain_id"]}"', row)
            self.assertIn(f'data-received-ns="{event["received_at_ns"] or ""}"', row)

    def test_failure_page_says_what_to_check_and_retries(self):
        html = render_failure('STORAGE_ERROR', 'ledger read failed: <unable to open>')
        self.assertIn('화면을 그릴 수 없습니다', html)
        self.assertIn('파일 경로', html)                         # the hint for STORAGE_ERROR
        self.assertIn('&lt;unable to open&gt;', html)
        self.assertIn('http-equiv="refresh"', html)
        self.assertIn('현장 비상 정지', html)
        self.assertNotIn('<script', html)
        self.assertIn('콘솔을 띄운 터미널', render_failure('SOMETHING_NEW', 'x'))
        self.assertTrue(failure_text('INVALID_INPUT', 'bad').startswith('INVALID_INPUT: '))

    def test_tab_title_carries_the_disaster_mode(self):
        self.assertIn('<title>[재난] 구조 관제</title>', render_page(self.view()))
        self.assertIn('<title>구조 관제</title>', render_page(self.view({})))
        self.assertIn("'[갱신 끊김] '", render_page(self.view()))   # set by the poller when updates stop

    def test_elapsed_time_switches_to_days(self):
        self.assertEqual([_elapsed(v) for v in (None, -1, 95, 3600 * 5 + 60 * 7, 86400 * 1060 + 3600 * 14 + 5)],
                         ['—', '—', '1분 35초', '5시간 07분', '1060일 14시간'])

    def test_ages_read_at_a_glance(self):
        self.assertEqual([_age(v) for v in (None, 0.44, 59.9, 185, 7800, 91_616_778.5, -3.0)],
                         ['—', '0.4s', '59.9s', '3분 05초', '2시간 10분', '1060일', '-3.0s'])

    def test_camera_streams_sit_outside_the_polled_fragment(self):
        cameras = (('cam_a', 'http://camera.invalid:8081/stream?q="x"'),)
        html = render_page(self.view(), cameras=cameras)
        live = html.index('<div id="live">')
        figure = html.index('<section id="cameras">')
        self.assertLess(figure, live)    # a poll replaces #live only, so the stream is never restarted
        self.assertIn('src="http://camera.invalid:8081/stream?q=&quot;x&quot;"', html)
        self.assertIn('영상 끊김', html)
        self.assertNotIn('id="cameras"', render_page(self.view()))
        self.assertEqual(re.findall(r"fetch\('([^']*)'", html), ['fragment'])

    def test_camera_argument_needs_an_id_and_http_url(self):
        from rescue_console.app import parse_camera
        self.assertEqual(parse_camera('cam_a=http://camera.invalid:8081/stream'),
                         ('cam_a', 'http://camera.invalid:8081/stream'))
        for bad in ('cam_a', '=http://x', 'cam_a=javascript:alert(1)', 'cam_a=file:///etc/passwd'):
            with self.subTest(bad=bad), self.assertRaises(ConsoleError) as caught:
                parse_camera(bad)
            self.assertEqual(caught.exception.detail_code, 'INVALID_INPUT')

    def test_untrusted_text_is_escaped(self):
        status = dict(self.status, site_events={'schema_version': '1.1', 'exit_pass_count': 1,
                                                'site_events': [site_event('x', detail='<script>alert(1)</script>')]})
        html = render_fragment(self.view(status))
        self.assertNotIn('<script>alert', html)
        self.assertIn('&lt;script&gt;', html)


class LayoutTests(unittest.TestCase):
    """#64: in a real browser window (1920x1080 monitor minus tab bar = 1853x963) the victims list was cut to 0 px.

    CI has no browser, so these pin the CSS rules that keep it visible; the window measurements are in the PR.
    """

    @staticmethod
    def rule(selector):
        from rescue_console.render import _STYLE
        found = re.search(r'(?m)^' + re.escape(selector) + r'\{([^}]*)\}', _STYLE)
        return '' if found is None else found.group(1)

    def test_victims_panel_keeps_room_for_its_list(self):
        height = re.search(r'min-height:(\d+)px', self.rule('#targets'))
        self.assertIsNotNone(height, '#targets needs a pixel min-height')
        self.assertGreaterEqual(int(height.group(1)), 230)

    def test_robot_cards_scroll_instead_of_squeezing_the_victims(self):
        robots = self.rule('#robots')
        self.assertIn('min-height:0', robots)
        self.assertIn('overflow:auto', robots)

    def test_event_log_row_fits_its_content(self):
        from rescue_console.render import _STYLE
        rows = re.search(r'grid-template-rows:([^;]*);', _STYLE).group(1)
        self.assertNotRegex(rows, r'(?<![(\w])230px')      # a fixed 230 px row left empty space under few events


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
        self.assertEqual(client.get('/fragment').status_code, 401)
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
        self.assertEqual(client.post('/logout', data={'csrf': self.csrf(client, '/')}).status_code, 302)
        self.assertEqual(client.get('/api/view').status_code, 401)
        self.assertEqual(digest_dir(self.dir), before)                      # ledgers untouched

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_repeated_wrong_passwords_lock_that_name_for_a_while(self):
        from rescue_console.app import create_app
        from rescue_console.throttle import LOGIN_LOCK_S, LOGIN_MAX_FAILURES, LoginThrottle
        clock = [1000.0]
        app = create_app(self.db, accounts_path=self.accounts, secret_key=SECRET,
                         login_throttle=LoginThrottle(clock=lambda: clock[0]))
        client = app.test_client()
        for _ in range(LOGIN_MAX_FAILURES):
            self.login(client, 'admin', 'wrong-pass-1')
        # Locked: even the right password is refused, and the page says to wait.
        self.assertEqual(self.login(client, *ADMIN).headers['Location'], '/login')
        self.assertIn(f'{LOGIN_LOCK_S}초 뒤', client.get('/login').get_data(as_text=True))
        self.assertEqual(client.get('/').status_code, 302)
        # Another name is not locked.
        other = app.test_client()
        self.assertEqual(self.login(other, *CUSTOMER).headers['Location'], '/')
        clock[0] += LOGIN_LOCK_S
        self.assertEqual(self.login(client, *ADMIN).headers['Location'], '/')

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_responses_carry_security_headers_and_data_is_not_cached(self):
        client = self.app().test_client()
        self.login(client, *ADMIN)
        for path in ('/login', '/', '/fragment', '/api/view'):
            with self.subTest(path=path):
                headers = client.get(path).headers
                self.assertEqual(headers['X-Content-Type-Options'], 'nosniff')
                self.assertEqual(headers['X-Frame-Options'], 'DENY')
                self.assertEqual(headers['Referrer-Policy'], 'no-referrer')
                self.assertIn("frame-ancestors 'none'", headers['Content-Security-Policy'])
                self.assertEqual(headers['Cache-Control'], 'no-store')

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
    def test_old_sessions_die_with_the_account_they_came_from(self):
        """AST-50-01: a session is bound to the account row and its auth version, not to the name."""
        app = self.app()
        admin = app.test_client()
        self.login(admin, *ADMIN)
        csrf = self.csrf(admin)
        old_customer = app.test_client()
        self.login(old_customer, *CUSTOMER)
        self.assertEqual(old_customer.get('/api/view').status_code, 200)
        # delete jane, recreate the same name as ADMIN: the old cookie must not inherit anything
        admin.post('/admin/accounts/jane', data={'csrf': csrf, '_action': 'delete'})
        admin.post('/admin/accounts', data={'csrf': csrf, 'username': 'jane', 'password': 'other-pass-9', 'role': 'ADMIN'})
        self.assertEqual(old_customer.get('/api/view').status_code, 401)
        self.assertNotEqual(old_customer.get('/admin/accounts').status_code, 200)
        self.assertNotEqual(old_customer.post('/admin/accounts', data={'csrf': csrf, 'username': 'evil',
                                                                       'password': 'evil-pass-9', 'role': 'ADMIN'}).status_code, 303)
        self.assertIsNone(AccountStore.open(self.accounts).get('evil'))
        # password reset ends other sessions of that account
        fresh = app.test_client()
        self.login(fresh, 'jane', 'other-pass-9')
        self.assertEqual(fresh.get('/api/view').status_code, 200)
        admin.post('/admin/accounts/jane', data={'csrf': csrf, '_action': 'update', 'display_name': 'jane', 'email': '',
                                                 'role': 'ADMIN', 'status': 'ACTIVE', 'password': 'newer-pass-9'})
        self.assertEqual(fresh.get('/api/view').status_code, 401)
        # deactivate then reactivate: a cookie saved before deactivation stays dead
        again = app.test_client()
        self.login(again, 'jane', 'newer-pass-9')
        for status in ('INACTIVE', 'ACTIVE'):
            admin.post('/admin/accounts/jane', data={'csrf': csrf, '_action': 'update', 'display_name': 'jane', 'email': '',
                                                     'role': 'ADMIN', 'status': status})
        self.assertEqual(again.get('/api/view').status_code, 401)
        # the admin's own session is untouched by changes to other accounts
        self.assertEqual(admin.get('/api/view').status_code, 200)

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_fragment_polling_gets_401_not_a_login_document(self):
        """AST-50-02: the poller must never receive the login page as a fragment."""
        app = self.app()
        anonymous = app.test_client()
        response = anonymous.get('/fragment')
        self.assertEqual(response.status_code, 401)
        self.assertNotIn('<!doctype', response.get_data(as_text=True).lower())
        customer = app.test_client()
        self.login(customer, *CUSTOMER)
        self.assertEqual(customer.get('/fragment').status_code, 200)
        admin = app.test_client()
        self.login(admin, *ADMIN)
        admin.post('/admin/accounts/jane', data={'csrf': self.csrf(admin), '_action': 'update', 'display_name': 'Jane',
                                                 'email': 'jane@example.com', 'role': 'CUSTOMER', 'status': 'INACTIVE'})
        self.assertEqual(customer.get('/fragment').status_code, 401)
        page = render_page(self.view(), {'username': 'jane', 'display_name': 'Jane', 'role': 'CUSTOMER'})
        self.assertIn('401', page)                      # the poller handles it...
        self.assertIn("'/login'", page)                 # ...by leaving for the login page
        self.assertIn('clearInterval', page)            # ...and stops polling

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_logout_is_a_post_with_csrf(self):
        """AST-50-03: a link click must not end the session."""
        app = self.app()
        client = app.test_client()
        self.login(client, *ADMIN)
        self.assertEqual(client.get('/logout').status_code, 405)
        self.assertEqual(client.post('/logout').status_code, 400)
        self.assertEqual(client.get('/api/view').status_code, 200)
        csrf = self.csrf(client, '/')
        self.assertEqual(client.post('/logout', data={'csrf': csrf}).status_code, 302)
        self.assertEqual(client.get('/api/view').status_code, 401)

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_broken_ledger_is_a_visible_failure(self):
        client = self.app(self.dir / 'absent.sqlite3').test_client()
        self.login(client, *ADMIN)
        api = client.get('/api/view')
        self.assertEqual(api.status_code, 503)
        self.assertEqual(api.get_json()['error'], 'STORAGE_ERROR')
        self.assertIn('파일 경로', api.get_json()['hint'])
        page = client.get('/')
        self.assertEqual(page.status_code, 503)
        self.assertIn('화면을 그릴 수 없습니다', page.get_data(as_text=True))
        fragment = client.get('/fragment')
        self.assertEqual(fragment.status_code, 503)
        self.assertTrue(fragment.get_data(as_text=True).startswith('STORAGE_ERROR: '))
        self.assertFalse((self.dir / 'absent.sqlite3').exists())


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
class SiteStateWebTests(AccountsFixture):
    def test_console_serves_the_site_state_db_and_camera_streams(self):
        from rescue_console.app import create_app
        site_db = self.dir / 'site_state.sqlite3'
        write_site_state(site_db, self.status)
        app = create_app(self.db, accounts_path=self.accounts, secret_key=SECRET, site_state_path=site_db,
                         cameras=[('cam_a', 'http://camera.invalid:8081/stream')])
        client = app.test_client()
        self.login(client, *CUSTOMER)
        before = digest_dir(self.dir)
        model = client.get('/api/view').get_json()
        self.assertEqual(model['mission']['mode'], 'DISASTER')
        self.assertEqual([r['robot_id'] for r in model['robots'] if r['has_status']], ['robot_a', 'robot_b'])
        self.assertFalse([w for w in model['warnings'] if '입력 없음' in w])
        self.assertIn('src="http://camera.invalid:8081/stream"', client.get('/').get_data(as_text=True))
        self.assertNotIn('camera.invalid', client.get('/fragment').get_data(as_text=True))
        self.assertEqual(digest_dir(self.dir), before)

    def test_site_state_and_fixture_together_fail_at_startup(self):
        from rescue_console.app import create_app
        with self.assertRaises(ConsoleError) as caught:
            create_app(self.db, 'status.json', accounts_path=self.accounts, secret_key=SECRET,
                       site_state_path=self.dir / 'site_state.sqlite3')
        self.assertEqual(caught.exception.detail_code, 'INVALID_INPUT')


if __name__ == '__main__':
    unittest.main()
