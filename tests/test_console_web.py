"""Console page and web boundary: command buttons do nothing, page is read-only, Flask is optional."""
import importlib.util
import re
import sys
import unittest
from unittest import mock

from rescue_console.render import render_fragment, render_page
from rescue_console.view import COMMANDS, ConsoleError, request_command
from test_console_view import LedgerFixture, digest_dir, site_event

HAS_FLASK = importlib.util.find_spec('flask') is not None


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


class FlaskBoundaryTests(LedgerFixture):
    def test_missing_flask_is_a_clear_error_not_a_fallback(self):
        from rescue_console.app import create_app
        with mock.patch.dict(sys.modules, {'flask': None}), self.assertRaises(ConsoleError) as caught:
            create_app(self.db)
        self.assertEqual(caught.exception.detail_code, 'DEPENDENCY_MISSING')
        self.assertIn('pip install flask', str(caught.exception))

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_routes_are_read_only(self):
        from rescue_console.app import create_app
        app = create_app(self.db)
        methods = {method for rule in app.url_map.iter_rules() for method in rule.methods}
        self.assertLessEqual(methods, {'GET', 'HEAD', 'OPTIONS'})
        client = app.test_client()
        before = digest_dir(self.dir)
        self.assertEqual(client.get('/').status_code, 200)
        self.assertEqual(client.get('/fragment').status_code, 200)
        self.assertEqual(client.get('/api/view').get_json()['exit_count'], None)
        for name, _ in COMMANDS:
            self.assertIn(client.post(f'/api/commands/{name}').status_code, (404, 405))
        self.assertEqual(digest_dir(self.dir), before)

    @unittest.skipUnless(HAS_FLASK, 'Flask not installed: web routes NOT_RUN')
    def test_broken_ledger_is_a_visible_failure(self):
        from rescue_console.app import create_app
        response = create_app(self.dir / 'absent.sqlite3').test_client().get('/api/view')
        self.assertEqual(response.status_code, 503)
        self.assertFalse((self.dir / 'absent.sqlite3').exists())


if __name__ == '__main__':
    unittest.main()
