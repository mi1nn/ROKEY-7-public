"""폰에서 들어갈 때마다 로그인하지 않게(재범 2026-10-16).

로그인 쿠키에 만료 시각이 없으면 브라우저 세션 쿠키가 된다. 폰 브라우저는 탭이나 앱을 닫으면 이 쿠키를 버리므로
다시 들어갈 때마다 로그인 화면이 나왔다. 로그인하면 쿠키에 만료 시각(SESSION_DAYS 일)을 붙인다.
로그아웃, 비밀번호·역할 변경(auth_version)으로 쿠키가 무효가 되는 것은 그대로다.
"""
import unittest
from http.cookies import SimpleCookie

from test_console_web import ADMIN, HAS_FLASK, AccountsFixture


@unittest.skipUnless(HAS_FLASK, 'Flask not installed')
class StayLoggedInTests(AccountsFixture):
    def cookie(self, response):
        header = next(v for v in response.headers.getlist('Set-Cookie') if v.startswith('session='))
        return SimpleCookie(header)['session']

    def test_login_cookie_outlives_the_browser(self):
        response = self.login(self.app().test_client(), *ADMIN)
        cookie = self.cookie(response)
        self.assertTrue(cookie['expires'])                     # 만료 시각이 있어야 브라우저를 닫아도 남는다
        from rescue_console.app import SESSION_DAYS
        self.assertEqual(7, SESSION_DAYS)

    def test_a_new_client_with_the_cookie_is_still_logged_in(self):
        """폰 브라우저를 닫았다 연 것과 같다. 같은 비밀키로 다시 띄운 서버도 그 쿠키를 받는다."""
        client = self.app().test_client()
        self.login(client, *ADMIN)
        value = client.get_cookie('session').value
        again = self.app().test_client()
        again.set_cookie('session', value)
        self.assertEqual(200, again.get('/').status_code)

    def test_logout_still_ends_it(self):
        client = self.app().test_client()
        self.login(client, *ADMIN)
        client.post('/logout', data={'csrf': self.csrf(client, '/')})
        self.assertEqual(302, client.get('/').status_code)


if __name__ == '__main__':
    unittest.main()
