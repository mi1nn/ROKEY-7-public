"""화면 이동 줄(재범 2026-10-16 「네비게이션바로 리플레이랑 대시보드 잘 연결해야할 것 같은데」).

실시간 관제(/), 회차 다시 보기(/replay), 회차 초기화(/reset, ADMIN), 계정 관리(/admin/accounts, ADMIN) 네
화면이 맨 위에 같은 이동 줄을 쓴다. 지금 화면은 aria-current="page" 로 표시하고, 로그인 이름과 로그아웃도 같은 줄
오른쪽에 둔다. ADMIN 이 아니면 초기화·계정 관리 칸이 없다. 화면마다 달랐던 「관제 화면으로」 링크는 없앤다.
"""
import re
import unittest

from test_console_replay import ReplayFixture
from test_console_web import ADMIN, CUSTOMER, HAS_FLASK, SECRET, AccountsFixture

PAGES = (('/', '실시간 관제'), ('/replay', '회차 다시 보기'), ('/reset', '회차 초기화'),
         ('/admin/accounts', '계정 관리'))


def tabs(html):
    """이동 줄의 (주소, 이름, 지금 화면인가) 목록."""
    nav = re.search(r'<nav class="tabs"[^>]*>(.*?)</nav>', html, re.S)
    if nav is None:
        return None
    return [(href, text, 'aria-current="page"' in attrs)
            for href, attrs, text in re.findall(r'<a href="([^"]+)"([^>]*)>([^<]+)</a>', nav.group(1))]


@unittest.skipUnless(HAS_FLASK, 'Flask not installed: navigation NOT_RUN')
class NavBarTests(ReplayFixture, AccountsFixture):
    def client(self, account):
        from rescue_console.app import create_app
        app = create_app(None, accounts_path=self.accounts, secret_key=SECRET, site_state_path=self.site_db)
        client = app.test_client()
        self.login(client, *account)
        return client

    def test_admin_sees_every_screen_and_the_current_one_is_marked(self):
        client = self.client(ADMIN)
        for path, name in PAGES:
            with self.subTest(path=path):
                html = client.get(path).get_data(as_text=True)
                self.assertEqual([(p, n, p == path) for p, n in PAGES], tabs(html))

    def test_customer_has_no_reset_or_accounts_tab(self):
        client = self.client(CUSTOMER)
        for path in ('/', '/replay', '/reset'):
            with self.subTest(path=path):
                html = client.get(path).get_data(as_text=True)
                self.assertEqual([('/', '실시간 관제'), ('/replay', '회차 다시 보기')],
                                 [(p, n) for p, n, _current in tabs(html)])

    def test_name_and_logout_sit_in_the_same_bar_and_the_old_back_link_is_gone(self):
        client = self.client(ADMIN)
        for path, _name in PAGES:
            with self.subTest(path=path):
                html = client.get(path).get_data(as_text=True)
                bar = re.search(r'<header id="masthead".*?</header>', html, re.S).group(0)
                self.assertIn('action="/logout"', bar)
                self.assertIn('(ADMIN)', bar)
                self.assertNotIn('관제 화면으로', html)


if __name__ == '__main__':
    unittest.main()
