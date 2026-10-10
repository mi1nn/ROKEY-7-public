"""Flask boundary (optional dependency).

Login is required for every page. ADMIN manages accounts at /admin/accounts; CUSTOMER
only views. Data routes stay GET-only; the console still writes nothing to the
operations ledgers. Forms carry a per-session CSRF token.
"""

import functools
import secrets
import time

from .accounts import AccountStore
from .render import render_accounts, render_fragment, render_login, render_page
from .sources import load_ledger, load_site_state, load_status
from .view import ConsoleError, build_view


def current_view(ledger_path, status_path=None, clock=time.time_ns, *, site_state_path=None):
    """One poll. Status comes from the site-state DB, or (E2, no operations) from a fixture."""
    if site_state_path is not None and status_path is not None:
        raise ConsoleError('INVALID_INPUT', 'give the site-state DB or a status fixture, not both')
    status = load_site_state(site_state_path) if site_state_path is not None else load_status(status_path)
    return build_view(load_ledger(ledger_path), status, built_at_ns=clock())


def parse_camera(text):
    """'cam_a=http://host:port/path' -> ('cam_a', url). Only http(s); nothing is guessed."""
    camera_id, sep, url = text.partition('=')
    if not sep or not camera_id or not url.startswith(('http://', 'https://')):
        raise ConsoleError('INVALID_INPUT', f'camera must be ID=http(s)://... : {text!r}')
    return camera_id, url


def create_app(ledger_path, status_path=None, *, accounts_path, secret_key, site_state_path=None, cameras=()):
    try:
        from flask import Flask, Response, abort, flash, get_flashed_messages, jsonify, redirect, request, session
    except ImportError as exc:
        raise ConsoleError('DEPENDENCY_MISSING',
                           'Flask is required for the web console: python3 -m pip install flask') from exc
    if not isinstance(secret_key, str) or len(secret_key) < 16:
        raise ConsoleError('CONFIG_MISSING', 'secret_key (>= 16 chars) is required to sign login sessions')
    if site_state_path is not None and status_path is not None:
        raise ConsoleError('INVALID_INPUT', 'give the site-state DB or a status fixture, not both')
    cameras = tuple(cameras)
    accounts = AccountStore.open(accounts_path)   # fails now, not at the first login
    app = Flask(__name__)
    app.secret_key = secret_key
    app.config.update(SESSION_COOKIE_HTTPONLY=True, SESSION_COOKIE_SAMESITE='Lax')

    def view():
        return current_view(ledger_path, status_path, site_state_path=site_state_path)

    def user():
        """The account behind this session, or None.

        The cookie carries (account_id, auth_version). A deleted-and-recreated name has a new
        id; a password / role / status change bumps the version. Either way every cookie
        issued earlier stops matching here, on the server, whatever the client kept (AST-50-01).
        """
        account_id, version = session.get('account_id'), session.get('auth_version')
        if not account_id:
            return None
        account = accounts.get_by_id(account_id)
        if account is None or account['status'] != 'ACTIVE' or account['auth_version'] != version:
            session.clear()
            return None
        return account

    def csrf_token():
        if 'csrf' not in session:
            session['csrf'] = secrets.token_urlsafe(32)
        return session['csrf']

    def check_csrf():
        if not secrets.compare_digest(request.form.get('csrf', ''), session.get('csrf', '-')):
            abort(400, 'csrf token mismatch')

    def login_required(handler):
        @functools.wraps(handler)
        def wrapped(*args, **kwargs):
            account = user()
            if account is None:
                # Data routes answer 401 so the poller never swallows a login document (AST-50-02);
                # only whole pages redirect.
                if request.path.startswith('/api/') or request.path == '/fragment':
                    return jsonify({'error': 'login required'}), 401
                return redirect('/login')
            return handler(account, *args, **kwargs)
        return wrapped

    def admin_required(handler):
        @functools.wraps(handler)
        @login_required
        def wrapped(account, *args, **kwargs):
            if account['role'] != 'ADMIN':
                abort(403, 'admin only')
            return handler(account, *args, **kwargs)
        return wrapped

    @app.errorhandler(ConsoleError)
    def failed(exc):
        # Shown as a failure, never as an empty but healthy screen.
        return Response(f'console input failed: {exc}', status=503, mimetype='text/plain')

    # ---- login ----------------------------------------------------------------------
    @app.get('/login')
    def login_form():
        if user() is not None:
            return redirect('/')
        return Response(render_login(csrf_token(), get_flashed_messages(with_categories=True)), mimetype='text/html')

    @app.post('/login')
    def login():
        check_csrf()
        account = accounts.authenticate(request.form.get('username', ''), request.form.get('password', ''))
        if account is None:
            flash('아이디 또는 비밀번호가 틀립니다.', 'danger')
            return redirect('/login')
        session.clear()
        session['account_id'] = account['account_id']
        session['auth_version'] = account['auth_version']
        csrf_token()
        return redirect('/')

    @app.post('/logout')
    def logout():
        check_csrf()               # a link or an image tag cannot log the operator out (AST-50-03)
        session.clear()
        return redirect('/login')

    # ---- console (any logged-in role) -----------------------------------------------
    @app.get('/')
    @login_required
    def page(account):
        return Response(render_page(view(), account, csrf=csrf_token(), cameras=cameras), mimetype='text/html')

    @app.get('/fragment')
    @login_required
    def fragment(account):
        return Response(render_fragment(view()), mimetype='text/html')

    @app.get('/api/view')
    @login_required
    def api_view(account):
        return jsonify(view())

    # ---- accounts (ADMIN) ------------------------------------------------------------
    def accounts_page(account, status=200):
        return Response(render_accounts(accounts.list_accounts(), account, csrf_token(),
                                        get_flashed_messages(with_categories=True)),
                        status=status, mimetype='text/html')

    @app.get('/admin/accounts')
    @admin_required
    def accounts_list(account):
        return accounts_page(account)

    @app.post('/admin/accounts')
    @admin_required
    def accounts_create(account):
        check_csrf()
        form = request.form
        try:
            accounts.add(form.get('username', ''), form.get('password', ''), role=form.get('role', 'CUSTOMER'),
                         display_name=form.get('display_name', ''), email=form.get('email', ''))
            flash(f'계정 {form.get("username", "")} 추가', 'success')
        except ConsoleError as exc:
            flash(f'추가 실패: {exc}', 'danger')
            return redirect('/admin/accounts', code=303)
        return redirect('/admin/accounts', code=303)

    @app.post('/admin/accounts/<username>')
    @admin_required
    def accounts_change(account, username):
        check_csrf()
        form = request.form
        try:
            if form.get('_action') == 'delete':
                accounts.delete(username, by=account['username'])
                flash(f'계정 {username} 삭제', 'info')
            elif form.get('_action') == 'update':
                accounts.update(username, display_name=form.get('display_name', ''), email=form.get('email', ''),
                                role=form.get('role', ''), status=form.get('status', ''),
                                password=form.get('password') or None)
                flash(f'계정 {username} 저장', 'success')
            else:
                abort(400, 'unknown _action')
        except ConsoleError as exc:
            flash(f'변경 실패: {exc}', 'danger')
        return redirect('/admin/accounts', code=303)

    return app
