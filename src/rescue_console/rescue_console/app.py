"""Flask boundary (optional dependency).

Login is required for every page. ADMIN manages accounts at /admin/accounts and sends operator
commands (POST /commands, passed on to operations); CUSTOMER only views. Data routes stay GET-only;
the console writes nothing to the operations ledgers. Forms and commands carry a per-session CSRF token.
"""

import functools
import secrets
import time
import uuid

from .accounts import AccountStore
from .commands import build_request, command_access, parse_operations_url, post_json, send_command
from .mapimage import LiveMap, load_map, map_overlay
from .render import (FAILURE_HINT_OTHER, FAILURE_HINTS, camera_stale_ms, failure_text, render_accounts, render_failure,
                     render_fragment, render_login, render_page)
from .sources import load_ledger, load_site_state, load_status
from .throttle import LoginThrottle
from .view import ConsoleError, build_view


# Paths that answer 401 instead of redirecting to the login page (polled data, the map image, commands).
DATA_PATHS = ('/fragment', '/map.png', '/commands')
# Sent with every response ([제안] 관제·화면, 2026-10-10). The pages are never framed by another
# site, never content-sniffed, and leak no referrer to the camera URLs. A full script CSP needs the
# inline script and style to carry a nonce first (README 「후속」).
SECURITY_HEADERS = {
    'X-Content-Type-Options': 'nosniff',
    'X-Frame-Options': 'DENY',
    'Referrer-Policy': 'no-referrer',
    'Content-Security-Policy': "frame-ancestors 'none'; base-uri 'none'; form-action 'self'; object-src 'none'",
}


def _check_sources(ledger_path, status_path, site_state_path):
    if site_state_path is not None and status_path is not None:
        raise ConsoleError('INVALID_INPUT', 'give the site-state DB or a status fixture, not both')
    # The old ReportLedger is optional with the site-state DB (no operations code writes it yet,
    # 2026-10-13); the fixture (E2) still comes with its ledger. A given ledger must open.
    if ledger_path is None and site_state_path is None:
        raise ConsoleError('INVALID_INPUT', 'a ledger is required unless --site-state is given')


def current_view(ledger_path, status_path=None, clock=time.time_ns, *, site_state_path=None, map_image=None,
                 map_source=None, robot_names=None, support_dispatch=None):
    """One poll. Status comes from the site-state DB, or (E2, no operations) from a fixture."""
    _check_sources(ledger_path, status_path, site_state_path)
    status = load_site_state(site_state_path) if site_state_path is not None else load_status(status_path)
    ledger = None if ledger_path is None else load_ledger(ledger_path)
    view = build_view(ledger, status, built_at_ns=clock(), robot_names=robot_names, support_dispatch=support_dispatch)
    live = None
    if map_source is not None:   # --live-map: the SLAM map as last written, with its age and state
        map_image, live = map_source.current()
    if map_image is not None:
        view['map'] = map_overlay(map_image.meta, view['robots'], view['targets'])
        for where in view['map']['frame_mismatch']:
            view['warnings'].append(f"지도에 안 그림: {where['kind']} {where['id']} frame {where['frame_id']!r} ≠ 지도 frame")
        for where in view['map']['off_map']:
            view['warnings'].append(f"지도 밖 좌표: {where['kind']} {where['id']} ({where['x']:.2f}, {where['y']:.2f})")
    if live is not None:
        view['map']['live'] = live
        if live['message']:
            view['warnings'].append(live['message'])
    return view


def parse_robot_name(text):
    """'robot_a=TB-6' -> ('robot_a', 'TB-6'): the robot ID and the name shown for it (owner 2026-10-13)."""
    robot_id, sep, name = text.partition('=')
    if not sep or not robot_id.strip() or not name.strip():
        raise ConsoleError('INVALID_INPUT', f'robot name must be ROBOT_ID=NAME: {text!r}')
    return robot_id.strip(), name.strip()


def robot_name_map(pairs):
    names = {}
    for robot_id, name in pairs:
        if robot_id in names:
            raise ConsoleError('INVALID_INPUT', f'robot name given twice for {robot_id!r}')
        names[robot_id] = name
    return names


def parse_camera(text):
    """'cam_a=http://host:port/path' -> ('cam_a', url). Only http(s); nothing is guessed."""
    camera_id, sep, url = text.partition('=')
    if not sep or not camera_id or not url.startswith(('http://', 'https://')):
        raise ConsoleError('INVALID_INPUT', f'camera must be ID=http(s)://... : {text!r}')
    return camera_id, url


def create_app(ledger_path, status_path=None, *, accounts_path, secret_key, site_state_path=None, cameras=(),
               map_path=None, live_map_path=None, live_map_stale_s=None, login_throttle=None, robot_names=None, support_dispatch=None, operations_url=None,
               command_transport=None, camera_stale_s=None):
    """`operations_url`: base URL of operations' HTTP API. Without it the command buttons stay off
    (no default address). `command_transport` replaces the HTTP POST in tests."""
    cameras = tuple(cameras)
    camera_stale_ms(cameras, camera_stale_s)   # validate configuration before optional Flask import
    try:
        from flask import Flask, Response, abort, flash, get_flashed_messages, jsonify, redirect, request, session
    except ImportError as exc:
        raise ConsoleError('DEPENDENCY_MISSING',
                           'Flask is required for the web console: python3 -m pip install flask') from exc
    if not isinstance(secret_key, str) or len(secret_key) < 16:
        raise ConsoleError('CONFIG_MISSING', 'secret_key (>= 16 chars) is required to sign login sessions')
    _check_sources(ledger_path, status_path, site_state_path)
    accounts = AccountStore.open(accounts_path)   # fails now, not at the first login
    map_image = None if map_path is None else load_map(map_path)   # likewise: a bad map fails at startup
    if map_path is not None and live_map_path is not None:
        raise ConsoleError('INVALID_INPUT', '--map and --live-map are exclusive: one map panel')
    # The live map may not exist yet at startup (no SLAM map received); the page says so.
    live_map = None if live_map_path is None else LiveMap(live_map_path, stale_s=live_map_stale_s)
    operations = None if operations_url is None else parse_operations_url(operations_url)
    transport = post_json if command_transport is None else command_transport
    app = Flask(__name__)
    app.secret_key = secret_key
    app.config.update(SESSION_COOKIE_HTTPONLY=True, SESSION_COOKIE_SAMESITE='Lax')
    throttle = LoginThrottle() if login_throttle is None else login_throttle

    @app.after_request
    def harden(response):
        for name, value in SECURITY_HEADERS.items():
            response.headers.setdefault(name, value)
        # Live data and login pages are never served from a cache; /map.png sets its own policy.
        response.headers.setdefault('Cache-Control', 'no-store')
        return response

    def view():
        return current_view(ledger_path, status_path, site_state_path=site_state_path, map_image=map_image,
                            map_source=live_map,
                            robot_names=robot_names, support_dispatch=support_dispatch)

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
                if request.path.startswith('/api/') or request.path in DATA_PATHS:
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
        # The page gets a page that says what to check; the poller gets one line it can show;
        # the API gets the code. All 503, so nothing reads as a healthy empty screen.
        if request.path == '/':
            return Response(render_failure(exc.detail_code, exc.detail), status=503, mimetype='text/html')
        if request.path.startswith('/api/'):
            return jsonify({'error': exc.detail_code, 'message': exc.detail,
                            'hint': FAILURE_HINTS.get(exc.detail_code, FAILURE_HINT_OTHER)}), 503
        return Response(failure_text(exc.detail_code, exc.detail), status=503, mimetype='text/plain')

    # ---- login ----------------------------------------------------------------------
    @app.get('/login')
    def login_form():
        if user() is not None:
            return redirect('/')
        return Response(render_login(csrf_token(), get_flashed_messages(with_categories=True)), mimetype='text/html')

    @app.post('/login')
    def login():
        check_csrf()
        username, client = request.form.get('username', ''), request.remote_addr
        wait = throttle.locked_for(username, client)
        if wait:
            # Checked before the password, so a locked name gives no hint whether a guess was right.
            flash(f'로그인을 너무 많이 틀렸습니다. {wait}초 뒤에 다시 하세요.', 'danger')
            return redirect('/login')
        account = accounts.authenticate(username, request.form.get('password', ''))
        if account is None:
            throttle.failed(username, client)
            flash('아이디 또는 비밀번호가 틀립니다.', 'danger')
            return redirect('/login')
        throttle.succeeded(username, client)
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
        return Response(render_page(view(), account, csrf=csrf_token(), cameras=cameras, camera_stale_s=camera_stale_s,
                                    command_access=command_access(account['role'], operations is not None)),
                        mimetype='text/html')

    @app.get('/fragment')
    @login_required
    def fragment(account):
        return Response(render_fragment(view()), mimetype='text/html')

    @app.get('/api/view')
    @login_required
    def api_view(account):
        return jsonify(view())

    @app.get('/map.png')
    @login_required
    def map_png(account):
        image = live_map.current()[0] if live_map is not None else map_image
        if image is None:
            abort(404, 'no map: start with --map MAP_YAML or --live-map LIVE_YAML')
        # The URL carries the PGM hash (`?v=`), so a cached copy is never a different map.
        return Response(image.png, mimetype='image/png', headers={'Cache-Control': 'private, max-age=3600'})

    # ---- operator commands (ADMIN) -----------------------------------------------------
    @app.post('/commands')
    @login_required
    def command(account):
        """One press -> one POST to operations. The answer or the failure goes back to the page as JSON.

        Nothing here retries, guesses or writes; operations records the command (operator_commands)
        and the coordinator decides. Refusals before sending say so with sent=false.
        """
        def refuse(code, message, status):
            app.logger.warning('command refused before sending: %s %s (%s)', code, message, account['username'])
            return jsonify({'sent': False, 'error': code, 'message': message}), status
        if not secrets.compare_digest(request.form.get('csrf', ''), session.get('csrf', '-')):
            return refuse('CSRF', 'csrf token mismatch — reload the page', 400)
        allowed, reason = command_access(account['role'], operations is not None)
        if not allowed:
            return refuse('NOT_AUTHORIZED' if account['role'] != 'ADMIN' else 'NOT_CONNECTED', reason,
                          403 if account['role'] != 'ADMIN' else 503)
        try:
            payload = build_request(request.form, operator_id=account['username'], command_id=str(uuid.uuid4()))
        except ConsoleError as exc:
            return refuse(exc.detail_code, exc.detail, 400)
        result = send_command(operations, payload, transport=transport)
        log = app.logger.info if result['sent'] else app.logger.warning
        log('operator command %s %s by %s -> %s', payload['kind'], payload['command_id'], account['username'],
            {key: value for key, value in result.items() if key not in ('command_id', 'kind')})
        return jsonify(result), 200 if result['sent'] else 502

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
