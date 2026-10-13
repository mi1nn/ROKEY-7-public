"""Flask 경계 모듈(선택 의존성).

모든 페이지는 로그인이 필요하다. ADMIN 은 /admin/accounts 에서 계정을 관리하고 운영자 명령을
보낸다(POST /commands, operations 로 넘김). CUSTOMER 는 보기만 한다. 데이터 경로는 GET 만 받는다.
관제 웹은 operations 장부에 아무것도 쓰지 않는다. 폼과 명령은 세션마다 하나인 CSRF 토큰을 싣는다.
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
from .sources import load_ledger, load_robot_watch, load_site_state, load_status
from .throttle import LoginThrottle
from .view import ConsoleError, build_view


# 로그인 페이지로 보내지 않고 401 로 답하는 경로(폴링하는 데이터, 지도 이미지, 명령).
DATA_PATHS = ('/fragment', '/map.png', '/commands')
# 모든 응답에 붙인다([제안] 관제·화면, 2026-10-10). 페이지는 다른 사이트의 frame 안에 들어가지 않고,
# 내용 추측(content-sniffing)을 당하지 않으며, 카메라 URL 로 referrer 를 흘리지 않는다. 스크립트까지 막는 완전한 CSP 는
# 인라인 script 와 style 에 먼저 nonce 를 붙여야 쓸 수 있다(README 「후속」).
SECURITY_HEADERS = {
    'X-Content-Type-Options': 'nosniff',
    'X-Frame-Options': 'DENY',
    'Referrer-Policy': 'no-referrer',
    'Content-Security-Policy': "frame-ancestors 'none'; base-uri 'none'; form-action 'self'; object-src 'none'",
}


def _check_sources(ledger_path, status_path, site_state_path):
    """관제 웹이 보일 수 없는 입력 조합을 거부한다. ConsoleError INVALID_INPUT 을 낸다.

    상태는 정확히 한 곳(현장 상태 DB 또는 픽스처)에서 오고, 장부나 현장 상태 DB 중 하나는
    있어야 한다. 기동 때(create_app) 검사하고 폴링마다(current_view) 다시 검사한다.
    """
    if site_state_path is not None and status_path is not None:
        raise ConsoleError('INVALID_INPUT', 'give the site-state DB or a status fixture, not both')
    # 현장 상태 DB 가 있으면 옛 ReportLedger 는 선택이다(아직 그것을 쓰는 operations 코드가 없다,
    # 2026-10-13). 픽스처(E2)는 여전히 장부와 함께 온다. 장부를 줬으면 열려야 한다.
    if ledger_path is None and site_state_path is None:
        raise ConsoleError('INVALID_INPUT', 'a ledger is required unless --site-state is given')


def current_view(ledger_path, status_path=None, clock=time.time_ns, *, site_state_path=None, map_image=None,
                 map_source=None, robot_names=None, support_dispatch=None, search_start=None, robot_watch_path=None,
                 robot_watch_stale_s=None):
    """폴링 한 번. 상태는 현장 상태 DB 에서, 또는 (E2, operations 없음) 픽스처에서 온다.

    입력을 캐시 없이 새로 읽고 `build_view` 로 화면 모델을 만든 뒤, 지도가 있으면 로봇, 대상,
    가장 새 출구(QR) 관측을 지도 위에 얹는다. 다른 frame 의 위치나 지도 밖 위치는
    경고가 된다. 읽기나 모양 실패는 ConsoleError(STORAGE_ERROR / INVALID_INPUT)를 내고, 앱이
    그것을 503 으로 바꾼다.
    """
    _check_sources(ledger_path, status_path, site_state_path)
    status = load_site_state(site_state_path) if site_state_path is not None else load_status(status_path)
    ledger = None if ledger_path is None else load_ledger(ledger_path)
    watch = None if robot_watch_path is None else dict(zip(('data', 'problem'), load_robot_watch(robot_watch_path)),
                                                       stale_s=robot_watch_stale_s)
    view = build_view(ledger, status, built_at_ns=clock(), robot_names=robot_names, support_dispatch=support_dispatch,
                      search_start=search_start, robot_watch=watch)
    live = None
    if map_source is not None:   # --live-map: 마지막으로 쓰인 SLAM 지도와 그 나이·상태
        map_image, live = map_source.current()
    if map_image is not None:
        view['map'] = map_overlay(map_image.meta, view['robots'], view['targets'], view['exit'])
        # 표식을 조용히 옮기거나 빼지 않는다: map_overlay 가 놓지 못한 것은 화면에 알린다.
        for where in view['map']['frame_mismatch']:
            view['warnings'].append(f"지도에 안 그림: {where['kind']} {where['id']} frame {where['frame_id']!r} ≠ 지도 frame")
        for where in view['map']['off_map']:
            view['warnings'].append(f"지도 밖 좌표: {where['kind']} {where['id']} ({where['x']:.2f}, {where['y']:.2f})")
    if live is not None:
        # 실시간 지도가 아직 안 왔을 때도 그렇다: 그때 지도 칸은 실시간 지도 메시지(MISSING / ERROR)를 보인다.
        view['map']['live'] = live
        if live['message']:
            view['warnings'].append(live['message'])
    return view


def parse_robot_name(text):
    """'robot_a=TB-6' -> ('robot_a', 'TB-6'): 로봇 ID 와 화면에 보일 그 로봇의 이름(소유자 2026-10-13)."""
    robot_id, sep, name = text.partition('=')
    if not sep or not robot_id.strip() or not name.strip():
        raise ConsoleError('INVALID_INPUT', f'robot name must be ROBOT_ID=NAME: {text!r}')
    return robot_id.strip(), name.strip()


def robot_name_map(pairs):
    """(robot_id, name) 쌍 -> {robot_id: name}. 같은 로봇 ID 를 두 번 주면 마지막 값을 쓰지 않고 INVALID_INPUT 이다."""
    names = {}
    for robot_id, name in pairs:
        if robot_id in names:
            raise ConsoleError('INVALID_INPUT', f'robot name given twice for {robot_id!r}')
        names[robot_id] = name
    return names


def parse_camera(text):
    """'cam_a=http://host:port/path' -> ('cam_a', url). http(s) 만 받고 아무것도 추측하지 않는다."""
    camera_id, sep, url = text.partition('=')
    if not sep or not camera_id or not url.startswith(('http://', 'https://')):
        raise ConsoleError('INVALID_INPUT', f'camera must be ID=http(s)://... : {text!r}')
    return camera_id, url


def create_app(ledger_path, status_path=None, *, accounts_path, secret_key, site_state_path=None, cameras=(),
               map_path=None, live_map_path=None, live_map_stale_s=None, login_throttle=None, robot_names=None, support_dispatch=None, search_start=None, operations_url=None,
               command_transport=None, camera_stale_s=None, robot_watch_path=None, robot_watch_stale_s=None):
    """설정을 검사한 뒤 로그인·관제·명령·계정 경로를 가진 Flask 앱을 만든다.

    틀릴 수 있는 설정은 모두 여기서 ConsoleError 로 실패한다. 그래서 잘못된 플래그는 첫 요청이 아니라
    기동을 멈춘다. 장부, 현장 상태 DB, 픽스처는 여기서 열지 않는다: 폴링마다 읽고,
    거기서 난 실패는 그 요청의 503 이 된다.
    `operations_url`: operations HTTP API 의 기본 URL. 없으면 명령 버튼은 꺼진 채다
    (기본 주소는 없다). 시험에서 `command_transport` 는 HTTP POST 를, `login_throttle` 은
    기본 LoginThrottle 을 대신한다(시험용 시계).
    """
    cameras = tuple(cameras)
    camera_stale_ms(cameras, camera_stale_s)   # 선택 의존성 Flask 를 import 하기 전에 설정을 검증한다
    try:
        from flask import Flask, Response, abort, flash, get_flashed_messages, jsonify, redirect, request, session
    except ImportError as exc:
        raise ConsoleError('DEPENDENCY_MISSING',
                           'Flask is required for the web console: python3 -m pip install flask') from exc
    if not isinstance(secret_key, str) or len(secret_key) < 16:
        raise ConsoleError('CONFIG_MISSING', 'secret_key (>= 16 chars) is required to sign login sessions')
    _check_sources(ledger_path, status_path, site_state_path)
    accounts = AccountStore.open(accounts_path)   # 첫 로그인 때가 아니라 지금 실패한다
    map_image = None if map_path is None else load_map(map_path)   # 마찬가지로 잘못된 지도는 기동 때 실패한다
    if map_path is not None and live_map_path is not None:
        raise ConsoleError('INVALID_INPUT', '--map and --live-map are exclusive: one map panel')
    # 기동 때 실시간 지도는 아직 없을 수 있다(SLAM 지도를 아직 못 받음). 그때는 페이지가 그렇게 알린다.
    live_map = None if live_map_path is None else LiveMap(live_map_path, stale_s=live_map_stale_s)
    operations = None if operations_url is None else parse_operations_url(operations_url)
    transport = post_json if command_transport is None else command_transport
    app = Flask(__name__)
    # Flask 는 이 키로 서명한 쿠키에 세션을 둔다. 서버 쪽 세션 저장소는 없다.
    app.secret_key = secret_key
    # HttpOnly: 페이지 스크립트가 쿠키를 읽지 못한다. SameSite=Lax: 다른 사이트에서 온 POST 에는 쿠키가 실리지 않는다
    # (그래도 아래의 CSRF 토큰은 검사한다). Secure 는 켜지 않는다: 관제 웹은 평문 http 로 서비스한다.
    app.config.update(SESSION_COOKIE_HTTPONLY=True, SESSION_COOKIE_SAMESITE='Lax')
    throttle = LoginThrottle() if login_throttle is None else login_throttle

    @app.after_request
    def harden(response):
        """모든 응답에 SECURITY_HEADERS 와 `Cache-Control: no-store` 를 붙인다.

        `setdefault` 를 쓰므로 경로가 직접 정한 헤더(지도 이미지의 캐시 정책)는 그대로 남는다.
        """
        for name, value in SECURITY_HEADERS.items():
            response.headers.setdefault(name, value)
        # 실시간 데이터와 로그인 페이지는 캐시에서 내주지 않는다. /map.png 는 자기 정책을 따로 정한다.
        response.headers.setdefault('Cache-Control', 'no-store')
        return response

    def view():
        """기동 때 정한 입력으로 만든 이 요청의 화면 모델(`current_view` 한 번 호출)."""
        return current_view(ledger_path, status_path, site_state_path=site_state_path, map_image=map_image,
                            map_source=live_map,
                            robot_names=robot_names, support_dispatch=support_dispatch,
                            search_start=search_start, robot_watch_path=robot_watch_path,
                            robot_watch_stale_s=robot_watch_stale_s)

    def user():
        """이 세션의 계정, 없으면 None.

        쿠키는 (account_id, auth_version) 을 싣는다. 지웠다가 다시 만든 이름은 id 가 새로
        바뀌고, 비밀번호·역할·상태를 바꾸면 version 이 올라간다. 어느 쪽이든 그 전에 발급한
        쿠키는 클라이언트가 무엇을 들고 있든 서버의 이 검사에서 더는 맞지 않는다(AST-50-01).
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
        """이 세션의 CSRF 토큰. 처음 쓸 때 만들고(무작위 32 바이트) 세션을 비울 때까지 유지한다."""
        if 'csrf' not in session:
            session['csrf'] = secrets.token_urlsafe(32)
        return session['csrf']

    def check_csrf():
        """폼의 `csrf` 필드가 세션의 토큰과 같지 않으면 400 으로 중단한다."""
        # 상수 시간 비교라서 응답 시간으로 추측이 얼마나 맞았는지 드러나지 않는다. 토큰이 없는
        # 세션은 '-' 와 비교하므로, 빠지거나 빈 필드는 절대 통과하지 않는다.
        if not secrets.compare_digest(request.form.get('csrf', ''), session.get('csrf', '-')):
            abort(400, 'csrf token mismatch')

    def login_required(handler):
        """경로 데코레이터: 유효한 세션일 때만 `handler(account, ...)` 를 실행한다(`user` 참고).

        유효한 세션이 없으면 데이터 경로는 401 JSON 으로 답하고 페이지는 /login 으로 보낸다.
        """
        @functools.wraps(handler)
        def wrapped(*args, **kwargs):
            """요청마다 세션의 계정을 찾아 경로 함수에 첫 인자로 넘긴다."""
            account = user()
            if account is None:
                # 데이터 경로는 401 로 답해서 폴링하는 쪽이 로그인 문서를 삼키지 않게 한다(AST-50-02).
                # 페이지 전체만 로그인 페이지로 보낸다.
                if request.path.startswith('/api/') or request.path in DATA_PATHS:
                    return jsonify({'error': 'login required'}), 401
                return redirect('/login')
            return handler(account, *args, **kwargs)
        return wrapped

    def admin_required(handler):
        """경로 데코레이터: `login_required` 를 거친 뒤 계정의 역할이 ADMIN 이 아니면 403.

        역할은 이 요청에서 읽은 계정 DB 행(`user`)에서 오고, 쿠키에서 오지 않는다.
        """
        @functools.wraps(handler)
        @login_required
        def wrapped(account, *args, **kwargs):
            if account['role'] != 'ADMIN':
                abort(403, 'admin only')
            return handler(account, *args, **kwargs)
        return wrapped

    @app.errorhandler(ConsoleError)
    def failed(exc):
        """요청을 처리하다 난 ConsoleError 를 요청한 쪽에 맞는 모양의 503 으로 바꾼다.

        `/` 는 실패 페이지(스스로 새로고침), `/api/...` 는 {error, message, hint} JSON,
        그 밖의 경로(폴링하는 조각, 지도, 계정 페이지)는 평문 한 줄을 받는다.
        """
        # 페이지에는 무엇을 확인할지 적은 페이지를, 폴링하는 쪽에는 보일 수 있는 한 줄을,
        # API 에는 코드를 준다. 모두 503 이라서 어느 것도 정상인 빈 화면으로 읽히지 않는다.
        if request.path == '/':
            return Response(render_failure(exc.detail_code, exc.detail), status=503, mimetype='text/html')
        if request.path.startswith('/api/'):
            return jsonify({'error': exc.detail_code, 'message': exc.detail,
                            'hint': FAILURE_HINTS.get(exc.detail_code, FAILURE_HINT_OTHER)}), 503
        return Response(failure_text(exc.detail_code, exc.detail), status=503, mimetype='text/plain')

    # ---- 로그인 ----------------------------------------------------------------------
    @app.get('/login')
    def login_form():
        """로그인 페이지(CSRF 토큰과 flash 메시지 포함). 이미 로그인한 사용자는 바로 / 로 보낸다."""
        if user() is not None:
            return redirect('/')
        return Response(render_login(csrf_token(), get_flashed_messages(with_categories=True)), mimetype='text/html')

    @app.post('/login')
    def login():
        """로그인 폼을 검사하고 세션을 시작한다. CSRF 검사를 지나면 어떤 결과든 redirect 다.

        순서: CSRF, 그다음 (이름, 클라이언트 주소) 잠금, 그다음 비밀번호. 틀린 이름, 틀린
        비밀번호, 비활성 계정은 같은 메시지를 받는다(AccountStore.authenticate 가 None 을 돌려준다).
        """
        check_csrf()               # 로그인 폼에도 세션의 토큰이 필요하다(GET /login 이 발급)
        username, client = request.form.get('username', ''), request.remote_addr
        wait = throttle.locked_for(username, client)
        if wait:
            # 비밀번호보다 먼저 검사하므로, 잠긴 이름으로는 추측이 맞았는지 알 수 없다.
            flash(f'로그인을 너무 많이 틀렸습니다. {wait}초 뒤에 다시 하세요.', 'danger')
            return redirect('/login')
        account = accounts.authenticate(username, request.form.get('password', ''))
        if account is None:
            throttle.failed(username, client)
            flash('아이디 또는 비밀번호가 틀립니다.', 'danger')
            return redirect('/login')
        throttle.succeeded(username, client)
        # 빈 세션에서 시작한다: 로그인 전의 것(옛 CSRF 토큰, flash 메시지)은 하나도 넘어오지
        # 않는다. 그 뒤 쿠키는 계정 id, auth_version, CSRF 토큰을 가지며, `user` 가 앞의 둘을
        # 요청마다 계정 DB 와 다시 맞춰 본다.
        session.clear()
        session['account_id'] = account['account_id']
        session['auth_version'] = account['auth_version']
        csrf_token()               # 새 세션의 새 토큰
        return redirect('/')

    @app.post('/logout')
    def logout():
        """세션을 끝내고(POST 만) 로그인 페이지로 보낸다."""
        check_csrf()               # 링크나 이미지 태그로는 운영자를 로그아웃시킬 수 없다(AST-50-03)
        session.clear()
        return redirect('/login')

    # ---- 관제 화면 (로그인한 모든 역할) -----------------------------------------------
    @app.get('/')
    @login_required
    def page(account):
        """관제 페이지 전체. 명령 칸은 역할과 --operations 를 줬는지에 따라 켜지거나 꺼진다."""
        return Response(render_page(view(), account, csrf=csrf_token(), cameras=cameras, camera_stale_s=camera_stale_s,
                                    command_access=command_access(account['role'], operations is not None)),
                        mimetype='text/html')

    @app.get('/fragment')
    @login_required
    def fragment(account):
        """페이지에서 계속 바뀌는 부분을 HTML 로 낸다. 페이지 스크립트가 폴링마다 받아 갈아 끼운다."""
        return Response(render_fragment(view()), mimetype='text/html')

    @app.get('/api/view')
    @login_required
    def api_view(account):
        """화면 모델을 원래 코드 그대로 JSON 으로 낸다(페이지를 그리는 바탕)."""
        return jsonify(view())

    @app.get('/map.png')
    @login_required
    def map_png(account):
        """지도 이미지를 PNG 로 낸다: 마지막으로 읽은 실시간 SLAM 지도, 또는 고정 --map 이미지.

        지도를 설정하지 않았거나 실시간 지도가 아직 오지 않았으면 404.
        """
        image = live_map.current()[0] if live_map is not None else map_image
        if image is None:
            abort(404, 'no map: start with --map MAP_YAML or --live-map LIVE_YAML')
        # URL 에 PGM 해시(`?v=`)가 실리므로 캐시된 사본이 다른 지도일 수 없다.
        return Response(image.png, mimetype='image/png', headers={'Cache-Control': 'private, max-age=3600'})

    # ---- 운영자 명령 (ADMIN) -----------------------------------------------------
    @app.post('/commands')
    @login_required
    def command(account):
        """한 번 누름 -> operations 로 POST 한 번. 답이나 실패는 JSON 으로 페이지에 돌려준다.

        여기서는 재시도·추측·쓰기를 하지 않는다. 명령은 operations 가 기록하고(operator_commands)
        조정기가 결정한다. 보내기 전에 거부하면 sent=false 로 그렇게 알린다.
        """
        # admin_required 가 아니라 login_required 다: 페이지 스크립트는 JSON 을 읽으므로, 모든 거부(역할 포함)가
        # HTML 오류 페이지가 아니라 같은 {sent: false, error, message} 모양으로 돌아온다.
        def refuse(code, message, status):
            app.logger.warning('command refused before sending: %s %s (%s)', code, message, account['username'])
            return jsonify({'sent': False, 'error': code, 'message': message}), status
        # check_csrf 와 같은 검사이고, 같은 이유로 JSON 으로 답한다.
        if not secrets.compare_digest(request.form.get('csrf', ''), session.get('csrf', '-')):
            return refuse('CSRF', 'csrf token mismatch — reload the page', 400)
        allowed, reason = command_access(account['role'], operations is not None)
        if not allowed:
            # CUSTOMER 는 금지(403). --operations 없는 ADMIN 은 보낼 곳이 없다(503).
            return refuse('NOT_AUTHORIZED' if account['role'] != 'ADMIN' else 'NOT_CONNECTED', reason,
                          403 if account['role'] != 'ADMIN' else 503)
        try:
            # 운영자는 로그인한 계정이고 폼 필드가 아니다. 누를 때마다 새 command_id: 다시 누르면
            # 새 명령이고, 운영자는 이 id 로 명령 기록에서 이번 명령을 찾는다.
            payload = build_request(request.form, operator_id=account['username'], command_id=str(uuid.uuid4()))
        except ConsoleError as exc:
            return refuse(exc.detail_code, exc.detail, 400)
        result = send_command(operations, payload, transport=transport)
        log = app.logger.info if result['sent'] else app.logger.warning
        log('operator command %s %s by %s -> %s', payload['kind'], payload['command_id'], account['username'],
            {key: value for key, value in result.items() if key not in ('command_id', 'kind')})
        # 거절이어도 200: operations 가 답했기 때문이다. 답하지 않았으면(또는 기대한 모양이 아니면) 502.
        return jsonify(result), 200 if result['sent'] else 502

    # ---- 계정 (ADMIN) ------------------------------------------------------------
    def accounts_page(account, status=200):
        """계정 목록 페이지. 직전 폼 POST 가 남긴 flash 메시지를 같이 보인다."""
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
        """관리자 폼으로 계정을 더한 뒤 목록으로 돌아간다.

        저장소가 모든 필드(이름 형식, 비밀번호 길이, 역할, 중복 이름)를 검사한다. 거기서 난
        ConsoleError 는 오류 페이지가 아니라 목록 페이지의 flash 메시지가 된다.
        """
        check_csrf()
        form = request.form
        try:
            accounts.add(form.get('username', ''), form.get('password', ''), role=form.get('role', 'CUSTOMER'),
                         display_name=form.get('display_name', ''), email=form.get('email', ''))
            flash(f'계정 {form.get("username", "")} 추가', 'success')
        except ConsoleError as exc:
            flash(f'추가 실패: {exc}', 'danger')
            return redirect('/admin/accounts', code=303)
        # 303 See Other: 브라우저가 GET 으로 따라가므로 목록을 새로고침해도 폼을 다시 보내지 않는다.
        return redirect('/admin/accounts', code=303)

    @app.post('/admin/accounts/<username>')
    @admin_required
    def accounts_change(account, username):
        """계정 하나를 지우거나 고친 뒤(폼 필드 `_action`) 목록으로 돌아간다(303).

        저장소는 로그인한 계정을 지우는 것과 마지막 활성 ADMIN 을 없애는 것을 거부한다. 비밀번호,
        역할, 상태를 바꾸면 auth_version 이 올라가 그 계정의 기존 세션이 끝난다(`user`).
        저장소의 거부는 flash 메시지가 되고, 모르는 `_action` 은 400 이다.
        """
        check_csrf()
        form = request.form
        try:
            if form.get('_action') == 'delete':
                accounts.delete(username, by=account['username'])
                flash(f'계정 {username} 삭제', 'info')
            elif form.get('_action') == 'update':
                # 빈 비밀번호 칸은 「빈 비밀번호로 바꿈」이 아니라 「비밀번호 유지」(None)를 뜻한다.
                accounts.update(username, display_name=form.get('display_name', ''), email=form.get('email', ''),
                                role=form.get('role', ''), status=form.get('status', ''),
                                password=form.get('password') or None)
                flash(f'계정 {username} 저장', 'success')
            else:
                abort(400, 'unknown _action')   # HTTPException 이라서 아래 except 가 잡지 않는다
        except ConsoleError as exc:
            flash(f'변경 실패: {exc}', 'danger')
        return redirect('/admin/accounts', code=303)

    return app
