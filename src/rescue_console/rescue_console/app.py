"""Flask 경계 모듈(선택 의존성).

모든 페이지는 로그인이 필수다. ADMIN은 /admin/accounts에서 계정을 관리하고 운영자 명령을
전송한다(POST /commands, operations로 중계). CUSTOMER는 열람만 가능하다. 데이터 엔드포인트는 GET만 허용된다.
관제 웹은 operations 원장에 일체 직접 쓰기를 하지 않는다. 모든 폼과 명령 요청은 세션별 CSRF 토큰을 검증한다.
"""

import functools
import secrets
import time
import urllib.parse
import uuid

from .accounts import AccountStore
from .commands import (build_request, command_access, parse_operations_url, post_json, robot_camera_state,
                       select_robot_camera, send_command, switch_webcam, webcam_state)
from .mapimage import LiveMap, load_map, map_overlay
from .render import (FAILURE_HINT_OTHER, FAILURE_HINTS, camera_stale_ms, failure_text, render_accounts, render_failure,
                     render_fragment, render_login, render_page)
from .replay import ClosedRunCache, build_timeline, replay_at, run_choices
from .replay_page import render_replay_fragment, render_replay_page
from .sources import load_ledger, load_replay, load_robot_watch, load_runs, load_site_state, load_status
from .throttle import LoginThrottle
from .view import ConsoleError, build_view


# 로그인 페이지로 리다이렉트하지 않고 401로 직접 응답할 경로 목록(폴링 데이터, 지도 이미지, 명령).
DATA_PATHS = ('/fragment', '/map.png', '/commands', '/replay/fragment')
# 기억해 두는 닫힌 회차 시간 막대 수([제안] 관제·화면, 2026-10-15). 넘치면 가장 먼저 넣은 것을 버린다.
REPLAY_CACHE_RUNS = 4
# 모든 응답에 적용할 보안 헤더([제안] 관제·화면, 2026-10-10). 타 사이트 iframe 임베딩 방지,
# MIME 타입 추측(content-sniffing) 차단, 외부 카메라 URL에 referrer 누출 방지 등을 포함한다. 완전한 스크립트 실행 차단 CSP는
# 인라인 script 및 style에 사전 nonce가 적용된 후 적용할 수 있다(README 「후속」).
SECURITY_HEADERS = {
    'X-Content-Type-Options': 'nosniff',
    'X-Frame-Options': 'DENY',
    'Referrer-Policy': 'no-referrer',
    'Content-Security-Policy': "frame-ancestors 'none'; base-uri 'none'; form-action 'self'; object-src 'none'",
}


def _check_sources(ledger_path, status_path, site_state_path):
    """관제 웹이 수용할 수 없는 입력 소스 조합을 검증한다. 위반 시 ConsoleError INVALID_INPUT 발생.

    상태 소스는 정확히 하나(현장 상태 DB 또는 픽스처)여야 하며, 장부 또는 현장 상태 DB 중 최소 하나는
    지정되어야 한다. 기동 시(create_app) 및 매 폴링 시(current_view) 반복 검증한다.
    """
    if site_state_path is not None and status_path is not None:
        raise ConsoleError('INVALID_INPUT', 'give the site-state DB or a status fixture, not both')
    # 현장 상태 DB가 지정되면 레거시 ReportLedger는 선택 사항이다(해당 기능을 사용하는 operations 코드가 아직 없음,
    # 2026-10-13). 픽스처(E2) 모드는 여전히 장부와 함께 제공되어야 한다. 장부가 지정되었으면 정상 로드되어야 한다.
    if ledger_path is None and site_state_path is None:
        raise ConsoleError('INVALID_INPUT', 'a ledger is required unless --site-state is given')


def current_view(ledger_path, status_path=None, clock=time.time_ns, *, site_state_path=None, map_image=None,
                 map_source=None, robot_names=None, support_dispatch=None, search_start=None, robot_watch_path=None,
                 robot_watch_stale_s=None, robot_watch_map_robot=None):
    """단일 폴링 뷰 구성. 상태 정보는 현장 상태 DB 또는 (E2 모드, operations 부재 시) 픽스처에서 읽어온다.

    입력 소스를 캐시 없이 새로 읽어 `build_view`로 화면 모델을 생성하며, 지도가 지정된 경우 로봇, 타깃,
    최신 출구(QR) 마커를 지도 위에 투영한다. frame 불일치 좌표나 지도 경계 밖 좌표는
    경고로 분류한다. 읽기 실패나 스키마 불일치 시 ConsoleError(STORAGE_ERROR / INVALID_INPUT)를 발생시키며, 앱 계층에서
    이를 503 응답으로 변환한다.
    """
    _check_sources(ledger_path, status_path, site_state_path)
    status = load_site_state(site_state_path) if site_state_path is not None else load_status(status_path)
    ledger = None if ledger_path is None else load_ledger(ledger_path)
    watch = None if robot_watch_path is None else dict(zip(('data', 'problem'), load_robot_watch(robot_watch_path)),
                                                       stale_s=robot_watch_stale_s, map_robot=robot_watch_map_robot)
    view = build_view(ledger, status, built_at_ns=clock(), robot_names=robot_names, support_dispatch=support_dispatch,
                      search_start=search_start, robot_watch=watch)
    live = None
    if map_source is not None:   # --live-map: 마지막으로 수신된 SLAM 지도 및 갱신 주기·상태
        map_image, live = map_source.current()
    if map_image is not None:
        view['map'] = map_overlay(map_image.meta, view['robots'], view['targets'], view['exit'], view['passages'])
        # 마커 좌표를 임의로 보정하거나 누락하지 않음: map_overlay 투영 실패 항목은 화면에 경고로 노출한다.
        for where in view['map']['frame_mismatch']:
            view['warnings'].append(f"지도에 안 그림: {where['kind']} {where['name']} frame {where['frame_id']!r} ≠ 지도 frame")
        for where in view['map']['off_map']:
            view['warnings'].append(f"지도 밖 좌표: {where['kind']} {where['name']} ({where['x']:.2f}, {where['y']:.2f})")
    if live is not None:
        # 실시간 지도가 미수신 상태일 때도 포함: 지도 패널에 실시간 지도 상태 메시지(MISSING / ERROR)를 표시한다.
        view['map']['live'] = live
        if live['message']:
            view['warnings'].append(live['message'])
    return view


def parse_robot_name(text):
    """'robot_a=TB-6' -> ('robot_a', 'TB-6'): 로봇 식별자 및 화면 표시용 로봇 명칭 파싱(소유자 2026-10-13)."""
    robot_id, sep, name = text.partition('=')
    if not sep or not robot_id.strip() or not name.strip():
        raise ConsoleError('INVALID_INPUT', f'robot name must be ROBOT_ID=NAME: {text!r}')
    return robot_id.strip(), name.strip()


def robot_name_map(pairs):
    """(robot_id, name) 쌍 목록 -> {robot_id: name}. 동일 로봇 식별자 중복 시 덮어쓰지 않고 INVALID_INPUT 예외 발생."""
    names = {}
    for robot_id, name in pairs:
        if robot_id in names:
            raise ConsoleError('INVALID_INPUT', f'robot name given twice for {robot_id!r}')
        names[robot_id] = name
    return names


def parse_camera(text):
    """'cam_a=http://host:port/path' -> ('cam_a', url). http(s) 스키마만 허용하며 기본값을 추측하지 않는다."""
    camera_id, sep, url = text.partition('=')
    if not sep or not camera_id or not url.startswith(('http://', 'https://')):
        raise ConsoleError('INVALID_INPUT', f'camera must be ID=http(s)://... : {text!r}')
    return camera_id, url


def parse_robot_camera(url):
    """--robot-camera: robot_view_relay 의 MJPEG 주소(http(s)). 고르기는 같은 host:port 의 /select 로 한다."""
    parts = urllib.parse.urlsplit(url)
    if parts.scheme not in ('http', 'https') or not parts.netloc:
        raise ConsoleError('INVALID_INPUT', f'robot camera must be http(s)://host:port/path : {url!r}')
    return url


def create_app(ledger_path, status_path=None, *, accounts_path, secret_key, site_state_path=None, cameras=(),
               map_path=None, live_map_path=None, live_map_stale_s=None, login_throttle=None, robot_names=None, support_dispatch=None, search_start=None, operations_url=None,
               command_transport=None, camera_stale_s=None, robot_watch_path=None, robot_watch_stale_s=None,
               robot_watch_map_robot=None, robot_camera=None):
    """설정 유효성을 사전에 검증한 후 로그인·관제·명령·계정 관리 라우트를 등록한 Flask 앱을 생성한다.

    모든 설정 오류는 이 단계에서 즉시 ConsoleError로 실패하여, 잘못된 옵션 지정 시 첫 요청이 아닌
    서버 기동 시점에 즉각 중단된다. 반면 장부, 현장 상태 DB, 픽스처 파일은 여기서 열지 않고
    매 폴링마다 새로 로드하며, 로드 실패 시 해당 요청에 한해 503 응답으로 처리된다.
    `operations_url`: operations HTTP API 베이스 URL. 미지정 시 운영자 명령 버튼이 비활성화된다
    (임의의 기본 URL 없음). 테스트 실행 시 `command_transport`로 모의 HTTP POST를, `login_throttle`로
    모의 시계가 주입된 LoginThrottle 인스턴스를 대체할 수 있다.
    """
    cameras = tuple(cameras)
    camera_stale_ms(cameras, camera_stale_s)   # 선택 의존성인 Flask를 import하기 전에 설정을 조기 검증한다
    try:
        from flask import Flask, Response, abort, flash, get_flashed_messages, jsonify, redirect, request, session
    except ImportError as exc:
        raise ConsoleError('DEPENDENCY_MISSING',
                           'Flask is required for the web console: python3 -m pip install flask') from exc
    if not isinstance(secret_key, str) or len(secret_key) < 16:
        raise ConsoleError('CONFIG_MISSING', 'secret_key (>= 16 chars) is required to sign login sessions')
    _check_sources(ledger_path, status_path, site_state_path)
    accounts = AccountStore.open(accounts_path)   # 첫 로그인 시점이 아닌 서버 기동 시점에 DB 유효성을 즉시 검증한다
    map_image = None if map_path is None else load_map(map_path)   # 지도 파일 오류 역시 기동 시점에 즉각 감지한다
    if map_path is not None and live_map_path is not None:
        raise ConsoleError('INVALID_INPUT', '--map and --live-map are exclusive: one map panel')
    # 실시간 지도는 서버 기동 시점에 아직 생성되지 않았을 수 있음(SLAM 수신 대기). 이 경우 화면 패널에 대기 상태가 안내된다.
    live_map = None if live_map_path is None else LiveMap(live_map_path, stale_s=live_map_stale_s)
    operations = None if operations_url is None else parse_operations_url(operations_url)
    transport = post_json if command_transport is None else command_transport
    robot_select = (None if robot_camera is None else
                    urllib.parse.urlsplit(parse_robot_camera(robot_camera))._replace(path='/select', query='').geturl())
    app = Flask(__name__)
    # Flask는 이 키로 쿠키를 암호화 서명하여 세션을 유지한다. 서버 측 별도 세션 저장소는 두지 않는다.
    app.secret_key = secret_key
    # HttpOnly: 클라이언트 스크립트의 세션 쿠키 접근 차단. SameSite=Lax: 타 사이트 출발 POST 요청에 쿠키 전송 차단
    # (추가로 아래 CSRF 토큰을 이중 검증함). Secure 플래그는 미적용: 관제 웹 콘솔은 평문 http 환경에서 운영된다.
    app.config.update(SESSION_COOKIE_HTTPONLY=True, SESSION_COOKIE_SAMESITE='Lax')
    throttle = LoginThrottle() if login_throttle is None else login_throttle

    @app.after_request
    def harden(response):
        """모든 응답에 SECURITY_HEADERS 및 `Cache-Control: no-store` 헤더를 주입한다.

        `setdefault`를 사용하므로 라우트 핸들러가 명시적으로 지정한 헤더(예: 지도 이미지 캐시 정책)는 유지된다.
        """
        for name, value in SECURITY_HEADERS.items():
            response.headers.setdefault(name, value)
        # 실시간 데이터 및 로그인 화면은 브라우저 캐시 저장을 금지한다. /map.png는 개별 캐시 헤더를 적용한다.
        response.headers.setdefault('Cache-Control', 'no-store')
        return response

    def view():
        """기동 시 구성된 소스 정보로 현재 요청에 대한 화면 모델을 생성한다(`current_view` 1회 호출)."""
        return current_view(ledger_path, status_path, site_state_path=site_state_path, map_image=map_image,
                            map_source=live_map,
                            robot_names=robot_names, support_dispatch=support_dispatch,
                            search_start=search_start, robot_watch_path=robot_watch_path,
                            robot_watch_stale_s=robot_watch_stale_s, robot_watch_map_robot=robot_watch_map_robot)

    def user():
        """현재 세션의 유효 계정 정보를 반환하며, 유효하지 않으면 None을 반환한다.

        세션 쿠키는 (account_id, auth_version)을 포함한다. 계정을 삭제 후 재생성하면 account_id가 변경되고,
        비밀번호·역할·상태를 수정하면 auth_version이 증가한다. 어느 경우든 이전에 발급된
        세션 쿠키는 클라이언트의 변조 여부와 무관하게 서버 검증에서 즉시 실효된다(AST-50-01).
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
        """세션에 바인딩된 CSRF 토큰을 반환한다. 최초 요청 시 32바이트 난수로 생성되며 세션 만료 시까지 유지된다."""
        if 'csrf' not in session:
            session['csrf'] = secrets.token_urlsafe(32)
        return session['csrf']

    def check_csrf():
        """POST 폼의 `csrf` 필드가 세션 토큰과 일치하지 않으면 400 Bad Request로 요청을 중단한다."""
        # 상수 시간 비교를 수행하여 타이밍 분석 공격을 방어한다. 세션 토큰이 존재하지 않는 경우
        # '-'와 비교하여 누락되거나 빈 문자열인 토큰은 절대 검증을 통과할 수 없다.
        if not secrets.compare_digest(request.form.get('csrf', ''), session.get('csrf', '-')):
            abort(400, 'csrf token mismatch')

    def login_required(handler):
        """라우트 데코레이터: 유효한 세션이 인증된 경우에만 `handler(account, ...)`를 실행한다(`user` 참조).

        인증 세션이 없으면 데이터 엔드포인트는 401 JSON으로 응답하고 일반 페이지 요청은 /login으로 리다이렉트한다.
        """
        @functools.wraps(handler)
        def wrapped(*args, **kwargs):
            """요청마다 세션 계정을 식별하여 라우트 핸들러의 첫 번째 인자로 주입한다."""
            account = user()
            if account is None:
                # 데이터 엔드포인트는 401 JSON으로 응답하여 클라이언트 폴링 로직이 로그인 HTML을 삼키지 않도록 방어한다(AST-50-02).
                # 브라우저 직접 페이지 접근인 경우에만 로그인 페이지로 리다이렉트한다.
                if request.path.startswith('/api/') or request.path in DATA_PATHS:
                    return jsonify({'error': 'login required'}), 401
                return redirect('/login')
            return handler(account, *args, **kwargs)
        return wrapped

    def admin_required(handler):
        """라우트 데코레이터: `login_required` 통과 후 계정 역할이 ADMIN이 아니면 403 Forbidden으로 차단한다.

        역할 정보는 쿠키가 아닌 현재 요청에서 계정 DB를 직접 조회한 결과(`user`)를 기준으로 판정한다.
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
        """요청 처리 중 발생한 ConsoleError를 클라이언트 요청 형식에 맞는 503 Service Unavailable로 변환한다.

        `/` 경로는 자동 새로고침 안내 HTML, `/api/...`는 {error, message, hint} JSON 구조,
        그 외 경로(폴링 HTML 조각, 지도 이미지, 계정 페이지 등)는 단일 행 텍스트 응답을 반환한다.
        """
        # 페이지에는 점검 항목을 기술한 문서를, 조각 폴링에는 1줄 텍스트를, API에는 오류 코드를 반환한다.
        # 모두 503 상태 코드를 부여하여 오류 상태가 빈 정상 응답으로 오인되지 않도록 차단한다.
        if request.path in ('/', '/replay'):
            return Response(render_failure(exc.detail_code, exc.detail), status=503, mimetype='text/html')
        if request.path.startswith('/api/'):
            return jsonify({'error': exc.detail_code, 'message': exc.detail,
                            'hint': FAILURE_HINTS.get(exc.detail_code, FAILURE_HINT_OTHER)}), 503
        return Response(failure_text(exc.detail_code, exc.detail), status=503, mimetype='text/plain')

    # ---- 로그인 ----------------------------------------------------------------------
    @app.get('/login')
    def login_form():
        """로그인 화면 렌더링(CSRF 토큰 및 flash 메시지 포함). 이미 로그인된 계정은 즉시 루트(/)로 리다이렉트한다."""
        if user() is not None:
            return redirect('/')
        return Response(render_login(csrf_token(), get_flashed_messages(with_categories=True)), mimetype='text/html')

    @app.post('/login')
    def login():
        """로그인 폼 인증을 수행하고 세션을 생성한다. CSRF 검증 통과 후에는 결과와 무관하게 항상 리다이렉트한다.

        처리 순서: CSRF 검증 -> (아이디, 클라이언트 IP) 락아웃 검사 -> 비밀번호 해시 대조. 미등록 계정,
        비밀번호 불일치, 비활성화 계정 모두 동일한 오류 메시지를 출력한다(AccountStore.authenticate가 None 반환).
        """
        check_csrf()               # 로그인 폼 자체도 세션 CSRF 토큰을 검증해야 한다(GET /login 호출 시 발급됨)
        username, client = request.form.get('username', ''), request.remote_addr
        wait = throttle.locked_for(username, client)
        if wait:
            # 비밀번호 해시 대조 전에 잠금을 검사하므로 계정 잠금 상태에서는 비밀번호 일치 여부를 유추할 수 없다.
            flash(f'로그인을 너무 많이 틀렸습니다. {wait}초 뒤에 다시 하세요.', 'danger')
            return redirect('/login')
        account = accounts.authenticate(username, request.form.get('password', ''))
        if account is None:
            throttle.failed(username, client)
            flash('아이디 또는 비밀번호가 틀립니다.', 'danger')
            return redirect('/login')
        throttle.succeeded(username, client)
        # 빈 세션으로 초기화: 로그인 전 잔류 데이터(이전 CSRF 토큰, flash 메시지 등)가 절대 이월되지
        # 않는다. 인증 성공 후 세션은 계정 ID, auth_version, 신규 CSRF 토큰만 보관하며, `user` 함수가
        # 매 요청마다 계정 DB의 최신 정보와 일치 여부를 엄격히 대조한다.
        session.clear()
        session['account_id'] = account['account_id']
        session['auth_version'] = account['auth_version']
        csrf_token()               # 신규 세션 전용 신규 CSRF 토큰 생성
        return redirect('/')

    @app.post('/logout')
    def logout():
        """세션을 파기하고(POST 전용) 로그인 페이지로 리다이렉트한다."""
        check_csrf()               # 링크나 img 태그를 이용한 운영자 강제 로그아웃 공격(CSRF)을 방어한다(AST-50-03)
        session.clear()
        return redirect('/login')

    # ---- 관제 화면 (로그인한 모든 역할) -----------------------------------------------
    @app.get('/')
    @login_required
    def page(account):
        """전체 관제 화면 렌더링. 명령 패널 활성화 여부는 계정 역할 및 --operations 지정 여부에 따라 결정된다."""
        picker = None
        if cameras and operations is not None and account['role'] == 'ADMIN':   # 카메라 고르기(OPS-183): 관리자만
            picker = dict(zip(('cameras', 'current', 'problem'), webcam_state(operations, transport=transport)),
                          notices=get_flashed_messages(category_filter=['webcam']))
        current_view_ = view()
        robot_cam = None
        if robot_camera is not None:   # 로봇 카메라 칸(OPS-191): 누구나 고름
            robots, current, problem = robot_camera_state(robot_select, transport=transport)
            names = {row['robot_id']: row['name'] for row in current_view_['robots']}
            robot_cam = {'url': robot_camera, 'robots': [(rid, names.get(rid, rid)) for rid in robots],
                         'current': current, 'problem': problem,
                         'notices': get_flashed_messages(category_filter=['robot_camera'])}
        return Response(render_page(current_view_, account, csrf=csrf_token(), cameras=cameras,
                                    camera_stale_s=camera_stale_s,
                                    command_access=command_access(account['role'], operations is not None),
                                    webcam=picker, robot_camera=robot_cam),
                        mimetype='text/html')

    @app.post('/robot-camera')
    @login_required
    def robot_camera_pick(account):
        """로봇 카메라 칸의 TB-6/TB-5 고르기(OPS-191). 운영 명령이 아니라 보기 설정이라 누구나, 확인 창 없음."""
        check_csrf()
        if robot_camera is None:
            abort(404)
        ok, message = select_robot_camera(robot_select, request.form.get('robot', ''), transport=transport)
        if not ok:
            app.logger.warning('robot camera pick by %s: %s', account['username'], message)
            flash(message, 'robot_camera')
        return redirect('/')

    @app.post('/webcam')
    @admin_required
    def webcam(account):
        """웹캠 칸의 카메라 고르기(OPS-183) -> operations 로 한 번 보내고 답을 다음 화면에 보인다."""
        check_csrf()
        if operations is None:
            abort(503, 'no --operations')
        ok, message = switch_webcam(operations, request.form.get('device', ''), operator_id=account['username'],
                                    transport=transport)
        (app.logger.info if ok else app.logger.warning)('webcam switch by %s: %s', account['username'], message)
        flash(message, 'webcam')
        return redirect('/')

    @app.get('/fragment')
    @login_required
    def fragment(account):
        """주기적으로 갱신되는 화면 영역을 HTML 조각으로 렌더링한다. 프론트엔드 폴링 스크립트가 교체 삽입한다."""
        return Response(render_fragment(view()), mimetype='text/html')

    # ---- 리플레이 (로그인한 모든 역할, 읽기만) -------------------------------------------
    closed_runs = ClosedRunCache(REPLAY_CACHE_RUNS)   # 닫힌 회차는 행이 늘지 않으므로 다시 읽지 않는다

    def replay_timeline(run_text):
        """(시간 막대, 회차 목록). ``run_text`` 가 비면 열린 회차, 없으면 마지막 회차. 회차가 없으면 시간 막대는 None.

        목록에 없는 회차는 INVALID_INPUT 이다. 열린 회차는 매 요청 새로 읽는다. 픽스처·장부만으로는 이력이 없어 거부한다.
        """
        if site_state_path is None:
            raise ConsoleError('CONFIG_MISSING', 'replay needs --site-state')
        runs, default = run_choices(load_runs(site_state_path))
        run_id = run_text or default
        chosen = [run for run in runs if run['run_id'] == run_id]
        if run_id is None:
            return None, runs
        if not chosen:
            raise ConsoleError('INVALID_INPUT', 'run is not in this site-state DB')
        run = chosen[0]
        cached = None if run['open'] else closed_runs.get(run['run_id'])
        if cached is not None:
            return cached, runs
        timeline = build_timeline(load_replay(site_state_path, run['run_id']), run=run, robot_names=robot_names)
        if not run['open']:
            closed_runs.put(run['run_id'], timeline)
        return timeline, runs

    @app.get('/replay')
    @login_required
    def replay(account):
        """지난 회차 다시 보기(?run=회차 ID). 처음에는 끝 시점을 그린다. 명령 버튼·로봇 경로가 없다."""
        timeline, runs = replay_timeline(request.args.get('run', ''))
        state = None if timeline is None else replay_at(timeline, timeline['end_ns'])
        return Response(render_replay_page(timeline, state, runs, account, csrf=csrf_token()), mimetype='text/html')

    @app.get('/replay/fragment')
    @login_required
    def replay_fragment(account):
        """회차 시작부터 ``offset_ns`` 뒤 시점의 조각. 숫자가 아니거나 범위 밖이면 INVALID_INPUT(503 한 줄)."""
        timeline, _runs = replay_timeline(request.args.get('run', ''))
        text = request.args.get('offset_ns', '')
        # ASCII 숫자만 받는다. str.isdigit() 는 '²'(int() 가 거부, 500)와 '١'(1 로 읽힘)도 통과시킨다(REV-06).
        if not (text.isascii() and text.isdigit()) or timeline is None:
            raise ConsoleError('INVALID_INPUT', 'offset_ns must be a non-negative integer within the run')
        return Response(render_replay_fragment(replay_at(timeline, timeline['start_ns'] + int(text)), timeline),
                        mimetype='text/html')

    @app.get('/api/view')
    @login_required
    def api_view(account):
        """화면 모델 원본 데이터를 JSON으로 반환한다(클라이언트 렌더링 및 상태 디버깅 기반)."""
        return jsonify(view())

    @app.get('/map.png')
    @login_required
    def map_png(account):
        """지도 이미지 바이너리를 PNG로 반환한다: 최신 수신된 실시간 SLAM 지도 또는 정적 --map 이미지.

        지도 옵션이 설정되지 않았거나 실시간 지도가 아직 수신되지 않은 경우 404를 반환한다.
        """
        image = live_map.current()[0] if live_map is not None else map_image
        if image is None:
            abort(404, 'no map: start with --map MAP_YAML or --live-map LIVE_YAML')
        # URL 쿼리스트링에 PGM SHA-256 해시(`?v=`)가 포함되므로 브라우저 캐시 사본이 다른 지도와 혼동되지 않는다.
        return Response(image.png, mimetype='image/png', headers={'Cache-Control': 'private, max-age=3600'})

    # ---- 운영자 명령 (ADMIN) -----------------------------------------------------
    @app.post('/commands')
    @login_required
    def command(account):
        """운영자 명령 버튼 1회 클릭 -> operations 서비스로 1회 HTTP POST 중계. 처리 결과 또는 실패를 JSON으로 반환한다.

        본 관제 웹에서는 자체 재시도·추측·직접 쓰기를 하지 않는다. 모든 명령은 operations가 기록하고(operator_commands)
        중앙 조정기가 상태 전이를 결정한다. 중계 전 유효성 검증 실패 시 sent=false 응답을 반환한다.
        """
        # admin_required 대신 login_required 적용: 프론트엔드 스크립트가 JSON 응답을 기대하므로 모든 인가 실패를
        # HTML 오류 페이지 대신 일관된 {sent: false, error, message} JSON 형식으로 반환하기 위함이다.
        def refuse(code, message, status):
            app.logger.warning('command refused before sending: %s %s (%s)', code, message, account['username'])
            return jsonify({'sent': False, 'error': code, 'message': message}), status
        # check_csrf와 동일한 검증 로직이며, 동일한 사유로 실패 시 JSON 응답을 반환한다.
        if not secrets.compare_digest(request.form.get('csrf', ''), session.get('csrf', '-')):
            return refuse('CSRF', 'csrf token mismatch — reload the page', 400)
        allowed, reason = command_access(account['role'], operations is not None)
        if not allowed:
            # CUSTOMER 역할은 권한 없음(403). --operations가 미설정된 ADMIN은 중계 대상 서버 없음(503).
            return refuse('NOT_AUTHORIZED' if account['role'] != 'ADMIN' else 'NOT_CONNECTED', reason,
                          403 if account['role'] != 'ADMIN' else 503)
        try:
            # 운영자 ID는 위조 가능한 폼 파라미터가 아닌 현재 로그인 세션 계정명이다. 클릭마다 신규 UUID command_id를 생성하여
            # 중복 클릭 시 별도 명령으로 추적하며, 운영자는 이 식별자로 operations 원장에서 처리 결과를 대조한다.
            payload = build_request(request.form, operator_id=account['username'], command_id=str(uuid.uuid4()))
        except ConsoleError as exc:
            return refuse(exc.detail_code, exc.detail, 400)
        result = send_command(operations, payload, transport=transport)
        log = app.logger.info if result['sent'] else app.logger.warning
        log('operator command %s %s by %s -> %s', payload['kind'], payload['command_id'], account['username'],
            {key: value for key, value in result.items() if key not in ('command_id', 'kind')})
        # operations 거절 응답이어도 HTTP 200 반환: operations 서비스와의 정상 통신 결과이기 때문이다. 응답 부재 시 502.
        return jsonify(result), 200 if result['sent'] else 502

    # ---- 계정 (ADMIN) ------------------------------------------------------------
    def accounts_page(account, status=200):
        """계정 관리 화면 렌더링. 직전 POST 작업에서 전달된 flash 알림 메시지를 함께 출력한다."""
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
        """관리자 전용 계정 등록을 처리한 후 계정 목록 화면으로 리다이렉트한다(303).

        저장소 계층에서 모든 입력 형식(사용자명 포맷, 비밀번호 최소 길이, 역할 유효성, 중복 여부)을 검증한다.
        검증 실패로 발생한 ConsoleError는 503 오류 페이지 대신 목록 화면 상단의 flash 안내 메시지로 표시된다.
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
        # 303 See Other: 브라우저가 GET 요청으로 리다이렉트하므로 목록 화면 새로고침 시 폼 재전송 경고를 방지한다.
        return redirect('/admin/accounts', code=303)

    @app.post('/admin/accounts/<username>')
    @admin_required
    def accounts_change(account, username):
        """지정된 계정을 삭제하거나 정보를 수정한 후(폼 필드 `_action`) 목록 화면으로 리다이렉트한다(303).

        저장소는 로그인 중인 본인 계정 삭제 및 마지막 활성 관리자 박탈 요청을 엄격히 거부한다. 비밀번호·
        역할·상태 변경 시 auth_version이 증가하여 해당 계정으로 기 발급된 세션이 즉시 무효화된다(`user`).
        저장소 유효성 오류는 flash 메시지로 전환되며, 정의되지 않은 `_action` 요청은 400 Bad Request로 처리된다.
        """
        check_csrf()
        form = request.form
        try:
            if form.get('_action') == 'delete':
                accounts.delete(username, by=account['username'])
                flash(f'계정 {username} 삭제', 'info')
            elif form.get('_action') == 'update':
                # 비밀번호 입력란이 비어 있으면 '빈 비밀번호로 변경'이 아니라 '기존 비밀번호 유지'(None)로 처리한다.
                accounts.update(username, display_name=form.get('display_name', ''), email=form.get('email', ''),
                                role=form.get('role', ''), status=form.get('status', ''),
                                password=form.get('password') or None)
                flash(f'계정 {username} 저장', 'success')
            else:
                abort(400, 'unknown _action')   # HTTPException은 하위 except 블록에서 가로채지 않고 Flask가 직접 처리한다
        except ConsoleError as exc:
            flash(f'변경 실패: {exc}', 'danger')
        return redirect('/admin/accounts', code=303)

    return app
