"""관제 웹을 띄운다: python -m rescue_console [LEDGER] --accounts ACCOUNTS
[--site-state DB | --status FIXTURE] [--camera ID=URL ... --camera-stale-s S] [--map MAP_YAML | --live-map LIVE_YAML --live-map-stale-s S] [--operations URL] [--port N].

LEDGER(옛 ReportLedger)는 --site-state가 있으면 선택 사항이다. 미지정 시 페이지에 보고 내역 부재를
표시한다. 세션 비밀키는 환경 변수(기본 RESCUE_CONSOLE_SECRET)를 통해 주입받는다. 계정 파일은
`python -m rescue_console.accounts init` 서브커맨드로 사전에 생성한다.
"""

import argparse
import os
import sys

from .app import create_app, parse_camera, parse_robot_name, robot_name_map
from .view import ConsoleError


def main(argv=None):
    """커맨드라인 인자를 파싱하고 `create_app`으로 앱을 생성한 뒤 Flask 개발 서버를 실행한다.

    모든 유효성 검증은 서버 기동 전에 수행된다: ConsoleError 발생 시 stderr에 `CONSOLE FAILED: ...`를
    출력하고 서비스를 개시하지 않은 채 종료 코드 2를 반환한다. 서버 정상 종료 시에는 0을 반환한다.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ledger', nargs='?', default=None,
                        help='operations ReportLedger SQLite file (opened read-only); optional with --site-state')
    parser.add_argument('--accounts', required=True, help='console accounts SQLite file (rescue_console.accounts init)')
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--site-state', help='operations site-state SQLite file (opened read-only)')
    source.add_argument('--status', help='status fixture JSON in the operations read API shapes (E2 without operations)')
    parser.add_argument('--camera', action='append', default=[], metavar='ID=URL',
                        help='camera MJPEG stream shown on the page, e.g. cam_a=http://<camera host>:<port>/stream')
    parser.add_argument('--robot-name', action='append', default=[], metavar='ID=NAME',
                        help='name shown for a robot, e.g. robot_a=TB-6 (repeat per robot). No default: '
                             'without it the robot ID is shown')
    parser.add_argument('--support-dispatch', choices=('AUTO', 'OPERATOR'),
                        help="the coordinator's support_dispatch (field value). AUTO: no dispatch button, the "
                             'automatic dispatch state instead. Unset: the button as before')
    parser.add_argument('--search-start', choices=('AUTO', 'OPERATOR'),
                        help="the coordinator's search_start (field value). AUTO: no START_SEARCH button, the "
                             'automatic start state instead. Unset: the button as before')
    parser.add_argument('--camera-stale-s', type=float, metavar='S',
                        help='with --camera: seconds without a new frame before the stream shows 영상 끊김 and '
                             'is reconnected. No default')
    parser.add_argument('--map', help='map_server map YAML (binary PGM) drawn under robot and target markers')
    parser.add_argument('--live-map', help='SLAM map YAML written by the operations launch (map_topic/map_dir), '
                                           're-read when it changes; may not exist yet at startup')
    parser.add_argument('--live-map-stale-s', type=float,
                        help='with --live-map: no new map for this long shows the map as stale. No default')
    parser.add_argument('--robot-camera', metavar='URL',
                        help='robot camera panel: robot_view_relay MJPEG URL from the operations launch (robot_views), '
                             'e.g. http://<operations host>:<video_port+1>/robot_view.mjpg; needs --camera-stale-s')
    parser.add_argument('--robot-watch', help='robot watch file written by the operations launch '
                                              '(robot_namespaces/robot_watch_path): robot base topics seen directly')
    parser.add_argument('--robot-watch-stale-s', type=float,
                        help='with --robot-watch: nothing from a robot for this long shows it as stale. No default')
    parser.add_argument('--robot-watch-map-robot', metavar='ROBOT_ID',
                        help='with --robot-watch: the robot whose own map is the map shown (live SLAM map owner). '
                             'Other robots\' directly watched poses are in their own map frame and are not drawn')
    parser.add_argument('--operations', metavar='URL',
                        help='operations HTTP API base, e.g. http://<operations host>:<port>; '
                             'operator commands go to URL/api/operator-commands. No default: without it the '
                             'command buttons stay off')
    parser.add_argument('--secret-env', default='RESCUE_CONSOLE_SECRET',
                        help='environment variable holding the session secret (>= 16 chars)')
    parser.add_argument('--host', default='127.0.0.1',
                        help='bind address; the deployment profile decides any non-local value')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args(argv)
    try:
        # 세션 비밀키는 인자가 아닌 환경 변수로 전달받아 프로세스 목록 등에 노출되지 않는다.
        # 키 최소 길이는 create_app에서 검증한다.
        secret = os.environ.get(args.secret_env)
        if not secret:
            raise ConsoleError('CONFIG_MISSING', f'set {args.secret_env} to a random string of 16+ characters')
        # create_app에서도 동일한 규칙을 검증하지만(app._check_sources), 여기서는 커맨드라인 인자명을 명시한다.
        if args.ledger is None and args.site_state is None:
            raise ConsoleError('INVALID_INPUT', 'LEDGER is required unless --site-state is given')
        # 둘 중 하나만 전달되면 거부한다: 만료(stale) 기준 시간은 기본값이 없다.
        if len({args.robot_watch is None, args.robot_watch_stale_s is None, args.robot_watch_map_robot is None}) > 1:
            raise ConsoleError('INVALID_INPUT', '--robot-watch, --robot-watch-stale-s and --robot-watch-map-robot go together')
        if (args.live_map is None) != (args.live_map_stale_s is None):
            raise ConsoleError('INVALID_INPUT', '--live-map and --live-map-stale-s go together')
        cameras = [parse_camera(text) for text in args.camera]
        robot_names = robot_name_map(parse_robot_name(text) for text in args.robot_name)
        app = create_app(args.ledger, args.status, accounts_path=args.accounts, secret_key=secret,
                         site_state_path=args.site_state, cameras=cameras, map_path=args.map,
                         live_map_path=args.live_map, live_map_stale_s=args.live_map_stale_s,
                         robot_watch_path=args.robot_watch, robot_watch_stale_s=args.robot_watch_stale_s,
                         robot_watch_map_robot=args.robot_watch_map_robot,
                         robot_camera=args.robot_camera,
                         operations_url=args.operations, robot_names=robot_names,
                         support_dispatch=args.support_dispatch,
                         search_start=args.search_start, camera_stale_s=args.camera_stale_s)
    except ConsoleError as exc:
        print(f'CONSOLE FAILED: {exc}', file=sys.stderr)
        return 2
    if args.operations is None:
        # 정상 동작: 조회 전용 실행(E2 픽스처, 오프라인 시연)을 지원하며, 안내 로그로 버튼 비활성화 이유를 출력한다.
        print('operations 주소 없음: 명령 비활성 (--operations URL 로 켬)', file=sys.stderr)
    # Werkzeug 개발 서버 실행. 보안을 위해 debug=False로 대화형 디버거를 비활성화한다.
    app.run(host=args.host, port=args.port, debug=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
