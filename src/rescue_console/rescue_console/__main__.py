"""관제 웹을 띄운다: python -m rescue_console [LEDGER] --accounts ACCOUNTS
[--site-state DB | --status FIXTURE] [--camera ID=URL ... --camera-stale-s S] [--map MAP_YAML | --live-map LIVE_YAML --live-map-stale-s S] [--operations URL] [--port N].

LEDGER(옛 ReportLedger)는 --site-state 가 있으면 선택이다. 없으면 페이지가 보고 행이 없다고
알린다. 세션 비밀값은 환경 변수(기본 RESCUE_CONSOLE_SECRET)에서 온다. 계정 파일은
`python -m rescue_console.accounts init` 으로 한 번 만든다.
"""

import argparse
import os
import sys

from .app import create_app, parse_camera, parse_robot_name, robot_name_map
from .view import ConsoleError


def main(argv=None):
    """명령줄을 해석하고 `create_app` 으로 앱을 만든 뒤 Flask 개발 서버를 띄운다.

    모든 검사는 서버가 뜨기 전에 돈다: ConsoleError 가 나면 `CONSOLE FAILED: ...` 를 stderr 에
    찍고 서비스하지 않은 채 2 를 돌려준다. 서버가 멈추면 0 을 돌려준다.
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
        # 세션 비밀값은 인자가 아니라 환경 변수에서 오므로 명령줄에 드러나지 않는다.
        # 길이는 create_app 이 검사한다.
        secret = os.environ.get(args.secret_env)
        if not secret:
            raise ConsoleError('CONFIG_MISSING', f'set {args.secret_env} to a random string of 16+ characters')
        # create_app 도 같은 규칙을 검사한다(app._check_sources). 이 메시지는 명령줄 인자 이름을 댄다.
        if args.ledger is None and args.site_state is None:
            raise ConsoleError('INVALID_INPUT', 'LEDGER is required unless --site-state is given')
        # 둘 중 하나만 주면 거부한다: 낡음 기준에는 기본값이 없다.
        if (args.live_map is None) != (args.live_map_stale_s is None):
            raise ConsoleError('INVALID_INPUT', '--live-map and --live-map-stale-s go together')
        cameras = [parse_camera(text) for text in args.camera]
        robot_names = robot_name_map(parse_robot_name(text) for text in args.robot_name)
        app = create_app(args.ledger, args.status, accounts_path=args.accounts, secret_key=secret,
                         site_state_path=args.site_state, cameras=cameras, map_path=args.map,
                         live_map_path=args.live_map, live_map_stale_s=args.live_map_stale_s,
                         operations_url=args.operations, robot_names=robot_names,
                         support_dispatch=args.support_dispatch,
                         search_start=args.search_start, camera_stale_s=args.camera_stale_s)
    except ConsoleError as exc:
        print(f'CONSOLE FAILED: {exc}', file=sys.stderr)
        return 2
    if args.operations is None:
        # 오류가 아니다: 보기 전용 실행(E2 픽스처, 오프라인 시연)도 정상이다. 이 줄은 버튼이 꺼진 이유를 알린다.
        print('operations 주소 없음: 명령 비활성 (--operations URL 로 켬)', file=sys.stderr)
    # Werkzeug 개발 서버. debug=False 로 대화형 디버거를 끈 채 둔다.
    app.run(host=args.host, port=args.port, debug=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
