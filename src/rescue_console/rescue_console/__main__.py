"""Run the console: python -m rescue_console [LEDGER] --accounts ACCOUNTS
[--site-state DB | --status FIXTURE] [--camera ID=URL ... --camera-stale-s S] [--map MAP_YAML] [--operations URL] [--port N].

LEDGER (the old ReportLedger) is optional with --site-state; without it the page says report rows
are missing. The session secret comes from the environment (RESCUE_CONSOLE_SECRET by default); the
accounts file is created once with `python -m rescue_console.accounts init`.
"""

import argparse
import os
import sys

from .app import create_app, parse_camera, parse_robot_name, robot_name_map
from .view import ConsoleError


def main(argv=None):
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
    parser.add_argument('--camera-stale-s', type=float, metavar='S',
                        help='with --camera: seconds without a new frame before the stream shows 영상 끊김 and '
                             'is reconnected. No default')
    parser.add_argument('--map', help='map_server map YAML (binary PGM) drawn under robot and target markers')
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
        secret = os.environ.get(args.secret_env)
        if not secret:
            raise ConsoleError('CONFIG_MISSING', f'set {args.secret_env} to a random string of 16+ characters')
        if args.ledger is None and args.site_state is None:
            raise ConsoleError('INVALID_INPUT', 'LEDGER is required unless --site-state is given')
        cameras = [parse_camera(text) for text in args.camera]
        robot_names = robot_name_map(parse_robot_name(text) for text in args.robot_name)
        app = create_app(args.ledger, args.status, accounts_path=args.accounts, secret_key=secret,
                         site_state_path=args.site_state, cameras=cameras, map_path=args.map,
                         operations_url=args.operations, robot_names=robot_names, camera_stale_s=args.camera_stale_s)
    except ConsoleError as exc:
        print(f'CONSOLE FAILED: {exc}', file=sys.stderr)
        return 2
    if args.operations is None:
        print('operations 주소 없음: 명령 비활성 (--operations URL 로 켬)', file=sys.stderr)
    app.run(host=args.host, port=args.port, debug=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
