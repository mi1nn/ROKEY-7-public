"""Run the console: python -m rescue_console LEDGER --accounts ACCOUNTS [--status FIXTURE] [--map MAP_YAML] [--port N].

The session secret comes from the environment (RESCUE_CONSOLE_SECRET by default); the
accounts file is created once with `python -m rescue_console.accounts init`.
"""

import argparse
import os
import sys

from .app import create_app
from .view import ConsoleError


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ledger', help='operations ledger SQLite file (opened read-only)')
    parser.add_argument('--accounts', required=True, help='console accounts SQLite file (rescue_console.accounts init)')
    parser.add_argument('--status', help='status fixture JSON in the operations read API shapes')
    parser.add_argument('--map', help='map_server map YAML (binary PGM) drawn under robot and target markers')
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
        app = create_app(args.ledger, args.status, accounts_path=args.accounts, secret_key=secret,
                         map_path=args.map)
    except ConsoleError as exc:
        print(f'CONSOLE FAILED: {exc}', file=sys.stderr)
        return 2
    app.run(host=args.host, port=args.port, debug=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
