"""Run the read-only console: python -m rescue_console LEDGER [--status FIXTURE] [--port N]."""

import argparse
import sys

from .app import create_app
from .view import ConsoleError


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ledger', help='operations ledger SQLite file (opened read-only)')
    parser.add_argument('--status', help='status fixture JSON in the operations read API shapes')
    parser.add_argument('--host', default='127.0.0.1',
                        help='bind address; the deployment profile decides any non-local value')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args(argv)
    try:
        app = create_app(args.ledger, args.status)
    except ConsoleError as exc:
        print(f'CONSOLE FAILED: {exc}', file=sys.stderr)
        return 2
    app.run(host=args.host, port=args.port, debug=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
