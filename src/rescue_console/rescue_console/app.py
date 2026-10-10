"""Flask boundary (optional dependency). GET only: the console sends nothing anywhere."""

import time

from .render import render_fragment, render_page
from .sources import load_ledger, load_status
from .view import ConsoleError, build_view


def current_view(ledger_path, status_path, clock=time.time_ns):
    return build_view(load_ledger(ledger_path), load_status(status_path), built_at_ns=clock())


def create_app(ledger_path, status_path=None):
    try:
        from flask import Flask, Response, jsonify
    except ImportError as exc:
        raise ConsoleError('DEPENDENCY_MISSING',
                           'Flask is required for the web console: python3 -m pip install flask') from exc
    app = Flask(__name__)

    def view():
        return current_view(ledger_path, status_path)

    @app.errorhandler(ConsoleError)
    def failed(exc):
        # Shown as a failure, never as an empty but healthy screen.
        return Response(f'console input failed: {exc}', status=503, mimetype='text/plain')

    @app.get('/')
    def page():
        return Response(render_page(view()), mimetype='text/html')

    @app.get('/fragment')
    def fragment():
        return Response(render_fragment(view()), mimetype='text/html')

    @app.get('/api/view')
    def api_view():
        return jsonify(view())

    return app
