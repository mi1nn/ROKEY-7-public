"""Read-only inputs: the operations ledger, the site-state DB or a status fixture.

Never writes or creates a file.
"""

import json
from pathlib import Path

from rescue_operations.ledger import OperationsError, read_reports
from rescue_operations.site_state import read_robot_status, read_site_events, read_target_states

from .view import ConsoleError

# read_reports and read_site_events cap one read at 1000 rows (rescue_operations.ledger._limit).
LEDGER_READ_LIMIT = 1000
# read_site_events returns the OLDEST rows first (ORDER BY observed_at_ns LIMIT n), so the console
# asks for the cap; past 1000 site events the newest are missing (README 「후속」).
SITE_EVENT_READ_LIMIT = 1000
STATUS_KEYS = frozenset({'robot_status', 'site_events', 'target_states'})


def load_ledger(db_path):
    """Snapshot through the operations read-only API (mode=ro, query_only)."""
    try:
        return read_reports(db_path, limit=LEDGER_READ_LIMIT)
    except OperationsError as exc:
        raise ConsoleError('STORAGE_ERROR', f'ledger read failed: {exc}') from exc


def load_status(path):
    """Fixture with the operations read API shapes until those functions exist.

    Keys: robot_status (read_robot_status), site_events (read_site_events),
    target_states (read_target_states). A missing key is shown as missing input.
    """
    if path is None:
        return {}
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise ConsoleError('INVALID_INPUT', f'status fixture unreadable: {exc}') from exc
    if not isinstance(data, dict) or not set(data) <= STATUS_KEYS:
        raise ConsoleError('INVALID_INPUT', f'status fixture keys must be a subset of {sorted(STATUS_KEYS)}')
    return data


def load_site_state(db_path):
    """The three site-state read API results (PR #34), opened read-only one after another.

    Same shapes as the status fixture, so `build_view` does not know which one it got.
    The three reads are separate opens, not one transaction: a row written in between can
    show up in one table and not yet in another for one poll.
    """
    try:
        return {
            'robot_status': read_robot_status(db_path),
            'site_events': read_site_events(db_path, limit=SITE_EVENT_READ_LIMIT),
            'target_states': read_target_states(db_path),
        }
    except OperationsError as exc:
        raise ConsoleError('STORAGE_ERROR', f'site-state read failed: {exc}') from exc
