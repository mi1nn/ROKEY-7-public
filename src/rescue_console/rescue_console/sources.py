"""Read-only inputs: the operations ledger and a status fixture. Never writes or creates a file."""

import json
from pathlib import Path

from rescue_operations.ledger import OperationsError, read_reports

from .view import ConsoleError

# read_reports caps one read at 1000 rows (rescue_operations.ledger._limit).
LEDGER_READ_LIMIT = 1000
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
