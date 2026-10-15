"""Map archive for replay: the run's SLAM and frozen maps under ``<map_dir>/<run_id>/`` (no ROS import).

db_writer keeps the maps the replay page draws (작전·웹개발 2026-10-15). It runs inside db_writer because
``map_snapshots`` rows can only be written by the file's one writer; the map_snapshot node keeps writing
live_map.yaml for the console's live map and does not change.

- Live SLAM map (the SCOUT robot's, TB-6): at most one every ARCHIVE_PERIOD_NS, and an image already kept
  in the run is not kept again. Its revision is the SLAM robot's current MapState revision; before one is
  heard nothing is kept (never a guessed revision).
- Frozen maps (/rescue/map/frozen + /rescue/map/frozen_revision, R0+1 and R0+2): always kept, each once the
  revision text names its occupancy hash (rescue_contracts.occupancy, the hash the robot and the
  coordinator use). A grid without its text, or a text for another grid, waits; nothing is paired by guess.
- Between runs (after a reset closed one and before the next opened) nothing is kept.

Each kept map is a trinary PGM (map_snapshot.grid_to_pgm) and a map_server YAML next to it; the row's
sha256 is the PGM's. Failures go to error_log with the detail starting MAP_ARCHIVE_SKIPPED (error_log takes
C-09 codes only). A failing DB raises OperationsError to the caller (db_writer logs it).
"""
import hashlib
import os
from pathlib import Path
import uuid

from rescue_contracts.occupancy import occupancy_sha256, parse_frozen_revision, planar_origin

from .map_snapshot import FREE_PERCENT, OCCUPIED_PERCENT, grid_to_pgm
from .site_state import read_overhead_passages

# One live map per period per run [제안, 작전·웹개발 2026-10-15]. About 160 KB a PGM for a 20 m x 20 m map at
# 0.05 m, so a 15-minute run keeps about 29 MB [계산, 설계 메모 2026-10-15].
ARCHIVE_PERIOD_NS = 5_000_000_000
SNAPSHOT_NAMESPACE = uuid.uuid5(uuid.NAMESPACE_URL, 'rescue_operations/map_archive')  # same run + image -> same id


def grid_fields(msg):
    """nav_msgs/OccupancyGrid -> the rescue_contracts.occupancy fields (as the coordinator's FrozenGrid reads it).
    ValueError for a non-planar origin."""
    info = msg.info
    p, q = info.origin.position, info.origin.orientation
    return dict(frame=msg.header.frame_id, width=int(info.width), height=int(info.height),
                resolution=float(info.resolution), origin=planar_origin(p.x, p.y, p.z, q.x, q.y, q.z, q.w),
                data=[int(v) for v in msg.data])


def _replace(path, data):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes(data)
    os.replace(tmp, path)


class MapArchive:
    def __init__(self, ledger, *, out_dir, robot_id):
        self.ledger, self.out_dir, self.robot_id = ledger, Path(out_dir), robot_id
        self._live = {}            # run_id -> (last kept at_ns, kept PGM hashes)
        self._frozen_grids = {}    # occupancy hash -> fields, waiting for or matched with a revision text
        self._frozen_info = None   # latest parsed revision text
        self._frozen_kept = set()  # (run_id, revision, occupancy hash)
        self._skipped = set()      # reasons already written to error_log

    def on_live_map(self, fields, *, received_at_ns):
        """True when the map was kept."""
        run = self.ledger.run_id
        if run is None:
            return False
        last, kept = self._live.setdefault(run, (None, set()))
        if last is not None and received_at_ns - last < ARCHIVE_PERIOD_NS:
            return False
        current = read_overhead_passages(self.ledger.path)['current_map']
        if current is None:
            self._skip(received_at_ns, 'no MapState of the SLAM robot yet: a live map has no revision to keep it under')
            return False
        try:
            pgm = grid_to_pgm(fields['width'], fields['height'], fields['data'])
        except ValueError as exc:
            self._skip(received_at_ns, f'live map not drawable: {exc}')
            return False
        sha = hashlib.sha256(pgm).hexdigest()
        if sha in kept:
            return False
        self._store(run, fields, pgm, sha, kind='live', revision=current['map_revision'], at_ns=received_at_ns)
        kept.add(sha)
        self._live[run] = (received_at_ns, kept)
        return True

    def on_frozen_grid(self, fields, *, received_at_ns):
        """True when this grid completed a pair and was kept."""
        try:
            self._frozen_grids[occupancy_sha256(**fields)] = fields
        except ValueError as exc:
            self._skip(received_at_ns, f'frozen map not valid: {exc}')
            return False
        return self._pair(received_at_ns)

    def on_frozen_revision(self, text, *, received_at_ns):
        """True when this text completed a pair and the map was kept."""
        try:
            self._frozen_info = parse_frozen_revision(text)
        except ValueError as exc:
            self._skip(received_at_ns, f'frozen revision not valid: {exc}')
            return False
        return self._pair(received_at_ns)

    def _pair(self, received_at_ns):
        info, run = self._frozen_info, self.ledger.run_id
        if info is None or run is None:
            return False
        fields = self._frozen_grids.get(info['occupancy_sha256'])
        if fields is None:
            return False
        key = (run, info['map_revision'], info['occupancy_sha256'])
        if key in self._frozen_kept:
            return False
        try:
            pgm = grid_to_pgm(fields['width'], fields['height'], fields['data'])
        except ValueError as exc:
            self._skip(received_at_ns, f'frozen map not drawable: {exc}')
            return False
        self._store(run, fields, pgm, hashlib.sha256(pgm).hexdigest(), kind='frozen', revision=info['map_revision'],
                    at_ns=received_at_ns)
        self._frozen_kept.add(key)
        return True

    def _store(self, run, fields, pgm, sha, *, kind, revision, at_ns):
        folder = self.out_dir / run
        name = f'{kind}-r{revision}-{sha[:12]}'
        try:
            folder.mkdir(parents=True, exist_ok=True)
            _replace(folder / f'{name}.pgm', pgm)
            x, y, yaw = fields['origin']
            _replace(folder / f'{name}.yaml', (
                f'image: {name}.pgm\nmode: trinary\nresolution: {float(fields["resolution"])!r}\n'
                f'origin: [{float(x)!r}, {float(y)!r}, {float(yaw)!r}]\nnegate: 0\n'
                f'occupied_thresh: {OCCUPIED_PERCENT / 100}\nfree_thresh: {FREE_PERCENT / 100}\n').encode())
        except OSError as exc:
            self._skip(at_ns, f'{name} not written: {exc}')
            return
        snapshot = dict(snapshot_id=str(uuid.uuid5(SNAPSHOT_NAMESPACE, f'{run}:{kind}:{sha}')), robot_id=self.robot_id,
                        revision=revision, path=str(folder / f'{name}.pgm'), sha256=sha, at_ns=at_ns)
        self.ledger.record_map_snapshot(snapshot)

    def _skip(self, at_ns, why):
        """One error_log row per distinct reason; a map stream would otherwise fill the log."""
        if why in self._skipped:
            return
        self._skipped.add(why)
        self.ledger.record_error(dict(at_ns=at_ns, robot_id=None, task_id=None, code='INVALID_INPUT',
                                      detail=f'MAP_ARCHIVE_SKIPPED {why}'[:1024]))
