"""Short chime when TB-5 has reached the person (방식 B full scenario, user work order 2026-10-15).

The event is the coordinator's own TargetSnapshot (``/rescue/targets``): a target whose ``approach_state`` becomes
``APPROACH_COMPLETE``, which the coordinator sets only when TB-5's APPROACH_TARGET ended COMPLETED with its stop
confirmed (rescue_coordination AUTO_APPROACH_STATES 'ARRIVED'). Receiving a goal or starting the approach
(ASSIGNED, IN_PROGRESS) never plays it. One chime per (mission trigger event, target): snapshots repeat (latched,
republished on every change) and play nothing again; the next mission (a new DISASTER trigger event) or another
target plays again. With ``journal_path`` the played keys survive a restart of this node within the run.

Playing never touches the control flow: a missing player or audio device is logged (``FOUND_ALERT_NOT_PLAYED``) and
the run goes on; ``enabled:=false`` logs the event only. The player is the first of paplay, pw-play, aplay found on
PATH (no library is assumed); the sound is a 0.6 s two-tone chime written once to a temporary WAV, at a quarter of
full scale.
"""
import json
import math
import struct
import wave

APPROACH_COMPLETE = 'APPROACH_COMPLETE'
PLAYERS = (('paplay',), ('pw-play',), ('aplay', '-q'))
TONES = ((880.0, 0.15), (0.0, 0.05), (1318.5, 0.15), (0.0, 0.05), (1318.5, 0.2))  # Hz (0 = rest), seconds
RATE, AMPLITUDE = 22050, 0.25


class FoundAlert:
    """Which (trigger_event_id, target_id) to chime for; no ROS, no audio."""

    def __init__(self, played=()):
        self.played = set(tuple(k) for k in played)
        self.trigger = None

    def on_trigger(self, trigger_event_id):
        self.trigger = trigger_event_id or None

    def on_targets(self, targets):
        """``targets``: iterable of (target_id, approach_state). New keys to chime for, in order (each once)."""
        if self.trigger is None:
            return []   # no mission yet: nothing to announce (and no key to remember it by)
        new = []
        for target_id, state in targets:
            key = (self.trigger, target_id)
            if state == APPROACH_COMPLETE and key not in self.played:
                self.played.add(key)
                new.append(key)
        return new


def chime_wav(path):
    """Write the chime to ``path`` (16-bit mono WAV)."""
    frames = bytearray()
    for freq, seconds in TONES:
        n = int(RATE * seconds)
        for i in range(n):
            fade = min(1.0, i / (0.01 * RATE), (n - i) / (0.01 * RATE))  # 10 ms ramps: no clicks
            value = AMPLITUDE * fade * math.sin(2 * math.pi * freq * i / RATE) if freq else 0.0
            frames += struct.pack('<h', int(value * 32767))
    with wave.open(str(path), 'wb') as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(RATE)
        out.writeframes(bytes(frames))
    return path


def play(path, *, which, run, timeout_s=5.0):
    """('PLAYED', player) or ('NOT_PLAYED', reason). ``which``/``run`` are shutil.which/subprocess.run (injected)."""
    tried = []
    for player in PLAYERS:
        if which(player[0]) is None:
            continue
        try:
            done = run([*player, str(path)], capture_output=True, text=True, timeout=timeout_s)
        except Exception as exc:   # a broken player is a reason, never an exception into the run
            tried.append(f'{player[0]}: {exc}')
            continue
        if done.returncode == 0:
            return 'PLAYED', player[0]
        tried.append(f'{player[0]}: exit {done.returncode} {done.stderr.strip()[:120]}')
    return 'NOT_PLAYED', '; '.join(tried) or 'no player (paplay, pw-play, aplay) on PATH'


def main(args=None):
    import shutil
    import subprocess
    import tempfile
    import threading
    import time
    from pathlib import Path

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import SystemState, TargetSnapshot

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_found_alert')
    log = node.get_logger()
    enabled = bool(node.declare_parameter('enabled', True).value)
    journal = node.declare_parameter('journal_path', '').value
    journal = Path(journal) if journal else None
    played = []
    if journal is not None and journal.exists():
        played = json.loads(journal.read_text()).get('played', [])
    core = FoundAlert(played)
    sound = chime_wav(Path(tempfile.mkdtemp(prefix='found_alert_')) / 'chime.wav')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def sound_off(key, at_ns):
        if not enabled:
            log.info(f'FOUND_ALERT_DISABLED trigger={key[0]} target={key[1]} at_ns={at_ns}')
            return
        result, detail = play(sound, which=shutil.which, run=subprocess.run)
        # rclpy refuses two severities on one call site ('Logger severity cannot be changed between calls')
        if result == 'PLAYED':
            log.info(
                f'FOUND_ALERT_{result} trigger={key[0]} target={key[1]} at_ns={at_ns} {detail}')
        else:
            log.warning(
                f'FOUND_ALERT_{result} trigger={key[0]} target={key[1]} at_ns={at_ns} {detail}')

    def on_state(msg):
        core.on_trigger(msg.trigger_event_id if msg.has_trigger else '')

    def on_targets(msg):
        for key in core.on_targets((t.target_id, t.approach_state) for t in msg.targets):
            at = time.time_ns()
            log.info(f'FOUND_EVENT TB-5 reached target={key[1]} (approach APPROACH_COMPLETE) trigger={key[0]} at_ns={at}')
            if journal is not None:
                journal.parent.mkdir(parents=True, exist_ok=True)
                journal.write_text(json.dumps(dict(played=sorted(core.played))) + '\n')
            threading.Thread(target=sound_off, args=(key, at), daemon=True).start()

    node.create_subscription(SystemState, '/rescue/system_state', on_state, latched)
    node.create_subscription(TargetSnapshot, '/rescue/targets', on_targets, latched)
    log.info(f'found alert enabled={enabled} journal={journal} sound={sound} '
             f'player={next((p[0] for p in PLAYERS if shutil.which(p[0])), "none")}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
