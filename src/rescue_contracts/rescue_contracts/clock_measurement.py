"""Read-only OS clock boundary, shared by the two PC processes.

No ROS, files written, clock adjustment or synchronization configuration.
Backend and freshness/drift budgets are explicit deployment settings.
systemd's NTPMessage layout is from systemd v255 timesyncd-bus.c; timestamps
are microseconds. Bounds assume the declared upstream UTC reference is valid.
"""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from decimal import Decimal, ROUND_CEILING
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time


def _ns(seconds):
    value = Decimal(str(seconds)) * 10**9
    if not value.is_finite() or value < 0:
        raise ValueError('nonnegative finite clock error required')
    return int(value.to_integral_value(rounding=ROUND_CEILING))


class ClockUnavailable(ValueError):
    """No usable measurement; the message starts with its reason code (C13, 2026-10-16)."""


def parse_timesync(message, *, synchronized, now_ns, max_age_ns, drift_ppm):
    """Use the original NTP reply time, never the time it was read again.

    A reply timesyncd flagged as a spike is used too (C13 A): it is a valid NTP measurement that timesyncd only did
    not correct the clock with, and the bound below holds the whole offset and half the round trip, which covers
    that uncorrected clock (``ntp_spike`` in the record). A reply not used says why: NO_NTP_REPLY or
    NOT_SYNCHRONIZED."""
    if (not isinstance(message, list) or len(message) != 15
            or any(type(message[i]) is not int for i in (0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 13, 14))
            or type(message[12]) is not bool
            or message[0] != 0 or message[1] not in (3, 4) or message[2] != 4
            or not 0 < message[3] < 16 or message[13] <= 0):
        raise ClockUnavailable('NO_NTP_REPLY: no valid NTP reply from timesyncd')
    leap, version, mode, stratum, precision, root_delay, dispersion, reference, t1, t2, t3, t4, spike, count, jitter = message
    if not synchronized:
        raise ClockUnavailable('NOT_SYNCHRONIZED: timesyncd reports NTPSynchronized=no')
    if min(root_delay, dispersion, t1, t2, t3, t4, jitter) < 0 or not all((t1, t2, t3, t4)):
        raise ValueError('invalid NTP clock fields')
    delay_us = (t4 - t1) - (t3 - t2)
    if delay_us < 0 or t3 < t2 or t4 < t1:
        raise ValueError('invalid NTP round trip')
    observed = t4 * 1000
    if not 0 <= now_ns - observed < max_age_ns:
        raise ValueError('NTP reply is future or expired')
    offset_twice_us = abs((t2 - t1) + (t3 - t4))
    # Include upstream distance, peer delay, offset, jitter, precision and the
    # explicitly agreed oscillator drift since the original reply.
    precision_ns = _ns(Decimal(2) ** precision)
    error = ((root_delay + delay_us + offset_twice_us + 1) // 2 + dispersion + jitter) * 1000
    error += precision_ns + _ns(Decimal(now_ns - observed) / 10**9 * Decimal(str(drift_ppm)) / 10**6)
    return dict(state='SYNCED', measured_at_ns=observed, sampled_at_ns=now_ns,
                valid_until_ns=observed + max_age_ns, error_bound_ns=error, ntp_spike=spike)


def parse_chrony(raw, *, now_ns, max_age_ns, drift_ppm):
    fields = dict(line.split(':', 1) for line in raw.splitlines() if ':' in line)
    fields = {key.strip(): value.strip() for key, value in fields.items()}
    ref = fields['Reference ID'].split()[0]
    if fields['Leap status'] != 'Normal' or ref in ('00000000', '7F7F0101') or not 0 < int(fields['Stratum']) < 16:
        raise ValueError('chrony not synchronized')
    stamp = datetime.strptime(fields['Ref time (UTC)'], '%a %b %d %H:%M:%S %Y').replace(tzinfo=timezone.utc)
    observed = int(stamp.timestamp()) * 10**9
    if not 0 <= now_ns - observed < max_age_ns:
        raise ValueError('chrony reference observation expired')
    correction = abs(Decimal(fields['System time'].split()[0]))
    delay = abs(Decimal(fields['Root delay'].split()[0]))
    dispersion = Decimal(fields['Root dispersion'].split()[0])
    if not dispersion.is_finite() or dispersion < 0:
        raise ValueError('invalid chrony dispersion')
    drift = Decimal(now_ns - observed) / 10**9 * Decimal(str(drift_ppm)) / 10**6
    return dict(state='SYNCED', measured_at_ns=observed, sampled_at_ns=now_ns,
                valid_until_ns=observed + max_age_ns,
                error_bound_ns=_ns(correction + delay / 2 + dispersion + drift))


def collect(backend, *, max_age_ns, drift_ppm, run=subprocess.run, now_ns=time.time_ns):
    if backend not in ('chrony', 'timesyncd') or type(max_age_ns) is not int or max_age_ns <= 0:
        raise ValueError('explicit clock backend and age policy required')
    if type(drift_ppm) not in (int, float) or not math.isfinite(drift_ppm) or drift_ppm <= 0:
        raise ValueError('explicit positive drift bound required')
    def read(args):
        result = run(args, capture_output=True, text=True, timeout=5, check=False,
                     env={'PATH': '/usr/sbin:/usr/bin:/sbin:/bin', 'LC_ALL': 'C', 'SYSTEMD_PAGER': 'cat'})
        if result.returncode:
            raise ValueError('clock read failed: ' + args[0] + ': ' + result.stderr.strip())
        return result.stdout.strip()
    service = 'chrony.service' if backend == 'chrony' else 'systemd-timesyncd.service'
    invocation = read(['systemctl', 'show', service, '--property=InvocationID', '--value'])
    if not invocation or read(['systemctl', 'is-active', service]) != 'active':
        raise ValueError('clock daemon unavailable')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    machine = Path('/etc/machine-id').read_text().strip()
    if not machine or not boot:
        raise ValueError('host identity unavailable')
    if backend == 'chrony':
        record = parse_chrony(read(['chronyc', '-n', 'tracking']), now_ns=now_ns(),
                              max_age_ns=max_age_ns, drift_ppm=drift_ppm)
    else:
        synchronized = read(['timedatectl', 'show', '--property=NTPSynchronized', '--value']) == 'yes'
        raw = json.loads(read(['busctl', '--json=short', 'get-property', 'org.freedesktop.timesync1',
            '/org/freedesktop/timesync1', 'org.freedesktop.timesync1.Manager', 'NTPMessage']))
        if raw.get('type') != '(uuuuittayttttbtt)':
            raise ValueError('unexpected systemd NTPMessage type')
        data = raw['data']
        if len(data) == 1 and isinstance(data[0], list):
            data = data[0]
        record = parse_timesync(data, synchronized=synchronized, now_ns=now_ns(),
                               max_age_ns=max_age_ns, drift_ppm=drift_ppm)
    if read(['systemctl', 'show', service, '--property=InvocationID', '--value']) != invocation:
        raise ValueError('clock daemon restarted during measurement')
    record.update(host_id=machine, host_boot_id=boot, backend=backend,
                  epoch_id=hashlib.sha256((boot + ':' + invocation).encode()).hexdigest(),
                  drift_ppm=float(drift_ppm))
    return record


class ClockSampler:
    """One background measurement; ROS callbacks never block on OS commands."""
    def __init__(self, backend, max_age_ns, drift_ppm, *, collector=collect):
        if backend not in ('chrony','timesyncd') or type(max_age_ns) is not int or max_age_ns <= 0:
            raise ValueError('explicit clock backend and positive age policy required')
        if type(drift_ppm) not in (int,float) or not math.isfinite(drift_ppm) or drift_ppm <= 0:
            raise ValueError('explicit positive clock drift budget required')
        self.settings = dict(max_age_ns=max_age_ns, drift_ppm=drift_ppm)
        self.backend, self.collector = backend, collector
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ntp-read')
        self.pending = None
        self.record = dict(state='UNKNOWN', reason='waiting for clock measurement')
        self.last_pair = None
        self.step_sequence = 0

    def poll(self):
        if self.pending is not None and self.pending.done():
            try:
                record = self.pending.result()
                pair = (time.time_ns(), time.monotonic_ns())
                if self.last_pair is not None:
                    previous, old = self.last_pair
                    change = abs((pair[0] - pair[1]) - (old[0] - old[1]))
                    if change > record['error_bound_ns'] + previous['error_bound_ns']:
                        self.step_sequence += 1
                self.last_pair = (record, pair)
                record['epoch_id'] += ':' + str(self.step_sequence)
                self.record = record
            except Exception as exc:
                self.record = dict(state='UNKNOWN', reason=str(exc))
            self.pending = None
        if self.pending is None:
            self.pending = self.pool.submit(self.collector, self.backend, **self.settings)
        return dict(self.record)

    def close(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
