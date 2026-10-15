"""Rehearsal only: give one process its own host id in clock measurements (one-PC wiring rehearsal).

The executor refuses a coordinator on the same host (live_clock: 'LIVE_ROS expects two distinct PCs',
both clocks read /etc/machine-id). rehearsal.launch.py puts this folder first on PYTHONPATH of the two
task_input processes only, with REHEARSAL_HOST_ID=rehearsal-<robot>. Nothing else in the measurement
changes (same chrony reading, error bound and boot). Never used on a field PC.
"""
import os

_HOST = os.environ.get('REHEARSAL_HOST_ID')
if _HOST:
    import rescue_contracts.clock_measurement as _cm

    _collect = _cm.collect

    def _rehearsal_collect(*args, **kwargs):
        record = _collect(*args, **kwargs)
        record['host_id'] = _HOST
        return record

    _cm.collect = _rehearsal_collect
    _cm.ClockSampler.__init__.__kwdefaults__['collector'] = _rehearsal_collect
