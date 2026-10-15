"""Join measured local and configured coordinator clocks in memory.

No observed timestamps/expiry are changed by receiving or polling a record.
The designated service is a deployment trust boundary, like the existing
configured SystemState publisher; DDS cryptographic authentication is not added.
"""
from copy import deepcopy
import hashlib
import json
import math
import time

from rescue_contracts.clock_measurement import ClockSampler
from rescue_robot.executor import ContractError, identifier
from rescue_robot.task_policy import ClockBasis


class LiveClock:
    def __init__(self, runtime, settings):
        self.runtime, self.settings = runtime, settings
        self.peer = None
        self.pending = None
        self.pending_since = None
        self.sampler = ClockSampler(settings['backend'], settings['max_age_ns'], settings['drift_ppm'])
        self.client = None
        self.timer = None

    def attach(self):
        from rcl_interfaces.srv import GetParameters
        self.request_type = GetParameters.Request
        node = self.runtime.node
        self.client = node.create_client(GetParameters, self.settings['coordinator_service'])
        self.timer = node.create_timer(self.settings['poll_period_s'], self.poll)
        self.poll()

    def poll(self):
        self.sampler.poll()
        if (self.pending is not None and not self.pending.done()
                and time.monotonic_ns()-self.pending_since >= self.runtime.inputs.lease_policy.max_age_ns):
            self.pending.cancel()
            self.pending = None
            with self.runtime.adapter.lock:
                self.peer = None
                self.runtime._stop_input_session()
            self.runtime.node.get_logger().warning('central input service timed out; no input renewed')
        if self.pending is not None and self.pending.done():
            try:
                response = self.pending.result()
                if len(response.values) != 1 or response.values[0].type != 4:
                    raise ContractError('central task source response type')
                record = json.loads(response.values[0].string_value)
                self.receive(record)
            except Exception as exc:
                with self.runtime.adapter.lock:
                    self.peer = None
                    self.runtime._stop_input_session()
                self.runtime.node.get_logger().error('live central input rejected: ' + str(exc))
            self.pending = None
        if self.pending is None and self.client.service_is_ready():
            self.pending = self.client.call_async(self.request_type(names=['source']))
            self.pending_since = time.monotonic_ns()

    def receive(self, record):
        r = self.runtime
        with r.adapter.lock:
            c, a = r.inputs.config, r.config['coordinator_authority']
            if (record['format'] != 'rescue-live-task-source/1'
                    or (record['coordinator_id'], record['authority_epoch'], record['schema_version'], record['clock_domain_id'])
                    != (a['coordinator_id'], a['authority_epoch'], c['schema_version'], r.inputs.lease_policy.clock_domain_id)
                    or record['lease_duration_s'] != r.inputs.lease_policy.duration_s
                    or not 0 < record['lease_renew_period_s'] <= record['lease_duration_s'] / 2):
                raise ContractError('central identity/domain/lease policy differs', 'NOT_AUTHORIZED')
            identifier(record['publisher_boot_id'])
            if (record['coordinator_id'],record['publisher_boot_id']) in r._retired_coordinators:
                raise ContractError('retired central clock source','NOT_AUTHORIZED')
            if r._coordinator is not None and record['publisher_boot_id'] != r._coordinator[1]:
                r._stop_input_session()
                # A fresh SystemState must explicitly register the new boot.
            self.validate_measurement(record['clock'])
            if not isinstance(record['destinations'], dict):
                raise ContractError('central destination object required')
            # The launch home (profile home_positions) stays until the coordinator sends one for this robot,
            # which then wins (TB-5 QR exit, OPS-139/144). Replacing everything lost it (#192 on #132).
            destinations = deepcopy(r.config.get('configured_destinations', {}))
            for ref, point in record['destinations'].items():
                if isinstance(ref, str) and ref.startswith('home/'):
                    # RETURN_HOME home_ref 'home/<robot_id>': the only non-UUID key (OPS-144 a)
                    if ref == 'home/':
                        raise ContractError('home destination key needs a robot id')
                    if point['robot_id'] != ref[len('home/'):]:
                        raise ContractError('home destination point is for another robot than its key')
                else:
                    identifier(ref)
                if point['robot_id'] != r.adapter.robot_id:
                    continue
                # resolve_goal will compare frame/session/revisions to the
                # independent live map basis, never to an arriving goal.
                xy = point['xy_yaw']
                if len(xy) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in xy):
                    raise ContractError('invalid configured central destination')
                destinations[ref] = point
            self.peer = record
            r.inputs.config['configured_destinations'] = destinations

    def validate_measurement(self, m):
        now = self.runtime.adapter.wall_clock_ns()
        if (m.get('state') != 'SYNCED' or not m.get('host_id') or not m.get('host_boot_id') or not m.get('epoch_id')
                or any(type(m.get(k)) is not int for k in ('measured_at_ns','sampled_at_ns','valid_until_ns','error_bound_ns'))
                or m['error_bound_ns'] < 0 or m['measured_at_ns'] <= 0
                or m['sampled_at_ns'] < m['measured_at_ns']
                or m['sampled_at_ns'] > now + self.runtime.inputs.bound
                or now + self.runtime.inputs.bound >= m['valid_until_ns']
                or now - m['measured_at_ns'] + self.runtime.inputs.bound >= self.settings['max_age_ns']
                or type(m.get('drift_ppm')) not in (int,float) or not math.isfinite(m['drift_ppm']) or m['drift_ppm'] <= 0):
            raise ContractError('fresh synchronized OS measurement required', 'CLOCK_UNCERTAIN')
        return m

    def basis(self):
        if self.peer is None:
            raise ContractError('waiting for measured coordinator clock', 'CLOCK_UNCERTAIN')
        local = self.validate_measurement(self.sampler.record)
        peer = self.validate_measurement(self.peer['clock'])
        same_host = local['host_id'] == peer['host_id']
        if self.settings.get('allow_same_host', False):
            # one-PC run (방식 B, launch single_host:=true; co_map 2026-10-15): both measurements must be of this one
            # host, boot and clock epoch; the two error bounds are still both counted below (conservative)
            if not same_host or any(not local.get(k) or local.get(k) != peer.get(k)
                                    for k in ('host_boot_id', 'epoch_id', 'backend')):
                raise ContractError('single desktop requires matching measured host, boot and clock epoch',
                                    'CLOCK_UNCERTAIN')
        elif same_host:
            raise ContractError('LIVE_ROS expects two distinct PCs', 'CLOCK_UNCERTAIN')
        now = self.runtime.adapter.wall_clock_ns()
        # Conservatively age both error bounds between measurements and use.
        error = sum(m['error_bound_ns'] + math.ceil(max(0, now-m['sampled_at_ns']) * m['drift_ppm'] / 1e6)
                    for m in (local,peer))
        epoch = hashlib.sha256(json.dumps([self.peer['publisher_boot_id'],
            *[(m['host_id'],m['host_boot_id'],m['epoch_id']) for m in (local,peer)]]).encode()).hexdigest()
        return ClockBasis(self.runtime.inputs.lease_policy.clock_domain_id, 'SYNCED', error, epoch,
                          min(local['valid_until_ns'],peer['valid_until_ns']))

    def close(self):
        if self.timer is not None:
            self.runtime.node.destroy_timer(self.timer)
        if self.pending is not None:
            self.pending.cancel()
        if self.client is not None:
            self.runtime.node.destroy_client(self.client)
        self.sampler.close()
