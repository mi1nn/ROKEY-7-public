"""RobotStatus from the existing adapter; no fake lease, battery or evidence.

An idle ARMED executor has readiness but no CommandLease. The coordinator
must not require lease_valid before issuing its first task (#90 bootstrap gap).
"""
def fill_robot_status(msg, adapter, ingress, *, state_sequence):
    with adapter.lock:
        executor, now = adapter.executor, adapter.clock()
        active = executor.active
        msg.robot_id, msg.robot_boot_id = executor.robot_id, executor.robot_boot_id
        msg.authority_epoch, msg.state_sequence = executor.authority_epoch, state_sequence
        msg.observed_at_ns = adapter.wall_clock_ns()
        msg.active_task_id = active.task_id if active else ''
        msg.generation = active.generation if active else max(0, executor.highest_generation)
        msg.task_state = executor.state
        ready = adapter.readiness(now).valid(now, executor.policy)
        msg.readiness = 'READY' if ready and executor.latch == 'ARMED' else 'NOT_READY'
        msg.motion_state = 'STOPPING' if executor.state == 'STOPPING' else executor.motion
        if msg.motion_state == 'STOP_REQUESTED': msg.motion_state = 'STOPPING'
        msg.stop_reason = executor.stop_cause if executor.state in ('STOPPING', 'FINISHED') else ''
        if executor.state == 'FINISHED' and executor.rearm_refused:
            # Completed, but the automatic re-arm was refused (OPS-231): task_arm is needed.
            msg.stop_reason = 'REARM_REFUSED_' + executor.rearm_refused
        request = ingress.requests.get(active.command_id) if active else None
        msg.segment_index = (request[0].route.segment_index
                             if request and request[0].has_route else 0)
        msg.stop_evidence = []  # local stop gate is real; EvidenceRef records require a separate recorder
        msg.lease_valid = bool(active and now < executor.lease_until
                               and executor.state != 'STOPPING' and not executor.session_fault
                               and not ingress.clock_fault)
        undocked = adapter.status.get('undocked')
        msg.dock_state, msg.dock_observed_at_ns = 'UNKNOWN', 0
        if undocked is not None and 0 <= now - undocked[1] <= executor.policy.evidence_max_age:
            msg.dock_state = 'UNDOCKED' if undocked[0] else 'DOCKED'
            # Receipt time measured on this host, not a made-up device capture stamp.
            msg.dock_observed_at_ns = msg.observed_at_ns - round((now - undocked[1]) * 1e9)
        nav2 = adapter.status.get('nav2')
        msg.nav2_state = 'UNKNOWN'
        if nav2 is not None and 0 <= now - nav2[1] <= executor.policy.evidence_max_age:
            msg.nav2_state = 'READY' if nav2[0] else 'NOT_READY'
        msg.sensor_quality, msg.battery_known = 'UNKNOWN', False
        return msg
