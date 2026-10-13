"""ObserveTarget client for the recheck after a REOBSERVE_TARGET stop (C1-b, SDD IF-023).

Called from the ExecuteTask action's ``execute`` coroutine after the terminal report: it is
awaited (rclpy futures inside the executor), never blocked on, so the single-threaded
executor keeps serving the action client. The deadline is enforced here with a timer;
no server, a rejected goal or no answer by the deadline returns None (has_recheck stays
false). Server answers are returned unchanged.
"""


class RecheckClient:
    def __init__(self, runtime, *, action_name):
        from rclpy.action import ActionClient
        from rescue_interfaces.action import ObserveTarget
        self.runtime, self.ObserveTarget = runtime, ObserveTarget
        self.action_name = action_name
        self.client = ActionClient(runtime.node, ObserveTarget, action_name)

    async def _until(self, future, deadline_ns):
        """The future's result, or None once the wall clock passes deadline_ns."""
        from rclpy.task import Future
        r = self.runtime
        done = Future()
        future.add_done_callback(lambda f: done.done() or done.set_result(f.result()))
        remaining_s = max(0.01, (deadline_ns - r.adapter.wall_clock_ns()) / 1e9)
        timer = r.node.create_timer(remaining_s, lambda: done.done() or done.set_result(None))
        try:
            return await done
        finally:
            r.node.destroy_timer(timer)

    async def __call__(self, fields):
        r = self.runtime
        log = r.node.get_logger()
        if not self.client.server_is_ready():
            log.error(f'recheck server {self.action_name} not available')
            return None
        goal = self.ObserveTarget.Goal()
        with r.adapter.lock:
            r.server.stamp_envelope(goal.envelope)
        for name in ('request_id', 'task_id', 'target_id', 'robot_id', 'requested_window_ns', 'policy_version',
                     'deadline_ns'):
            setattr(goal, name, fields[name])
        goal.watched_native_point.x, goal.watched_native_point.y, goal.watched_native_point.z = \
            fields['watched_native_point']
        for name, value in fields['frame_ref'].items():
            setattr(goal.frame_ref, name, value)
        handle = await self._until(self.client.send_goal_async(goal), fields['deadline_ns'])
        if handle is None or not handle.accepted:
            log.error(f'recheck {fields["request_id"]} not accepted by {self.action_name}')
            return None
        wrapped = await self._until(handle.get_result_async(), fields['deadline_ns'])
        if wrapped is None:
            log.error(f'recheck {fields["request_id"]} had no answer by its deadline; canceling')
            handle.cancel_goal_async()
            return None
        return wrapped.result
