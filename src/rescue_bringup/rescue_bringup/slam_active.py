"""Is TB-6 slam_toolbox active after its launch? (실습 기록 #339 B4, 2026-10-15)

turtlebot4_navigation slam.launch.py (slam_toolbox online_sync_launch.py) sends activate only when launch sees
the configure -> inactive transition event; launched in the background the node stayed inactive [추정: the
event left before launch's subscription matched; launch_ros waits for the change_state service only].
After wait_s this reads <node>/get_state and logs one line:
  SLAM_ACTIVE <node>                          active
  SLAM_NOT_ACTIVE <node> state=<label> ...    anything else, or NO_ANSWER
An inactive node is sent activate once (activate_once, logged) and read again. Nothing else is redone:
configure is launch's job, so an unconfigured node or no answer is only logged.

    slam_active_check --ros-args -p node:=/robot6/slam_toolbox -p wait_s:=15.0 -p activate_once:=true
"""


def judge(state, *, node, activate_once, sent):
    """(log line, 'ACTIVATE' or None) for a lifecycle state label (None = no answer)."""
    after = ' after activate' if sent else ''
    if state == 'active':
        return f'SLAM_ACTIVE {node}{after}', None
    label = state or 'NO_ANSWER'
    if state == 'inactive' and activate_once and not sent:
        return f'SLAM_NOT_ACTIVE {node} state={label}: sending activate once', 'ACTIVATE'
    if sent:
        return f'SLAM_NOT_ACTIVE {node} state={label}{after}: start it by hand (ros2 lifecycle set)', None
    return f'SLAM_NOT_ACTIVE {node} state={label}: not changed here', None


def main(args=None):
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState, GetState

    rclpy.init(args=args)
    node = rclpy.create_node('tb6_slam_active_check')
    try:
        node.declare_parameter('node', '')
        node.declare_parameter('wait_s', 0.0)
        node.declare_parameter('activate_once', False)
        target = node.get_parameter('node').value
        wait_s = float(node.get_parameter('wait_s').value)
        activate_once = bool(node.get_parameter('activate_once').value)
        log = node.get_logger()
        if not target or wait_s <= 0:
            log.fatal('node and a positive wait_s are required')
            raise SystemExit(2)
        get_state = node.create_client(GetState, target + '/get_state')
        change = node.create_client(ChangeState, target + '/change_state')

        def call(client, request, timeout_s):
            if not client.wait_for_service(timeout_sec=timeout_s):
                return None
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
            return future.result() if future.done() else None

        def state():
            answer = call(get_state, GetState.Request(), 5.0)
            return answer.current_state.label if answer is not None else None

        end = node.get_clock().now().nanoseconds + int(wait_s * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        line, action = judge(state(), node=target, activate_once=activate_once, sent=False)
        (log.info if line.startswith('SLAM_ACTIVE') else log.error)(line)
        if action == 'ACTIVATE':
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_ACTIVATE
            answer = call(change, request, 5.0)
            log.warning(f'SLAM_ACTIVATE_SENT {target} success={None if answer is None else answer.success}')
            line, _ = judge(state(), node=target, activate_once=activate_once, sent=True)
            (log.info if line.startswith('SLAM_ACTIVE') else log.error)(line)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
