"""One-PC full scenario runner (방식 B live shared map; user work order 2026-10-15).

    bash scripts/field/run_full_scenario.sh --profile <profile.json> [options]   # sources ROS, then this module

One command after "the webcam saw an earthquake" (assumed: no webcam judgement runs here). The runner

1. checks the environment (ROS distro, ROS_DOMAIN_ID against the profile, Discovery Server, the clock, packages) and
   writes the run inputs to a new run folder: the merged live-map field file (config/field/live_map.overlay.json on
   search_area.field.json) with the measured TB-5 start, so every launch reads one file;
2. starts the stages in dependency order, each as its own process group with its own log, and waits for real
   readiness (a message, a publisher, a service, an action server, an ALIGNED start_align, a lifecycle state), never
   for "the process started"; a stage not ready in time stops the run with the stage name and the reason;
3. refreshes the coordinator state and arms both executors (task_arm, the field step 3 order), then publishes ONE
   DISASTER_SIGN on /rescue/site_events marked synthetic (camera ``cam_full_scenario_start``, policy
   ``FULL_SCENARIO_SYNTHETIC_START``): the same contract the webcam node uses, so the coordinator's own gates
   (search_start AUTO, READY, lease, clock) start the search;
4. watches /rescue/mission_outcome, /rescue/system_state and the robots until the coordinator reports
   MISSION_FINISHED (TB-5 waiting at its exit, TB-6 docked at its start pose) or a failure;
5. on any failure, Ctrl+C or the end: safe stop through the existing robot interfaces (executor task_stop, cancel
   every Nav2 and dock goal, cmd_vel zero; the field stop_robot.sh steps) for both robots, then INT -> TERM -> KILL
   to every process group it started, and reports what is left.

``--rehearsal`` (scripts/field/full-scenario-rehearsal/run.sh) replaces only the robots, their Pis and the doll
detector with a 2D simulation (sim_world.py); SLAM, AMCL, start_align, Nav2, executors, watchdogs, coordinator,
operations nodes, dock node and alert run for real. A rehearsal is not a field result.

The plan and the verdict are plain functions (``stage_plan``, ``judge``) tested without ROS.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

REPO_HINT = 'run from the repository root or pass --field'
DEFAULT_OVERLAY = Path('config/field/live_map.overlay.json')
SYNTHETIC_CAMERA = 'cam_full_scenario_start'
SYNTHETIC_POLICY = 'FULL_SCENARIO_SYNTHETIC_START'
# Placeholder Phase 1 route/observe goals: unused in FRONTIER + AUTO dispatch + EXIT_WAIT_SCOUT_DOCK (the coordinator
# launch still requires them, #132 ran with these placeholders; field README 1단계).
DEFAULT_COORDINATOR_ARGS = {'route_segment_goals': '[0.0, 0.0, 0.0]', 'observe_goal': '[0.0, 0.0, 0.0]',
                            'route_segment_passage_free': '[false]'}
# How long each stage may take to become ready (s) [제안 2026-10-15, not measured on the field PCs]
STAGE_TIMEOUTS = {'sim_world': 30, 'tb6_slam': 60, 'live_map': 30, 'passage': 60, 'robots': 120,
                  'tb5_localization': 150, 'perception': 90, 'operations': 60, 'coordinator': 60, 'alert': 30,
                  'web': 30, 'rviz': 20, 'arm': 90, 'disaster': 30}
# Stages started again once when not ready in time: tb6_slam only. Its lifecycle activation answer was lost once at
# start (rehearsal 2026-10-15: 'failed to send response to /robot6/slam_toolbox/change_state (timeout)', so
# slam_toolbox never activated); restarting it is safe while it has published no map (the map guard refuses a
# second publisher otherwise). Nothing else is retried: a robot launch that did not come up stops the run.
# The same restart covers a standing TB-6's one SLAM pose lost to the slam_pose_latest relay (it is published right
# after activation, before DDS matched every subscriber; rehearsal 2026-10-15): without it the executor has no
# covariance and TB-6 never gets its first goal (SCOUT_POSE_UNKNOWN).
STAGE_RETRIES = {'tb6_slam': 2}
NAV2_HEALTH_PERIOD_S = 10.0   # how often the mission watch asks both bt_navigators for their lifecycle state
TERMINAL_FAIL = {'approach': ('FAILED',), 'support_exit': ('FAILED', 'CANCELED'), 'scout_return': ('FAILED',),
                 'scout_dock': ('FAILED',)}


# start_align failures worth a restart while TB-5 stands on its spot: the live map may have too little of TB-5's
# surroundings yet (LOW_MATCH) or AMCL needs more updates (NOT_CONVERGED). A seed far off (JUMP_TOO_LARGE), a docked
# or moving robot, missing scans or AMCL services are not retried: the run stops with that reason.
ALIGN_RETRYABLE = ('START_ALIGN_LOW_MATCH', 'START_ALIGN_NOT_CONVERGED')
ALIGN_RESTARTS, ALIGN_RESTART_GAP_S = 3, 5.0   # [제안 2026-10-15]


class AlignSupervisor:
    """What the runner does with each start_align status: 'done', 'wait', 'restart' or ('fail', reason)."""

    def __init__(self, *, restarts=ALIGN_RESTARTS, gap_s=ALIGN_RESTART_GAP_S):
        self.left, self.gap_s, self.last_restart = restarts, gap_s, None

    def decide(self, status, now_s):
        if not status:
            return 'wait'
        if status.get('status') == 'ALIGNED':
            return 'done'
        if status.get('status') != 'FAILED':
            return 'wait'
        reason = status.get('reason', '')
        if reason not in ALIGN_RETRYABLE:
            return ('fail', f'start_align FAILED {reason}: not retried ({json.dumps(status, sort_keys=True)})')
        if self.left <= 0:
            return ('fail', f'start_align FAILED {reason} after {ALIGN_RESTARTS} restarts '
                            f'({json.dumps(status, sort_keys=True)})')
        if self.last_restart is not None and now_s - self.last_restart < self.gap_s:
            return 'wait'
        self.left -= 1
        self.last_restart = now_s
        return 'restart'


class ScenarioError(RuntimeError):
    """A stage that cannot start or did not become ready: (stage, reason)."""

    def __init__(self, stage, reason):
        super().__init__(f'[{stage}] {reason}')
        self.stage, self.reason = stage, reason


# ---- plan (no ROS) -------------------------------------------------------------------------------------------
def tb5_seed(argument, env):
    """The measured TB-5 start [x m, y m, yaw rad] (relative to the TB-6 start pose) from --tb5-start or
    TB5_START_X/Y/YAW; None when neither is given (the field file may still hold one). Half a value is refused."""
    if argument is not None:
        values = list(argument)
    else:
        names = ('TB5_START_X', 'TB5_START_Y', 'TB5_START_YAW')
        given = [env.get(name, '') for name in names]
        if not any(given):
            return None
        if not all(given):
            raise ScenarioError('preflight', f'TB5_START_X/Y/YAW: all three or none ({dict(zip(names, given))})')
        try:
            values = [float(v) for v in given]
        except ValueError as exc:
            raise ScenarioError('preflight', f'TB5_START_X/Y/YAW must be numbers: {exc}') from None
    if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
        raise ScenarioError('preflight', 'TB-5 start must be three finite numbers x y yaw (m, m, rad)')
    return [float(v) for v in values]


def run_field(field_path, seed):
    """Merged field run (overlay applied) with the TB-5 seed written in; raw dict ready for the run folder."""
    from .field_launch import FieldLaunchError, load_field
    try:
        field = load_field(field_path)
    except FieldLaunchError as exc:
        raise ScenarioError('preflight', f'field file {field_path}: {exc}') from None
    raw = json.loads(json.dumps(field.raw))
    if raw['coordinator']['map_mode'] != 'SLAM_LIVE' or raw['coordinator']['completion_mode'] != 'EXIT_WAIT_SCOUT_DOCK':
        raise ScenarioError('preflight', 'the full scenario needs the live-map field (map_mode SLAM_LIVE, '
                                         'completion_mode EXIT_WAIT_SCOUT_DOCK): config/field/live_map.overlay.json')
    support = [r for r, s in raw['robots'].items() if s['task.saved_map.mode'] == 'LIVE_MAP_AMCL']
    if len(support) != 1:
        raise ScenarioError('preflight', 'exactly one LIVE_MAP_AMCL robot (TB-5) is needed')
    section = raw['robots'][support[0]]
    if seed is not None:
        section['initial_pose_xy_yaw'] = seed
        raw['source'] += f' || TB-5 start {seed} (m, m, rad, relative to the TB-6 start pose) given at the run'
    elif 'initial_pose_xy_yaw' not in section:
        raise ScenarioError('preflight', 'TB-5 시작 자세가 없다: --tb5-start X Y YAW 또는 TB5_START_X/Y/YAW (m, m, rad, '
                                         'TB-6 출발 자세 기준, 현장 측정값). 예시값을 넣지 않는다')
    return raw, support[0]


def roles_of(deployment):
    roles = {role: robot for robot, role in deployment.mission.roles}
    return roles['SCOUT'], roles['SUPPORT']


def stage_plan(*, profile, field, run_dir, scout, support, ns, options):
    """[(stage, [(label, argv)])] in start order. ``ns``: robot id -> namespace. ``options``: rviz, alert,
    overhead, rehearsal, sim_world, perception_args (dict), coordinator_args (dict), webcam_params, web (dict|None),
    api_port, python."""
    launch = ['ros2', 'launch', 'rescue_bringup']
    base = [f'profile_path:={profile}', f'field_path:={field}']
    plan = []
    if options['rehearsal']:
        plan.append(('sim_world', [('sim_world', [options['python'], str(options['sim_world']), '--profile', str(profile),
                                                  '--field', str(field), '--world', str(options['world'])])]))
    # The start pose recorder first: slam_toolbox publishes <ns>/pose only for the scans it processes, and a TB-6
    # standing still gets none after its first one (minimum_travel_distance), so a recorder started after SLAM may
    # never see a pose before TB-6 moves (rehearsal 2026-10-15). It waits; tb6_slam is ready with the map AND it.
    plan.append(('live_map', [('live_map', launch + ['live_map.launch.py', *base,
                                                    f'output_dir:={run_dir / "live_map"}'])]))
    plan.append(('tb6_slam', [('tb6_slam', launch + ['tb6_slam.launch.py', *base, f'robot_id:={scout}'])]))
    if not options['rehearsal']:   # the simulation publishes empty passage points itself
        passage = [(f'passage_{r}', launch + ['passage_points.launch.py', *base, f'robot_id:={r}'])
                   for r in (scout, support)]
        if options['overhead']:
            passage += [(f'overhead_{r}', launch + ['overhead_passage.launch.py', *base, f'robot_id:={r}'])
                        for r in (scout, support)]
        plan.append(('passage', passage))
    # TB-5 localization before the robot launches: TB-5's Nav2 costmaps (global frame map, static layer on the live
    # map) only activate once AMCL publishes TB-5's map -> odom (rehearsal 2026-10-15: bt_navigator never active
    # otherwise), and start_align needs no Nav2. So the work order's step 4 runs before step 3 for TB-5.
    plan.append(('tb5_localization', [('tb5_localization', launch + [
        'live_localization.launch.py', *base, f'robot_id:={support}', 'start_pose_wait_s:=30.0'])]))
    plan.append(('robots', [(f'robot_{r}', launch + ['robot.launch.py', *base, f'robot_id:={r}', 'single_host:=true'])
                            for r in (scout, support)]))
    if options['rehearsal']:   # the real target_locator / exit_locator; sim_world.py stands in for the detector
        plan.append(('perception', [('perception', ['ros2', 'launch', str(options['rehearsal_perception']), *base])]))
    else:
        args = [f'{k}:={v}' for k, v in sorted(options['perception_args'].items())]
        plan.append(('perception', [('perception', launch + ['amr_perception.launch.py', f'profile_path:={profile}',
                                                             *args])]))
    ops = ['operations.launch.py', f'profile_path:={profile}', f'db_path:={run_dir / "site_state.sqlite3"}',
           f'api_port:={options["api_port"]}', 'command_timeout_s:=3.0',
           f'report_db_path:={run_dir / "reports.sqlite3"}',
           f'envelope_schema_version:={options["envelope_schema_version"]}',
           f'report_publisher_id:={options["report_publisher_id"]}']
    if options['webcam_params']:
        ops.append(f'webcam_params_file:={options["webcam_params"]}')
    if options['web'] is not None:
        ops += [f'map_topic:={ns[scout]}/map', f'map_dir:={run_dir / "map_snapshot"}']
    plan.append(('operations', [('operations', launch + ops)]))
    coordinator = dict(DEFAULT_COORDINATOR_ARGS, **options['coordinator_args'])
    plan.append(('coordinator', [('coordinator', launch + ['coordinator.launch.py', *base,
                                                          *[f'{k}:={v}' for k, v in sorted(coordinator.items())]])]))
    plan.append(('alert', [('found_alert', ['ros2', 'run', 'rescue_operations', 'found_alert', '--ros-args',
                                            '-p', f'enabled:={str(options["alert"]).lower()}',
                                            '-p', f'journal_path:={run_dir / "found_alert.json"}'])]))
    if options['web'] is not None:
        web = options['web']
        plan.append(('web', [('web', [options['python'], '-m', 'rescue_console', '--accounts', web['accounts'],
                                      '--site-state', str(run_dir / 'site_state.sqlite3'),
                                      '--operations', f'http://127.0.0.1:{options["api_port"]}',
                                      '--robot-name', f'{scout}=TB-6', '--robot-name', f'{support}=TB-5',
                                      '--support-dispatch', 'AUTO', '--search-start', 'AUTO',
                                      '--live-map', str(run_dir / 'map_snapshot' / 'live_map.yaml'),
                                      '--live-map-stale-s', '15', '--port', str(web['port'])])]))
    if options['rviz']:
        plan.append(('rviz', [('rviz', launch + ['shared_map_view.launch.py', *base, 'show_all:=true'])]))
    return plan


def judge(outcome, *, child_exits=(), timed_out=False, support_task_after_exit='', nav2_down=''):
    """('RUNNING' | 'SUCCESS' | 'FINISHED_INCOMPLETE' | 'FAILED', reason) from the coordinator's outcome JSON.

    SUCCESS only when the coordinator says MISSION_FINISHED, the exploration ended with EXPLORATION_COMPLETE (not
    the time limit), TB-6's dock was confirmed and TB-5 reached its exit (or no survivor was found, so TB-5 never
    left). A finish on EXPLORATION_TIMEOUT is FINISHED_INCOMPLETE: TB-6 still came home and docked, but the map was
    not completed and is not reported as such."""
    if child_exits:
        label, code = child_exits[0]
        return 'FAILED', f'PROCESS_EXITED {label} code={code}'
    if support_task_after_exit:
        return 'FAILED', f'SUPPORT_TASK_AFTER_EXIT {support_task_after_exit}'
    if nav2_down:   # a Nav2 stack fell over while its launch process lives on (rehearsal 2026-10-15)
        return 'FAILED', f'NAV2_DOWN {nav2_down}'
    outcome = outcome or {}
    for key, bad in TERMINAL_FAIL.items():
        if outcome.get(key) in bad:
            detail = outcome.get('approach_failure') if key == 'approach' else outcome.get('scout_dock_reason', '')
            return 'FAILED', f'{key.upper()}_{outcome[key]} {detail or ""}'.strip()
    if outcome.get('mode') == 'MISSION_FINISHED':
        if outcome.get('scout_dock') != 'DOCKED':
            return 'FAILED', 'FINISHED_WITHOUT_DOCK'
        if not (outcome.get('support_exit') == 'AT_EXIT' or outcome.get('no_survivor')):
            return 'FAILED', 'FINISHED_WITHOUT_SUPPORT_AT_EXIT'
        if outcome.get('exploration_reason') != 'EXPLORATION_COMPLETE':
            return 'FINISHED_INCOMPLETE', f'exploration ended by {outcome.get("exploration_reason") or "?"}'
        return 'SUCCESS', 'NO_SURVIVOR_FOUND' if outcome.get('no_survivor') else 'TB-5 AT_EXIT, TB-6 DOCKED'
    if timed_out:
        return 'FAILED', 'MISSION_TIMEOUT'
    return 'RUNNING', ''


def profile_domains(deployment):
    """The ROS_DOMAIN_ID values the profile's hosts name (one PC runs them all)."""
    out = set()
    for host in deployment.hosts:
        network = getattr(host, 'network', None)
        if network is not None and getattr(network, 'domain_id', None) is not None:
            out.add(network.domain_id)
    return out


def env_checks(env, *, rehearsal, domains):
    """[(name, ok, detail)] for the DDS/ROS environment this PC runs everything in."""
    checks = [('ROS_DISTRO', env.get('ROS_DISTRO') == 'jazzy', env.get('ROS_DISTRO', '(unset)'))]
    domain = env.get('ROS_DOMAIN_ID', '')
    if not domain.isdigit():
        checks.append(('ROS_DOMAIN_ID', False, f'{domain or "(unset)"}: set it explicitly'))
    elif rehearsal:
        checks.append(('ROS_DOMAIN_ID', int(domain) >= 10, f'{domain} (rehearsal: >= 10, apart from the robots)'))
    elif domains:
        checks.append(('ROS_DOMAIN_ID', int(domain) in domains, f'{domain}; profile hosts {sorted(domains)}'))
    else:
        checks.append(('ROS_DOMAIN_ID', True, f'{domain} (profile names no domain)'))
    server = env.get('ROS_DISCOVERY_SERVER', '')
    if rehearsal:   # this PC only: a local Discovery Server on 127.0.0.1, or LOCALHOST simple discovery
        local = server and all(part.strip().startswith('127.0.0.1:') for part in server.split(';') if part.strip())
        checks.append(('discovery (this PC only)', bool(local) or (
            not server and env.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'),
            f'ROS_DISCOVERY_SERVER={server or "(unset)"} '
            f'ROS_AUTOMATIC_DISCOVERY_RANGE={env.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "(unset)")}'))
    else:
        checks.append(('ROS_DISCOVERY_SERVER', True, server or '(unset: plain multicast discovery)'))
    checks.append(('RMW_IMPLEMENTATION', True, env.get('RMW_IMPLEMENTATION', '(default)')))
    # Two Nav2 stacks + SLAM + AMCL + ours on one PC are about 80 nodes and 600 services. With Fast DDS's default
    # shared-memory transport discovery stalled (rehearsal 2026-10-15: the TB-5 executor never matched the
    # coordinator clock service in 90 s); UDPv4 only, as the field robot launches already run (field README 2단계).
    transports = env.get('FASTDDS_BUILTIN_TRANSPORTS', '')
    checks.append(('FASTDDS_BUILTIN_TRANSPORTS', transports == 'UDPv4', f'{transports or "(unset)"} (UDPv4 required)'))
    return checks


# ---- processes -------------------------------------------------------------------------------------------------
class Children:
    """Child process groups, one log each; stopped INT -> TERM -> KILL, never by name (AGENTS 6)."""

    def __init__(self, log_dir, say):
        self.log_dir, self.say, self.items, self.env = Path(log_dir), say, [], None
        self.log_dir.mkdir(parents=True, exist_ok=True)

    def start(self, label, argv, env=None):
        log = open(self.log_dir / f'{label}.log', 'ab')
        log.write(f'$ {shlex.join(argv)}\n'.encode())
        log.flush()
        proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                env=env or self.env)
        self.items.append((label, proc, log))
        self.say(f'  시작 {label} pid={proc.pid} log={self.log_dir / (label + ".log")}')
        (self.log_dir.parent / 'pids.json').write_text(json.dumps({lbl: p.pid for lbl, p, _ in self.items}, indent=1))
        return proc

    def stop(self, procs):
        """Stop some of the started processes (a stage started again) and forget them."""
        for proc in procs:
            for sig, wait in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
                if proc.poll() is not None:
                    break
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + wait
                while time.monotonic() < deadline and proc.poll() is None:
                    time.sleep(0.2)
        self.items = [(label, proc, log) for label, proc, log in self.items if proc not in procs]

    def exited(self):
        return [(label, proc.returncode) for label, proc, _ in self.items if proc.poll() is not None]

    def stop_all(self, *, grace_s=10.0):
        """Stop in reverse start order; returns the labels still alive after KILL."""
        alive = [(label, proc) for label, proc, _ in reversed(self.items) if proc.poll() is None]
        for sig, wait in ((signal.SIGINT, grace_s), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
            if not alive:
                break
            for label, proc in alive:
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline and any(p.poll() is None for _, p in alive):
                time.sleep(0.2)
            alive = [(label, proc) for label, proc in alive if proc.poll() is None]
            if alive:
                self.say(f'  {sig.name} 뒤에도 남음: {[label for label, _ in alive]}')
        left = []
        for label, proc, log in self.items:
            try:
                os.killpg(proc.pid, 0)
                left.append(label)   # the group still has members (a grandchild survived)
            except ProcessLookupError:
                pass
            log.close()
        return left


# ---- ROS -------------------------------------------------------------------------------------------------------
class Ros:
    """The runner's own node: readiness probes, arm, the start event, the watch and the safe stop."""

    def __init__(self, *, scout, support, ns, field):
        import threading
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rescue_interfaces.msg import RobotStatus, SystemState
        from std_msgs.msg import String

        from rclpy.signals import SignalHandlerOptions
        # The runner owns SIGINT: rclpy's own handler shut the context down on Ctrl+C before the safe stop could
        # call a single service (rehearsal 2026-10-15: 'rcl node's context is invalid'). Python's KeyboardInterrupt
        # reaches main(), which stops the robots first and only then closes ROS.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy, self.lock = rclpy, threading.Lock()
        self.node = rclpy.create_node('full_scenario_runner')
        self.scout, self.support, self.ns, self.field = scout, support, ns, field
        self.latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = {'system': None, 'outcome': None, 'align': None, 'dock': None, 'status': {}, 'seen': {}}
        n = self.node
        n.create_subscription(SystemState, '/rescue/system_state', lambda m: self._put('system', m), self.latched)
        n.create_subscription(String, '/rescue/mission_outcome', lambda m: self._put('outcome', json.loads(m.data)),
                              self.latched)
        n.create_subscription(String, ns[support] + '/rescue_start_align',
                              lambda m: self._put('align', json.loads(m.data)), self.latched)
        n.create_subscription(String, f'/rescue/robots/{scout}/dock_state',
                              lambda m: self._put('dock', json.loads(m.data)), self.latched)
        for robot in (scout, support):
            n.create_subscription(RobotStatus, f'/rescue/robots/{robot}/status',
                                  lambda m, robot=robot: self._status(robot, m), 10)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _put(self, key, value):
        with self.lock:
            self.state[key] = value

    def _status(self, robot, msg):
        with self.lock:
            self.state['status'][robot] = (msg, time.time_ns())

    def put_align(self, value):
        self._put('align', value)

    def get(self, key):
        with self.lock:
            return self.state[key]

    def status(self, robot):
        with self.lock:
            return self.state['status'].get(robot)

    # probes
    def message_on(self, topic, kind, qos=None, *, key=None):
        """Start a one-time watch; returns a callable that is True once a message arrived."""
        from rclpy.qos import qos_profile_sensor_data
        got = []
        sub = self.node.create_subscription(kind, topic, got.append, qos or qos_profile_sensor_data)
        self.state['seen'][key or topic] = sub
        return lambda: bool(got)

    def node_up(self, full_name):
        return full_name in {f'{n_ns.rstrip("/")}/{n}' for n, n_ns in self.node.get_node_names_and_namespaces()}

    def publishers(self, topic):
        return self.node.count_publishers(topic)

    def has_service(self, name):
        return any(n == name for n, _ in self.node.get_service_names_and_types())

    def action_ready(self, kind, name):
        from rclpy.action import ActionClient
        key = ('action', name)
        if key not in self.state['seen']:
            self.state['seen'][key] = ActionClient(self.node, kind, name)
        return self.state['seen'][key].server_is_ready()

    def call(self, kind, name, request, timeout_s=5.0):
        """(True, response) or (False, reason)."""
        client = self.state['seen'].get(('client', name))
        if client is None:
            client = self.state['seen'][('client', name)] = self.node.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, 'service not available'
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False, f'no answer within {timeout_s} s'
        if future.exception() is not None:
            return False, str(future.exception())
        return True, future.result()

    def lifecycle_active(self, node_name):
        from lifecycle_msgs.srv import GetState
        ok, res = self.call(GetState, node_name + '/get_state', GetState.Request(), timeout_s=2.0)
        return ok and res.current_state.id == 3

    def publish_start_event(self, coordinator):
        """One synthetic DISASTER_SIGN (the webcam node's contract, marked synthetic); returns its event_id."""
        import hashlib
        from uuid import uuid4
        from rescue_contracts.clock_measurement import ClockSampler
        from rescue_interfaces.msg import SiteEvent
        sampler = ClockSampler(coordinator['live_inputs.clock_backend'], coordinator['live_inputs.clock_max_age_ns'],
                               coordinator['live_inputs.clock_drift_ppm'])
        deadline = time.monotonic() + 10.0   # the sampler measures in the background: poll until it has one
        record = sampler.poll()
        while record.get('state') != 'SYNCED' and time.monotonic() < deadline:
            time.sleep(0.2)
            record = sampler.poll()
        sampler.close()
        if record.get('state') != 'SYNCED' or type(record.get('error_bound_ns')) is not int:
            raise ScenarioError('disaster', f'host clock not SYNCED: {record.get("state")}')
        pub = self.node.create_publisher(SiteEvent, '/rescue/site_events', 10)
        deadline = time.monotonic() + 10
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.1)
        if pub.get_subscription_count() == 0:
            raise ScenarioError('disaster', 'no subscriber on /rescue/site_events (coordinator not connected)')
        stamp, boot = time.time_ns(), str(uuid4())
        event = SiteEvent()
        event.event_id, event.camera_id, event.source_boot_id = str(uuid4()), SYNTHETIC_CAMERA, boot
        event.kind, event.confidence, event.visibility_state = 'DISASTER_SIGN', 1.0, 'VALID'
        event.policy_version = SYNTHETIC_POLICY
        event.metrics_name, event.metrics_value = ['SYNTHETIC_SCENARIO_START'], [1.0]
        t = event.time
        t.observed_at_ns, t.valid_until_ns = stamp, stamp + round(coordinator['status_max_age_s'] * 1e9)
        t.stamp_semantics, t.clock_state = 'DRIVER_RECEIVE', 'SYNCED'
        t.clock_domain_id = coordinator['live_inputs.clock_domain_id']
        t.has_clock_error_bound, t.clock_error_bound_ns = True, record['error_bound_ns']
        env = event.envelope
        env.schema_version, env.publisher_id = coordinator['envelope_schema_version'], 'full_scenario.start'
        env.publisher_boot_id, env.sequence, env.published_at_ns = boot, 1, stamp
        payload = json.dumps(dict(synthetic=True, purpose='assumed webcam earthquake; start gate only',
                                  event_id=event.event_id, observed_at_ns=stamp), sort_keys=True).encode()
        e = event.evidence
        e.evidence_id, e.sha256, e.mime_type = str(uuid4()), hashlib.sha256(payload).hexdigest(), 'application/json'
        e.source_id, e.source_boot_id, e.observed_at_ns = event.camera_id, boot, stamp
        pub.publish(event)
        return event.event_id, json.loads(payload)

    def safe_stop(self, say):
        """Both robots: task_stop, cancel Nav2 and dock goals, cmd_vel zero (scripts/field/stop_robot.sh steps)."""
        from action_msgs.srv import CancelGoal
        from geometry_msgs.msg import TwistStamped
        from std_srvs.srv import Trigger
        for robot in (self.support, self.scout):
            ns = self.ns[robot]
            steps = [(Trigger, f'{ns}/rescue_command_adapter/task_stop', Trigger.Request(), '실행기 task_stop')]
            steps += [(CancelGoal, f'{ns}/{a}/_action/cancel_goal', CancelGoal.Request(), f'Nav2 {a} 목표 전부 취소')
                      for a in ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints')]
            if robot == self.scout:
                steps += [(Trigger, f'{ns}/rescue_dock/cancel', Trigger.Request(), '도킹 노드 취소'),
                          (CancelGoal, f'{ns}/dock/_action/cancel_goal', CancelGoal.Request(), 'Create3 dock 목표 취소')]
            for kind, name, request, label in steps:
                if not self.has_service(name):
                    say(f'  [없음] {robot} {label}: {name}')
                    continue
                ok, res = self.call(kind, name, request, timeout_s=3.0)
                say(f'  [{"보냄" if ok else "실패"}] {robot} {label}: {name}'
                    + ('' if ok else f' ({res})'))
            pub = self.node.create_publisher(TwistStamped, f'{ns}/cmd_vel', 10)
            for _ in range(5):
                msg = TwistStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                pub.publish(msg)
                time.sleep(0.1)
            say(f'  [보냄] {robot} cmd_vel 0 x5: {ns}/cmd_vel (구독자 {pub.get_subscription_count()})')

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=2.0)
            self.node.destroy_node()
        finally:
            self.rclpy.try_shutdown()


def wait_until(stage, checks, timeout_s, say, *, progress=None, children=None):
    """Wait until every (name, predicate) is True; ScenarioError(stage, names still false) on timeout or a dead
    child. ``progress`` (callable -> str) is printed when its text changes."""
    deadline, said = time.monotonic() + timeout_s, None
    while True:
        pending = [name for name, ok in checks if not ok()]
        if not pending:
            return
        if children is not None and children.exited():
            label, code = children.exited()[0]
            raise ScenarioError(stage, f'process {label} exited (code {code}) before ready; see its log')
        text = progress() if progress else None
        if text and text != said:
            said = text
            say(f'  … {text}')
        if time.monotonic() > deadline:
            raise ScenarioError(stage, f'not ready within {timeout_s} s: {pending}' + (f' ({text})' if text else ''))
        time.sleep(0.5)


def readiness(stage, ros, *, scout, support, ns, options):
    """[(name, predicate)] that make ``stage`` ready; ``progress`` callable or None."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from sensor_msgs.msg import LaserScan, PointCloud2
    progress = None
    if stage == 'sim_world':
        checks = [(f'{ns[r]}/scan', ros.message_on(f'{ns[r]}/scan', LaserScan)) for r in (scout, support)]
    elif stage == 'live_map':
        checks = [(name, lambda name=name: ros.node_up(name)) for name in ('/rescue_slam_start_pose',
                                                                            '/rescue_map_correction',
                                                                            '/rescue_slam_pose_latest')]
    elif stage == 'tb6_slam':
        topic = f'/rescue/map/start_pose/{scout}'
        latest = f'{ns[scout]}/rescue/slam_pose_latest'   # field_launch.SLAM_POSE_LATEST: the executor's covariance
        checks = [(f'{ns[scout]}/map', ros.message_on(f'{ns[scout]}/map', OccupancyGrid, ros.latched)),
                  (topic, ros.message_on(topic, PoseWithCovarianceStamped, ros.latched)),
                  (latest, ros.message_on(latest, PoseWithCovarianceStamped))]
    elif stage == 'passage':
        checks = [(f'{ns[r]}/passage_points', ros.message_on(f'{ns[r]}/passage_points', PointCloud2))
                  for r in (scout, support)]
    elif stage == 'robots':
        checks = []
        for r in (scout, support):
            checks += [(f'{ns[r]}/rescue_command_adapter/task_arm',
                        lambda r=r: ros.has_service(f'{ns[r]}/rescue_command_adapter/task_arm')),
                       (f'{ns[r]}/bt_navigator active', lambda r=r: ros.lifecycle_active(f'{ns[r]}/bt_navigator')),
                       (f'{ns[r]}/navigate_to_pose', lambda r=r: ros.action_ready(NavigateToPose,
                                                                                   f'{ns[r]}/navigate_to_pose'))]
        checks.append((f'/rescue/robots/{scout}/dock', lambda: ros.has_service(f'/rescue/robots/{scout}/dock')))
    elif stage == 'tb5_localization':
        supervisor = AlignSupervisor()

        def align():
            from std_srvs.srv import Trigger
            a = ros.get('align')
            action = supervisor.decide(a, time.monotonic())
            if isinstance(action, tuple):
                raise ScenarioError('tb5_localization', action[1])
            if action == 'restart':
                ros.put_align(None)   # the next latched status is the restarted run's
                ok, res = ros.call(Trigger, f'{ns[support]}/rescue_start_align/restart', Trigger.Request())
                ros.say(f'  … start_align {a.get("reason")} → restart ({supervisor.left} left): '
                        f'{res.message if ok else res}')
            return action == 'done'
        checks = [(f'{ns[support]}/rescue_start_align ALIGNED', align)]

        def progress():
            a = ros.get('align') or {}
            return f'start_align {a.get("status", "?")} {a.get("reason", "")}'.strip()
    elif stage == 'perception':
        checks = [('/rescue/target_locations publisher', lambda: ros.publishers('/rescue/target_locations') > 0)]
    elif stage == 'operations':
        checks = [('/rescue/report_receipts publisher', lambda: ros.publishers('/rescue/report_receipts') > 0)]
    elif stage == 'coordinator':
        checks = [('/rescue/system_state', lambda: ros.get('system') is not None),
                  ('/mission_coordinator/refresh_state', lambda: ros.has_service('/mission_coordinator/refresh_state'))]
    elif stage == 'alert':
        checks = [('/rescue_found_alert', lambda: ros.node_up('/rescue_found_alert'))]
    else:   # web, rviz: the process staying up is all there is to see
        start = time.monotonic()
        checks = [('process up 3 s', lambda: time.monotonic() - start > 3)]
    return checks, progress


def arm_both(ros, *, scout, support, ns, timeout_s, say):
    """refresh_state then task_arm per robot (field step 3), retried until success or the timeout."""
    from std_srvs.srv import Trigger
    deadline = time.monotonic() + timeout_s
    pending, last = [support, scout], {}
    while pending:
        robot = pending[0]
        ok, res = ros.call(Trigger, '/mission_coordinator/refresh_state', Trigger.Request())
        refresh = res.message if ok else res
        ok, res = ros.call(Trigger, f'{ns[robot]}/rescue_command_adapter/task_arm', Trigger.Request())
        if ok and res.success:
            say(f'  {robot} arm 성공 ({res.message})')
            pending.pop(0)
            continue
        reason = res.message if ok else res
        if last.get(robot) != reason:
            last[robot] = reason
            say(f'  … {robot} arm 거절: {reason} (refresh_state: {refresh})')
        if time.monotonic() > deadline:
            raise ScenarioError('arm', f'{robot} task_arm refused until timeout: {reason}')
        time.sleep(1.0)


def robot_ready(ros, robot, max_age_ns):
    s = ros.status(robot)
    if s is None:
        return False, 'no RobotStatus'
    msg, _ = s
    age = time.time_ns() - msg.observed_at_ns
    if not 0 <= age <= max_age_ns:
        return False, f'RobotStatus {age / 1e9:.1f} s old'
    if msg.readiness != 'READY' or msg.motion_state != 'STOPPED' or msg.active_task_id:
        return False, f'{msg.readiness}/{msg.motion_state}/task={msg.active_task_id or "-"}/stop={msg.stop_reason}'
    return True, 'READY STOPPED idle'


# ---- main ------------------------------------------------------------------------------------------------------
def parse_args(argv):
    p = argparse.ArgumentParser(description='One-PC full scenario (live shared map): one command after the assumed '
                                            'webcam earthquake. Robots move: watch them on site.')
    p.add_argument('--profile', required=True, help='deployment profile v2.1 JSON (outside the repository)')
    p.add_argument('--field', default=str(DEFAULT_OVERLAY), help='live-map field overlay or field file')
    p.add_argument('--run-dir', help='new folder for inputs, logs and results (default ~/rokey7_runs/full-<time>)')
    p.add_argument('--tb5-start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                   help='measured TB-5 start, m m rad, relative to the TB-6 start pose (or TB5_START_X/Y/YAW)')
    p.add_argument('--perception-args', help='JSON object of amr_perception.launch.py arguments (field values)')
    p.add_argument('--coordinator-args', help='JSON object overriding coordinator.launch.py route/observe/ROI args')
    p.add_argument('--webcam-params', help='webcam_node YAML for operations.launch.py (optional; the start is '
                                           'synthetic either way)')
    p.add_argument('--api-port', type=int, default=18765, help='operations command API port (127.0.0.1)')
    p.add_argument('--no-rviz', dest='rviz', action='store_false', help='do not start RViz (load)')
    p.add_argument('--no-alert', dest='alert', action='store_false', help='found alert: log only, no sound')
    p.add_argument('--no-overhead', dest='overhead', action='store_false',
                   help='do not start overhead_passage (depth processing for the map tunnels; load)')
    p.add_argument('--web-accounts', help='rescue_console accounts DB: also start the operator web console')
    p.add_argument('--web-port', type=int, default=8080)
    p.add_argument('--align-timeout', type=float, default=STAGE_TIMEOUTS['tb5_localization'])
    p.add_argument('--mission-timeout', type=float, help='s from the start event (default exploration_timeout_s + 900)')
    p.add_argument('--keep-running', action='store_true', help='after the end, wait for Ctrl+C before stopping')
    p.add_argument('--rehearsal', action='store_true', help='robot-free rehearsal with sim_world.py (not a field run)')
    p.add_argument('--sim-world', help='rehearsal: sim_world.py path')
    p.add_argument('--world', help='rehearsal: world JSON for sim_world.py')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    started = time.strftime('%Y%m%d-%H%M%S')
    run_dir = Path(args.run_dir or Path.home() / 'rokey7_runs' / f'full-{started}').resolve()
    if run_dir.exists() and any(run_dir.iterdir()):
        print(f'[preflight] FAILED run folder {run_dir} is not empty: use a new one per run', flush=True)
        return 2
    (run_dir / 'inputs').mkdir(parents=True, exist_ok=True)
    log_lines = []

    def say(line):
        text = f'{time.strftime("%H:%M:%S")} {line}'
        print(text, flush=True)
        log_lines.append(text)
        with open(run_dir / 'runner.log', 'a') as out:
            out.write(text + '\n')

    children, ros, verdict = None, None, ('FAILED', 'not started')
    events = []
    try:
        say(f'== 한 PC 전체 시나리오 ({"리허설: 로봇 없음, 모의 센서" if args.rehearsal else "실기: 사람이 현장을 지켜볼 것"}) '
            f'run={run_dir}')
        # 1. preflight
        say('[preflight] 시작')
        from .profile import load_deployment
        from .field_launch import load_field
        deployment = load_deployment(args.profile)
        domains = profile_domains(deployment)
        failed = False
        for name, ok, detail in env_checks(os.environ, rehearsal=args.rehearsal, domains=domains):
            say(f'  {"PASSED" if ok else "FAILED"} {name} = {detail}')
            failed |= not ok
        if failed:
            raise ScenarioError('preflight', 'environment checks failed (above)')
        seed = tb5_seed(args.tb5_start, os.environ)
        raw, support_id = run_field(args.field, seed)
        field_path = run_dir / 'inputs' / 'field.json'
        field_path.write_text(json.dumps(raw, indent=2, ensure_ascii=False) + '\n')
        field = load_field(field_path)
        scout, support = roles_of(deployment)
        if support != support_id:
            raise ScenarioError('preflight', f'profile SUPPORT {support} is not the field LIVE_MAP_AMCL robot {support_id}')
        ns = {r.id: r.parameters.namespace for r in deployment.robots}
        say(f'  PASSED field {args.field} -> {field_path} (SLAM_LIVE, EXIT_WAIT_SCOUT_DOCK); TB-6={scout} {ns[scout]}, '
            f'TB-5={support} {ns[support]}; TB-5 start {raw["robots"][support]["initial_pose_xy_yaw"]} '
            f'(m, m, rad, TB-6 출발 자세 기준, {"이번 실행 입력" if seed else "현장 파일"})')
        perception_args = {}
        if not args.rehearsal:
            if not args.perception_args:
                raise ScenarioError('preflight', '--perception-args <json> (amr_perception.launch.py field values) '
                                                 'is required for a field run')
            perception_args = json.loads(Path(args.perception_args).read_text())
        coordinator_args = json.loads(Path(args.coordinator_args).read_text()) if args.coordinator_args else {}
        from shutil import which
        missing = [tool for tool in ('ros2',) if which(tool) is None]
        if missing:
            raise ScenarioError('preflight', f'not on PATH: {missing} (source ROS and the workspace)')
        from uuid import uuid4
        options = dict(rviz=args.rviz, alert=args.alert, overhead=args.overhead, rehearsal=args.rehearsal,
                       sim_world=args.sim_world, world=args.world, perception_args=perception_args,
                       rehearsal_perception=(Path(args.sim_world).with_name('perception.launch.py')
                                             if args.sim_world else None),
                       coordinator_args=coordinator_args, webcam_params=args.webcam_params,
                       web=dict(accounts=args.web_accounts, port=args.web_port) if args.web_accounts else None,
                       api_port=args.api_port, python=sys.executable,
                       envelope_schema_version=field.coordinator['envelope_schema_version'],
                       report_publisher_id=f'full-scenario-report-{uuid4()}')
        if args.rehearsal and not (args.sim_world and args.world):
            raise ScenarioError('preflight', '--rehearsal needs --sim-world and --world')
        plan = stage_plan(profile=Path(args.profile).resolve(), field=field_path, run_dir=run_dir, scout=scout,
                          support=support, ns=ns, options=options)
        (run_dir / 'inputs' / 'plan.json').write_text(json.dumps(plan, indent=1, default=str))
        say('[preflight] 준비 완료')

        # 2. stages
        children = Children(run_dir / 'logs', say)
        # Under a Discovery Server only a super client sees the whole graph (publisher counts, services): the runner
        # alone is one; every node it starts stays a plain client (field README 2단계, wiring_check.py).
        children.env = dict(os.environ, ROS_SUPER_CLIENT='False')
        if os.environ.get('ROS_DISCOVERY_SERVER'):
            os.environ['ROS_SUPER_CLIENT'] = 'True'
        ros = Ros(scout=scout, support=support, ns=ns, field=field)
        ros.say = say
        timeouts = dict(STAGE_TIMEOUTS, tb5_localization=args.align_timeout)
        for stage, commands in plan:
            say(f'[{stage}] 시작')
            for attempt in range(STAGE_RETRIES.get(stage, 0) + 1):
                procs = [children.start(label, argv) for label, argv in commands]
                checks, progress = readiness(stage, ros, scout=scout, support=support, ns=ns, options=options)
                try:
                    wait_until(stage, checks, timeouts[stage], say, progress=progress, children=children)
                    break
                except ScenarioError as exc:
                    if attempt == STAGE_RETRIES.get(stage, 0):
                        raise
                    say(f'  {exc.reason} → 이 단계만 다시 시작 ({attempt + 1}/{STAGE_RETRIES[stage]})')
                    children.stop(procs)
            say(f'[{stage}] 준비 완료: {[name for name, _ in checks]}')

        # 3. arm + the assumed earthquake
        say('[arm] 시작 (refresh_state → task_arm, 두 로봇)')
        arm_both(ros, scout=scout, support=support, ns=ns, timeout_s=timeouts['arm'], say=say)
        say('[arm] 준비 완료')
        say('[disaster] 시작: 웹캠 지진 감지를 가정한 합성 DISASTER_SIGN 1회 (실제 판정 아님)')
        max_age = round(field.coordinator['status_max_age_s'] * 1e9)
        wait_until('disaster', [(f'{r} READY', lambda r=r: robot_ready(ros, r, max_age)[0]) for r in (scout, support)]
                   + [('coordinator NORMAL', lambda: ros.get('system') is not None and ros.get('system').mode == 'NORMAL')],
                   timeouts['disaster'], say, children=children,
                   progress=lambda: '; '.join(f'{r}: {robot_ready(ros, r, max_age)[1]}' for r in (scout, support)))
        event_id, payload = ros.publish_start_event(field.coordinator)
        (run_dir / 'start_event.json').write_text(json.dumps(payload, indent=1) + '\n')
        wait_until('disaster', [('SystemState DISASTER', lambda: ros.get('system').mode in ('DISASTER', 'MISSION_FINISHED')
                                 and event_id in list(ros.get('system').cause_event_ids))],
                   timeouts['disaster'], say, children=children)
        say(f'[disaster] 준비 완료: 조정기 DISASTER, 원인 event {event_id}')

        # 4. watch
        limit = args.mission_timeout or field.coordinator['exploration_timeout_s'] + 900
        say(f'[mission] 진행 감시 (제한 {limit:.0f} s). 정지: Ctrl+C (안전 정지 후 종료)')
        start, said, last_system, after_exit_task = time.monotonic(), None, None, ''
        health = {'next': 0.0, 'misses': {}, 'down': ''}
        while True:
            if time.monotonic() >= health['next'] and not health['down']:   # Nav2 alive (two misses in a row)
                health['next'] = time.monotonic() + NAV2_HEALTH_PERIOD_S
                for r in (scout, support):
                    active = ros.lifecycle_active(f'{ns[r]}/bt_navigator')
                    health['misses'][r] = 0 if active else health['misses'].get(r, 0) + 1
                    if health['misses'][r] >= 2:
                        health['down'] = f'{r} {ns[r]}/bt_navigator not active'
            outcome = ros.get('outcome') or {}
            system = ros.get('system')
            if outcome.get('support_exit') == 'AT_EXIT':
                s = ros.status(support)
                if s is not None and s[0].active_task_id:
                    after_exit_task = s[0].active_task_id
            state, reason = judge(outcome, child_exits=children.exited(), timed_out=time.monotonic() - start > limit,
                                  support_task_after_exit=after_exit_task, nav2_down=health['down'])
            key = json.dumps(outcome, sort_keys=True)
            if key != said:
                said = key
                say(f'  outcome {key}')
                events.append(dict(at=time.time(), outcome=outcome))
            if system is not None and (system.phase, system.reason) != last_system:
                last_system = (system.phase, system.reason)
                say(f'  system mode={system.mode} phase={system.phase} reason={system.reason}')
            dock = ros.get('dock')
            if state != 'RUNNING':
                verdict = (state, reason)
                break
            time.sleep(0.5)
        say(f'[mission] 결과 {verdict[0]}: {verdict[1]} (dock={dock})')
        if args.keep_running and verdict[0] in ('SUCCESS', 'FINISHED_INCOMPLETE'):
            say('  --keep-running: Ctrl+C 로 정리한다')
            while True:
                time.sleep(1)
    except ScenarioError as exc:
        verdict = ('FAILED', str(exc))
        say(f'[{exc.stage}] FAILED {exc.reason}')
    except KeyboardInterrupt:
        verdict = (verdict[0] if verdict[0] in ('SUCCESS', 'FINISHED_INCOMPLETE') else 'STOPPED', 'Ctrl+C')
        say('[stop] Ctrl+C: 안전 정지 후 정리')
    except Exception as exc:   # unexpected: still stop the robots and the processes
        verdict = ('FAILED', f'{type(exc).__name__}: {exc}')
        say(f'[runner] FAILED {verdict[1]}')
    finally:
        if ros is not None:
            say('[safe_stop] 두 로봇 정지 명령 (기존 인터페이스)')
            try:
                ros.safe_stop(say)
            except Exception as exc:
                say(f'  [실패] safe stop: {exc}')
        if children is not None:
            say('[cleanup] 프로세스 그룹 정리 (INT → TERM → KILL)')
            left = children.stop_all()
            say(f'  남은 프로세스 그룹: {left or "없음"}')
        if ros is not None:
            ros.close()
        result = dict(verdict=verdict[0], reason=verdict[1], run_dir=str(run_dir), rehearsal=args.rehearsal,
                      outcome=(ros.get('outcome') if ros is not None else None), events=events)
        (run_dir / 'result.json').write_text(json.dumps(result, indent=1, default=str) + '\n')
        say(f'== 최종: {verdict[0]} — {verdict[1]} (result.json, runner.log, logs/)')
    return 0 if verdict[0] == 'SUCCESS' else 3 if verdict[0] == 'FINISHED_INCOMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
