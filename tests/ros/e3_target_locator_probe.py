"""E3 (partial): fake observation + TF -> target_locator -> mission_coordinator target (B2, 2026-10-13).

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy and a built rescue_interfaces).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    colcon build --packages-select rescue_interfaces   # in a scratch workspace
    source <scratch>/install/setup.bash
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=71 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_mapping:<REPO>/src/rescue_perception:<REPO>/src/rescue_coordination:<REPO>/src/rescue_bringup:<REPO>/tests/ros:$PYTHONPATH \
      python3 tests/ros/e3_target_locator_probe.py <REPO>   # not -I: ROS needs PYTHONPATH

Both nodes run as child processes: the coordinator with the e3_coordinator_probe parameters
(mission_frame_id map, the #132 map basis), target_locator with the same basis and its TF
remapped to a namespaced robot TF. This probe publishes the robot TF and TargetObservations.
Checks: 1. target_locator refuses to start without parameters (exit 2); 2. an observation
with capture-time TF -> one VALID TargetLocation at the expected map point with the
coordinator's basis; 3. the coordinator makes a target of it (RescueReport naming that
location, TargetSnapshot with one target); 4. an observation with no TF at its capture
time -> nothing published; 5. an observation without a VALID camera point -> nothing published.
Exit code 0 = all held.
"""
import math
import os
import subprocess
import sys
import time
from uuid import uuid4

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rescue_interfaces.msg import RescueReport, TargetLocation, TargetObservation, TargetSnapshot
from tf2_msgs.msg import TFMessage

from e3_coordinator_probe import MAP, NODE_ARGS, node_command

TF_TOPIC, TF_STATIC_TOPIC = '/e3robot/tf', '/e3robot/tf_static'
CAMERA = 'oakd_rgb_camera_optical_frame'
LOCATOR_ARGS = ['-p', 'robot_id:=robot_a', '-p', 'observations_topic:=/rescue/observations',
                '-p', 'target_locations_topic:=/rescue/target_locations', '-p', 'map_frame:=map',
                '-p', 'mission_frame:=map', '-p', f'map_session_id:={MAP}', '-p', 'map_revision:=0',
                '-p', 'transform_revision:=1', '-p', 'alignment_xy_yaw:=[0.0, 0.0, 0.0]',
                '-p', 'alignment_position_error_m:=0.0', '-p', 'alignment_rotation_error_rad:=0.0',
                '-p', 'tf_wait_s:=1.0', '-p', 'publisher_id:=amr.target_locator',
                '-p', "envelope_schema_version:='0.1'",
                '-r', f'/tf:={TF_TOPIC}', '-r', f'/tf_static:={TF_STATIC_TOPIC}']


def locator_command(extra):
    code = 'from rescue_perception.target_locator import main; raise SystemExit(main())'
    return [sys.executable, '-c', code, '--ros-args'] + extra


def coordinator_args(profile):
    args = list(NODE_ARGS)
    args[args.index('mission_frame_id:=mission')] = 'mission_frame_id:=map'  # field value (#132)
    return ['-p', f'profile_path:={profile}'] + args


def transform(parent, child, stamp_ns, xyz, yaw):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec, t.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


class Probe(Node):
    def __init__(self):
        super().__init__('e3_target_locator_probe')
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf = self.create_publisher(TFMessage, TF_TOPIC, 100)
        self.tf_static = self.create_publisher(TFMessage, TF_STATIC_TOPIC, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.observations = self.create_publisher(TargetObservation, '/rescue/observations', latched)
        self.locations, self.reports, self.snapshots = [], [], []
        self.create_subscription(TargetLocation, '/rescue/target_locations', self.locations.append, latched)
        self.create_subscription(RescueReport, '/rescue/reports', self.reports.append, 10)
        self.create_subscription(TargetSnapshot, '/rescue/targets', self.snapshots.append, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def publish_static(self):
        # odom -> camera fixed mount 0.3 m up; map -> odom comes as dynamic TF around each capture.
        self.tf_static.publish(TFMessage(transforms=[transform('odom', CAMERA, 0, (0.0, 0.0, 0.3), 0.0)]))

    def publish_map_odom(self, stamp_ns):
        self.tf.publish(TFMessage(transforms=[transform('map', 'odom', s, (1.0, 2.0, 0.0), math.pi / 2)
                                              for s in (stamp_ns - 50_000_000, stamp_ns + 50_000_000)]))

    def publish_observation(self, stamp_ns, status='VALID'):
        msg = TargetObservation(observation_id=str(uuid4()), source_robot='robot_a', source_camera='amr_rgbd',
                                source_boot_id=str(uuid4()), class_label='survivor', confidence=0.9,
                                model_id='e3', model_sha256='a' * 64, tracking_mode='DETECTION_ONLY',
                                sensor_frame=CAMERA, localization_status=status, depth_status='LOCATED')
        msg.time.observed_at_ns, msg.time.valid_until_ns = stamp_ns, stamp_ns + 2_000_000_000
        msg.time.stamp_semantics, msg.time.clock_state = 'UNKNOWN', 'UNKNOWN'
        msg.sensor_position.x, msg.sensor_position.y, msg.sensor_position.z = (
            (1.0, 0.0, 0.0) if status == 'VALID' else (math.nan,) * 3)
        msg.position_covariance = [0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04]
        self.observations.publish(msg)
        return msg.observation_id


def wait(executor, condition, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)
        if condition():
            return True
    return bool(condition())


def main():
    repo = sys.argv[1]
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    checks = {}
    refused = subprocess.run(locator_command([]), capture_output=True, text=True, timeout=30)
    checks['locator: missing parameters -> exit 2'] = (refused.returncode == 2
                                                       and 'missing parameters' in refused.stderr)
    profile = os.path.join(repo, 'config', 'deployment.v2.example.json')
    coordinator = subprocess.Popen(node_command(repo, coordinator_args(profile)),
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    locator = subprocess.Popen(locator_command(LOCATOR_ARGS), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True)
    rclpy.init()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    try:
        wait(executor, lambda: probe.count_subscribers('/rescue/observations') >= 1
             and probe.count_subscribers('/rescue/target_locations') >= 2, 20)
        probe.publish_static()
        stamp = probe.get_clock().now().nanoseconds
        probe.publish_map_odom(stamp)
        wait(executor, lambda: False, 0.5)
        observation_id = probe.publish_observation(stamp)
        got = wait(executor, lambda: probe.locations, 5)
        loc = probe.locations[0] if got else None
        f = loc.frame_ref if loc else None
        checks['observation + TF -> VALID TargetLocation'] = bool(loc) and (
            loc.projection_status, loc.observation.observation_id, f.robot_id, f.native_frame, f.mission_frame,
            f.map_session_id, f.map_revision, f.transform_revision, f.alignment_status) == (
            'VALID', observation_id, 'robot_a', 'map', 'map', MAP, 0, 1, 'VALID')
        checks['map point = TF applied at capture time'] = bool(loc) and all(
            abs(a - b) < 1e-6 for a, b in zip((loc.mission_position.x, loc.mission_position.y,
                                               loc.mission_position.z), (1.0, 3.0, 0.3)))
        got_report = wait(executor, lambda: probe.reports, 10)
        checks['coordinator: RescueReport names the location'] = got_report and bool(loc) and list(
            probe.reports[0].record_ids) == [loc.location_id]
        checks['coordinator: TargetSnapshot has one target'] = wait(
            executor, lambda: probe.snapshots and len(probe.snapshots[-1].targets) == 1, 10)
        before = len(probe.locations)
        probe.publish_observation(probe.get_clock().now().nanoseconds + 30_000_000_000)  # no TF then
        probe.publish_observation(stamp, status='UNKNOWN')
        wait(executor, lambda: False, 2.5)
        checks['no TF at capture / no camera point -> nothing published'] = len(probe.locations) == before
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.try_shutdown()
        outputs = {}
        for name, child in (('coordinator', coordinator), ('locator', locator)):
            child.terminate()
            try:
                outputs[name] = child.communicate(timeout=10)[0]
            except subprocess.TimeoutExpired:
                child.kill()
                outputs[name] = child.communicate()[0]
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    if not all(checks.values()):
        for name, text in outputs.items():
            print(f'--- {name} output ---\n{text}')
    else:
        print('--- locator log (tail) ---\n' + '\n'.join(outputs['locator'].splitlines()[-6:]))
    return 0 if checks and all(checks.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
