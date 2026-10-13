"""TB-6 pose while searching = TF map -> base_link (map lifecycle, OPS-177); no ROS.

slam_toolbox publishes <ns>/pose only for the scans it processes, and it processes a scan only
after the robot moved far enough, so a stopped TB-6 gets no new SLAM pose [estimate, slam_toolbox
defaults]. In the SLAM phase the executor therefore reads TF on its poll period: map -> odom
(slam_toolbox, republished periodically) composed with odom -> base_link (Create3). This is the
pose source of the phase, not a fallback. Its time is the older of the two edges (a stopped
edge makes the pose old, never fresh). The covariance is the last <ns>/pose's (TF has none):
older than ``covariance_max_age_ns`` it is still shown but the pose is not valid. Without both
edges or before any SLAM pose there is no pose.
"""
import math
from types import SimpleNamespace as NS

from rescue_contracts import ros_time_to_ns
from rescue_contracts.occupancy import compose_planar, planar_origin
from rescue_robot.executor import ContractError


class SlamTfPose:
    def __init__(self, *, map_frame, odom_frame, base_frame, covariance_max_age_ns):
        for name, value in (('map_frame', map_frame), ('odom_frame', odom_frame), ('base_frame', base_frame)):
            if not isinstance(value, str) or not value:
                raise ContractError(f'SLAM TF pose needs {name}')
        if type(covariance_max_age_ns) is not int or covariance_max_age_ns <= 0:
            raise ContractError('SLAM TF pose needs a positive integer covariance_max_age_ns')
        self.edges = {(map_frame, odom_frame): None, (odom_frame, base_frame): None}   # -> (stamp_ns, x, y, yaw)
        self.covariance = None   # (stamp_ns, list) of the last SLAM pose
        self.covariance_max_age_ns = covariance_max_age_ns

    def on_tf(self, message):
        """Keep the newest of each edge; returns how many edges were refused (not planar)."""
        refused = 0
        for t in message.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if key not in self.edges:
                continue
            p, q = t.transform.translation, t.transform.rotation
            try:
                x, y, yaw = planar_origin(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            except ValueError:
                refused += 1
                continue
            stamp = ros_time_to_ns(t.header.stamp.sec, t.header.stamp.nanosec)
            if self.edges[key] is None or stamp > self.edges[key][0]:
                self.edges[key] = (stamp, x, y, yaw)
        return refused

    def on_covariance(self, stamp_ns, covariance):
        if len(covariance) != 36:
            raise ContractError('6x6 covariance required')
        if self.covariance is None or stamp_ns > self.covariance[0]:
            self.covariance = (stamp_ns, [float(v) for v in covariance])

    def pose(self, *, now_ns):
        (to_odom, to_base) = self.edges.values()
        if to_odom is None or to_base is None or self.covariance is None:
            return None
        x, y, yaw = compose_planar(to_odom[1:], to_base[1:])
        stamp, covariance = self.covariance
        return dict(stamp_ns=min(to_odom[0], to_base[0]), x=x, y=y, yaw=yaw, covariance=list(covariance),
                    covariance_fresh=now_ns - stamp <= self.covariance_max_age_ns)


def pose_message(pose, frame, factory=None):
    """A PoseWithCovarianceStamped (``factory``) or a look-alike for SavedMapSources.receive_pose."""
    sec, nanosec = divmod(pose['stamp_ns'], 1_000_000_000)
    qz, qw = math.sin(pose['yaw'] / 2), math.cos(pose['yaw'] / 2)
    if factory is None:
        return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec)),
                  pose=NS(pose=NS(position=NS(x=pose['x'], y=pose['y'], z=0.0), orientation=NS(x=0.0, y=0.0, z=qz, w=qw)),
                          covariance=list(pose['covariance'])))
    message = factory()
    message.header.frame_id = frame
    message.header.stamp.sec, message.header.stamp.nanosec = sec, nanosec
    position, orientation = message.pose.pose.position, message.pose.pose.orientation
    position.x, position.y, position.z = float(pose['x']), float(pose['y']), 0.0
    orientation.x, orientation.y, orientation.z, orientation.w = 0.0, 0.0, qz, qw
    message.pose.covariance = [float(v) for v in pose['covariance']]
    return message
