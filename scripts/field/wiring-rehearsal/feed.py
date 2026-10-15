"""Synthetic robot data for the one-PC rehearsal probes (freeze_probe.py, mission_probe.py). Not field data.

What slam_toolbox, AMCL and the robot Pis would publish, on the field names:
- TB-6: /robot6/map (a grid: free middle, unknown border so FRONTIER finds a frontier), /robot6/pose (small
  covariance), /robot6/tf map->odom->base_link. ``tf='measured'`` uses the TB-6 TF of the 2026-10-14 map
  freeze failure (#291 tests/test_robot_slam_tf_pose.py measured_tb6_tf: roll/pitch about 0.5 deg, z mm,
  odom->base_link 0.1 s older); ``tf='flat'`` is an exact plane at the origin.
- TB-5: /robot5/amcl_pose, /robot5/tf flat at the origin (TB-5 TF not measured), and the start alignment verdict
  /robot5/rescue_start_align ALIGNED (what rescue_robot start_align latches once TB-5 is aligned, #317; its own
  rehearsal is scripts/field/start-align-rehearsal).
- Both: zero-velocity odom, undocked dock_status, scan and the readiness camera topics, RGB image or camera_info
  (empty payloads).
"""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
COV = [0.0] * 36
COV[0] = COV[7] = COV[35] = 0.001
S = 1_000_000_000
# #291 measured_tb6_tf (2026-10-14, TB-6 stationary after undocking): (parent, child, stamp offset ns, xyz, xyzw)
MEASURED_TB6 = (
    ('map', 'odom', 0, (0.02000981357266791, 1.4404437134362097e-05, 0.0014779624437087533),
     (-0.004797844888163447, 0.0025875895498232732, -1.2195844091517151e-05, 0.9999851423476311)),
    ('odom', 'base_link', -S // 10, (-0.14913737773895264, -0.0034079807810485363, -0.002282583387568593),
     (-0.0023667090572416782, -0.00456047710031271, 0.9999651908874512, 0.006571151316165924)))
FLAT = (('map', 'odom', 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ('odom', 'base_link', 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))


class Feeder:
    def __init__(self, node, *, tf='measured'):
        if tf not in ('measured', 'flat'):
            raise ValueError('tf must be measured or flat')
        self.node, self.tb6_edges = node, MEASURED_TB6 if tf == 'measured' else FLAT
        self.slam = True
        p = node.create_publisher
        self.pubs = {'map6': p(OccupancyGrid, '/robot6/map', LATCHED),
                     'pose6': p(PoseWithCovarianceStamped, '/robot6/pose', 10),
                     'tf6': p(TFMessage, '/robot6/tf', 10), 'tf5': p(TFMessage, '/robot5/tf', 10),
                     'amcl5': p(PoseWithCovarianceStamped, '/robot5/amcl_pose', LATCHED),
                     'align5': p(String, '/robot5/rescue_start_align', LATCHED)}
        self.pubs['align5'].publish(String(data='{"reason": "", "status": "ALIGNED"}'))
        self.sensors = []
        for ns in ('/robot6', '/robot5'):
            from irobot_create_msgs.msg import DockStatus
            from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
            for kind, name in ((Odometry, '/odom'), (DockStatus, '/dock_status'), (LaserScan, '/scan'),
                               (CompressedImage, '/oakd/rgb/image_raw/compressed'),
                               (CameraInfo, '/oakd/rgb/camera_info'),   # readiness camera of the robot files (B7)
                               (CompressedImage, '/oakd/stereo/image_raw/compressedDepth')):
                self.sensors.append((kind, p(kind, ns + name, qos_profile_sensor_data)))
        self.grid = OccupancyGrid()
        g = self.grid
        g.header.frame_id = 'map'
        g.info.resolution, g.info.width, g.info.height = 0.05, 80, 80
        g.info.origin.position.x = g.info.origin.position.y = -2.0
        g.info.origin.orientation.w = 1.0
        # free 2 x 2 m in the middle, unknown around it: a frontier for the coordinator (search_mode FRONTIER)
        g.data = [0 if 20 <= x < 60 and 20 <= y < 60 else -1 for y in range(80) for x in range(80)]
        self.amcl6 = None
        node.create_timer(0.05, self.fast)
        node.create_timer(0.5, self.slow)

    def stamp(self, offset_ns=0):
        return rclpy.time.Time(nanoseconds=self.node.get_clock().now().nanoseconds + offset_ns).to_msg()

    def edges(self, edges):
        out = []
        for parent, child, offset, xyz, xyzw in edges:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = self.stamp(offset), parent, child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            r = t.transform.rotation
            r.x, r.y, r.z, r.w = xyzw
            out.append(t)
        return TFMessage(transforms=out)

    def pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp, msg.header.frame_id = self.stamp(), 'map'
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = COV
        return msg

    def fast(self):
        self.pubs['tf6'].publish(self.edges(self.tb6_edges))
        self.pubs['tf5'].publish(self.edges(FLAT))
        for kind, pub in self.sensors:
            msg = kind()
            if hasattr(msg, 'header'):
                msg.header.stamp = self.stamp()
            if hasattr(msg, 'is_docked'):
                msg.is_docked = False
            pub.publish(msg)

    def slow(self):
        if self.slam:
            self.grid.header.stamp = self.stamp()
            self.pubs['map6'].publish(self.grid)
            self.pubs['pose6'].publish(self.pose())
        elif self.amcl6 is not None:
            self.amcl6.publish(self.pose())
        self.pubs['amcl5'].publish(self.pose())

    def stop_slam(self):
        """TB-6 stops SLAM (its /robot6/map and /robot6/pose go away); AMCL publishes /robot6/amcl_pose."""
        self.slam = False
        self.node.destroy_publisher(self.pubs.pop('map6'))
        self.node.destroy_publisher(self.pubs.pop('pose6'))
        self.amcl6 = self.node.create_publisher(PoseWithCovarianceStamped, '/robot6/amcl_pose', LATCHED)
