"""Print every /robot5/rescue_start_align result and the last AMCL pose; exit 0 on a final state."""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('e3_watch')
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
state = {'final': None, 'amcl': None}


def on_status(m):
    r = json.loads(m.data)
    print('STATUS', r['status'], r['reason'], json.dumps({k: r.get(k) for k in (
        'pose', 'first_guess', 'sigma_xy_m', 'sigma_yaw_rad', 'jump_xy_m', 'jump_yaw_rad', 'match_ratio',
        'match_points', 'updates')}), flush=True)
    if r['status'] in ('ALIGNED', 'FAILED'):
        state['final'] = r


def on_amcl(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    state['amcl'] = (p.x, p.y, 2 * math.atan2(q.z, q.w), math.sqrt(max(m.pose.covariance[0], 0.0)))


node.create_subscription(String, '/robot5/rescue_start_align', on_status, latched)
node.create_subscription(PoseWithCovarianceStamped, '/robot5/amcl_pose', on_amcl, latched)
end = time.time() + float(sys.argv[1])
while time.time() < end and state['final'] is None:
    rclpy.spin_once(node, timeout_sec=0.2)
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.1)
print('AMCL_LAST', state['amcl'], flush=True)
print('RESULT', 'TIMEOUT' if state['final'] is None else state['final']['status'] + ' ' + state['final']['reason'])
sys.exit(0 if state['final'] else 3)
