"""Tunnels the vision judgement found blocked -> a Nav2 keepout mask for both robots (재범 decision 2026-10-16
01:5x, option A, grade 4).

PassageStop (a depth stop in collision_monitor) is gone: it stopped the robots in front of walls and docks
(C1 #363, run 4). Each robot's overhead_passage publishes its judged structures
(/rescue/robots/<id>/overhead_passages, contracts C-04b); this node reads both lists and paints a disc of
``radius_m`` around each place to keep out into one OccupancyGrid (map frame, 100 = keep out, 0 = free).
The global and local costmaps of TB-6 and TB-5 read it through a Nav2 KeepoutFilter (nav2_field), so paths
do not cross it, and a lidar beam passing under a cover cannot clear it.

- Place: items within ``merge_m`` of each other, over both robots and every map revision (frozen maps keep
  the SLAM frame). The newest item (last_seen_ns) of a place decides.
- Painted: BLOCKED; UNKNOWN only with reason NO_FLOOR_NEAR (the structure reached the box's near end and the
  floor under it was never seen). Other UNKNOWN and PASSABLE are not painted: the robot keeps approaching
  under PassageSlow and measures again.
- Approval: a person approves an item (``KeepoutCore.approve``); its place leaves the mask for the run,
  whatever is measured there later. Who and when is logged. The ROS service for it
  (/rescue/passage_keepout/approve) needs a new interface in the SDD JSON first and comes in the next PR.
Nothing else is painted or removed.
"""
import math

from rescue_contracts.overhead import parse_overhead_passages

MASK_TOPIC = '/rescue/passage_keepout/mask'
KEEP_OUT, FREE = 100, 0


class KeepoutCore:
    def __init__(self, *, radius_m, resolution_m, merge_m, map_frame):
        for name, value in (('radius_m', radius_m), ('resolution_m', resolution_m), ('merge_m', merge_m)):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'passage keepout {name} must be a positive number')
        if not isinstance(map_frame, str) or not map_frame:
            raise ValueError('passage keepout needs the map frame')
        self.radius_m, self.resolution_m, self.merge_m, self.map_frame = radius_m, resolution_m, merge_m, map_frame
        self.lists = {}       # robot_id -> items
        self.approved = []    # (x, y, item_id, by, now_ns)

    def on_items(self, robot_id, items):
        self.lists[robot_id] = [dict(i, robot_id=robot_id) for i in items]

    def on_text(self, text):
        """One robot's published list (C-04b); ValueError for anything the contract refuses."""
        parsed = parse_overhead_passages(text)
        if parsed['map_frame'] != self.map_frame:
            raise ValueError(f'overhead passages in {parsed["map_frame"]!r}, not {self.map_frame!r}')
        self.on_items(parsed['robot_id'], parsed['items'])
        return parsed['robot_id']

    def _all(self):
        return [i for items in self.lists.values() for i in items]

    def places(self):
        """[(newest item, items of the place)], newest first: greedy by last_seen_ns within merge_m."""
        left = sorted(self._all(), key=lambda i: (-i['last_seen_ns'], i['id']))
        out = []
        while left:
            head = left.pop(0)
            group = [head] + [i for i in left if math.dist((i['x'], i['y']), (head['x'], head['y'])) <= self.merge_m]
            left = [i for i in left if i not in group]
            out.append((head, group))
        return out

    def _approved(self, x, y):
        return any(math.dist((x, y), (ax, ay)) <= self.merge_m for ax, ay, *_ in self.approved)

    def painted(self):
        """The newest items whose place is kept out."""
        out = []
        for head, _ in self.places():
            paint = head['state'] == 'BLOCKED' or (head['state'] == 'UNKNOWN' and head['reason'] == 'NO_FLOOR_NEAR')
            if paint and not self._approved(head['x'], head['y']):
                out.append(head)
        return out

    def approve(self, item_id, *, by, now_ns):
        """(accepted, log line). The item must be in a list now; ``by`` names the person."""
        if not isinstance(by, str) or not by.strip():
            return False, f'PASSAGE_APPROVE_REFUSED {item_id}: who approves is required'
        found = [i for i in self._all() if i['id'] == item_id]
        if not found:
            return False, f'PASSAGE_APPROVE_REFUSED {item_id}: no such overhead passage item'
        i = found[0]
        self.approved.append((i['x'], i['y'], item_id, by.strip(), now_ns))
        return True, (f'PASSAGE_APPROVED {item_id} by {by.strip()} at {now_ns} ns: place ({i["x"]:.2f}, {i["y"]:.2f}) '
                      f'{i["state"]} lowest={i["lowest_m"]} from {i["robot_id"]} leaves the keepout for this run')

    def mask(self):
        """OccupancyGrid fields (dict): the discs of painted places, or a 1-cell free grid when there is none."""
        res, r = self.resolution_m, self.radius_m
        heads = self.painted()
        if not heads:
            return dict(frame_id=self.map_frame, resolution=res, origin=(0.0, 0.0), width=1, height=1, data=[FREE])
        x0 = math.floor((min(h['x'] for h in heads) - r) / res) * res
        y0 = math.floor((min(h['y'] for h in heads) - r) / res) * res
        width = int(math.ceil((max(h['x'] for h in heads) + r - x0) / res)) + 1
        height = int(math.ceil((max(h['y'] for h in heads) + r - y0) / res)) + 1
        data = [FREE] * (width * height)
        for j in range(height):
            cy = y0 + (j + 0.5) * res
            for i in range(width):
                cx = x0 + (i + 0.5) * res
                if any(math.dist((cx, cy), (h['x'], h['y'])) <= r for h in heads):
                    data[j * width + i] = KEEP_OUT
        return dict(frame_id=self.map_frame, resolution=res, origin=(x0, y0), width=width, height=height, data=data)


def main(args=None):
    import rclpy
    from nav2_msgs.msg import CostmapFilterInfo
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from rescue_contracts.overhead import overhead_passages_topic
    from rescue_robot.ros_node import git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_passage_keepout')
    log = node.get_logger()
    defaults = {'robot_ids': [''], 'map_frame': '', 'info_topic': '', 'radius_m': 0.0, 'resolution_m': 0.0,
                'merge_m': 0.0}
    values = {name: node.declare_parameter(name, default).value for name, default in defaults.items()}
    try:
        robots = [r for r in values['robot_ids'] if r]
        if not robots or not values['info_topic']:
            raise ValueError('passage keepout needs robot_ids and info_topic')
        core = KeepoutCore(radius_m=values['radius_m'], resolution_m=values['resolution_m'], merge_m=values['merge_m'],
                           map_frame=values['map_frame'])
    except ValueError as exc:
        log.fatal(str(exc))
        raise SystemExit(2)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    mask_pub = node.create_publisher(OccupancyGrid, MASK_TOPIC, latched)
    info_pub = node.create_publisher(CostmapFilterInfo, values['info_topic'], latched)
    state = {'painted': None}

    def publish(force=False):
        heads = sorted(h['id'] for h in core.painted())
        if not force and heads == state['painted']:
            return
        state['painted'] = heads
        m = core.mask()
        msg = OccupancyGrid()
        msg.header.frame_id = m['frame_id']
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.info.resolution = float(m['resolution'])
        msg.info.width, msg.info.height = m['width'], m['height']
        msg.info.origin.position.x, msg.info.origin.position.y = (float(v) for v in m['origin'])
        msg.info.origin.orientation.w = 1.0
        msg.data = list(m['data'])
        mask_pub.publish(msg)
        log.info(f'PASSAGE_KEEPOUT {len(heads)} place(s) kept out: {heads}')

    def on_list(message):
        try:
            core.on_text(message.data)
        except ValueError as exc:   # a list the contract refuses changes nothing
            log.error(f'overhead passages refused: {exc}')
            return
        publish()

    log.info(f'git_sha={git_sha()} parameters={values} mask={MASK_TOPIC}')
    info = CostmapFilterInfo()
    info.header.frame_id = values['map_frame']
    info.type, info.filter_mask_topic, info.base, info.multiplier = 0, MASK_TOPIC, 0.0, 1.0   # 0 = keepout
    info_pub.publish(info)
    publish(force=True)
    for robot in robots:
        node.create_subscription(String, overhead_passages_topic(robot), on_list, latched)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
