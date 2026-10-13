"""Measured ROS inputs for the existing task runtime, not another executor.

A configured measured alignment and the map hash bind the coordinate basis: a declared
saved-map hash, or the one frozen TB-6 SLAM map (SavedMapSources). TF ticks never
manufacture SLAM correction events; the revision moves once, at the freeze.
"""
from copy import deepcopy
import math

from rescue_contracts import ros_time_to_ns
from rescue_contracts.occupancy import footprint_clear, occupancy_sha256, planar_origin
from rescue_robot.executor import ContractError


def stamp(message):
    return ros_time_to_ns(message.header.stamp.sec,message.header.stamp.nanosec)


def planar(pose):
    p,q=pose.position,pose.orientation
    try: return planar_origin(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
    except ValueError as exc: raise ContractError(str(exc)) from None


class Grid:
    """One received OccupancyGrid; hash and free-footprint rule are rescue_contracts.occupancy (shared)."""
    def __init__(self,message):
        info=message.info
        self.frame=message.header.frame_id
        self.width,self.height=info.width,info.height
        self.resolution=info.resolution; self.origin=planar(info.origin)
        self.data=tuple(message.data)
        try:
            # Resource identity deliberately excludes publication stamp/load time.
            self.sha256=occupancy_sha256(**self._fields())
        except ValueError:
            raise ContractError('invalid occupancy grid') from None
        self.resolution=float(self.resolution)

    def _fields(self):
        return dict(frame=self.frame,width=self.width,height=self.height,resolution=self.resolution,
                    origin=self.origin,data=self.data)

    def point(self,x,y):
        ox,oy,yaw=self.origin; c,s=math.cos(yaw),math.sin(yaw)
        return ox+c*x-s*y,oy+s*x+c*y

    def cell(self,x,y):
        ox,oy,yaw=self.origin; c,s=math.cos(yaw),math.sin(yaw)
        return math.floor((c*(x-ox)+s*(y-oy))/self.resolution),math.floor((-s*(x-ox)+c*(y-oy))/self.resolution)

    def value(self,x,y):
        ix,iy=self.cell(x,y)
        return self.data[iy*self.width+ix] if 0<=ix<self.width and 0<=iy<self.height else None

    def clear(self,x,y,radius):
        return footprint_clear(**self._fields(),x=x,y=y,radius=radius)


MAP_MODES=('SAVED_MAP_AMCL','SLAM_THEN_FROZEN','FROZEN_MAP_AMCL')


class SavedMapSources:
    """This robot's map and localization inputs, one of three declared modes (no fallback between them).

    SAVED_MAP_AMCL: a saved map with a measured hash (tests without robots). SLAM_THEN_FROZEN
    (TB-6): its live SLAM map and SLAM pose at revision R0 while searching, then the frozen map
    with AMCL at R0+1. FROZEN_MAP_AMCL (TB-5): nothing before the freeze, then the frozen map
    with AMCL at R0+1. The frozen map (/rescue/map/frozen) is loaded only when this robot's own
    map topic carries exactly that grid; a second, different frozen map blocks the map.
    """
    def __init__(self,runtime,settings):
        self.runtime=runtime; self.settings=deepcopy(settings); self.maps={}; self.poses={}; self.faults={}
        self.slam_maps={}; self.received={}; self.frozen=None
        self.publisher=None  # live_pose.LivePosePublisher for this robot (RobotPose / MapState)
        if not settings: raise ContractError('saved_maps requires measured robot settings')
        for robot,c in self.settings.items():
            if c['mode'] not in MAP_MODES: raise ContractError('SLAM backend correction adapter not configured')
            topics=['map_topic','localization_topic']
            if c['mode']!='SAVED_MAP_AMCL': topics.append('frozen_topic')
            if c['mode']=='SLAM_THEN_FROZEN': topics.append('slam_localization_topic')
            for name in topics:
                if not isinstance(c.get(name),str) or not c[name].startswith('/'):
                    raise ContractError('absolute saved-map input topic required')
            if c['mode']=='SAVED_MAP_AMCL':
                if len(c['occupancy_sha256'])!=64: raise ContractError('measured occupancy sha256 required')
            elif c['occupancy_sha256']!='':
                raise ContractError('the freeze decides the map hash; no declared occupancy sha256')
            for name in ('max_position_variance_m2','max_yaw_variance_rad2'):
                if not math.isfinite(c[name]) or c[name]<=0: raise ContractError('explicit localization variance policy')

    def phase(self,robot):
        mode=self.settings[robot]['mode']
        if mode=='SAVED_MAP_AMCL': return 'SAVED'
        if self.frozen is not None: return 'FROZEN'
        return 'SLAM' if mode=='SLAM_THEN_FROZEN' else 'UNFROZEN'

    def loaded(self,robot):
        return robot in (self.slam_maps if self.phase(robot)=='SLAM' else self.maps)

    def basis(self,robot):
        """The basis this robot publishes now; occupancy_key = the map hash, or what stands in for it."""
        c=self.settings[robot]; phase=self.phase(robot)
        key={'SAVED':c['occupancy_sha256'],'SLAM':'slam','UNFROZEN':'unfrozen'}.get(phase)
        return dict(map_frame=c['map_frame'],map_session_id=c['map_session_id'],
                    map_revision=c['map_revision']+(1 if phase=='FROZEN' else 0),
                    transform_revision=c['transform_revision'],
                    occupancy_key=self.frozen.sha256 if phase=='FROZEN' else key)

    def _load(self,robot):
        """Frozen phase: load this robot's last received map only if it is exactly the frozen grid."""
        grid=self.received.get(robot)
        if robot in self.faults or grid is None or grid.sha256!=self.frozen.sha256:
            if self.maps.pop(robot,None) is not None or grid is not None:
                self.runtime.node.get_logger().error(f'{robot} map is not the frozen map; not loaded')
            return
        self.maps[robot]=grid

    def receive_frozen(self,message):
        r=self.runtime
        with r.adapter.lock:
            frozen_robots=[robot for robot,c in self.settings.items() if c['mode']!='SAVED_MAP_AMCL']
            if not frozen_robots: return
            try:
                grid=Grid(message)
                if any(grid.frame!=self.settings[robot]['map_frame'] for robot in frozen_robots):
                    raise ContractError('frozen map is not in the map frame')
            except (ContractError,ValueError,TypeError) as exc:
                r.node.get_logger().error('frozen map refused: '+str(exc)); return
            if self.frozen is not None and self.frozen.sha256!=grid.sha256:
                for robot in frozen_robots:
                    self.faults[robot]='frozen map changed; new run required'; self.maps.pop(robot,None)
                r.node.get_logger().error('frozen map changed after the freeze; map blocked (new run required)')
            elif self.frozen is None:
                self.frozen=grid
                for robot in frozen_robots:
                    self.slam_maps.pop(robot,None); self.poses.pop(robot,None)
                r.node.get_logger().info(f'MAP_FROZEN sha256={grid.sha256} {grid.width}x{grid.height} '
                                         f'res={grid.resolution}')
            for robot in frozen_robots:
                self._load(robot); self._publish_map_state(robot)

    def receive_map(self,robot,message):
        r=self.runtime
        with r.adapter.lock:
            try:
                grid=Grid(message); c=self.settings[robot]; phase=self.phase(robot)
                if phase=='SAVED':
                    if (grid.frame,grid.sha256)!=(c['map_frame'],c['occupancy_sha256']):
                        raise ContractError('loaded map changed; new measured session required','FRAME_REVISION_MISMATCH')
                    if robot not in self.faults: self.maps[robot]=grid
                elif grid.frame!=c['map_frame']:
                    raise ContractError('map is not in the map frame')
                elif phase=='SLAM':
                    self.slam_maps[robot]=grid  # changes while searching; never a footprint map
                else:
                    self.received[robot]=grid
                    if phase=='FROZEN': self._load(robot)
            except (ContractError,ValueError,TypeError) as exc:
                self.maps.pop(robot,None); self.slam_maps.pop(robot,None)
                if self.settings[robot]['mode']=='SAVED_MAP_AMCL': self.faults[robot]=str(exc)
                r.node.get_logger().error('saved map blocked: '+str(exc))
            self._publish_map_state(robot)

    def receive_pose(self,robot,message,source='localization'):
        """``source`` 'slam' (SLAM pose topic) counts only while searching, 'localization' (AMCL) otherwise."""
        r=self.runtime
        with r.adapter.lock:
            if (source=='slam')!=(self.phase(robot)=='SLAM'): return
            try:
                c=self.settings[robot]; observed=stamp(message)
                if observed<=0 or observed>r.adapter.wall_clock_ns()+r.inputs.bound:
                    raise ContractError('AMCL stamp missing or future')
                if observed<=self.poses.get(robot,(0,False))[0]: return
                planar(message.pose.pose); covariance=message.pose.covariance
                valid=message.header.frame_id==c['map_frame'] and len(covariance)==36 and all(math.isfinite(v) for v in covariance)
                if valid:
                    vx,vy,cxy=covariance[0],covariance[7],covariance[1]
                    valid=(vx>=0 and vy>=0 and math.isclose(cxy,covariance[6],abs_tol=1e-9)
                        and cxy*cxy<=vx*vy and (vx+vy+math.hypot(vx-vy,2*cxy))/2<=c['max_position_variance_m2']
                        and 0<=covariance[35]<=c['max_yaw_variance_rad2'])
                self.poses[robot]=(observed,valid)
                if self.publisher is not None and robot==self.publisher.robot:
                    self.publisher.on_pose(message,observed,valid)
            except (ContractError,ValueError,TypeError) as exc:
                self.poses.pop(robot,None); r.node.get_logger().warning('AMCL input rejected: '+str(exc))
                self._publish_map_state(robot)

    def _publish_map_state(self,robot):
        if self.publisher is not None and robot==self.publisher.robot:
            self.publisher.publish_map_state(localization_valid=self.poses.get(robot,(0,False))[1])

    def records(self):
        result={}
        for robot,c in self.settings.items():
            observed,valid=self.poses.get(robot,(0,False))
            result[robot]={key:deepcopy(c[key]) for key in ('map_frame','map_session_id','map_revision',
                'transform_revision','transform_xy_yaw','authority_id','clock_domain_id','valid_from_ns',
                'transform_valid_until_ns','position_error_m','rotation_error_rad')}
            result[robot].update(map_revision=self.basis(robot)['map_revision'],
                observed_at_ns=observed,valid_until_ns=observed+self.runtime.inputs.max_age,
                map_update_sequence=0,localization_status='VALID' if valid and self.loaded(robot) else 'UNKNOWN')
        return result


def attach_sources(runtime):
    """Bind real ROS inputs; polling is asynchronous and never moves the robot."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PoseWithCovarianceStamped
    r=runtime; snapshot=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def subscribe(kind,topic,callback,qos,invalidate):
        def receive(message):
            with r.adapter.lock:
                try: callback(message)
                except (ContractError,ValueError,TypeError,KeyError) as exc:
                    invalidate(); r.node.get_logger().warning('live input rejected: '+str(exc))
        handle=r.node.create_subscription(kind,topic,receive,qos)
        r.input_handles.append(('destroy_subscription',handle))

    if r.saved_maps is not None:
        if any(c['mode']!='SAVED_MAP_AMCL' for c in r.saved_maps.settings.values()):
            topic=next(c['frozen_topic'] for c in r.saved_maps.settings.values() if c['mode']!='SAVED_MAP_AMCL')
            subscribe(OccupancyGrid,topic,r.saved_maps.receive_frozen,snapshot,lambda:None)
        for robot,c in r.saved_maps.settings.items():
            if c['mode']=='SLAM_THEN_FROZEN':
                subscribe(PoseWithCovarianceStamped,c['slam_localization_topic'],
                          lambda m,robot=robot:r.saved_maps.receive_pose(robot,m,source='slam'),qos_profile_sensor_data,
                          lambda robot=robot:r.saved_maps.poses.pop(robot,None))
            subscribe(OccupancyGrid,c['map_topic'],lambda m,robot=robot:r.saved_maps.receive_map(robot,m),snapshot,
                      lambda robot=robot:r.saved_maps.maps.pop(robot,None))
            subscribe(PoseWithCovarianceStamped,c['localization_topic'],lambda m,robot=robot:r.saved_maps.receive_pose(robot,m),qos_profile_sensor_data,
                      lambda robot=robot:r.saved_maps.poses.pop(robot,None))
