"""Measured ROS inputs for the existing task runtime, not another executor.

Saved-map AMCL only: a configured measured alignment and exact occupancy hash
bind the coordinate basis. TF ticks never manufacture SLAM correction events.
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


class SavedMapSources:
    def __init__(self,runtime,settings):
        self.runtime=runtime; self.settings=deepcopy(settings); self.maps={}; self.poses={}; self.faults={}
        self.publisher=None  # live_pose.LivePosePublisher for this robot (RobotPose / MapState)
        if not settings: raise ContractError('saved_maps requires measured robot settings')
        for robot,c in self.settings.items():
            if c['mode']!='SAVED_MAP_AMCL': raise ContractError('SLAM backend correction adapter not configured')
            for name in ('map_topic','localization_topic'):
                if not isinstance(c[name],str) or not c[name].startswith('/'):
                    raise ContractError('absolute saved-map input topic required')
            if len(c['occupancy_sha256'])!=64: raise ContractError('measured occupancy sha256 required')
            for name in ('max_position_variance_m2','max_yaw_variance_rad2'):
                if not math.isfinite(c[name]) or c[name]<=0: raise ContractError('explicit localization variance policy')

    def receive_map(self,robot,message):
        r=self.runtime
        with r.adapter.lock:
            try:
                grid=Grid(message); c=self.settings[robot]
                if (grid.frame,grid.sha256)!=(c['map_frame'],c['occupancy_sha256']):
                    raise ContractError('loaded map changed; new measured session required','FRAME_REVISION_MISMATCH')
                if robot not in self.faults: self.maps[robot]=grid
            except (ContractError,ValueError,TypeError) as exc:
                self.maps.pop(robot,None); self.faults[robot]=str(exc)
                r.node.get_logger().error('saved map blocked: '+str(exc))
            self._publish_map_state(robot)

    def receive_pose(self,robot,message):
        r=self.runtime
        with r.adapter.lock:
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
            result[robot].update(observed_at_ns=observed,valid_until_ns=observed+self.runtime.inputs.max_age,
                map_update_sequence=0,localization_status='VALID' if valid and robot in self.maps else 'UNKNOWN')
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
        for robot,c in r.saved_maps.settings.items():
            subscribe(OccupancyGrid,c['map_topic'],lambda m,robot=robot:r.saved_maps.receive_map(robot,m),snapshot,
                      lambda robot=robot:r.saved_maps.maps.pop(robot,None))
            subscribe(PoseWithCovarianceStamped,c['localization_topic'],lambda m,robot=robot:r.saved_maps.receive_pose(robot,m),qos_profile_sensor_data,
                      lambda robot=robot:r.saved_maps.poses.pop(robot,None))
