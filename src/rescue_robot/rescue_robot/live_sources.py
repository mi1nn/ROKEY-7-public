"""Measured ROS inputs for the existing task runtime, not another executor.

A configured measured alignment and the map hash bind the coordinate basis: a declared
saved-map hash, or the one frozen TB-6 SLAM map (SavedMapSources). TF ticks never
manufacture SLAM correction events; the revision moves once, at the freeze.
"""
from copy import deepcopy
import json
import math

from rescue_contracts import ros_time_to_ns
from rescue_contracts.occupancy import (footprint_clear, occupancy_sha256, paired_revision,
                                        parse_frozen_revision, planar_origin)
from rescue_robot.executor import ContractError


def stamp(message):
    return ros_time_to_ns(message.header.stamp.sec,message.header.stamp.nanosec)


VARIANCE_ROUNDING=1e-9   # m^2 / rad^2: a variance this far below zero is rounding, read as 0


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
FREEZES=2  # per run (OPS-154): R0 + 1 doll found (TB-5 goes), R0 + 2 search done (TB-6 SLAM -> AMCL)


class SavedMapSources:
    """This robot's map and localization inputs, one of three declared modes (no fallback between them).

    SAVED_MAP_AMCL: a saved map with a measured hash (tests without robots). The TB-6 map is
    frozen twice per run; a frozen grid (/rescue/map/frozen) has a revision only through its
    pair on /rescue/map/frozen_revision (same hash, rescue_contracts.occupancy). SLAM_THEN_FROZEN
    (TB-6): its live SLAM map and SLAM pose at R0 until the last freeze (R0 + 2), then that map
    with AMCL. FROZEN_MAP_AMCL (TB-5): nothing before the first freeze, then the highest frozen
    revision with AMCL. A higher revision is taken only with no task and odometry STOPPED
    (else MAP_REVISION_DEFERRED, retried on tick); TB-5 re-aligning then restarts AMCL from its
    own last valid pose (``reinitialize``) once its map topic carries the new grid. A frozen
    map is loaded only when this robot's own map topic carries exactly that grid; another grid
    for a revision already known blocks the map (new run).
    """
    def __init__(self,runtime,settings):
        self.runtime=runtime; self.settings=deepcopy(settings); self.maps={}; self.poses={}; self.faults={}
        self.slam_maps={}; self.received={}
        self.frozen_grids={}; self.frozen_info=None  # received grids by hash, latest pair: matched in _pair
        self.known={}         # revision -> Grid, frozen maps of this run
        self.adopted={}       # robot -> frozen revision it uses
        self.valid_after={}   # robot -> wall ns; localization poses stamped before do not count
        self.last_valid={}    # robot -> last valid localization pose message (re-alignment start)
        self.reinit={}        # robot -> pose message to put on its AMCL once its map topic carries the grid
        self.deferred={}      # robot -> reason last logged
        self.slam_tf={}  # robot -> SlamTfPose (SLAM_THEN_FROZEN while searching)
        self.amcl_tf={}  # robot -> AmclTfPose (frozen phase: a standing robot has no new amcl_pose, run 3 2026-10-16)
        self.aligned={}  # FROZEN_MAP_AMCL robot -> last start_align verdict is ALIGNED (#317, decision (나))
        self.publisher=None  # live_pose.LivePosePublisher for this robot (RobotPose / MapState)
        self.reinitialize=None  # (robot, PoseWithCovarianceStamped-like) -> publishes <ns>/initialpose
        if not settings: raise ContractError('saved_maps requires measured robot settings')
        for robot,c in self.settings.items():
            if c['mode'] not in MAP_MODES: raise ContractError('SLAM backend correction adapter not configured')
            topics=['map_topic','localization_topic']
            if c['mode']!='SAVED_MAP_AMCL': topics+=['frozen_topic','frozen_revision_topic']
            if c['mode']=='SLAM_THEN_FROZEN': topics+=['slam_localization_topic','slam_tf_topic']
            if c['mode']=='FROZEN_MAP_AMCL': topics+=['initial_pose_topic','start_align_topic']
            for name in topics:
                if not isinstance(c.get(name),str) or not c[name].startswith('/'):
                    raise ContractError('absolute saved-map input topic required')
            if c['mode']=='SAVED_MAP_AMCL':
                if len(c['occupancy_sha256'])!=64: raise ContractError('measured occupancy sha256 required')
            elif c['occupancy_sha256']!='':
                raise ContractError('the freeze decides the map hash; no declared occupancy sha256')
            for name in ('max_position_variance_m2','max_yaw_variance_rad2'):
                if not math.isfinite(c[name]) or c[name]<=0: raise ContractError('explicit localization variance policy')
            if c['mode']=='SLAM_THEN_FROZEN':  # searching: pose = TF map -> base_link (OPS-177)
                from rescue_robot.slam_tf_pose import SlamTfPose
                self.slam_tf[robot]=SlamTfPose(map_frame=c['map_frame'],odom_frame=c.get('slam_odom_frame'),
                    base_frame=c.get('slam_base_frame'),covariance_max_age_ns=c.get('slam_covariance_max_age_ns'))
            if c['mode']!='SAVED_MAP_AMCL' and 'amcl_tf_topic' in c:  # frozen phase: TF map -> base_link
                from rescue_robot.slam_tf_pose import AmclTfPose
                self.amcl_tf[robot]=AmclTfPose(map_frame=c['map_frame'],odom_frame=c.get('amcl_odom_frame'),
                    base_frame=c.get('amcl_base_frame'),update_min_d=c.get('amcl_update_min_d'),
                    update_min_a=c.get('amcl_update_min_a'))

    def _frozen_robots(self):
        return [robot for robot,c in self.settings.items() if c['mode']!='SAVED_MAP_AMCL']

    def phase(self,robot):
        mode=self.settings[robot]['mode']
        if mode=='SAVED_MAP_AMCL': return 'SAVED'
        if robot in self.adopted: return 'FROZEN'
        return 'SLAM' if mode=='SLAM_THEN_FROZEN' else 'UNFROZEN'

    def loaded(self,robot):
        return robot in (self.slam_maps if self.phase(robot)=='SLAM' else self.maps)

    def basis(self,robot):
        """The basis this robot publishes now; occupancy_key = the map hash, or what stands in for it."""
        c=self.settings[robot]; phase=self.phase(robot)
        key={'SAVED':c['occupancy_sha256'],'SLAM':'slam','UNFROZEN':'unfrozen'}.get(phase)
        frozen=phase=='FROZEN'
        return dict(map_frame=c['map_frame'],map_session_id=c['map_session_id'],
                    map_revision=self.adopted[robot] if frozen else c['map_revision'],
                    transform_revision=c['transform_revision'],
                    occupancy_key=self.known[self.adopted[robot]].sha256 if frozen else key)

    def _target(self,robot):
        """The frozen revision this robot should be on: TB-6 only the last, TB-5 the highest so far."""
        c=self.settings[robot]
        if c['mode']=='SLAM_THEN_FROZEN':
            last=c['map_revision']+FREEZES
            return last if last in self.known else None
        return max(self.known) if self.known else None

    def pending(self,robot):
        """A higher frozen revision this robot has not taken yet (it is not stopped, or has a task)."""
        if self.settings[robot]['mode']=='SAVED_MAP_AMCL' or robot in self.faults: return False
        target=self._target(robot)
        return target is not None and target!=self.adopted.get(robot)

    def check_new_goal(self,robot):
        """D2 (OPS-159): no new goal on the old revision while a newer one waits; the task in progress goes on."""
        if self.runtime.adapter.executor.active is None and self.pending(robot):
            raise ContractError(f'map revision pending (R{self._target(robot)}); new goals wait until it is taken',
                                'FRAME_REVISION_MISMATCH')

    def tick(self):
        """Take a pending revision once the robot is stopped with no task (the timer of attach_sources)."""
        with self.runtime.adapter.lock:
            for robot in self._frozen_robots(): self._try_adopt(robot)

    def _try_adopt(self,robot):
        if not self.pending(robot): return
        r=self.runtime; executor=r.adapter.executor; target=self._target(robot)
        reason=('task active' if executor.active is not None else
                None if executor.motion=='STOPPED' else f'not stopped (motion={executor.motion})')
        if reason is not None:
            if self.deferred.get(robot)!=reason:
                self.deferred[robot]=reason
                r.node.get_logger().warning(f'MAP_REVISION_DEFERRED {robot} R{self.adopted.get(robot)} -> R{target}: '
                                            f'{reason}; taken once stopped with no task')
            return
        previous=self.adopted.get(robot)
        self.deferred.pop(robot,None)
        self.adopted[robot]=target
        self.valid_after[robot]=r.adapter.wall_clock_ns()
        self.maps.pop(robot,None); self.slam_maps.pop(robot,None); self.poses.pop(robot,None)
        if previous is not None and robot in self.last_valid:   # TB-5 re-aligning: start from where it stands
            self.reinit[robot]=self.last_valid[robot]
        r.node.get_logger().info(f'MAP_REVISION_ADOPTED {robot} R{previous} -> R{target} '
                                 f'sha256={self.known[target].sha256}')
        grid=self.received.get(robot)
        if grid is not None and grid.sha256==self.known[target].sha256: self._load(robot)  # else: its relay copies it
        self._publish_map_state(robot)   # the relay copies the grid of the revision this MapState names

    def _load(self,robot):
        """Frozen phase: load this robot's last received map only if it is exactly its frozen grid."""
        grid=self.received.get(robot); expected=self.known[self.adopted[robot]]
        if robot in self.faults or grid is None or grid.sha256!=expected.sha256:
            if self.maps.pop(robot,None) is not None or grid is not None:
                self.runtime.node.get_logger().error(f'{robot} map is not the frozen map; not loaded')
            return
        self.maps[robot]=grid
        start=self.reinit.pop(robot,None)
        if start is not None:
            # AMCL poses until now were on the previous grid, even those stamped after the adoption (REV-242-01)
            self.valid_after[robot]=self.runtime.adapter.wall_clock_ns(); self.poses.pop(robot,None)
            if self.reinitialize is not None: self.reinitialize(robot,start)
            x,y,yaw=planar(start.pose.pose)
            self.runtime.node.get_logger().info(f'MAP_REALIGN {robot} R{self.adopted[robot]} AMCL start = its last '
                                                f'valid pose ({x:.3f}, {y:.3f}, {yaw:.3f})')

    def receive_frozen(self,message):
        r=self.runtime
        with r.adapter.lock:
            frozen_robots=self._frozen_robots()
            if not frozen_robots: return
            try:
                grid=Grid(message)
                if any(grid.frame!=self.settings[robot]['map_frame'] for robot in frozen_robots):
                    raise ContractError('frozen map is not in the map frame')
            except (ContractError,ValueError,TypeError) as exc:
                r.node.get_logger().error('frozen map refused: '+str(exc)); return
            self.frozen_grids[grid.sha256]=grid
            self._pair()

    def receive_frozen_revision(self,message):
        r=self.runtime
        with r.adapter.lock:
            frozen_robots=self._frozen_robots()
            if not frozen_robots: return
            try:
                info=parse_frozen_revision(message.data)
                for robot in frozen_robots:
                    c=self.settings[robot]
                    if info['map_session_id']!=c['map_session_id'] or info['map_revision'] not in range(
                            c['map_revision']+1,c['map_revision']+FREEZES+1):
                        raise ContractError(f'frozen revision R{info["map_revision"]} of session '
                                            f'{info["map_session_id"]} is not a freeze of this run')
            except (ContractError,ValueError,TypeError,AttributeError) as exc:
                r.node.get_logger().error('frozen revision refused: '+str(exc)); return
            self.frozen_info=info
            self._pair()

    def _pair(self):
        """A frozen grid becomes revision R only with its pair (same hash); never guessed."""
        r=self.runtime; info=self.frozen_info
        grid=None if info is None else self.frozen_grids.get(info['occupancy_sha256'])
        if grid is None: return
        revision=paired_revision(info,grid.sha256)
        frozen_robots=self._frozen_robots()
        old=self.known.get(revision)
        if old is not None and old.sha256!=grid.sha256:
            for robot in frozen_robots:
                self.faults[robot]=f'frozen map R{revision} changed; new run required'; self.maps.pop(robot,None)
            r.node.get_logger().error(f'frozen map R{revision} changed after the freeze; map blocked (new run required)')
        elif old is None:
            self.known[revision]=grid
            r.node.get_logger().info(f'MAP_FROZEN R{revision} sha256={grid.sha256} {grid.width}x{grid.height} '
                                     f'res={grid.resolution}')
        for robot in frozen_robots:
            self._try_adopt(robot)
            if robot in self.adopted: self._load(robot)
            self._publish_map_state(robot)

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

    def receive_pose(self,robot,message,source='localization',covariance_fresh=True):
        """``source`` 'slam' (TF map -> base_link, slam_tf_pose) counts only while searching, 'localization'
        (amcl_pose) and 'amcl_tf' (TF map -> base_link with the last amcl_pose's covariance) otherwise. ``covariance_fresh`` False (the SLAM pose giving the covariance is too old): not valid."""
        r=self.runtime
        with r.adapter.lock:
            if (source=='slam')!=(self.phase(robot)=='SLAM'): return
            try:
                c=self.settings[robot]; observed=stamp(message)
                if observed<=0 or observed>r.adapter.wall_clock_ns()+r.inputs.bound:
                    raise ContractError('AMCL stamp missing or future')
                if source!='slam' and observed<=self.valid_after.get(robot,0): return  # before the map switch
                if source=='localization' and robot in self.amcl_tf and self.phase(robot)=='FROZEN':
                    # the TF pose's covariance, even when a TF pose stamped later already went out
                    self.amcl_tf[robot].on_covariance(observed,list(message.pose.covariance))
                if observed<=self.poses.get(robot,(0,False))[0]: return
                planar(message.pose.pose); covariance=message.pose.covariance
                valid=message.header.frame_id==c['map_frame'] and len(covariance)==36 and all(math.isfinite(v) for v in covariance)
                if valid:
                    # AMCL with zero spread publishes rounding just below zero (yaw -5.7e-14, #317 E3): 0
                    vx,vy,cxy,vyaw=(max(covariance[0],0.),max(covariance[7],0.),covariance[1],max(covariance[35],0.))
                    valid=(min(covariance[0],covariance[7],covariance[35])>=-VARIANCE_ROUNDING
                        and math.isclose(cxy,covariance[6],abs_tol=1e-9)
                        and cxy*cxy<=vx*vy+VARIANCE_ROUNDING
                        and (vx+vy+math.hypot(vx-vy,2*cxy))/2<=c['max_position_variance_m2']
                        and vyaw<=c['max_yaw_variance_rad2'])
                valid=valid and covariance_fresh
                self.poses[robot]=(observed,valid)
                if valid and self.phase(robot)=='FROZEN': self.last_valid[robot]=deepcopy(message)
                if self.publisher is not None and robot==self.publisher.robot:
                    self.publisher.on_pose(message,observed,valid and self._valid(robot))
            except (ContractError,ValueError,TypeError) as exc:
                self.poses.pop(robot,None); r.node.get_logger().warning('AMCL input rejected: '+str(exc))
                self._publish_map_state(robot)

    def receive_tf(self,robot,message):
        """<ns>/tf for the frozen-phase TF pose (map -> odom from AMCL, odom -> base_link from Create3)."""
        with self.runtime.adapter.lock:
            return self.amcl_tf[robot].on_tf(message)

    def tick_tf_pose(self,robot):
        """Frozen phase, every poll: the pose from TF with the last amcl_pose's covariance (AmclTfPose). A
        standing robot gets no amcl_pose (Nav2 AMCL updates on motion), so without this its pose stops."""
        from rescue_robot.slam_tf_pose import pose_message
        r=self.runtime
        with r.adapter.lock:
            if robot not in self.amcl_tf or self.phase(robot)!='FROZEN': return
            pose=self.amcl_tf[robot].pose(now_ns=r.adapter.wall_clock_ns(),after_ns=self.valid_after.get(robot,0))
            if pose is None: return   # no amcl_pose since the map switch, no TF edge, or tilted: no pose
            self.receive_pose(robot,pose_message(pose,self.settings[robot]['map_frame']),source='amcl_tf',
                              covariance_fresh=pose['covariance_fresh'])

    def _valid(self,robot):
        """Pose valid and, for a FROZEN_MAP_AMCL robot, its start alignment ALIGNED: AMCL starts that robot at
        the measured start with zero spread [Nav2 jazzy amcl_node.cpp:283-295], so its covariance alone says
        nothing about the start (재범 decision 2026-10-15 (나))."""
        aligned=self.settings[robot]['mode']!='FROZEN_MAP_AMCL' or self.aligned.get(robot,False)
        return self.poses.get(robot,(0,False))[1] and aligned

    def receive_start_align(self,robot,message):
        """rescue_robot.start_align verdict (JSON, latched): only status ALIGNED counts; anything else is not."""
        r=self.runtime
        with r.adapter.lock:
            if self.settings.get(robot,{}).get('mode')!='FROZEN_MAP_AMCL':
                raise ContractError(f'{robot} has no start alignment (only FROZEN_MAP_AMCL)')
            try:
                verdict=json.loads(message.data)
                status,reason=verdict['status'],verdict.get('reason','')
                if not isinstance(status,str) or not isinstance(reason,str): raise TypeError('status and reason text')
            except (ValueError,TypeError,KeyError,AttributeError) as exc:
                status,reason='UNREADABLE',str(exc)
            aligned=status=='ALIGNED'
            changed=self.aligned.get(robot)!=aligned
            self.aligned[robot]=aligned
            log=r.node.get_logger(); line=f'START_ALIGN {robot} {status} {reason}'.strip()
            # one severity per call site: rclpy raises when a site changes severity (simulation 2026-10-15)
            if aligned: log.info(line)
            else: log.warning(line+': localization not VALID')
            if changed: self._publish_map_state(robot)

    def _publish_map_state(self,robot):
        if self.publisher is not None and robot==self.publisher.robot:
            self.publisher.publish_map_state(localization_valid=self._valid(robot))

    def records(self):
        result={}
        for robot,c in self.settings.items():
            observed,valid=self.poses.get(robot,(0,False))
            result[robot]={key:deepcopy(c[key]) for key in ('map_frame','map_session_id','map_revision',
                'transform_revision','transform_xy_yaw','authority_id','clock_domain_id','valid_from_ns',
                'transform_valid_until_ns','position_error_m','rotation_error_rad')}
            result[robot].update(map_revision=self.basis(robot)['map_revision'],
                observed_at_ns=observed,valid_until_ns=observed+self.runtime.inputs.max_age,
                map_update_sequence=0,localization_status='VALID' if self._valid(robot) and self.loaded(robot) else 'UNKNOWN')
        return result


def attach_slam_tf_pose(r,robot,c,subscribe):
    """TB-6 searching: TF map -> base_link on the poll period, covariance from <ns>/pose (OPS-177)."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    from rescue_robot.slam_tf_pose import MAX_TILT_RAD, pose_message
    source=r.saved_maps.slam_tf[robot]; said={'refused':False,'tilted':False}

    def on_tf(message):
        if source.on_tf(message) and not said['refused']:
            said['refused']=True
            r.node.get_logger().warning(f'SLAM_TF_POSE {robot}: a map/odom/base edge that is not finite or not a unit rotation was not used')

    def on_slam_pose(message):
        if message.header.frame_id!=c['map_frame']: raise ContractError('SLAM pose is not in the map frame')
        source.on_covariance(stamp(message),list(message.pose.covariance))

    def tick():
        with r.adapter.lock:
            pose=source.pose(now_ns=r.adapter.wall_clock_ns())
            if pose is None:   # no TF edge, no SLAM pose yet or tilted: no pose, MapState stays UNKNOWN
                if source.tilt_rad is not None and source.tilt_rad>MAX_TILT_RAD and not said['tilted']:
                    said['tilted']=True
                    r.node.get_logger().warning(f'SLAM_TF_POSE {robot}: map -> base tilted '
                                                f'{math.degrees(source.tilt_rad):.1f} deg > '
                                                f'{math.degrees(MAX_TILT_RAD):.0f} deg: no pose')
                return
            r.saved_maps.receive_pose(robot,pose_message(pose,c['map_frame'],PoseWithCovarianceStamped),
                                      source='slam',covariance_fresh=pose['covariance_fresh'])

    subscribe(TFMessage,c['slam_tf_topic'],on_tf,100,lambda:None)
    subscribe(PoseWithCovarianceStamped,c['slam_localization_topic'],on_slam_pose,qos_profile_sensor_data,lambda:None)
    timer=r.node.create_timer(r.config['poll_period_s'],tick)
    r.input_handles.append(('destroy_timer',timer))


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
        frozen=[c for c in r.saved_maps.settings.values() if c['mode']!='SAVED_MAP_AMCL']
        if frozen:
            from std_msgs.msg import String
            subscribe(OccupancyGrid,frozen[0]['frozen_topic'],r.saved_maps.receive_frozen,snapshot,lambda:None)
            subscribe(String,frozen[0]['frozen_revision_topic'],r.saved_maps.receive_frozen_revision,snapshot,
                      lambda:None)
            # a pending revision is taken once the robot is stopped with no task
            timer=r.node.create_timer(r.config['poll_period_s'],r.saved_maps.tick)
            r.input_handles.append(('destroy_timer',timer))
        publishers={robot:r.node.create_publisher(PoseWithCovarianceStamped,c['initial_pose_topic'],10)
                    for robot,c in r.saved_maps.settings.items() if c['mode']=='FROZEN_MAP_AMCL'}
        for handle in publishers.values(): r.input_handles.append(('destroy_publisher',handle))

        def reinitialize(robot,last):
            """AMCL start on the newer frozen map = this robot's last valid pose (same SLAM frame)."""
            message=PoseWithCovarianceStamped()
            message.header.frame_id=last.header.frame_id
            message.header.stamp=r.node.get_clock().now().to_msg()
            message.pose.pose=last.pose.pose
            message.pose.covariance=[float(v) for v in last.pose.covariance]
            publishers[robot].publish(message)
        if publishers: r.saved_maps.reinitialize=reinitialize
        for robot,c in r.saved_maps.settings.items():
            if c['mode']=='SLAM_THEN_FROZEN':
                attach_slam_tf_pose(r,robot,c,subscribe)
            if robot in r.saved_maps.amcl_tf:   # frozen phase: TF pose on the poll period (standing robot)
                from tf2_msgs.msg import TFMessage
                subscribe(TFMessage,c['amcl_tf_topic'],lambda m,robot=robot:r.saved_maps.receive_tf(robot,m),100,
                          lambda:None)
                timer=r.node.create_timer(r.config['poll_period_s'],lambda robot=robot:r.saved_maps.tick_tf_pose(robot))
                r.input_handles.append(('destroy_timer',timer))
            subscribe(OccupancyGrid,c['map_topic'],lambda m,robot=robot:r.saved_maps.receive_map(robot,m),snapshot,
                      lambda robot=robot:r.saved_maps.maps.pop(robot,None))
            if c['mode']=='FROZEN_MAP_AMCL':   # TB-5 VALID only after start_align ALIGNED (#317)
                from std_msgs.msg import String
                subscribe(String,c['start_align_topic'],lambda m,robot=robot:r.saved_maps.receive_start_align(robot,m),
                          snapshot,lambda robot=robot:r.saved_maps.aligned.pop(robot,None))
            subscribe(PoseWithCovarianceStamped,c['localization_topic'],lambda m,robot=robot:r.saved_maps.receive_pose(robot,m),qos_profile_sensor_data,
                      lambda robot=robot:r.saved_maps.poses.pop(robot,None))
