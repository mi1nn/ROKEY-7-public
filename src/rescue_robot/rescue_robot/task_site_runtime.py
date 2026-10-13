"""Installed robot input runtime; watchdog is an independent readiness input.

Local measured records retain their original observation times. Live boot IDs
come directly from the Executor and configured coordinator's SystemState;
they are held in memory, never written back to the measured records.
Legacy configurations may still read a complete local IPC snapshot.
This file trust model assumes its writer and directory are controlled; it is
not DDS cryptographic authentication. Missing/stale input rejects commands
while the ROS action/status endpoints remain available.
"""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
from uuid import uuid4

from rescue_contracts import RescueError
from rescue_robot.executor import ContractError, GoalPose, generation, identifier, number, wall_time
from rescue_robot.phase1_route import Phase1RouteGate, phase1_endpoint
from rescue_robot.task_policy import ClockBasis, LeasePolicy, validate_clock_basis
from rescue_robot.task_server import attach_task_runtime
from rescue_robot.task_wire import COMPLETION_SCOPES

# Target moves the coordinator may send without an operator approval (C1, 2026-10-13).
AUTO_TARGET_KINDS = ('APPROACH_TARGET','REOBSERVE_TARGET','RETURN_HOME')
# Kinds resolved by _resolve_auto_target; RETURN_HOME keeps its configured-destination resolver and is
# only a per-robot permission in the policy (decision 5A, least privilege, 2026-10-13).
AUTO_TARGET_MOVES = ('APPROACH_TARGET','REOBSERVE_TARGET')


def recheck_settings(value):
    """ObserveTarget call after a REOBSERVE stop: observation window and the whole call's deadline (ns)."""
    if not isinstance(value,dict) or set(value)!={'window_ns','deadline_ns'}:
        raise ContractError('recheck needs exactly window_ns and deadline_ns')
    window,deadline=value['window_ns'],value['deadline_ns']
    if type(window) is not int or type(deadline) is not int or not 0<window<=deadline:
        raise ContractError('recheck window_ns and deadline_ns must be integers, 0 < window <= deadline')
    return dict(value)


def auto_target_policy(value):
    """Check the explicit LIVE_ROS automatic target-move policy; no field has a default."""
    if not isinstance(value,dict) or set(value)!={'enabled','kinds','goal_clear_radius_m','policy_version'}:
        raise ContractError('auto_target needs exactly enabled, kinds, goal_clear_radius_m, policy_version')
    kinds=value['kinds']
    if type(value['enabled']) is not bool:
        raise ContractError('auto_target.enabled must be a bool')
    if (not isinstance(kinds,list) or not kinds or len(set(kinds))!=len(kinds)
            or any(kind not in AUTO_TARGET_KINDS for kind in kinds)):
        raise ContractError('auto_target.kinds must be distinct kinds from '+str(list(AUTO_TARGET_KINDS)))
    radius=value['goal_clear_radius_m']
    if type(radius) not in (int,float) or not math.isfinite(radius) or radius<=0:
        raise ContractError('auto_target.goal_clear_radius_m must be finite and positive')
    if not isinstance(value['policy_version'],str) or not value['policy_version']:
        raise ContractError('auto_target.policy_version required')
    return deepcopy(value)


def atomic_json(path, value):
    """Publish a complete local IPC record; never silently replace its timestamps."""
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    fd, tmp=tempfile.mkstemp(prefix='.'+path.name+'.',dir=path.parent)
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as stream:
            json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
            stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        os.replace(tmp,path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def positive_ns(value,name):
    if type(value) is not int or not 0<value<2**63: raise ContractError(name)
    return value


def load_json(path):
    """Reject ambiguous keys and nonfinite values at the local file boundary."""
    def pairs(items):
        value={}
        for key,item in items:
            if key in value: raise ContractError('duplicate JSON key: '+key)
            value[key]=item
        return value
    def constant(value):
        raise ContractError('nonfinite JSON value: '+value)
    def floating(value):
        result=float(value)
        if not math.isfinite(result): raise ContractError('nonfinite JSON number: '+value)
        return result
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'),
                          object_pairs_hook=pairs,parse_constant=constant,parse_float=floating)
    except (OSError,ValueError) as exc:
        if isinstance(exc,RescueError): raise
        raise ContractError('JSON input unreadable: '+str(exc)) from exc


class InputStore:
    """Freshness-checked facade used by both phase-one gate and goal binder."""
    def __init__(self,config,*,wall_clock_ns=time.time_ns,read=None):
        self.config=deepcopy(config); self.wall_clock_ns=wall_clock_ns
        c=self.config
        if c.get('format')!='rescue-robot-runtime/1': raise ContractError('runtime format')
        self.mission_frame=c['mission_frame']
        if not isinstance(self.mission_frame,str) or not self.mission_frame: raise ContractError('mission_frame')
        self.lease_policy=LeasePolicy(**c['lease'])
        self.bound=positive_ns(c['clock_error_bound_ns'],'clock_error_bound_ns')
        if self.bound>self.lease_policy.max_error_ns: raise ContractError('task bound exceeds lease policy')
        self.max_age=positive_ns(c['input_max_age_ns'],'input_max_age_ns')
        for k in ('max_position_error_m','max_rotation_error_rad'): number(c[k],k,positive=True)
        path=c.get('input_source_file',c.get('input_file'))
        if 'saved_maps' not in c and (not isinstance(path,str) or not Path(path).is_absolute()):
            raise ContractError('explicit absolute local input path required')
        clock_path=c.get('clock_basis_file')
        if 'saved_maps' in c and ((clock_path is None and 'live_clock' not in c)
                or not isinstance(c.get('coordinator_authority'),dict)):
            raise ContractError('saved_maps requires measured clock provider and coordinator_authority')
        if clock_path is not None and (not isinstance(clock_path,str) or not Path(clock_path).is_absolute()):
            raise ContractError('clock_basis_file must be an absolute path')
        if not isinstance(c.get('configured_destinations',{}),dict):
            raise ContractError('configured_destinations must be an object')
        self.read=read or (lambda: load_json(path))
        self._last_time=None
        self._map_versions={}; self._retired_sessions=set()
        self._transform_meanings={}
        self.live_clock_provider=None

    def _window(self,start,until,now):
        positive_ns(start,'input observation time'); positive_ns(until,'input expiry')
        if start>now or until<=start or now+self.bound>=until or now-start+self.bound>self.max_age:
            raise ContractError('trusted input missing, future or stale','STALE_OBSERVATION')

    def current(self):
        now=wall_time(self.wall_clock_ns())
        if self._last_time is not None and now<self._last_time:
            raise ContractError('input clock regressed','CLOCK_UNCERTAIN')
        self._last_time=now
        try: data=self.read()
        except RescueError: raise
        except (OSError,ValueError) as exc: raise ContractError('trusted input unavailable','NOT_AUTHORIZED') from exc
        live='input_source_file' in self.config or 'saved_maps' in self.config
        expected='rescue-robot-sources/1' if live else 'rescue-robot-inputs/1'
        if not isinstance(data,dict) or data.get('format')!=expected:
            raise ContractError('input snapshot format')
        if not live: self._window(data['generated_at_ns'],data['valid_until_ns'],now)
        return data

    def clock_basis(self,data=None):
        # clock_capture combine already publishes this exact ClockBasis shape.
        # An explicit provider must fail on error, never fall back to stale data.
        path=self.config.get('clock_basis_file')
        if self.live_clock_provider is not None:
            basis=self.live_clock_provider()
        elif 'live_clock' in self.config:
            raise ContractError('live clock provider not attached','CLOCK_UNCERTAIN')
        else:
            record=load_json(path) if path is not None else (self.current() if data is None else data)['clock']
            basis=ClockBasis(**record)
        error=validate_clock_basis(basis,policy=self.lease_policy,now_ns=self.wall_clock_ns())
        if error>self.bound: raise ContractError('measured error exceeds task bound','CLOCK_UNCERTAIN')
        return basis

    def map_state(self,robot):
        data=self.current(); m=data['maps'][robot]
        self._window(m['observed_at_ns'],m['valid_until_ns'],self.wall_clock_ns())
        if m['localization_status']!='VALID': raise ContractError('localization uncertain','FRAME_REVISION_MISMATCH')
        identifier(m['map_session_id']); generation(m['map_revision']); generation(m['map_update_sequence'])
        if not isinstance(m['map_frame'],str) or not m['map_frame']: raise ContractError('native map frame')
        key=(robot,m['map_session_id']); current=(m['map_revision'],m['map_update_sequence'])
        old=self._map_versions.get(robot)
        if key in self._retired_sessions: raise ContractError('retired map session','FRAME_REVISION_MISMATCH')
        if old is not None:
            if old[0]==key and (current[0]<old[1][0] or current[1]<old[1][1]):
                raise ContractError('map revision rolled back','FRAME_REVISION_MISMATCH')
            if old[0]!=key: self._retired_sessions.add(old[0])
        self._map_versions[robot]=(key,current)
        return SimpleNamespace(source_robot=robot,map_frame=m['map_frame'],map_session_id=m['map_session_id'],
            map_revision=m['map_revision'],map_update_sequence=m['map_update_sequence'])

    def current_alignment(self,robot,*,now_ns):
        from rescue_mapping import FrameAlignment, RigidTransform
        state=self.map_state(robot); m=self.current()['maps'][robot]
        if (m['map_frame'],m['map_session_id'],m['map_revision'])!=(
                state.map_frame,state.map_session_id,state.map_revision):
            raise ContractError('mapping changed while reading input','FRAME_REVISION_MISMATCH')
        # File inputs carry a calibration record and must name its evidence (C-04). Live saved maps
        # declare their alignment in the runtime configuration and have no such record.
        evidence=m.get('evidence_id')
        if (m['authority_id']!=self.config['mapping_authority']
                or m['clock_domain_id']!=self.lease_policy.clock_domain_id
                or ('saved_maps' not in self.config and not (isinstance(evidence,str) and evidence))):
            raise ContractError('calibration authority/clock/evidence missing','NOT_AUTHORIZED')
        if not (m['valid_from_ns']<=now_ns-self.bound and now_ns+self.bound<m['transform_valid_until_ns']):
            raise ContractError('alignment expired','FRAME_REVISION_MISMATCH')
        if (m['position_error_m']>self.config['max_position_error_m']
                or m['rotation_error_rad']>self.config['max_rotation_error_rad']):
            raise ContractError('alignment exceeds configured error','FRAME_REVISION_MISMATCH')
        alignment=FrameAlignment(robot,state.map_frame,self.mission_frame,state.map_session_id,
            state.map_revision,m['transform_revision'],m['authority_id'],m['clock_domain_id'],
            RigidTransform.planar(*m['transform_xy_yaw']),m['valid_from_ns'],m['transform_valid_until_ns'],
            m['position_error_m'],m['rotation_error_rad'])
        key=(robot,state.map_session_id,state.map_revision,alignment.transform_revision)
        meaning=(alignment.map_frame,alignment.transform,alignment.authority_id,evidence)
        old=self._transform_meanings.get(key)
        if old is not None and old!=meaning:
            raise ContractError('same transform revision changed meaning','ID_CONTENT_CONFLICT')
        self._transform_meanings[key]=meaning
        return alignment

    def current_target_revision(self,target):
        return generation(self.current()['targets'][target]['revision'])

    def authorize_operator(self,approval,goal):
        record=self.current()['approvals'].get(approval.approval_id)
        if not isinstance(record,dict): return False
        # Independent operator record, never created from an arriving goal.
        return all(getattr(approval,key,None)==value for key,value in record.items()) and set(record)=={
            'approval_id','command_id','operator_id','robot_id','robot_boot_id','authority_epoch',
            'target_id','target_estimate_revision','route_id','route_revision','segment_index',
            'approved_at_ns','valid_until_ns','scope'}

    def passage_guard(self,goal):
        record=self.current()['protected_routes'].get(goal.route.route_id)
        if not isinstance(record,dict): return False
        self._window(record['observed_at_ns'],record['valid_until_ns'],self.wall_clock_ns())
        pose=phase1_endpoint(goal); f=goal.frame_ref
        expected=dict(robot_id=goal.robot_id,target_id=goal.target_id,target_revision=goal.target_estimate_revision,
            route_revision=goal.route.route_revision,segment_index=goal.route.segment_index,
            endpoint=[pose.x,pose.y,pose.yaw],map_session_id=f.map_session_id,
            map_revision=f.map_revision,transform_revision=f.transform_revision)
        return (all(record.get(k)==v for k,v in expected.items())
            and record.get('protection_loaded') is True and record.get('passage_free') is True
            and isinstance(record.get('evidence_id'),str) and bool(record['evidence_id']))


class SiteRuntime:
    def __init__(self,node,adapter,config,inputs,*,bindings=None):
        self.node,self.adapter,self.inputs=node,adapter,inputs
        self.config=deepcopy(config); self.leases={}; self.closed=False
        unsupported={'nav2_protection','operator_report_topic','operator_receipt_topic','request_stop_service'} & config.keys()
        if unsupported:
            raise ContractError('optional non-SEARCH live extensions not installed: '+', '.join(sorted(unsupported)))
        self.last_lease_rejection=''
        self.auto_target=auto_target_policy(config['auto_target']) if 'auto_target' in config else None
        self.recheck=recheck_settings(config['recheck']) if 'recheck' in config else None
        if self.auto_target is not None and 'REOBSERVE_TARGET' in self.auto_target['kinds'] and self.recheck is None:
            raise ContractError('automatic REOBSERVE_TARGET needs recheck settings (window_ns, deadline_ns)')
        # TB-5 passage stop from collision_monitor (live_passage); None = this robot reports none
        self.passage=None
        if 'passage' in config:
            from rescue_robot.live_passage import PassageMonitor, passage_settings
            self.passage=PassageMonitor(passage_settings(config['passage']))
        self.rechecks={}      # command_id -> ObserveTarget result to carry in TaskResult.recheck
        self.observe=None     # async goal-fields -> ObserveTarget result or None (attached with ROS)
        self.input_handles=[]; self._coordinator=None
        self._retired_coordinators=set(); self._input_error=''
        self.saved_maps=None
        self.live_clock=None
        if 'live_clock' in config:
            from rescue_robot.live_clock import LiveClock
            self.live_clock=LiveClock(self,config['live_clock'])
            inputs.live_clock_provider=self.live_clock.basis
        if 'saved_maps' in config:
            from rescue_robot.live_sources import SavedMapSources
            if adapter.robot_id not in config['saved_maps']:
                raise ContractError('saved_maps requires this robot')
            self.saved_maps=SavedMapSources(self,config['saved_maps'])
        self._targets={}; self._target_versions={}; self._target_sequence=-1; self._pending_targets=None
        if 'coordinator_targets_topic' in config:
            if not ('input_source_file' in config or 'saved_maps' in config):
                raise ContractError('live targets require coordinator session input')
            inputs.current_target_revision=self._target_revision
        if config['robot_id']!=adapter.robot_id: raise ContractError('runtime robot differs','WRONG_ROBOT')
        self.gate=Phase1RouteGate(adapter,registry=inputs,authorize_operator=inputs.authorize_operator,
            current_target_revision=inputs.current_target_revision,passage_guard=inputs.passage_guard,
            clock_error_bound_ns=inputs.bound)
        self.server=attach_task_runtime(node,adapter,registry=inputs,
            authorize=self.authorize,resolve_goal=self.resolve_goal,target_gate=self.target_gate,
            segment_gate=self.gate,phase1_gate=self.gate,clock_error_bound_ns=inputs.bound,
            lease_policy=inputs.lease_policy,clock_basis=inputs.clock_basis,
            schema_version=config['schema_version'],publisher_id=config['publisher_id'],
            poll_period_s=config['poll_period_s'],lookup_lease=lambda goal:self.leases.get(goal.lease_id),
            accept_lease=self.accept_lease,encode_result=self.encode_result,bindings=bindings,
            auto_resolve=self._auto,before_result=self.before_result,feedback_reason=self.feedback_reason,
            lease_rejection_reason=lambda:self.last_lease_rejection,
            **config['endpoints'])
        self.services=[]
        self.install_live_authority_guard()

    def _authority(self):
        a=self.inputs.current()['authority']
        if not ('input_source_file' in self.config or 'saved_maps' in self.config): return a
        return dict(a,robot_boot_id=self.adapter.executor.robot_boot_id,
                    publisher_id=self._coordinator[0],publisher_boot_id=self._coordinator[1])

    def _allowed(self,kind):
        """This robot's explicit LIVE_ROS policy lets it take this kind (no operator approval)."""
        policy=self.auto_target
        return (self.config.get('input_mode')=='LIVE_ROS' and policy is not None
                and policy['enabled'] is True and kind in policy['kinds'])

    def _auto(self,kind):
        """An automatic target move resolved by _resolve_auto_target."""
        return kind in AUTO_TARGET_MOVES and self._allowed(kind)

    def _issuer_rejection(self,message,*,boot_field):
        kind=getattr(message,'kind','SEARCH_AREA')
        if self.config.get('input_mode')=='LIVE_ROS' and kind!='SEARCH_AREA' and not self._allowed(kind):
            return 'LIVE_SEARCH_ONLY'
        a=self._authority(); e=message.envelope
        checks=((message.robot_id==self.adapter.robot_id,'ROBOT_ID_MISMATCH'),
            (getattr(message,boot_field)==a['robot_boot_id']==self.adapter.executor.robot_boot_id,
             'ROBOT_BOOT_ID_MISMATCH'),
            (message.authority_epoch==a['authority_epoch']==self.adapter.executor.authority_epoch,
             'AUTHORITY_EPOCH_MISMATCH'),
            (message.coordinator_id==a['coordinator_id'],'COORDINATOR_ID_MISMATCH'),
            (e.schema_version==self.config['schema_version'],'SCHEMA_VERSION_MISMATCH'),
            (e.publisher_id==a['publisher_id'],'PUBLISHER_ID_MISMATCH'),
            (e.publisher_boot_id==a['publisher_boot_id'],'PUBLISHER_BOOT_ID_MISMATCH'))
        return next((reason for accepted,reason in checks if not accepted),'')

    def _issuer(self,message,*,boot_field):
        return not self._issuer_rejection(message,boot_field=boot_field)

    def authorize(self,goal,lease):
        return self._issuer(goal,boot_field='expected_robot_boot_id') and (
            lease is None or self._issuer(lease,boot_field='robot_boot_id'))

    def accept_lease(self,lease):
        # TaskActionServer reads this diagnostic under the shared adapter lock.
        self.last_lease_rejection=self._issuer_rejection(lease,boot_field='robot_boot_id')
        if self.last_lease_rejection: return False
        old=self.leases.get(lease.lease_id)
        identity=lambda m:(m.lease_id,m.robot_id,m.robot_boot_id,m.authority_epoch,m.coordinator_id,m.generation)
        if old is not None and identity(old)!=identity(lease):
            self.last_lease_rejection='LEASE_IDENTITY_CONFLICT'
            return False
        if old is not None and lease.renewal_sequence<=old.renewal_sequence:
            self.last_lease_rejection='NONINCREASING_SEQUENCE'
            return False
        self.leases[lease.lease_id]=deepcopy(lease)
        return True

    def _resolve_auto_target(self,goal):
        """Automatic target move (C1): the coordinator's approach goal on this robot's own map basis.

        One shared map (profile map_basis): the goal's frame_ref must be this robot's current
        declared alignment; the target source robot's live map is not read (another PC may not
        see it). Path basis: the goal footprint is free in the saved map this robot loaded and
        hashed. Nav2 plans the real path on its live costmap (scene changes after the quake);
        an unplannable goal ends UNREACHABLE. No operator approval is read or recorded.
        """
        if goal.has_route is not True or goal.has_approval is not False:
            raise ContractError('automatic target move needs a route and no operator approval','NOT_AUTHORIZED')
        if goal.kind=='REOBSERVE_TARGET':
            self._current_target(goal.target_id,goal.target_estimate_revision)
        pose=phase1_endpoint(goal)  # no planner evidence, passage UNKNOWN, planar mission-frame goal
        a=self.inputs.current_alignment(goal.robot_id,now_ns=self.adapter.wall_clock_ns())
        f=goal.frame_ref
        if (f.mission_frame,f.map_session_id,f.map_revision,f.transform_revision)!=(
                self.inputs.mission_frame,a.map_session_id,a.map_revision,a.transform_revision):
            raise ContractError('target goal is not on this robot\'s current map basis','FRAME_REVISION_MISMATCH')
        grid=(self.saved_maps.maps.get(goal.robot_id) if self.saved_maps is not None else None)
        if grid is None:
            raise ContractError('saved map not loaded; cannot check the goal footprint','LOCALIZATION_UNCERTAIN')
        x,y,_=a.transform.inverse().apply((pose.x,pose.y,0.))
        if grid.clear(x,y,self.auto_target['goal_clear_radius_m']) is not True:
            raise ContractError('goal footprint is not free in the saved map','UNREACHABLE')
        return GoalPose(self.inputs.mission_frame,pose.x,pose.y,pose.yaw,a.map_session_id,a.map_revision,
                        a.transform_revision)

    def _current_target(self,target_id,revision):
        """The coordinator's TargetSnapshot record of this target at this revision (IF-070)."""
        record=self._targets.get(target_id)
        if record is None:
            raise ContractError('target not in the coordinator snapshot','STALE_OBSERVATION')
        self.inputs._window(record['observed_at_ns'],record['valid_until_ns'],self.adapter.wall_clock_ns())
        if record['revision']!=revision:
            raise ContractError('target revision changed','FRAME_REVISION_MISMATCH')
        return record

    def _recheck_goal(self,goal):
        """ObserveTarget goal fields: the target's current position in this robot's native map frame."""
        record=self._current_target(goal.target_id,goal.target_estimate_revision)
        a=self.inputs.current_alignment(goal.robot_id,now_ns=self.adapter.wall_clock_ns())
        point=tuple(float(v) for v in a.transform.inverse().apply(tuple(record['mission_position'])))
        now=self.adapter.wall_clock_ns()
        return dict(request_id=str(uuid4()),task_id=goal.task_id,target_id=goal.target_id,robot_id=goal.robot_id,
            watched_native_point=point,requested_window_ns=self.recheck['window_ns'],
            policy_version=self.auto_target['policy_version'],deadline_ns=now+self.recheck['deadline_ns'],
            frame_ref=dict(robot_id=goal.robot_id,native_frame=a.map_frame,mission_frame=self.inputs.mission_frame,
                map_session_id=a.map_session_id,map_revision=a.map_revision,transform_revision=a.transform_revision,
                alignment_status='VALID'))

    async def before_result(self,report,goal):
        """After a REOBSERVE stop is confirmed, ask the recheck server; carry its answer as it is.

        Only COMPLETED + stop_confirmed asks. No target position, no server or no answer by the
        deadline leaves has_recheck false (nothing is made up); the task outcome never changes.
        """
        if (goal.kind!='REOBSERVE_TARGET' or not self._auto(goal.kind)
                or report.result.outcome!='COMPLETED' or report.result.stop_confirmed is not True):
            return
        log=self.node.get_logger()
        try:
            with self.adapter.lock:
                fields=self._recheck_goal(goal)
        except Exception as exc:
            log.error('recheck not requested for '+goal.command_id+': '+str(exc)); return
        if self.observe is None:
            log.error('recheck not requested for '+goal.command_id+': no ObserveTarget client'); return
        answer=await self.observe(fields)
        if answer is None:
            log.error('recheck '+fields['request_id']+' got no answer; has_recheck=false'); return
        log.info('RECHECK '+goal.command_id+' request='+fields['request_id']+' outcome='+str(answer.outcome)
                 +' reason='+str(answer.reason_code))
        with self.adapter.lock:
            self.rechecks[goal.command_id]=answer

    def resolve_goal(self,goal):
        if goal.kind=='HOLD': return None
        if self._auto(goal.kind): return self._resolve_auto_target(goal)
        if goal.kind not in ('SEARCH_AREA','RETURN_HOME'):
            raise ContractError('target movement requires phase-one route; automatic approach unavailable','NOT_AUTHORIZED')
        ref=goal.search_area_ref if goal.kind=='SEARCH_AREA' else goal.home_ref
        configured=self.inputs.config.get('configured_destinations',{})
        if ref in configured:
            p=configured[ref]
            if p['frame_id']!=self.inputs.mission_frame:
                raise ContractError('configured destination frame differs','FRAME_REVISION_MISMATCH')
        else:
            p=self.inputs.current().get('destinations',{}).get(ref)
            if p is None:
                what='home destination' if goal.kind=='RETURN_HOME' else 'search destination'
                raise ContractError(what+' not configured: '+str(ref),'NOT_AUTHORIZED')
            self.inputs._window(p['observed_at_ns'],p['valid_until_ns'],self.adapter.wall_clock_ns())
        a=self.inputs.current_alignment(goal.robot_id,now_ns=self.adapter.wall_clock_ns())
        if (p['robot_id'],p['map_session_id'],p['map_revision'],p['transform_revision'])!=(
                goal.robot_id,a.map_session_id,a.map_revision,a.transform_revision):
            raise ContractError('destination basis changed','FRAME_REVISION_MISMATCH')
        return GoalPose(self.inputs.mission_frame,*p['xy_yaw'],a.map_session_id,a.map_revision,a.transform_revision)

    def target_gate(self,task):
        # attach_task_runtime runs ingress.check (the live Phase1RouteGate)
        # before the binder. Do not run that same gate a second time here.
        request=self.server.ingress.requests.get(task.command_id)
        if request and self._auto(task.kind):
            return True  # resolved again by guard() on every step (_resolve_auto_target)
        return bool(request and task.kind in ('MOVE_SEGMENT','REOBSERVE_TARGET')
                    and task.command_id in self.gate.requests)

    def feedback_reason(self):
        """Non-latched ExecuteTask feedback reason: PASSAGE_BLOCKED while the passage polygon holds the active task."""
        if self.passage is not None and self.adapter.executor.active is not None and self.passage.blocked():
            return 'PASSAGE_BLOCKED'
        return ''

    def encode_result(self,report,goal):
        response=self.server.bindings.ExecuteTask.Result(); result=response.result
        result.task_id,result.command_id,result.robot_id=report.task_id,report.command_id,report.robot_id
        result.generation=report.generation
        result.outcome,result.terminal_reason=report.result.outcome,report.result.reason
        # Nav2 gave up after the passage polygon held TB-5: say why (original_cause keeps Nav2's cause).
        if (self.passage is not None and result.terminal_reason=='ACTION_ABORTED'
                and self.passage.blocked_within(now_ns=self.adapter.wall_clock_ns())):
            result.terminal_reason='PASSAGE_BLOCKED'
        result.original_cause,result.stop_confirmed=report.result.original_cause,report.result.stop_confirmed
        result.motion_state='STOPPED' if result.stop_confirmed else 'STOP_UNCONFIRMED'
        result.completion_scope=COMPLETION_SCOPES[goal.kind]
        answer=self.rechecks.pop(goal.command_id,None) if goal.kind=='REOBSERVE_TARGET' else None
        result.has_recheck=answer is not None
        if answer is not None:
            result.recheck=answer.observation  # the server's observation, unchanged (OBSERVED/UNKNOWN/FAILED)
        result.used_record_ids=[]; result.stop_evidence=[]
        # Action + fresh odometry prove stop. No pose or perception/evidence
        # record is invented; independent evidence recorders may add those.
        pose=result.final_pose
        pose.robot_id,pose.source_boot_id=report.robot_id,report.robot_boot_id
        pose.mission_pose_status='UNKNOWN'
        pose.time.stamp_semantics,pose.time.clock_state='UNKNOWN','UNKNOWN'
        self.server.stamp_envelope(pose.envelope)
        return response

    def guard(self,task):
        request=self.server.ingress.requests.get(task.command_id)
        if request is None: return False
        try:
            if not self.authorize(request[0],None): return False
            # RETURN_HOME is resolved continuously by TaskIngress.check.
            if (task.kind=='SEARCH_AREA' or self._auto(task.kind)) and self.resolve_goal(request[0])!=task.goal_pose:
                raise ContractError('destination changed; require a new command','FRAME_REVISION_MISMATCH')
            return self._base_guard(task)
        except Exception as exc:
            self.node.get_logger().error('live task input guard rejected: '+str(exc))
            return False

    def install_live_authority_guard(self):
        self._base_guard=self.adapter.guards
        self.adapter.guards=self.guard

    def arm(self):
        with self.adapter.lock:
            a=self._authority()
            if (a['robot_boot_id'],a['authority_epoch'])!=(
                    self.adapter.executor.robot_boot_id,self.adapter.executor.authority_epoch): return False
            self.inputs.clock_basis()
            now=self.adapter.clock()
            return self.adapter.executor.arm(a['authority_epoch'],now=now,readiness=self.adapter.readiness(now))

    def attach_services(self):
        from std_srvs.srv import Trigger
        def arm(_request,response):
            try: response.success=self.arm(); response.message='ARMED' if response.success else 'STOP_OR_READINESS_NOT_READY'
            except Exception as exc: response.success=False; response.message=str(exc)
            return response
        def stop(_request,response):
            with self.adapter.lock: self.adapter.request_stop('CANCEL_USER')
            response.success=True; response.message='STOP_REQUESTED; confirm RobotStatus/TaskResult'
            return response
        self.services=[self.node.create_service(Trigger,'~/task_arm',arm),
                       self.node.create_service(Trigger,'~/task_stop',stop)]

    def _input_source(self):
        data=(dict(format='rescue-robot-sources/1',maps=self.saved_maps.records())
              if self.saved_maps is not None else load_json(self.config['input_source_file']))
        if not isinstance(data,dict) or data.get('format')!='rescue-robot-sources/1':
            raise ContractError('input source format')
        # Control identity is deployment configuration, not an observation or
        # an identity claimed by an arriving goal. Keep legacy source support.
        if 'coordinator_authority' in self.config:
            data['authority']=deepcopy(self.config['coordinator_authority'])
        a=data['authority']
        identifier(a['coordinator_id'])
        if a['authority_epoch']!=self.adapter.executor.authority_epoch:
            raise ContractError('source authority differs from configured executor','NOT_AUTHORIZED')
        self.inputs.clock_basis(data)
        return data

    def _central_envelope(self,envelope):
        a=self._authority(); now=self.adapter.wall_clock_ns(); stamp=wall_time(envelope.published_at_ns)
        if ((envelope.schema_version,envelope.publisher_id,envelope.publisher_boot_id)!=
                (self.config['schema_version'],a['publisher_id'],a['publisher_boot_id'])
                or stamp<=0 or stamp>now+self.inputs.bound
                or now-stamp+self.inputs.bound>self.inputs.lease_policy.max_age_ns):
            raise ContractError('unregistered or stale central record','NOT_AUTHORIZED')
        generation(envelope.sequence)





    def _target_revision(self,target):
        self._live_inputs()
        record=self._targets[target]
        self.inputs._window(record['observed_at_ns'],record['valid_until_ns'],self.adapter.wall_clock_ns())
        return record['revision']

    def _receive_targets(self,message):
        """Consume IF-070 full snapshots; never create approval/path evidence."""
        with self.adapter.lock:
            e=message.envelope; key=(e.publisher_id,e.publisher_boot_id)
            registered=False
            try:
                data=self._input_source(); now=self.adapter.wall_clock_ns()
                if (e.schema_version!=self.config['schema_version']
                        or e.publisher_id!=data['authority']['coordinator_id']):
                    raise ContractError('unregistered target publisher','NOT_AUTHORIZED')
                identifier(e.publisher_boot_id); identifier(message.snapshot_id); generation(e.sequence)
                stamp=wall_time(e.published_at_ns)
                if stamp<=0 or stamp>now+self.inputs.bound or key in self._retired_coordinators:
                    raise ContractError('invalid target snapshot session/time','NOT_AUTHORIZED')
                registered=True
                if self._coordinator is not None and key==self._coordinator[:2] and e.sequence<=self._target_sequence:
                    return
                if self._pending_targets is not None and key==self._pending_targets[0] and e.sequence<=self._pending_targets[1]:
                    return
                targets={}; seen=set()
                for item in message.targets:
                    identifier(item.target_id); generation(item.estimate_revision)
                    start=positive_ns(item.latest_observed_at_ns,'target observation')
                    until=positive_ns(item.valid_until_ns,'target expiry')
                    if start>now+self.inputs.bound or until<=start or item.target_id in seen:
                        raise ContractError('invalid target snapshot record')
                    if item.confirmation_state not in ('CANDIDATE','CONFIRMED','NEEDS_REOBSERVATION','DISMISSED'):
                        raise ContractError('invalid target confirmation state')
                    seen.add(item.target_id)
                    position=item.mission_position
                    if not all(math.isfinite(v) for v in (position.x,position.y,position.z)):
                        raise ContractError('invalid target position')
                    record=dict(revision=item.estimate_revision,observed_at_ns=start,valid_until_ns=until,
                                mission_position=(float(position.x),float(position.y),float(position.z)))
                    old=self._target_versions.get(item.target_id) if self._coordinator is not None and key==self._coordinator[:2] else None
                    if old is not None and (record['revision']<old['revision']
                            or record['revision']==old['revision'] and record!=old):
                        raise ContractError('target revision regressed or changed meaning','ID_CONTENT_CONFLICT')
                    if item.confirmation_state=='DISMISSED': continue
                    targets[item.target_id]=record
                snapshot=(key,e.sequence,targets)
                if self._coordinator is None or key!=self._coordinator[:2]:
                    self._pending_targets=snapshot
                else:
                    self._targets=targets; self._target_sequence=e.sequence
                    self._target_versions.update(targets)
            except (RescueError,OSError,ValueError,TypeError,KeyError) as exc:
                # An invalid new snapshot from the registered session cannot
                # leave obsolete target authorization usable. Foreign packets
                # do not revoke the current session.
                if (registered and self._coordinator is not None and key==self._coordinator[:2]
                        and e.sequence>self._target_sequence):
                    self._targets={}
                    self._target_sequence=e.sequence
                self.node.get_logger().warning('target snapshot rejected: '+str(exc))

    def _live_inputs(self):
        data=self._input_source()
        if self._coordinator is None or self._coordinator[0]!=data['authority']['coordinator_id']:
            raise ContractError('waiting for fresh configured coordinator SystemState','NOT_AUTHORIZED')
        if self.live_clock is not None and (self.live_clock.peer is None
                or self.live_clock.peer['publisher_boot_id']!=self._coordinator[1]):
            raise ContractError('waiting for matching clock/coordinator session','CLOCK_UNCERTAIN')
        return data

    def _stop_input_session(self):
        if self.adapter.executor.active is not None:
            self.adapter.request_stop('GUARD_INVALID','FAILED')
        elif self.adapter.executor.latch!='FAULT_LATCHED':
            self.adapter.executor.latch='DISARMED'

    def _check_input_session(self):
        with self.adapter.lock:
            try:
                self.inputs.current()
                if self._input_error:
                    self.node.get_logger().info('input session ready; explicit arm required')
                self._input_error=''
                return True
            except (RescueError,OSError,ValueError,TypeError,KeyError) as exc:
                self._stop_input_session()
                if str(exc)!=self._input_error:
                    self.node.get_logger().error('input session blocked: '+str(exc))
                self._input_error=str(exc)
                return False

    def _receive_coordinator(self,message):
        with self.adapter.lock:
            registered=False
            try:
                data=self._input_source(); e=message.envelope; now=self.adapter.wall_clock_ns()
                if (e.schema_version!=self.config['schema_version']
                        or e.publisher_id!=data['authority']['coordinator_id']):
                    raise ContractError('unregistered coordinator SystemState','NOT_AUTHORIZED')
                identifier(e.publisher_boot_id); generation(e.sequence); stamp=wall_time(e.published_at_ns)
                if (stamp<=0 or stamp>now+self.inputs.bound
                        or now-stamp+self.inputs.bound>self.inputs.lease_policy.max_age_ns):
                    raise ContractError('coordinator SystemState is future or stale','CLOCK_UNCERTAIN')
                key=(e.publisher_id,e.publisher_boot_id)
                if key in self._retired_coordinators:
                    raise ContractError('retired coordinator boot','NOT_AUTHORIZED')
                if self._coordinator is not None and key==self._coordinator[:2] and e.sequence<=self._coordinator[2]: return
                registered=True
                if self._coordinator is not None:
                    if key==self._coordinator[:2] and e.sequence<=self._coordinator[2]: return
                    if key!=self._coordinator[:2]:
                        self._retired_coordinators.add(self._coordinator[:2])
                        self.leases.clear(); self._stop_input_session()
                        self._targets={}; self._target_versions={}; self._target_sequence=-1
                self._coordinator=(*key,e.sequence)
                if self._pending_targets is not None and self._pending_targets[0]==key:
                    _,self._target_sequence,self._targets=self._pending_targets
                    self._target_versions.update(self._targets)
                    self._pending_targets=None
                self._check_input_session()
            except (RescueError,OSError,ValueError,TypeError,KeyError) as exc:
                if registered:
                    self._stop_input_session()
                self.node.get_logger().warning('coordinator session rejected: '+str(exc))

    def attach_input_updates(self):
        """Observe the coordinator session; measured records remain read-only."""
        if not ('input_source_file' in self.config or 'saved_maps' in self.config): return
        if self.live_clock is not None: self.live_clock.attach()
        topic=self.config['coordinator_state_topic']
        if not isinstance(topic,str) or not topic.startswith('/'):
            raise ContractError('explicit absolute coordinator_state_topic required')
        from rescue_interfaces.msg import SystemState
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.inputs.read=self._live_inputs
        self._check_input_session()
        subscription=self.node.create_subscription(SystemState,topic,self._receive_coordinator,qos)
        self.input_handles.append(('destroy_subscription',subscription))
        if 'coordinator_targets_topic' in self.config:
            from rescue_interfaces.msg import TargetSnapshot
            topic=self.config['coordinator_targets_topic']
            if not isinstance(topic,str) or not topic.startswith('/'):
                raise ContractError('explicit absolute coordinator_targets_topic required')
            subscription=self.node.create_subscription(TargetSnapshot,topic,self._receive_targets,qos)
            self.input_handles.append(('destroy_subscription',subscription))
        if self.recheck is not None and self.auto_target is not None and 'REOBSERVE_TARGET' in self.auto_target['kinds']:
            from rescue_robot.live_recheck import RecheckClient
            self.observe=RecheckClient(self,action_name=f'/rescue/robots/{self.adapter.robot_id}/observe_target')
        if self.passage is not None:
            from rescue_robot.live_passage import attach as attach_passage
            attach_passage(self,'collision_monitor_state')  # relative: <ns>/collision_monitor_state (Nav2)
        if self.saved_maps is not None:
            from rescue_robot.live_sources import attach_sources
            if 'pose_valid_ns' in self.config:
                from rescue_robot.live_pose import LivePosePublisher
                self.saved_maps.publisher=LivePosePublisher(self,pose_valid_ns=self.config['pose_valid_ns'],
                                                            sensor_clock=self.config['sensor_host_clock'])
            attach_sources(self)
        timer=self.node.create_timer(self.config['poll_period_s'],self._check_input_session)
        self.input_handles.append(('destroy_timer',timer))

    def destroy(self):
        if self.closed: return
        self.closed=True
        if self.live_clock is not None: self.live_clock.close()
        for method,handle in reversed(self.input_handles): getattr(self.node,method)(handle)
        self.server.destroy()
        for service in self.services: self.node.destroy_service(service)


def create_runtime(node,adapter):
    from rcl_interfaces.msg import ParameterType, ParameterDescriptor
    from rescue_robot.ros_node import declare_required
    node.declare_parameter('task.input_mode','FILES',descriptor=ParameterDescriptor(
        type=ParameterType.PARAMETER_STRING,read_only=True))
    mode=node.get_parameter('task.input_mode').value
    if mode=='LIVE_ROS':
        from rescue_robot.live_parameters import read_config
        config=read_config(node,adapter)
    elif mode=='FILES':
        values,missing=declare_required(node,{'task.runtime_config':ParameterType.PARAMETER_STRING})
        if missing: raise ContractError('task.runtime_config required')
        path=Path(values['task.runtime_config'])
        if not path.is_absolute(): raise ContractError('task.runtime_config must be an explicit absolute path')
        config=load_json(path)
    else:
        raise ContractError('task.input_mode must be LIVE_ROS or FILES')
    if not isinstance(config,dict): raise ContractError('runtime configuration must be an object')
    try:
        inputs=InputStore(config,wall_clock_ns=adapter.wall_clock_ns)
        runtime=SiteRuntime(node,adapter,config,inputs)
    except (KeyError,TypeError) as exc:
        raise ContractError('runtime configuration missing or malformed: '+str(exc)) from exc
    try:
        runtime.attach_services()
        runtime.attach_input_updates()
    except Exception:
        runtime.destroy()
        raise
    node.get_logger().info('task runtime configuration='+json.dumps(config,sort_keys=True,allow_nan=False))
    node.get_logger().info('installed task runtime; input_mode='+mode+'; explicit arm required')
    return runtime
