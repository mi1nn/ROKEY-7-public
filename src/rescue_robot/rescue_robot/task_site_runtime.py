"""Installed robot input runtime; watchdog is an independent readiness input.

The explicit local input file is a trusted IPC snapshot from the deployment's
authority, calibration and route-protection adapters. It must be atomically
replaced with original observation times. Reading never extends its validity.
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

from rescue_contracts import RescueError, TARGET_TASK_KINDS
from rescue_robot.executor import ContractError, GoalPose, generation, identifier, number, wall_time
from rescue_robot.phase1_route import Phase1RouteGate, phase1_endpoint
from rescue_robot.task_policy import ClockBasis, LeasePolicy, validate_clock_basis
from rescue_robot.task_server import attach_task_runtime
from rescue_robot.task_wire import COMPLETION_SCOPES


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


SEARCH_INPUT_FORMAT = 'rescue-robot-search-inputs/1'


def search_snapshot(data, config):
    """One robot and one verified interval; never derive freshness from reading.

    The producer confirms the complete snapshot together. Independent source
    observation times must use the existing full format instead.
    """
    robot = config['robot_id']
    start, until = data['generated_at_ns'], data['valid_until_ns']
    mapping = dict(data['map'], observed_at_ns=start, valid_until_ns=until,
        valid_from_ns=start, transform_valid_until_ns=until,
        authority_id=config['mapping_authority'], clock_domain_id=data['clock']['domain_id'])
    destinations = {ref: dict(robot_id=robot, xy_yaw=pose,
        map_session_id=mapping['map_session_id'], map_revision=mapping['map_revision'],
        transform_revision=mapping['transform_revision'], observed_at_ns=start, valid_until_ns=until)
        for ref, pose in data['destinations'].items()}
    return dict(data, maps={robot: mapping}, destinations=destinations,
                targets={}, approvals={}, protected_routes={})


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
        if not isinstance(c['input_file'],str) or not Path(c['input_file']).is_absolute():
            raise ContractError('explicit absolute local input_file required')
        self.read=read or (lambda: json.loads(Path(c['input_file']).read_text(encoding='utf-8')))
        self._last_time=None
        self._map_versions={}; self._retired_sessions=set()
        self._transform_meanings={}

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
        except (OSError,ValueError) as exc: raise ContractError('trusted input unavailable','NOT_AUTHORIZED') from exc
        if not isinstance(data,dict) or data.get('format') not in ('rescue-robot-inputs/1', SEARCH_INPUT_FORMAT):
            raise ContractError('input snapshot format')
        self._window(data['generated_at_ns'],data['valid_until_ns'],now)
        if data['format'] == SEARCH_INPUT_FORMAT:
            data = search_snapshot(data, self.config)
        return data

    def clock_basis(self):
        data=self.current()
        basis=ClockBasis(**data['clock'])
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
        if (m['authority_id']!=self.config['mapping_authority']
                or m['clock_domain_id']!=self.lease_policy.clock_domain_id or not m['evidence_id']):
            raise ContractError('calibration authority/evidence missing','NOT_AUTHORIZED')
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
        meaning=(alignment.map_frame,alignment.transform,alignment.authority_id,m['evidence_id'])
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
        self.last_lease_rejection=''
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
            lease_rejection_reason=lambda:self.last_lease_rejection,
            **config['endpoints'])
        self.services=[]
        self.install_live_authority_guard()

    def _issuer_rejection(self,message,*,boot_field):
        data=self.inputs.current()
        if (data['format'] == SEARCH_INPUT_FORMAT and getattr(message,'kind',None) is not None
                and message.kind not in ('SEARCH_AREA','RETURN_HOME','HOLD')):
            return 'SEARCH_INPUT_KIND_UNSUPPORTED'
        a=data['authority']; e=message.envelope
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

    def resolve_goal(self,goal):
        if goal.kind=='HOLD': return None
        if goal.kind not in ('SEARCH_AREA','RETURN_HOME'):
            raise ContractError('target movement requires phase-one route; automatic approach unavailable','NOT_AUTHORIZED')
        ref=goal.search_area_ref if goal.kind=='SEARCH_AREA' else goal.home_ref
        data=self.inputs.current(); p=data['destinations'][ref]
        self.inputs._window(p['observed_at_ns'],p['valid_until_ns'],self.adapter.wall_clock_ns())
        a=self.inputs.current_alignment(goal.robot_id,now_ns=self.adapter.wall_clock_ns())
        if (p['robot_id'],p['map_session_id'],p['map_revision'],p['transform_revision'])!=(
                goal.robot_id,a.map_session_id,a.map_revision,a.transform_revision):
            raise ContractError('destination basis changed','FRAME_REVISION_MISMATCH')
        return GoalPose(self.inputs.mission_frame,*p['xy_yaw'],a.map_session_id,a.map_revision,a.transform_revision)

    def target_gate(self,task):
        request=self.server.ingress.requests.get(task.command_id)
        return bool(request and task.kind in ('MOVE_SEGMENT','REOBSERVE_TARGET')
                    and self.gate.check(task.command_id))

    def encode_result(self,report,goal):
        response=self.server.bindings.ExecuteTask.Result(); result=response.result
        result.task_id,result.command_id,result.robot_id=report.task_id,report.command_id,report.robot_id
        result.generation=report.generation
        result.outcome,result.terminal_reason=report.result.outcome,report.result.reason
        result.original_cause,result.stop_confirmed=report.result.original_cause,report.result.stop_confirmed
        result.motion_state='STOPPED' if result.stop_confirmed else 'STOP_UNCONFIRMED'
        result.completion_scope=COMPLETION_SCOPES[goal.kind]
        result.has_recheck=False; result.used_record_ids=[]; result.stop_evidence=[]
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
            if task.kind in ('SEARCH_AREA','RETURN_HOME') and self.resolve_goal(request[0])!=task.goal_pose:
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
            a=self.inputs.current()['authority']
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

    def destroy(self):
        if self.closed: return
        self.closed=True
        self.server.destroy()
        for service in self.services: self.node.destroy_service(service)


def create_runtime(node,adapter):
    from rcl_interfaces.msg import ParameterType
    from rescue_robot.ros_node import declare_required
    values,missing=declare_required(node,{'task.runtime_config':ParameterType.PARAMETER_STRING})
    if missing: raise ContractError('task.runtime_config required')
    path=Path(values['task.runtime_config'])
    config=json.loads(path.read_text(encoding='utf-8'))
    inputs=InputStore(config,wall_clock_ns=adapter.wall_clock_ns)
    runtime=SiteRuntime(node,adapter,config,inputs)
    runtime.attach_services()
    node.get_logger().info('installed task runtime; input_file='+config['input_file']+'; explicit arm required')
    return runtime
