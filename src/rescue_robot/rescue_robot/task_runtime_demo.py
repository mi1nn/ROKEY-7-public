"""Watchdog-independent task runtime rehearsal using synthetic motion inputs.

Both modes use the installed SiteRuntime. E2 runs without ROS. E3 uses real
ExecuteTask/CommandLease/RobotStatus transports on a selected LOCALHOST domain,
but NavigationPort and odometry remain synthetic. No cmd_vel, Nav2 ActionClient,
TF/device subscription, hardware operation or field policy is created here.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict
import json
import time
from types import SimpleNamespace as NS
from uuid import uuid4

from rescue_robot.executor import Executor, Policy
from rescue_robot.nav2_adapter import CommandAdapter
from rescue_robot.task_policy import ClockBasis, LeasePolicy
from rescue_robot.task_site_runtime import InputStore, SiteRuntime
from rescue_robot.task_server_probe import probe_namespace

DEMO_POLICY=Policy('SYNTHETIC-ONLY',1.,2.,.2,.15,1.,.01,.01)
DEMO_LEASE=LeasePolicy(.6,1_000_000_000,20_000_000,'SYNTHETIC-CLOCK')
SECOND=10**9


def fill(obj,values):
    for key,value in values.items():
        if isinstance(value,dict):
            nested=getattr(obj,key,None)
            if nested is None: nested=NS(); setattr(obj,key,nested)
            fill(nested,value)
        else: setattr(obj,key,value)
    return obj


def result_message():
    return NS(envelope=NS(),result=NS(final_pose=NS(envelope=NS(),time=NS())))


class DemoNode:
    def __init__(self): self.messages=[]; self.errors=[]
    def create_publisher(self,*args): return NS(publish=self.messages.append)
    def create_subscription(self,*args,**kwargs): return NS()
    def create_timer(self,*args,**kwargs): return NS()
    def destroy_timer(self,*args): pass
    def destroy_subscription(self,*args): pass
    def destroy_publisher(self,*args): pass
    def get_logger(self):
        return NS(error=self.errors.append,warning=self.errors.append,debug=lambda message:None)


def fake_bindings():
    class Feedback:
        def __new__(cls): return NS(envelope=NS(),current_pose=NS(envelope=NS(),time=NS()))
    return NS(ActionServer=lambda *a,**k:NS(destroy=lambda:None),
        ReentrantCallbackGroup=lambda:None,GoalResponse=NS(ACCEPT=True,REJECT=False),
        ExecuteTask=NS(Result=result_message,Feedback=Feedback),
        TaskAdmission=lambda:NS(envelope=NS()),RobotStatus=lambda:NS(envelope=NS()),CommandLease=NS)


class SimulatedNav:
    def __init__(self): self.goals=[]; self.cancels=[]; self.adapter=None
    def send_goal(self,command,pose):
        self.goals.append((command,pose)); self.adapter.on_goal_response(command,True)
    def cancel_goal(self,command):
        self.cancels.append(command)
        self.adapter.on_cancel_response(command,True); self.adapter.on_terminal(command,5)


class Fixture:
    """All numeric policies/measurements/authority here are synthetic test data."""
    def __init__(self,node,*,clock=time.monotonic,wall=time.time_ns,bindings=None):
        self.node=node; self.clock=clock; self.wall=wall
        robot='synthetic_robot'; boot=str(uuid4()); epoch=str(uuid4()); session=str(uuid4())
        self.prefix=probe_namespace()
        now=wall(); until=now+180*SECOND
        self.port=SimulatedNav()
        self.adapter=CommandAdapter(Executor(robot,boot,epoch,DEMO_POLICY,allowed_robot_ids=(robot,'synthetic_other')),
            self.port,guards=lambda _:False,clock_error_bound_s=.01,clock=clock,wall_clock_ns=wall)
        self.port.adapter=self.adapter
        self.frame=dict(robot_id=robot,native_frame='synthetic_map',mission_frame='synthetic_mission',
            map_session_id=session,map_revision=0,transform_revision=1,alignment_status='VALID')
        self.data=dict(format='rescue-robot-inputs/1',generated_at_ns=now,valid_until_ns=until,
            clock=asdict(ClockBasis(DEMO_LEASE.clock_domain_id,'SYNCED',1_000_000,'synthetic-epoch',until)),
            authority=dict(coordinator_id='synthetic_coordinator',authority_epoch=epoch,robot_boot_id=boot,
                publisher_id='synthetic_coordinator',publisher_boot_id=str(uuid4())),
            maps={robot:dict(map_frame='synthetic_map',map_session_id=session,map_revision=0,map_update_sequence=0,
                observed_at_ns=now,valid_until_ns=until,localization_status='VALID',transform_revision=1,
                authority_id='synthetic_mapping',clock_domain_id=DEMO_LEASE.clock_domain_id,
                valid_from_ns=now-SECOND,transform_valid_until_ns=until,transform_xy_yaw=[2.,0.,0.],
                position_error_m=.01,rotation_error_rad=.01,evidence_id='SYNTHETIC-CALIBRATION')},
            targets={},approvals={},protected_routes={},destinations={})
        self.config=dict(format='rescue-robot-runtime/1',robot_id=robot,mission_frame='synthetic_mission',
            schema_version='SYNTHETIC-ONLY',publisher_id='synthetic_executor',clock_error_bound_ns=10_000_000,
            lease=asdict(DEMO_LEASE),mapping_authority='synthetic_mapping',max_position_error_m=.2,
            max_rotation_error_rad=.1,input_max_age_ns=180*SECOND,poll_period_s=.05,input_file='/unused/synthetic.json',
            endpoints=dict(action_name=self.prefix+'/execute_task',lease_topic=self.prefix+'/command_lease',
                admission_topic=self.prefix+'/task_admission',status_topic=self.prefix+'/status'))
        self.inputs=InputStore(self.config,wall_clock_ns=wall,read=lambda:deepcopy(self.data))
        self.runtime=SiteRuntime(node,self.adapter,self.config,self.inputs,bindings=bindings)
        self.generation=0

    def observe(self):
        # Explicit simulated observations, not a claim that a watchdog exists.
        self.adapter.observe_status(driver=True,sensors=True,slam=True,nav2=True,undocked=True,watchdog=True)
        sec,ns=divmod(self.wall(),SECOND)
        self.adapter.on_odom(stamp_sec=sec,stamp_nanosec=ns,linear=(0.,0.,0.),angular=(0.,0.,0.))
        self.adapter.step(); self.runtime.server.poll()

    def envelope(self):
        a=self.data['authority']
        return dict(schema_version='SYNTHETIC-ONLY',publisher_id=a['publisher_id'],
            publisher_boot_id=a['publisher_boot_id'],sequence=self.generation,published_at_ns=self.wall())

    def goal_values(self,kind):
        self.generation+=1; a=self.data['authority']; now=self.wall(); until=now+10*SECOND
        goal=dict(envelope=self.envelope(),command_id=str(uuid4()),task_id=str(uuid4()),
            robot_id=self.adapter.robot_id,expected_robot_boot_id=a['robot_boot_id'],
            authority_epoch=a['authority_epoch'],coordinator_id=a['coordinator_id'],generation=self.generation,
            kind=kind,issued_at_ns=now,deadline_ns=until,lease_id=str(uuid4()),policy_version=DEMO_POLICY.version,
            target_id='',target_estimate_revision=0,search_area_ref='',home_ref='',has_route=False,
            has_approval=False,frame_ref=deepcopy(self.frame))
        if kind in ('SEARCH_AREA','RETURN_HOME'):
            ref=str(uuid4()); goal['search_area_ref' if kind=='SEARCH_AREA' else 'home_ref']=ref
            self.data['destinations'][ref]=dict(robot_id=goal['robot_id'],map_session_id=self.frame['map_session_id'],
                map_revision=0,transform_revision=1,xy_yaw=[3.,0.,0.],observed_at_ns=now,valid_until_ns=until)
            if kind=='SEARCH_AREA': goal['frame_ref'].update(native_frame='',alignment_status='')
        if kind in ('REOBSERVE_TARGET','MOVE_SEGMENT'):
            target=str(uuid4()); route=str(uuid4()); goal.update(target_id=target,has_route=True)
            self.data['targets'][target]=dict(revision=0)
            goal['route']=dict(route_id=route,route_revision=1,robot_id=goal['robot_id'],target_id=target,
                segment_index=0,frame_ref=deepcopy(self.frame),policy_version=DEMO_POLICY.version,
                generated_at_ns=now,valid_until_ns=until,path_sha256='',planner_evidence_id='',approval_corridor_sha256='',
                planned_path=dict(poses=[]),assessment=dict(status='UNKNOWN',width_known=False,
                    height_known=False,floor_geometry_known=False),
                approach_goal=dict(header=dict(frame_id=self.frame['mission_frame']),
                    pose=dict(position=dict(x=3.,y=0.,z=0.),orientation=dict(x=0.,y=0.,z=0.,w=1.))))
            self.data['protected_routes'][route]=dict(route_revision=1,segment_index=0,robot_id=goal['robot_id'],
                target_id=target,target_revision=0,endpoint=[3.,0.,0.],map_session_id=self.frame['map_session_id'],
                map_revision=0,transform_revision=1,valid_until_ns=until,observed_at_ns=now,
                protection_loaded=True,passage_free=True,evidence_id='SYNTHETIC-PROTECTED-PATH')
            if kind=='MOVE_SEGMENT':
                approval=dict(approval_id=str(uuid4()),command_id=str(uuid4()),operator_id='synthetic_operator',
                    robot_id=goal['robot_id'],robot_boot_id=a['robot_boot_id'],authority_epoch=a['authority_epoch'],
                    target_id=target,target_estimate_revision=0,route_id=route,route_revision=1,segment_index=0,
                    scope='ONE_SEGMENT',approved_at_ns=now,valid_until_ns=until)
                goal.update(has_approval=True,approval=approval)
                self.data['approvals'][approval['approval_id']]=deepcopy(approval)
        return goal

    def lease_values(self,goal,sequence):
        return dict(envelope=self.envelope(),lease_id=goal.lease_id,robot_id=goal.robot_id,
            robot_boot_id=goal.expected_robot_boot_id,coordinator_id=goal.coordinator_id,
            authority_epoch=goal.authority_epoch,generation=goal.generation,renewal_sequence=sequence,
            issued_at_ns=self.wall(),valid_until_ns=self.wall()+SECOND)


def stage_kinds(stage):
    """#85 priority, not a new task contract or a deployment default."""
    kinds={'common':('SEARCH_AREA','REOBSERVE_TARGET','HOLD'),
           'fallback':('MOVE_SEGMENT',),
           'all':('SEARCH_AREA','REOBSERVE_TARGET','MOVE_SEGMENT','RETURN_HOME','HOLD')}
    if stage not in kinds: raise ValueError('stage must be common, fallback or all')
    return kinds[stage]


def run_e2(*,stage='all'):
    kinds=stage_kinds(stage)
    class Clock:
        now=10.
        def steady(self): return self.now
        def wall(self): return 1_800_000_000*SECOND+round(self.now*SECOND)
    clock=Clock(); node=DemoNode(); f=Fixture(node,clock=clock.steady,wall=clock.wall,bindings=fake_bindings())
    def tick(n=1):
        for _ in range(n): clock.now=round(clock.now+.05,6); f.observe()
    tick(6); scenarios=[]
    for kind in kinds:
        if not f.runtime.arm(): raise RuntimeError('synthetic explicit arm failed')
        goal=fill(NS(),f.goal_values(kind))
        if f.runtime.server.on_goal(goal) is not True: raise RuntimeError(str(node.errors))
        lease=fill(NS(),f.lease_values(goal,0))
        if not f.runtime.server.on_lease(lease): raise RuntimeError('first lease rejected')
        tick(2)
        if kind!='HOLD': f.adapter.on_terminal(goal.command_id,4)
        tick(6)
        report=f.runtime.server.ingress.report(goal.command_id)
        if report is None or not report.result.stop_confirmed: raise RuntimeError('synthetic stop not completed')
        scenarios.append(dict(kind=kind,outcome=report.result.outcome,
                              scope=f.runtime.encode_result(report,goal).result.completion_scope))
    expiry_kind='MOVE_SEGMENT' if stage=='fallback' else 'REOBSERVE_TARGET'
    f.runtime.arm(); goal=fill(NS(),f.goal_values(expiry_kind))
    if not f.runtime.server.on_goal(goal): raise RuntimeError('expiry scenario admission')
    tick(21)
    report=f.runtime.server.ingress.report(goal.command_id)
    if not report or report.result.outcome!='EXPIRED' or not report.result.stop_confirmed:
        raise RuntimeError('lease expiry did not cancel and prove synthetic stop')
    late=fill(NS(),f.lease_values(goal,100))
    if f.runtime.server.on_lease(late) and f.adapter.executor.active is not None:
        raise RuntimeError('late lease revived expired task')
    extra={}
    if stage!='fallback':
        f.runtime.arm(); goal=fill(NS(),f.goal_values('SEARCH_AREA'))
        if not f.runtime.server.on_goal(goal): raise RuntimeError('renewal admission')
        started=clock.steady(); sequence=0
        # Run longer than the initial lease, without a Nav terminal callback.
        for _ in range(6):
            lease=fill(NS(),f.lease_values(goal,sequence))
            if not f.runtime.server.on_lease(lease): raise RuntimeError('renewal rejected')
            before=f.adapter.executor.lease_until
            if f.runtime.server.on_lease(lease) or f.adapter.executor.lease_until!=before:
                raise RuntimeError('duplicate renewal extended lease')
            tick(4); sequence+=1
            if f.adapter.executor.active is None or f.adapter.executor.state=='STOPPING':
                raise RuntimeError('active search expired despite fresh renewals')
        extra['renewal']=dict(status='PASSED',active_for_s=round(clock.steady()-started,6),
            initial_duration_s=DEMO_LEASE.duration_s,last_sequence=sequence-1,duplicate_rejected=True)
        f.adapter.request_stop('CANCEL_USER'); f.adapter.step()
        if f.runtime.server.ingress.report(goal.command_id) is not None:
            raise RuntimeError('cancel acknowledgment incorrectly proved stop')
        tick(6); report=f.runtime.server.ingress.report(goal.command_id)
        if not report or report.result.outcome!='CANCELED' or not report.result.stop_confirmed:
            raise RuntimeError('cancel did not wait for fresh synthetic odometry')
        extra['cancel']='CANCELED_STOP_CONFIRMED'
        f.runtime.arm(); goal=fill(NS(),f.goal_values('SEARCH_AREA'))
        if not f.runtime.server.on_goal(goal): raise RuntimeError('missing stop evidence admission')
        tick(2); f.adapter.request_stop('CANCEL_USER'); f.adapter.step()
        # Action cancel/terminal exists, but no odometry after its fence.
        for _ in range(45):
            clock.now=round(clock.now+.05,6); f.adapter.step(); f.runtime.server.poll()
        report=f.runtime.server.ingress.report(goal.command_id)
        if not report or report.result.stop_confirmed or report.result.reason!='STOP_UNCONFIRMED':
            raise RuntimeError('missing odometry incorrectly produced a stopped success')
        extra['no_stop_evidence']='STOP_UNCONFIRMED'
    return dict(status='PASSED',stage=stage,layer='E2-inprocess',scenarios=scenarios,lease_expiry='EXPIRED_STOP_CONFIRMED',
        late_renewal='NO_REVIVAL',**extra,
        sent_map_goals=[dict(frame=pose.frame_id,x=pose.x,y=pose.y) for _,pose in f.port.goals],
        inputs='SYNTHETIC',watchdog_process_created=False,real_nav2_client_created=False,
        robot_validation='NOT_RUN',ros_transport='NOT_RUN')


def run_ros(domain_id,timeout,*,stage='all'):
    kinds=stage_kinds(stage)
    from rescue_robot.task_server_probe import check_environment
    check_environment(domain_id)
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rescue_interfaces.action import ExecuteTask
    from rescue_interfaces.msg import CommandLease,TaskAdmission,RobotStatus
    rclpy.init(args=[]); node=Node('synthetic_task_runtime_'+uuid4().hex[:8])
    f=Fixture(node); received=[]; status=[]; results=[]
    node.create_subscription(TaskAdmission,f.prefix+'/task_admission',received.append,10)
    node.create_subscription(RobotStatus,f.prefix+'/status',status.append,10)
    publisher=node.create_publisher(CommandLease,f.prefix+'/command_lease',10)
    client=ActionClient(node,ExecuteTask,f.prefix+'/execute_task')
    node.create_timer(.05,f.observe); end=time.monotonic()+timeout
    def wait(predicate):
        while not predicate():
            if time.monotonic()>=end: raise RuntimeError('E3 local transport timeout')
            rclpy.spin_once(node,timeout_sec=.02)
    try:
        wait(lambda:client.server_is_ready() and f.adapter.executor.motion=='STOPPED'
             and publisher.get_subscription_count()>0 and len(status)>0
             and f.runtime.server.publisher.get_subscription_count()>0)
        for kind in kinds:
            if not f.runtime.arm(): raise RuntimeError('synthetic arm')
            goal=fill(ExecuteTask.Goal(),f.goal_values(kind)); sent=client.send_goal_async(goal)
            wait(sent.done); handle=sent.result()
            if not handle.accepted: raise RuntimeError(kind+' rejected')
            done=handle.get_result_async()
            lease=fill(CommandLease(),f.lease_values(goal,0)); publisher.publish(lease)
            wait(lambda:f.adapter.executor.lease_sequence==0)
            if kind!='HOLD':
                wait(lambda:goal.command_id in f.adapter.sent)
                f.adapter.on_terminal(goal.command_id,4)
            # HOLD also completes asynchronously after the stop gate and ROS
            # result delivery; lease receipt alone does not finish its Future.
            wait(done.done)
            result=done.result().result.result
            if not result.stop_confirmed: raise RuntimeError('synthetic stop missing')
            if result.outcome!='COMPLETED': raise RuntimeError('completion outcome')
            results.append(dict(kind=kind,outcome=result.outcome,scope=result.completion_scope))
        wait(lambda:len(received)>=len(kinds))
        expiry_kind='MOVE_SEGMENT' if stage=='fallback' else 'REOBSERVE_TARGET'
        if not f.runtime.arm(): raise RuntimeError('expiry arm')
        expiry=fill(ExecuteTask.Goal(),f.goal_values(expiry_kind))
        sent=client.send_goal_async(expiry); wait(sent.done); handle=sent.result()
        if not handle.accepted: raise RuntimeError('expiry admission')
        done=handle.get_result_async(); wait(done.done)
        result=done.result().result.result
        if result.outcome!='EXPIRED' or not result.stop_confirmed:
            raise RuntimeError('lease expiry did not cancel and prove synthetic stop')
        extra={}
        if stage!='fallback':
            if not f.runtime.arm(): raise RuntimeError('renewal arm')
            goal=fill(ExecuteTask.Goal(),f.goal_values('SEARCH_AREA'))
            sent=client.send_goal_async(goal); wait(sent.done); handle=sent.result()
            if not handle.accepted: raise RuntimeError('renewal admission')
            done=handle.get_result_async(); started=time.monotonic()
            for sequence in range(6):
                publisher.publish(fill(CommandLease(),f.lease_values(goal,sequence)))
                wait(lambda:f.adapter.executor.lease_sequence==sequence)
                until=time.monotonic()+.2; wait(lambda:time.monotonic()>=until)
                if done.done() or f.adapter.executor.state=='STOPPING':
                    raise RuntimeError('active search expired despite ROS renewals')
            extra['renewal']=dict(status='PASSED',active_for_s=time.monotonic()-started,
                initial_duration_s=DEMO_LEASE.duration_s,last_sequence=sequence)
            canceled=handle.cancel_goal_async(); wait(canceled.done)
            if not canceled.result().goals_canceling: raise RuntimeError('ROS cancel rejected')
            wait(done.done); result=done.result().result.result
            if result.outcome!='CANCELED' or not result.stop_confirmed:
                raise RuntimeError('ROS cancel result did not confirm synthetic stop')
            extra['cancel']='CANCELED_STOP_CONFIRMED'
        return dict(status='PASSED',stage=stage,layer='E3-LOCALHOST',scenarios=results,admission_count=len(received),
            lease_expiry='EXPIRED_STOP_CONFIRMED',**extra,
            status_count=len(status),inputs='SYNTHETIC',watchdog_process_created=False,
            real_nav2_client_created=False,robot_validation='NOT_RUN')
    finally:
        f.runtime.destroy(); client.destroy(); node.destroy_node(); rclpy.try_shutdown()


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros',action='store_true'); parser.add_argument('--domain-id',type=int)
    parser.add_argument('--stage',choices=('common','fallback','all'),default='all',
                        help='common first, then the minimal segment fallback (#85)')
    parser.add_argument('--timeout',type=float,default=20.,help='test-only maximum run time')
    opt=parser.parse_args(args)
    if not 0<opt.timeout<=60: parser.error('timeout must be in (0,60]')
    if opt.ros and opt.domain_id is None: parser.error('--ros requires an explicitly unused --domain-id')
    result=run_ros(opt.domain_id,opt.timeout,stage=opt.stage) if opt.ros else run_e2(stage=opt.stage)
    print(json.dumps(result,ensure_ascii=False)); return 0


if __name__=='__main__': raise SystemExit(main())
