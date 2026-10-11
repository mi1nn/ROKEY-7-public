"""Generate task runtime files and report missing deployment inputs.

Creates only files in the requested output directory. Never updates the team
profile, starts nodes, chooses motion policy, arms or sends commands. Null
fields mean NOT_CONFIGURED, not usable measurements or deployment defaults.
"""
import argparse
import json
from pathlib import Path

from rescue_robot.task_site_runtime import atomic_json, InputStore


def prepare(directory,*,robot_id,namespace,settings=None):
    if not robot_id or not namespace.startswith('/') or namespace=='/' or '//' in namespace:
        raise ValueError('explicit logical robot_id and physical namespace required')
    root=Path(directory).resolve(); root.mkdir(parents=True,exist_ok=True)
    c=dict(format='rescue-robot-runtime/1',robot_id=robot_id,mission_frame=None,schema_version=None,
        publisher_id=None,clock_error_bound_ns=None,lease=dict(duration_s=None,max_age_ns=None,
        max_error_ns=None,clock_domain_id=None),mapping_authority=None,max_position_error_m=None,
        max_rotation_error_rad=None,input_max_age_ns=None,poll_period_s=None,
        input_file=str(root/'inputs.json'),endpoints=dict(action_name=f'/rescue/robots/{robot_id}/execute_task',
        lease_topic=f'/rescue/robots/{robot_id}/command_lease',admission_topic=f'/rescue/robots/{robot_id}/task_admission',
        status_topic=f'/rescue/robots/{robot_id}/status'))
    if settings:
        # Explicit supplied settings only. Keep identity/path/endpoints local to this robot.
        for k in ('mission_frame','schema_version','publisher_id','clock_error_bound_ns','lease',
                  'mapping_authority','max_position_error_m','max_rotation_error_rad','input_max_age_ns','poll_period_s'):
            if k in settings: c[k]=settings[k]
    missing=[]
    def visit(value,prefix=''):
        for key,v in value.items():
            name=prefix+key
            if isinstance(v,dict): visit(v,name+'.')
            elif v is None: missing.append(name)
    visit(c)
    if not missing: InputStore(c)  # structural policy/configuration check, not readiness
    atomic_json(root/'runtime.json',c)
    # Preserve live inputs during regeneration; do not reset/refill them.
    if not (root/'inputs.json').exists():
        atomic_json(root/'inputs.json',dict(format='rescue-robot-inputs/1',generated_at_ns=0,valid_until_ns=0,
            clock=dict(domain_id=None,state='UNKNOWN',error_bound_ns=None,epoch_id=None,valid_until_ns=0),
            authority={},maps={},targets={},approvals={},protected_routes={},destinations={}))
    params={'task.runtime_factory':'rescue_robot.task_site_runtime:create_runtime',
            'task.runtime_config':str(root/'runtime.json')}
    # JSON is also valid YAML. This overlays existing adapter parameters without
    # copying TEST_ONLY policy into them. ROS namespace stays a launch argument.
    atomic_json(root/'task.params.yaml',{'/**':{'ros__parameters':params}})
    report=dict(status='NOT_CONFIGURED' if missing else 'CONFIGURED_ONLY',missing=missing,
        logical_robot_id=robot_id,physical_namespace=namespace,robot_motion='NOT_RUN',
        watchdog_required_to_generate=False,
        commands=dict(run=f'ros2 run rescue_robot task_input --ros-args --params-file <existing-adapter-params.yaml> --params-file {root / "task.params.yaml"} -r __ns:={namespace}',
                      arm=f'ros2 service call {namespace}/rescue_command_adapter/task_arm std_srvs/srv/Trigger "{{}}"',
                      stop=f'ros2 service call {namespace}/rescue_command_adapter/task_stop std_srvs/srv/Trigger "{{}}"'))
    atomic_json(root/'preparation.json',report); return report


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',required=True); parser.add_argument('--robot-id',required=True)
    parser.add_argument('--namespace',required=True); parser.add_argument('--settings',help='explicit agreed runtime settings JSON')
    opt=parser.parse_args(args)
    settings=json.loads(Path(opt.settings).read_text()) if opt.settings else None
    report=prepare(opt.directory,robot_id=opt.robot_id,namespace=opt.namespace,settings=settings)
    print(json.dumps(report,ensure_ascii=False)); return 0 if report['status']=='CONFIGURED_ONLY' else 3


if __name__=='__main__': raise SystemExit(main())
