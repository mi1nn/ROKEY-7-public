"""Read-only chrony measurements and a two-host clock basis, without ROS.

Accuracy estimate: |System time| + Root dispersion + |Root delay| / 2.
Source: https://chrony-project.org/doc/4.5/chronyc.html#tracking
This assumes the agreed upstream time reference is correct. A domain name is
deployment policy, never inferred from a successful command or matching names.
No makestep, config change, sudo, service restart or motion command is issued.
"""
import argparse
from dataclasses import asdict
from decimal import Decimal, InvalidOperation, ROUND_CEILING
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

from rescue_robot.executor import ContractError
from rescue_robot.task_policy import ClockBasis
from rescue_robot.task_site_runtime import atomic_json, positive_ns


def parse_tracking(raw):
    result={'state':'UNKNOWN','error_bound_ns':None}
    try:
        fields=dict(line.split(':',1) for line in raw.splitlines() if ':' in line)
        fields={k.strip():v.strip() for k,v in fields.items()}
        ref=fields['Reference ID'].split()[0]
        stratum=int(fields['Stratum'])
        if fields['Leap status']!='Normal' or ref in ('7F7F0101','00000000') or not 0<stratum<16:
            return result
        correction=abs(Decimal(fields['System time'].split()[0]))
        delay=abs(Decimal(fields['Root delay'].split()[0]))
        dispersion=Decimal(fields['Root dispersion'].split()[0])
        if not all(v.is_finite() for v in (correction,delay,dispersion)) or dispersion<0: return result
        error=(correction+dispersion+delay/2)*Decimal(10**9)
        result.update(state='SYNCED',error_bound_ns=int(error.to_integral_value(rounding=ROUND_CEILING)),
                      reference_id=ref,stratum=stratum)
    except (KeyError,ValueError,InvalidOperation,IndexError,AttributeError): pass
    return result


def collect(host_id,domain_id,*,run=subprocess.run,now_ns=time.time_ns):
    if not host_id or not domain_id: raise ContractError('explicit host and clock domain required')
    raw={}
    # These commands only read; retain exit codes and raw output on failures.
    commands={'tracking':['chronyc','-n','tracking'], 'sources':['chronyc','-n','sources','-v'],
              'chronyd_process':['ps','-C','chronyd','-o','pid=,lstart=']}
    for key,cmd in commands.items():
        try:
            reply=run(cmd,capture_output=True,text=True,timeout=5,check=False,
                      env={**os.environ,'LC_ALL':'C'})
            raw[key]={'command':cmd,'exit_code':reply.returncode,'stdout':reply.stdout,'stderr':reply.stderr}
        except (OSError,subprocess.TimeoutExpired) as exc:
            raw[key]={'command':cmd,'exit_code':None,'stdout':'','stderr':str(exc)}
    try: boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError: boot=''
    parsed=parse_tracking(raw['tracking']['stdout']) if raw['tracking']['exit_code']==0 else {'state':'UNKNOWN','error_bound_ns':None}
    process=raw['chronyd_process']['stdout'].strip()
    if not boot or not process or raw['chronyd_process']['exit_code']!=0:
        parsed={'state':'UNKNOWN','error_bound_ns':None}
    return dict(parsed,format='rescue-clock-measurement/1',host_id=host_id,host_boot_id=boot,
                clock_session_id=hashlib.sha256((boot+'\n'+process).encode()).hexdigest(),
                clock_domain_id=domain_id,measured_at_ns=now_ns(),raw=raw)


def combine_measurements(sender,receiver,*,domain_id,max_age_ns,max_error_ns,now_ns):
    positive_ns(max_age_ns,'clock measurement max_age_ns'); positive_ns(max_error_ns,'clock max_error_ns')
    if sender['host_id']==receiver['host_id']: raise ContractError('two distinct hosts required','CLOCK_UNCERTAIN')
    for m in (sender,receiver):
        error=m.get('error_bound_ns'); stamp=m.get('measured_at_ns')
        if (m.get('state')!='SYNCED' or m.get('clock_domain_id')!=domain_id or not m.get('host_boot_id')
                or type(error) is not int or error<0 or type(stamp) is not int
                or stamp>now_ns or now_ns>=stamp+max_age_ns):
            raise ContractError('two fresh synchronized clock measurements required','CLOCK_UNCERTAIN')
    error=sender['error_bound_ns']+receiver['error_bound_ns']
    if error>max_error_ns: raise ContractError('combined clock error exceeds policy','CLOCK_UNCERTAIN')
    epoch=hashlib.sha256(json.dumps([(m['host_id'],m['host_boot_id'],m.get('clock_session_id'))
                                  for m in (sender,receiver)],sort_keys=True).encode()).hexdigest()
    return ClockBasis(domain_id,'SYNCED',error,epoch,
                      min(m['measured_at_ns']+max_age_ns for m in (sender,receiver)))


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    capture=sub.add_parser('capture'); capture.add_argument('--host-id',required=True)
    capture.add_argument('--clock-domain',required=True); capture.add_argument('--output',required=True)
    capture.add_argument('--interval-s',type=float,help='repeat locally until Ctrl-C; no remote commands')
    combine=sub.add_parser('combine')
    for name in ('sender','receiver','clock-domain','max-age-ns','max-error-ns','output'):
        combine.add_argument('--'+name,required=True,type=int if name in ('max-age-ns','max-error-ns') else str)
    opt=parser.parse_args(args)
    if opt.command=='capture':
        if opt.interval_s is not None and (not math.isfinite(opt.interval_s) or not 0<opt.interval_s<=60):
            parser.error('interval must be positive and at most 60 seconds')
        try:
            while True:
                measurement=collect(opt.host_id,opt.clock_domain); atomic_json(opt.output,measurement)
                print(json.dumps({k:measurement[k] for k in ('state','error_bound_ns','measured_at_ns')}),flush=True)
                if opt.interval_s is None: return 0 if measurement['state']=='SYNCED' else 2
                time.sleep(opt.interval_s)
        except KeyboardInterrupt: return 0
    try:
        sender=json.loads(Path(opt.sender).read_text()); receiver=json.loads(Path(opt.receiver).read_text())
        basis=combine_measurements(sender,receiver,domain_id=opt.clock_domain,max_age_ns=opt.max_age_ns,
                                   max_error_ns=opt.max_error_ns,now_ns=time.time_ns())
        atomic_json(opt.output,asdict(basis)); print(json.dumps(asdict(basis))); return 0
    except (OSError,ValueError,KeyError,ContractError) as exc:
        print(json.dumps({'state':'UNKNOWN','error':str(exc)})); return 2


if __name__=='__main__': raise SystemExit(main())
