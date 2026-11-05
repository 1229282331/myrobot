"""Opt-in single J6 viewing step; fixed baseline, <=2 deg/step, +/-5 deg total.

No enable, disable, IO, gripper operation, retry or automatic return. The caller
reviews physical clearance and a new camera image before every invocation.
Existing commissioning-step limits in e6_step.py remain unchanged.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import time

try:
    from .e6_zero import Dashboard, E6Error, read_state, validate_joints
    from .e6_step import check_envelope, move_command
except ImportError:
    from e6_zero import Dashboard, E6Error, read_state, validate_joints
    from e6_step import check_envelope, move_command


def plan(baseline, start_offset, target_offset):
    validate_joints(baseline)
    if not all(math.isfinite(x) for x in (start_offset,target_offset)):
        raise ValueError('Non-finite offset')
    if max(abs(start_offset),abs(target_offset))>5 or not 0<abs(target_offset-start_offset)<=2:
        raise ValueError('Require J6 step <=2 deg and fixed-baseline offsets within +/-5 deg')
    start,target=list(baseline),list(baseline)
    start[5]+=start_offset;target[5]+=target_offset
    validate_joints(start);validate_joints(target)
    return start,target


def run(robot,start,target,report,execute=False,clock=time.monotonic,sleep=time.sleep):
    report.update(expected_start=start,target=target,states=[],command_log=[],
                  motion_command=move_command(target),automatic_disable=False)
    def sample():
        state=read_state(robot,clock)
        report['states'].append({'host_monotonic_ns':time.monotonic_ns(),
                                'mode':state.mode,'joints_deg':list(state.joints)})
        return state
    def command(value):
        entry={'command':value,'host_monotonic_ns':time.monotonic_ns()}
        report['command_log'].append(entry)
        entry['payload']=robot.command(value)
        entry['reply_received_monotonic_ns']=time.monotonic_ns()
        return entry['payload']
    def require_start(state):
        if state.mode!=5 or max(abs(a-b) for a,b in zip(state.joints,start))>.15:
            raise E6Error('Require enabled idle robot at reviewed start; no recovery')
    require_start(sample())
    if not execute:return 'preview'
    require_start(sample())
    try:
        queue_id=command(move_command(target))
        if not re.fullmatch(r'\d+',queue_id):raise E6Error('Invalid queue ID')
        report['queue_id']=int(queue_id)
        sent,stable=clock(),0
        while True:
            state=sample()
            if state.mode not in (5,7):raise E6Error(f'Unexpected mode {state.mode}')
            check_envelope(start,target,state.joints)
            error=max(abs(a-b) for a,b in zip(state.joints,target))
            stable=stable+1 if state.mode==5 and error<=.05 else 0
            if stable>=5:
                report.update(motion_elapsed_s=clock()-sent,arrival_max_error_deg=error)
                return 'reached'
            if clock()-sent>15:raise E6Error('15s arrival timeout')
            sleep(.1)
    except BaseException:
        try:
            command('Stop()');report['stop_acknowledged']=True
        except BaseException as exc:report['stop_error']=repr(exc)
        raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--start-offset',type=float,required=True)
    p.add_argument('--target-offset',type=float,required=True)
    p.add_argument('--scene-report',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--ip',default='192.168.1.4')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    data=json.loads(a.baseline.read_text(encoding='utf-8'))
    if data.get('error') or not data.get('frames'):p.error('Require a successful passive baseline')
    baseline=data['frames'][-1]['decoded']['q_actual_deg']
    for frame in data['frames']:
        d=frame['decoded']
        if d['robot_mode']!=5 or d['enable_status']!=1 or d['error_status'] or max(abs(x-y) for x,y in zip(d['q_actual_deg'],baseline))>.15:
            p.error('Require stable enabled alarm-free baseline')
    try:start,target=plan(baseline,a.start_offset,a.target_offset)
    except ValueError as exc:p.error(str(exc))
    scene=json.loads(a.scene_report.read_text(encoding='utf-8'))
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(scene['ended_at_utc'])).total_seconds()
    if scene.get('status')!='completed' or any(scene.get(k) for k in ['cleanup_errors','processing_errors','receiver_errors']) or not 0<=age<=180:
        p.error('Require successful scene capture within 180 seconds')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x',encoding='utf-8') as out:
        report={'started_utc':datetime.now(timezone.utc).isoformat(),'execute':a.execute,
                'baseline_file':str(a.baseline),'scene_report':str(a.scene_report),
                'scene_age_s':age,'start_offset_deg':a.start_offset,
                'target_offset_deg':a.target_offset,'status':'starting',
                'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'baseline_sha256':hashlib.sha256(a.baseline.read_bytes()).hexdigest()}
        try:
            with Dashboard(a.ip,timeout=.5) as robot:report['result']=run(robot,start,target,report,a.execute)
            report['status']='completed'
        except BaseException as exc:report.update(status='failed',error=repr(exc))
        report['ended_utc']=datetime.now(timezone.utc).isoformat()
        json.dump(report,out,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k!='states'},indent=2))
    return int(report['status']!='completed')


if __name__=='__main__':raise SystemExit(main())
