"""One bounded E6 commissioning step; default is a read-only preview.

Enable-and-hold or a single joint step/return relative to a saved passive
baseline. J1-J5 are restricted to +0.25 degree; J6 permits +0.25 or +1 degree.
No disable, IO, error clearing, payload changes, retries or reconnect.
The caller must establish physical clearance and emergency-stop access.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import time

try:
    from .e6_zero import Dashboard, E6Error, read_state, require_idle, validate_joints
except ImportError:
    from e6_zero import Dashboard, E6Error, read_state, require_idle, validate_joints

START_TOLERANCE = .15
ARRIVAL_TOLERANCE = .05
ENVELOPE_MARGIN = .15


def expected_joints(baseline, action, axis=6, delta=1.0):
    if type(axis) is not int or not 1 <= axis <= 6 or delta not in (.25,1.0):
        raise ValueError('Require one joint and a fixed bounded positive step')
    if axis != 6 and delta != .25:
        raise ValueError('J1-J5 commissioning is restricted to 0.25 degree')
    validate_joints(baseline)
    start = tuple(baseline)
    target = list(baseline)
    if action == 'out':
        target[axis-1] += delta
    elif action == 'return':
        values=list(start)
        values[axis-1] += delta
        start=tuple(values)
    elif action != 'enable':
        raise ValueError('Unsupported step')
    validate_joints(start)
    validate_joints(target)
    return start, tuple(target)


def check_envelope(start, target, actual, margin=ENVELOPE_MARGIN):
    validate_joints(actual)
    for i, (a, b, q) in enumerate(zip(start, target, actual), 1):
        if not min(a, b)-margin <= q <= max(a, b)+margin:
            raise E6Error(f'J{i}={q:.4f} outside this step envelope')


def move_command(target):
    validate_joints(target)
    values = ','.join(f'{x:.6f}' for x in target)
    return f'MovJ(joint={{{values}}},a=1,v=1,cp=0)'


def run_step(robot, baseline, action, report, execute=False, clock=time.monotonic, sleep=time.sleep, axis=6, delta=1.0):
    start, target = expected_joints(baseline, action, axis, delta)
    report.update(expected_start=list(start), target=list(target), action=action,
                  axis=axis, delta_deg=delta,
                  motion_command=None if action == 'enable' else move_command(target),
                  states=[], command_log=[], automatic_disable=False)

    def sample():
        state = read_state(robot, clock)
        report['states'].append({'host_monotonic_ns':time.monotonic_ns(),
                                 'mode':state.mode,'joints_deg':list(state.joints)})
        return state

    def command(value, **kw):
        entry={'command':value,'host_monotonic_ns':time.monotonic_ns()}
        report['command_log'].append(entry)
        entry['payload']=robot.command(value, **kw)
        entry['reply_received_monotonic_ns']=time.monotonic_ns()
        return entry['payload']

    current=sample()
    require_idle(current)
    if max(abs(a-b) for a,b in zip(current.joints,start)) > START_TOLERANCE:
        raise E6Error('Current pose changed from reviewed baseline; refuse action')
    if not execute:
        return 'preview'
    if action != 'enable' and current.mode != 5:
        raise E6Error('Motion requires enabled idle mode 5; no automatic enabling')
    attempted=False
    try:
        if action == 'enable':
            if current.mode == 4:
                attempted=True
                command('EnableRobot()',timeout=15)
            deadline=clock()+10
            hold_started=None
            while True:
                current=sample()
                if current.mode not in (4,5):
                    raise E6Error(f'Unexpected enable mode {current.mode}')
                check_envelope(start,target,current.joints)
                if current.mode == 5:
                    if hold_started is None:
                        hold_started=clock()
                    if clock()-hold_started >= 5:
                        report['hold_seconds']=clock()-hold_started
                        return 'enabled_and_held'
                elif hold_started is not None:
                    raise E6Error('Enable lost during hold; no recovery')
                if clock() >= deadline:
                    raise E6Error('Enable/hold timeout')
                sleep(.1)
        current=sample()
        if current.mode != 5 or max(abs(a-b) for a,b in zip(current.joints,start)) > START_TOLERANCE:
            raise E6Error('Pose or mode changed immediately before motion')
        attempted=True
        queue_id=command(move_command(target))
        if not re.fullmatch(r'\d+',queue_id):
            raise E6Error('Invalid command queue ID; no retry')
        report['queue_id']=int(queue_id)
        sent=clock()
        stable=0
        while True:
            current=sample()
            if current.mode not in (5,7):
                raise E6Error(f'Unexpected motion mode {current.mode}')
            check_envelope(start,target,current.joints)
            error=max(abs(a-b) for a,b in zip(current.joints,target))
            stable=stable+1 if current.mode==5 and error <= ARRIVAL_TOLERANCE else 0
            if stable >= 5:
                report.update(motion_elapsed_s=clock()-sent,arrival_max_error_deg=error)
                return 'reached'
            if clock()-sent > 15:
                raise E6Error('15s arrival timeout; no retry')
            sleep(.1)
    except BaseException:
        if attempted:
            try:
                command('Stop()')
                report['stop_acknowledged']=True
            except BaseException as exc:
                report['stop_error']=repr(exc)
        raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--action',choices=['enable','out','return'],required=True)
    p.add_argument('--axis',type=int,choices=range(1,7),default=6)
    p.add_argument('--delta',type=float,choices=[.25,1.0],default=1.0)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--ip',default='192.168.1.4')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.axis!=6 and a.delta!=.25:
        p.error('J1-J5 only permit --delta 0.25')
    data=json.loads(a.baseline.read_text(encoding='utf-8'))
    if data.get('error') or not data.get('frames'):
        p.error('Baseline must be a successful passive payload capture')
    baseline=data['frames'][-1]['decoded']['q_actual_deg']
    # Check the whole baseline capture, not just an isolated last sample.
    for f in data['frames']:
        d=f['decoded']
        if d['error_status'] or d['robot_mode'] not in (4,5) or max(abs(x-y) for x,y in zip(d['q_actual_deg'],baseline)) > .15:
            p.error('Baseline is not idle, alarm-free and stable')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x',encoding='utf-8') as out:
        report={'started_utc':datetime.now(timezone.utc).isoformat(),'robot_ip':a.ip,
                'execute':a.execute,'baseline_file':str(a.baseline),
                'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'baseline_sha256':hashlib.sha256(a.baseline.read_bytes()).hexdigest(),
                'status':'starting'}
        try:
            with Dashboard(a.ip,timeout=.5) as robot:
                report['result']=run_step(robot,baseline,a.action,report,a.execute,axis=a.axis,delta=a.delta)
            report['status']='completed'
        except BaseException as exc:
            report.update(status='failed',error=repr(exc))
        report['ended_utc']=datetime.now(timezone.utc).isoformat()
        json.dump(report,out,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k not in ('states',)},indent=2))
    return int(report['status']!='completed')


if __name__=='__main__':
    raise SystemExit(main())
