"""Independent original-packet/action/Stop audit; never contacts devices."""
import argparse
import base64
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.protocol import decode
from e6_observer.control_math import pose,angle


def audit_transaction(tick,action,run_passed):
    started=action.get('transaction_started_ns',tick['control_tick_monotonic_ns'])
    assert tick['control_tick_monotonic_ns']<=started<=action['gateway_sent_ns']<=action['gateway_accepted_ns']
    elapsed=action['gateway_accepted_ns']-started
    limits=(tick.get('status') or {}).get('limits') or {}
    hard_s=limits.get('transaction_s',.05)
    assert hard_s in (.05,.075), 'Unknown transaction budget'
    if elapsed<=round(hard_s*1e9):return None
    # A received ACK remains evidence of acceptance even when the overall
    # transaction missed its budget. Preserve that failed cycle explicitly;
    # it can never become a passing acceptance or a completed action.
    assert not run_passed
    assert action.get('stop_requested_after_acceptance') is True
    assert action.get('cycle_completed') is False
    assert tick['ik_result']['accepted'] is False
    return dict(command_id=action['command_id'],transaction_ms=elapsed/1e6,
                acceptance='controller_acknowledged_after_budget; stopped failed cycle')


def audit(directory):
    directory=Path(directory);summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    path=directory/'frames.jsonl';opener=open
    if not path.exists():path=path.with_suffix('.jsonl.gz');opener=gzip.open
    receives={};commands={};actions={};ticks=set();endpoints={};count=0;invalid=0;begin_ns=None;missed_budgets=[]
    with opener(path,'rt',encoding='utf-8') as stream:
        for line in stream:
            f=json.loads(line);count+=1;invalid+=int(not f['observation']['valid'])
            assert f['control_stage']=='live'
            s=f['sources']['robot'];raw=base64.b64decode(s['raw_base64'],validate=True);d=decode(raw,True)
            for name,value in d.items():
                if isinstance(value,list):np.testing.assert_allclose(value,s['decoded'][name],atol=1e-12,rtol=0)
                else:assert value==s['decoded'][name]
            receives[s['x5_receive_monotonic_ns']]=d
            control=f.get('control') or {};tick=control.get('tick') or {}
            if begin_ns is None:begin_ns=f['observation']['gateway_monotonic_ns']
            for event in control.get('command_events',[]):commands[event['x5_sent_ns']]=event
            identity=tick.get('control_tick_id')
            if identity is None or identity in ticks:continue
            ticks.add(identity)
            for event in tick.get('events',[]):
                if event['kind']=='servoj_endpoint_completed':endpoints[event['command_id']]=event
            a=tick.get('effective_action')
            if not a:continue
            assert a['command_id'] not in actions
            assert a['acceptance']=='controller_acknowledged_not_completion'
            assert tick['pre_action_robot']['x5_receive_monotonic_ns']<=tick['control_tick_monotonic_ns']<=a['gateway_sent_ns']<=a['gateway_accepted_ns']
            # The transaction includes integration, IK/FK and sending; the
            # observation tick also precedes health/pending-target checks.
            missed=audit_transaction(tick,a,summary['passed'])
            if missed:missed_budgets.append(missed)
            actions[a['command_id']]=a
    assert count==summary['frames'] and invalid==summary['invalid']
    assert len(actions)==summary['action_ticks']
    ordered=sorted(receives.items());stops=[];motion=[]
    for sent,event in sorted(commands.items()):
        # The shared gateway retains commands from separate earlier suites.
        if sent<begin_ns:continue
        if event['command'].startswith('ServoJ('):
            motion.append(event)
            if summary['passed']:assert event['acknowledged'] and not event.get('uncertain')
            if event.get('acknowledged'):
                command_id=int(event['raw_reply'].split('{',1)[1].split('}',1)[0]);assert command_id in actions
                assert (event['x5_reply_ns']-sent)<=round(summary['final_status'].get('limits',{}).get('transaction_s',.05)*1e9)
        elif event['command']=='Stop()':
            stop=dict(event);before=[r for r in ordered if r[0]<=sent]
            stationary=None;window=[]
            for ns,d in ordered:
                if ns<=sent:continue
                good=(d['robot_mode']==5 and d['enable_status']==1 and not d['error_status'] and not d['collision_state']
                    and not d['run_queued_command'] and np.max(np.abs(np.asarray(d['q_target_deg'])-d['q_actual_deg']))<=.05)
                window=(window+[(ns,d)])[-3:] if good else []
                if len(window)==3 and np.max(np.ptp([r[1]['q_actual_deg'] for r in window],axis=0))<=.01:
                    stationary=window[-1];break
            stop['stationary_feedback_receive_ns']=None if stationary is None else stationary[0]
            stop['request_to_stationary_bound_ms']=None if stationary is None else (stationary[0]-event.get('stop_requested_ns',sent))/1e6
            if before and stationary:
                q0=before[-1][1]['q_actual_deg'];q1=stationary[1]['q_actual_deg'];p0,r0=pose(before[-1][1]['controller_pose_raw_mm_deg']);p1,r1=pose(stationary[1]['controller_pose_raw_mm_deg'])
                stop['stationary_change_from_pre_send']=dict(max_joint_deg=float(np.max(np.abs(np.asarray(q1)-q0))),position_mm=float(1000*np.linalg.norm(p1-p0)),rotation_deg=math.degrees(angle(r0,r1)))
            if summary['passed']:
                assert event['send_confirmed'] and event['send_duration_ms']<=50 and event['acknowledged'] and stationary
                assert event['confirmation_duration_ms']<=1000
            stops.append(stop)
    def percentiles(v):return dict(zip(('p50','p95','p99','max'),np.percentile(v,[50,95,99,100]).tolist())) if v else {}
    result=dict(passed=bool(summary['passed']),retained_failed_run=not summary['passed'],raw_sdk_frames_checked=count,invalid=invalid,
        unique_ticks=len(ticks),accepted_action_ticks=len(actions),motion_commands=len(motion),stop_requests=len(stops),
        endpoint_completions=len(endpoints),tolerated_transaction_jitters=sum(bool(a.get('transaction_jitter_tolerated')) for a in actions.values()),motion_response_ms=percentiles([(e['x5_reply_ns']-e['x5_sent_ns'])/1e6 for e in motion if e.get('acknowledged')]),
        stop_send_ms=percentiles([e['send_duration_ms'] for e in stops if 'send_duration_ms' in e]),
        stop_confirmation_ms=percentiles([e['confirmation_duration_ms'] for e in stops if 'confirmation_duration_ms' in e]),
        stops=stops,missed_transaction_budgets=missed_budgets,audit_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),synthetic_inputs_only=True,physical_gamepad_acceptance_required=True,
        stationary_measurement_semantics='three sampled distinct controller receives after request; bounds, not physical brake timing')
    (directory/'frame_readback.json').write_text(json.dumps(result,indent=2),encoding='utf-8');return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
    result=audit(a.directory);print(json.dumps({k:v for k,v in result.items() if k!='stops'},indent=2))
