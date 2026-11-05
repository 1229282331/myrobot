"""Audit recorded formal-page MoveIt candidates, never device IO.

Raw controller poses provide the independent direction reference. Small
feedback quantization remains in each delta; summaries retain every cosine.
"""
import argparse
import base64
import bisect
from collections import Counter
import hashlib
import gzip
import json
import math
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation


def recorded_input(inputs,source):
    """Resolve a per-connection sequence using the shared X5 receipt timeline.

    A reload resets browser sequence numbers. Never overwrite earlier receipts,
    use a later connection's same-numbered input, or search by matching payload
    (which would conceal a corrupt source). The newest preceding receipt must
    itself match and remain within the existing 250 ms input freshness bound.
    """
    candidates=inputs.get(source['sequence'],[])
    before=[r for r in candidates if r['at_ns']<=source['at_ns']]
    if not before:return None
    latest=max(before,key=lambda r:r['at_ns'])
    if source['at_ns']-latest['at_ns']>250_000_000:return None
    return latest


def cpp_errors(candidate, raw, rows):
    """Validate explicit C++ provenance without inventing a ROS trajectory."""
    errors=[];c=candidate;src=c['source_input'];state=c.get('cpp') or {}
    if c.get('input_association')!='official_cpp_step_with_smoother_seed' or c.get('trajectory_sources')!=[]:
        errors.append('cpp_association')
    stamp=c['pre_action_received_ns']
    if stamp not in raw:errors.append('cpp_exact_observation_missing')
    elif not np.allclose(state.get('actual_q_rad',[]),raw[stamp][0],atol=1e-12,rtol=0):
        errors.append('cpp_actual_mismatch')
    if not (stamp<=c['received_ns']<=c['computed_ns']<c['received_ns']+50_000_000):
        errors.append('cpp_compute_times')
    expected=min(stamp+100_000_000,src['received_ns']+250_000_000)
    if c['valid_until_ns']!=expected or not 0<=c['received_ns']-src['received_ns']<=250_000_000:
        errors.append('cpp_original_deadlines')
    if state.get('period_s')!=.05:errors.append('cpp_period')
    parent=rows.get(state.get('parent_id'))
    effective=src.get('effective_intent',src['intent'])
    before_workspace=src['intent']
    ramp=src.get('input_ramp')
    if ramp:
        try:
            from copy import deepcopy
            tau=ramp['time_constant_s'];dt=ramp['period_s']
            assert ramp['kind']=='critically_damped_zoh' and .05<=tau<=1 and dt==.05
            prior=np.array(ramp['prior_state']);assert prior.shape==(2,12) and np.isfinite(prior).all()
            fields=('linear_m_s','angular_rad_s','joint_rad_s')
            raw_velocity=np.array([v for k in fields for v in src['intent'][k]])
            h=dt/tau;alpha=math.exp(-h)
            expected_state=np.array([[alpha,0],[h*alpha,alpha]])@prior+np.array([[1-alpha],[1-(1+h)*alpha]])@raw_velocity[None,:]
            assert np.allclose(expected_state,ramp['state'],atol=1e-12,rtol=0)
            if ramp['reset']:assert not prior.any()
            elif parent:
                assert np.allclose(prior,parent['source_input']['input_ramp']['state'],atol=1e-12,rtol=0)
            expected_intent=deepcopy(src['intent']);offset=0
            for field in fields:
                n=len(expected_intent[field]);expected_intent[field]=expected_state[1,offset:offset+n].tolist();offset+=n
            assert all(np.allclose(expected_intent[k],ramp['intent'][k],atol=1e-12,rtol=0) for k in fields)
            assert all(expected_intent[k]==ramp['intent'][k] for k in expected_intent if k not in fields)
            before_workspace=ramp['intent']
        except (AssertionError,ValueError,KeyError,TypeError):errors.append('cpp_input_ramp')
    damper=src.get('workspace_limit')
    if damper:
        from e6_phosphobot.workspace_velocity import damp_translation
        expected,_=damp_translation(before_workspace,damper['reference_position_m'],
            Rotation.from_quat(damper['reference_quaternion_xyzw']),damper['workspace'])
        if effective!=expected:errors.append('cpp_workspace_velocity')
    elif effective!=before_workspace:errors.append('cpp_unexplained_input_change')
    if state.get('reset'):
        if state.get('parent_id') is not None or state.get('seed_positions')!=state.get('actual_q_rad') or any(state.get('seed_velocities',[])):
            errors.append('cpp_reset')
    elif parent:
        if parent['epoch']!=c['epoch'] or parent['joints_rad']!=state['seed_positions'] or parent['cpp']['target_velocities']!=state['seed_velocities']:
            errors.append('cpp_parent')
    return errors


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--provenance-profile',choices=['legacy-single-point','jazzy-timed-window','cpp-step'],default='legacy-single-point',
                   help='Must match the recorded bridge protocol; never changes a production deadline')
    a=p.parse_args();root=Path(__file__).resolve().parents[1]
    sys.path[:0]=[str(root/'upstream/dobot-sdk'),str(root/'src')]
    from dobot_sdk.protocol.data_types import MyType
    sources={n:a.directory/(n if (a.directory/n).is_file() else n+'.gz') for n in ('raw.jsonl','events.jsonl')}
    def open_source(name):
        path=sources[name]
        return gzip.open(path,'rt',encoding='utf-8') if path.suffix=='.gz' else path.open(encoding='utf-8')
    raw={}
    for line in open_source('raw.jsonl'):
        r=json.loads(line);d=np.frombuffer(base64.b64decode(r['raw_b64']),dtype=MyType)[0]
        raw[r['received_ns']]=(np.deg2rad(d['QActual']),np.array(d['ToolVectorActual']))
    times=sorted(raw);inputs={};rows={};events=Counter();groups={};errors=[]
    for line in open_source('events.jsonl'):
        r=json.loads(line);events[r['event']]+=1
        if r['event']=='input' and 'intent' in r['data']:
            inputs.setdefault(r['data']['seq'],[]).append(r)
        if r['event']=='cycle':
            c=(r['state'].get('candidate') or {}).get('moveit')
            if c:rows.setdefault(c['id'],c)
    ages=[]
    for c in rows.values():
        source=c['source_input'];i=source['intent'];seq=source['sequence']
        recorded=recorded_input(inputs,source)
        if recorded is None or recorded['data']['intent']!=i:errors.append(['input_mismatch',c['id'],seq])
        if source['epoch']!=c['epoch']:errors.append(['epoch_mismatch',c['id']])
        age=c['received_ns']-source['at_ns'];ages.append(age/1e6)
        # Legacy output represented one immediately selected point. Jazzy
        # output is sampled from a timed rolling window; its recorded source
        # can predate the latest intent. Check the bound actually enforced in
        # jazzy_worker.sample (250 ms), and retain/report the full distribution.
        source_bound=250_000_000 if a.provenance_profile=='jazzy-timed-window' else 100_000_000
        if not 0<=age<=source_bound:errors.append(['source_age',c['id']])
        if a.provenance_profile=='jazzy-timed-window' and 'trajectory_sources' not in c:
            errors.append(['missing_window_provenance',c['id']])
        if a.provenance_profile=='cpp-step':
            errors.extend([error,c['id']] for error in cpp_errors(c,raw,rows))
            i=source.get('effective_intent',i)
        elif 'trajectory_sources' in c:
            if not c['bracket_ros_ns'][0]<=c['sample_ros_ns']<=c['bracket_ros_ns'][1]:
                errors.append(['sample_outside_bracket',c['id']])
            for point in c['trajectory_sources']:
                src=point['input']
                if point['kind']=='s':
                    if src is not None or point['input_ns']!=0:errors.append(['invalid_seed',c['id']])
                    continue
                recorded_point=recorded_input(inputs,src) if src else None
                if (not src or not recorded_point or recorded_point['data']['intent']!=src['intent']
                        or src['epoch']!=c['epoch'] or src['ros_stamp_ns']!=point['input_ns']):
                    errors.append(['window_input_mismatch',c['id'],point['input_ns']])
            if c['pre_action_received_ns']>c['received_ns']:errors.append(['late_observation',c['id']])
        stamp=source['observation_received_ns'];j=bisect.bisect_right(times,stamp)-1
        if j<0:errors.append(['missing_observation',c['id']]);continue
        q,pose=raw[times[j]];r0=Rotation.from_euler('xyz',pose[3:],degrees=True)
        if any(i['joint_rad_s']):
            kind='joint';expected=np.array(i['joint_rad_s']);delta=np.array(c['joints_rad'])-q
        else:
            kind='translation' if any(i['linear_m_s']) else 'rotation'
            expected=np.array(i['linear_m_s'] if kind=='translation' else i['angular_rad_s'])
            delta=np.array(c['position_m'])-pose[:3]/1000 if kind=='translation' else (Rotation.from_quat(c['quaternion_xyzw'])*r0.inv()).as_rotvec()
            if i['frame']=='tool':delta=r0.inv().apply(delta)
        norm=float(np.linalg.norm(delta));en=float(np.linalg.norm(expected))
        cosine=float(delta@expected/(norm*en)) if norm and en else 0
        key=(kind,i['frame'],tuple(np.sign(expected).astype(int)))
        group=groups.setdefault(str(key),dict(count=0,cosines=[],deltas=[],epochs=set()))
        group['count']+=1;group['cosines'].append(cosine);group['deltas'].append(delta.tolist());group['epochs'].add(c['epoch'])
    for g in groups.values():
        g['epochs']=sorted(g['epochs']);g['median_direction_cosine']=float(np.median(g['cosines']))
    result=dict(candidate_count=len(rows),case_count=len(groups),events=dict(events),errors=errors,cases=groups,
        unrecorded_cpp_parents=sum(bool(c.get('cpp',{}).get('parent_id') and c['cpp']['parent_id'] not in rows) for c in rows.values()),
        input_matching='Newest same-sequence receipt preceding source dispatch on X5; max age 250 ms; no payload search or future input substitution',
        provenance_profile=a.provenance_profile,selected_source_age_ms=np.percentile(ages,[50,95,99,100]).tolist() if ages else [],
        legacy_100ms_exceedances=sum(v>100 for v in ages),
        all_median_directions_pass=all(g['median_direction_cosine']>.97 for g in groups.values()),
        source_hashes={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in sources.values()},
        motion_is_proven=False,notes='Candidates are not accepted actions. Direction relative to native raw feedback; per-sample quantization retained.')
    result['audit_passed']=bool(rows) and not errors and result['all_median_directions_pass']
    a.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='cases'},ensure_ascii=False))
    print('minimum median cosine',min((g['median_direction_cosine'] for g in groups.values()),default=None))
    if not result['audit_passed']:raise SystemExit(1)


if __name__=='__main__':main()
