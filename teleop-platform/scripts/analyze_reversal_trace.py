"""Compare isolated production-bridge runs, never hardware acceptance evidence."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def slope(t, y, start, end):
    selected=(t>=start)&(t<end)
    if np.sum(selected)<3:
        return None
    return float(np.polyfit(t[selected],y[selected],1)[0])


def analyze(folder, fixture):
    group=fixture['groups'][0]
    origin=group['start_ns']
    phase_times=[(p['start_ns']-origin)/1e9 for p in group['phases']]
    end=group['duration_ms']/1000
    phase_ends=[(p['end_ns']-origin)/1e9 for p in group['phases']]
    axis=np.array(group['phases'][0]['intent']['angular_rad_s'])
    axis/=np.linalg.norm(axis)
    summary=json.loads((folder/'summary.json').read_text())
    data=json.loads((folder/'numeric.json').read_text())
    assert summary['completed'] and summary['error'] is None and not summary['forbidden_imports']
    candidate_t=np.array([c['at_s'] for c in data['candidates']])
    candidate_q=np.rad2deg([c['candidate']['joints_rad'] for c in data['candidates']])
    candidate_angle=(Rotation.from_quat([c['candidate']['quaternion_xyzw'] for c in data['candidates']])*
                     Rotation.from_quat(group['base']['quaternion_xyzw']).inv()).as_rotvec(degrees=True)@axis
    actual_t=np.array([s['t_s'] for s in data['samples']])
    actual_angle=np.array([s['angle_deg'] for s in data['samples']])@axis
    actual_q=np.rad2deg([s['q_rad'] for s in data['samples']])
    joint=int(np.argmax(np.ptp(candidate_q,axis=0)))
    polarity=np.sign(candidate_q[min(3,len(candidate_q)-1),joint]-candidate_q[0,joint])
    assert polarity
    phases=[]
    for i,(start,stop) in enumerate(zip(phase_times,phase_ends)):
        direction=float(np.array(group['phases'][i]['intent']['angular_rad_s'])@axis)
        dispatch=next(d for d in data['dispatch'] if d['phase']==i)
        adopted=next((c for c in data['candidates'] if
                      c['candidate']['source_input']['sequence']>=dispatch['sequence']),None)
        row=dict(phase=i,start_s=start,end_s=stop,requested_deg_s=float(np.rad2deg(direction)),
                 dispatch_after_phase_ms=(dispatch['at_s']-start)*1000,
                 new_source_candidate_after_dispatch_ms=(adopted['at_s']-dispatch['at_s'])*1000 if adopted else None,
                 slopes={},turns={})
        for name,t,y,q in [('candidate',candidate_t,candidate_angle,candidate_q),
                           ('virtual_actual',actual_t,actual_angle,actual_q)]:
            row['slopes'][name]=dict(whole_deg_s=slope(t,y,start,stop),
                                     middle_deg_s=slope(t,y,start+.35,stop-.1))
            if 0<i<len(phase_times)-1:
                selected=np.flatnonzero((t>=start)&(t<min(stop,start+.8)))
                k=selected[np.argmin(q[selected,joint]) if direction*polarity>0 else np.argmax(q[selected,joint])]
                row['turns'][name]=dict(after_phase_ms=float((t[k]-start)*1000),
                    after_dispatch_ms=float((t[k]-dispatch['at_s'])*1000),
                    surrounding_sample_spacing_ms=float((t[min(k+1,len(t)-1)]-t[max(k-1,0)])*1000))
        phases.append(row)
    result=dict(label=folder.name,hardware_acceptance=False,virtual_tau_s=summary['virtual_tau_s'],
        joint_acceleration_deg_s2=summary['profile']['solver_joint_acceleration_deg_s2'],
        source_sha256=dict(numeric=sha(folder/'numeric.json'),summary=sha(folder/'summary.json')),
        first_candidate_ms=summary['first_candidate_s']*1000,selected_joint=joint+1,
        phase_metrics=phases,drift_candidate_deg=float(candidate_angle[-1]),drift_virtual_actual_deg=float(actual_angle[-1]),
        middle_window='Original phase start +350ms through phase end -100ms; half phases retained.',
        turn_method='First selected-joint extremum within 800ms of reversal, not first-motion latency.',
        fixture_stop='Instant virtual freeze, not an E6 stop-time measurement.',
        candidate_turn_after_dispatch_ms_p50_p95_max=np.percentile(
            [p['turns']['candidate']['after_dispatch_ms'] for p in phases[1:-1]],[50,95,100]).tolist(),
        middle_full_phase_candidate_abs_deg_s_range=[float(min(abs(p['slopes']['candidate']['middle_deg_s']) for p in phases[1:-1])),
                                                    float(max(abs(p['slopes']['candidate']['middle_deg_s']) for p in phases[1:-1]))],
        middle_full_phase_actual_abs_deg_s_range=[float(min(abs(p['slopes']['virtual_actual']['middle_deg_s']) for p in phases[1:-1])),
                                                 float(max(abs(p['slopes']['virtual_actual']['middle_deg_s']) for p in phases[1:-1]))])
    return result,(candidate_t,candidate_angle,actual_t,actual_angle)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fixture',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('runs',nargs='+',type=Path)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    fixture=json.loads(args.fixture.read_text(encoding='utf8'))
    fig,axs=plt.subplots(len(args.runs)+1,1,figsize=(12,3*(len(args.runs)+1)),sharex=True,layout='constrained')
    group=fixture['groups'][0];base=group['start_ns']
    times=[(x['start_ns']-base)/1e9 for x in group['phases']]+[group['duration_ms']/1000]
    rates=[float(np.rad2deg(x['intent']['angular_rad_s'][2])) for x in group['phases']]+[0]
    axs[0].step(times,rates,where='post');axs[0].set_ylabel('Intent deg/s')
    results=[]
    for ax,folder in zip(axs[1:],args.runs):
        summary=json.loads((folder/'summary.json').read_text())
        # Deployment retained CRLF->LF transport normalization in source.json.
        # Verify those exact executed bytes and retain the original file hash.
        executed_hash=hashlib.sha256(args.fixture.read_bytes().replace(b'\r\n',b'\n')).hexdigest()
        assert summary['fixture_sha256']==executed_hash
        manifest=json.loads((folder/'source.json').read_text())
        assert manifest['files']['fixture.json']['executed_sha256']==executed_hash
        result,values=analyze(folder,fixture);results.append(result)
        ct,cy,at,ay=values
        ax.plot(ct,cy,'.-',label='20Hz selected candidate',markersize=2)
        ax.plot(at,ay,label='Virtual plant')
        ax.set_title(folder.name);ax.set_ylabel('Rotation deg');ax.legend()
        for t in times:ax.axvline(t,color='#888',alpha=.2)
    axs[-1].set_xlabel('Seconds since fixture begins (unshifted)')
    fig.suptitle('Isolated MoveIt bridge: recorded intent phases, newly measured dispatch timing\nVirtual plant is an assumption, not an identified E6 model')
    fig.savefig(args.output/'comparison.png',dpi=140)
    (args.output/'analysis.json').write_text(json.dumps(dict(fixture_sha256=sha(args.fixture),
        executed_fixture_lf_sha256=executed_hash,
        script_sha256=sha(Path(__file__)),runs=results),indent=2),encoding='utf8')
    print(json.dumps([{k:v for k,v in r.items() if k!='phase_metrics'} for r in results],indent=2))


if __name__=='__main__':main()
