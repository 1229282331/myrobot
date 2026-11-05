"""Offline separation of page, sent, controller-target and actual rotation.

Consumes analyze_servoj_following output with original X5 times. A fitted time
shift is a waveform diagnostic, not causal network latency or acceptance data.
No device connections, trajectory generation or controller parameter changes.
"""
import argparse
import csv
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


def fit_shift(source_t, source_y, target_t, target_y, start, end):
    # Keep the same comparison samples for every shift, retain an explicitly
    # reported constant offset, and never use this fit to alter acceptance times.
    times=np.arange(start,end,.01)
    observed=np.interp(times,target_t,target_y)
    options=[]
    for lag in np.arange(-.5,.751,.001):
        difference=observed-np.interp(times-lag,source_t,source_y)
        offset=float(np.median(difference))
        rmse=float(np.sqrt(np.mean((difference-offset)**2)))
        options.append((rmse,float(lag),offset))
    rmse,lag,offset=min(options)
    return dict(best_shift_ms=lag*1000,constant_offset_deg=offset,rmse_deg=rmse,
                samples=len(times),fitting_window_s=[start,end],
                positive_shift='target waveform occurs later than source',
                not_latency_percentile=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('evidence',type=Path);p.add_argument('tracking',type=Path)
    p.add_argument('--group-index',type=int,default=0)
    a=p.parse_args()
    phases_path=a.evidence/'continuous.json'
    phases=json.loads(phases_path.read_text(encoding='utf8'))
    tracking_path=a.tracking/'analysis.json'
    tracking=json.loads(tracking_path.read_text(encoding='utf8'))
    assert tracking['phase_report_sha256']==sha(phases_path)
    assert len(phases['groups'])==len(tracking['groups'])
    assert 0<=a.group_index<len(phases['groups'])
    group=phases['groups'][a.group_index];g=tracking['groups'][a.group_index]
    csv_path=a.tracking/g['raw_csv']['file']
    assert sha(csv_path)==g['raw_csv']['sha256']
    with csv_path.open(newline='',encoding='utf8') as f:
        data=list(csv.DictReader(f))
    values=lambda name:np.array([float(r[name]) for r in data])
    times=values('since_input_ms')/1000
    base=Rotation.from_quat(group['base']['quaternion_xyzw'])
    vector=np.array(group['phases'][0]['intent']['angular_rad_s'])
    axis=vector/np.linalg.norm(vector)
    assert all(x['intent']['frame']=='base' and x['kind']=='rotation' for x in group['phases'])
    angles={}
    for name in ('ToolVectorTarget','ToolVectorActual'):
        angles[name]=(Rotation.from_euler('xyz',np.array([values(name+'_'+str(i)) for i in (4,5,6)]).T,
                                          degrees=True)*base.inv()).as_rotvec(degrees=True)@axis
    commands=g['commands']
    sent_t=np.array([c['since_input_ms']/1000 for c in commands])
    sent_j=np.array([c['joints_deg'] for c in commands])
    joint=int(np.argmax(np.ptp(sent_j,axis=0)))+1
    internal=values('QTarget_'+str(joint));actual=values('QActual_'+str(joint))
    end=group['duration_ms']/1000
    compare=[1.,end-.75]
    result=dict(hardware_connected=False,group_index=a.group_index,selected_joint=joint,
        sent_to_internal=fit_shift(sent_t,sent_j[:,joint-1],times,internal,*compare),
        internal_to_actual=fit_shift(times,internal,times,actual,*compare),
        sent_to_actual=fit_shift(sent_t,sent_j[:,joint-1],times,actual,*compare),
        max_actual_orientation_deg=float(np.max(np.abs(angles['ToolVectorActual']))),
        actual_csv_tail_orientation_projection_deg=float(np.median(angles['ToolVectorActual'][-12:])),
        csv_ends_after_release_ms=float((times[-1]-end)*1000),
        notes='Shape fits only, fixed window and explicit offset. No time-shifted speed acceptance. '
              'Raw ToolVectorTarget is retained; no vendor field replaced by an inferred value.',
        source_hashes=dict(continuous=sha(phases_path),tracking=sha(tracking_path),raw_csv=sha(csv_path)),
        script_sha256=sha(Path(__file__)))
    polarity=float(np.sign(sent_j[min(3,len(sent_j)-1),joint-1]-sent_j[0,joint-1]))
    assert polarity, 'Selected joint initial direction is unresolved'
    result['joint_polarity_relative_to_initial_rotation']=polarity
    reversals=[]
    for index,phase in enumerate(group['phases'][1:-1],1):
        at=(phase['start_ns']-group['start_ns'])/1e9
        direction=float(np.array(phase['intent']['angular_rad_s'])@axis)
        peaks={}
        for name,t,y in (('sent',sent_t,sent_j[:,joint-1]),('internal',times,internal),('actual',times,actual)):
            selected=np.flatnonzero((t>=at)&(t<at+.8))
            # Use the selected joint's measured initial sign; this is a turn
            # extremum, not a filtered velocity trace or first-response time.
            point=selected[np.argmin(y[selected]) if direction*polarity>0 else np.argmax(y[selected])]
            peaks[name]=dict(since_input_ms=float((t[point]-at)*1000),sample_time_s=float(t[point]),
                surrounding_sample_spacing_ms=float((t[min(point+1,len(t)-1)]-t[max(point-1,0)])*1000))
        reversals.append(dict(phase=index,input_received_s=at,turns=peaks))
    result['reversal_extrema']=reversals
    result['reversal_notes']='First extremum within 800ms after each full reversal; original samples, no smoothing. '
    result['reversal_notes']+='Sampling spacing is retained. Extrema are turning points, not first-response latency.'
    phase_t=[(x['start_ns']-group['start_ns'])/1e9 for x in group['phases']]+[end]
    rate=[float(np.rad2deg(x['intent']['angular_rad_s'])@axis) for x in group['phases']]+[0.]
    fig,axs=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
    axs[0].step(phase_t,rate,where='post',label='Page intent received at X5')
    axs[0].set_ylabel('Requested deg/s')
    axs[1].plot(times,angles['ToolVectorTarget'],label='Controller ToolVectorTarget')
    axs[1].plot(times,angles['ToolVectorActual'],label='Controller ToolVectorActual')
    axs[1].set_ylabel('Base rotation projection (deg)')
    axs[2].plot(sent_t,sent_j[:,joint-1],'.-',label='Sent ServoJ target',markersize=3)
    axs[2].plot(times,internal,label='Controller internal QTarget')
    axs[2].plot(times,actual,label='Controller QActual')
    axs[2].set_ylabel(f'Joint {joint} (deg)');axs[2].set_xlabel('Seconds since X5 received active input')
    for ax in axs:
        ax.axvline(end,color='#555',linestyle=':',label='Release received')
        ax.grid(alpha=.2);ax.legend(loc='best')
    speed=float(np.linalg.norm(np.rad2deg(vector)))
    fig.suptitle(f'Actual {speed:g} deg/s input trial, group {a.group_index}: original unshifted waveforms\nDiagnostic CSV ends 500ms after release; complete raw evidence retained separately')
    suffix='' if a.group_index==0 else '-group-'+str(a.group_index)
    fig.savefig(a.tracking/('rotation-following'+suffix+'.png'),dpi=140)
    (a.tracking/('waveform-diagnosis'+suffix+'.json')).write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result))


if __name__=='__main__':main()
