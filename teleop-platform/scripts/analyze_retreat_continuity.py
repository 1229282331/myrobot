"""Plot isolated delayed-feedback comparisons; no robot IO."""
from pathlib import Path
import argparse,json,collections
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from e6_phosphobot.tool_frame import get_tool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('root',type=Path)
    ap.add_argument('--baseline',default='attempt-b/continuity-b-baseline')
    ap.add_argument('--candidate',default='attempt-d/continuity-d-candidate')
    ap.add_argument('--cases',nargs='*')
    ap.add_argument('--prefix',default='')
    ap.add_argument('--baseline-budget-ms',type=float,default=35)
    a=ap.parse_args();root=a.root
    groups={};tool=get_tool('omnipicker-r4')
    for version in ('baseline','candidate'):
        rows=json.loads((root/getattr(a,version)/'frames.json').read_text());g=collections.defaultdict(list)
        for row in rows:
            if row['reply']['active']:g[row['case']].append(row)
        groups[version]=g
    names=a.cases or ['original-near-base-0.006-6','recorded-epoch95-0.006-6','recorded-epoch95-0.015-6']
    summaries=[]
    for name in names:
        fig,ax=plt.subplots(3,2,figsize=(12,8),sharex=True,layout='constrained')
        for col,version in enumerate(('baseline','candidate')):
            rows=groups[version][name];t=np.arange(len(rows))*.05;xyz=[]
            for row in rows:
                fk=row['reply']['actual_fk'];p,_=tool.apply(np.array(fk['position_m']),Rotation.from_quat(fk['quaternion_xyzw']));xyz.append(p*1000)
            direction=np.array(rows[0]['requested'][:3]);direction/=np.linalg.norm(direction)
            vel=savgol_filter(np.array(xyz),7,2,deriv=1,delta=.05,axis=0)@direction
            request=np.array([r['requested'][:3] for r in rows])@direction*1000
            ax[0,col].plot(t,request,label='Requested TCP velocity');ax[0,col].plot(t,vel,label='Delayed feedback derivative')
            ax[0,col].set_title(version+' | '+name);ax[0,col].set_ylabel('mm/s');ax[0,col].legend(fontsize=8)
            scale=[r['reply']['collision_retreat']['selected_scale'] for r in rows]
            ax[1,col].step(t,scale,where='post');ax[1,col].set_ylabel('Collision scale');ax[1,col].set_ylim(-.05,1.05)
            cb=[r['reply']['callback_us']/1000 for r in rows]
            ax[2,col].plot(t,cb);ax[2,col].axhline(a.baseline_budget_ms if version=='baseline' else 37,color='orange',ls='--');ax[2,col].axhline(42,color='red',ls='--')
            ax[2,col].set_ylabel('Callback ms');ax[2,col].set_xlabel('Simulated trajectory time (s)')
            stops=[i for i,r in enumerate(rows) if r['reply']['status'] in (-1,2,5,6)]
            for i in stops:
                for panel in ax[:,col]:panel.axvline(t[i],color='red',alpha=.15)
            summaries.append(dict(case=name,version=version,stops=len(stops),scale_transitions=sum(abs(x-y)>.2 for x,y in zip(scale,scale[1:]))))
        for panel in ax.flat:panel.grid(alpha=.2)
        fig.suptitle('Isolated official solver | 300 ms feedback delay | not a physical motion test')
        fig.savefig(root/(a.prefix+name+'.png'),dpi=140);plt.close(fig)
    (root/(a.prefix+'curve-summary.json')).write_text(json.dumps(summaries,indent=2))
    print(json.dumps(summaries))

if __name__=='__main__':main()
