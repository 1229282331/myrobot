"""Offline small-increment official-SDK probe analysis; no device access."""
from pathlib import Path
import argparse,base64,json,sys
import numpy as np
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'teleop-platform/upstream/dobot-sdk'))
from dobot_sdk.protocol.data_types import MyType,TEST_VALUE_MAGIC
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();d=a.directory
s=json.loads((d/'summary.json').read_text());rows=[];states=set()
for line in (d/'frames.jsonl').read_text().splitlines():
 e=json.loads(line);wire=base64.b64decode(e['wire_base64'])
 assert len(wire)==1440,'Only unenveloped frames supported in this diagnostic'
 raw=np.frombuffer(wire,dtype=MyType,count=1)[0]
 assert raw['len']==1440 and raw['TestValue']==TEST_VALUE_MAGIC
 assert np.array_equal(raw['QActual'],e['status']['joint_state']['q_actual'])
 rows.append(np.r_[e['received_ns'],raw['QActual'],raw['QTarget'],raw['ToolVectorActual']])
 states.add((int(raw['RobotMode']),int(raw['EnableStatus']),int(raw['ErrorStatus']),int(raw['CollisionState'])))
r=np.array(rows);cmds=[c for c in s['commands'] if c['command'].startswith(('ServoP(','ServoJ('))]
start=cmds[0]['sent_ns'];end=s['release_received_ns'];sub=r[(r[:,0]>=start)&(r[:,0]<end)]
t=(sub[:,0]-start)/1e9;qt=sub[:,7:13];qa=sub[:,1:7]
bound=np.r_[0,np.flatnonzero(np.max(abs(np.diff(qt,axis=0)),axis=1)>1e-7)+1,len(qt)]
plateaus=[dict(from_s=float(t[l]),to_s=float(t[h-1]),duration_ms=float((t[h-1]-t[l])*1000)) for l,h in zip(bound[:-1],bound[1:]) if h-l>2]
actual=Rotation.from_euler('xyz',sub[:,16:19],degrees=True)
direction=np.array([0.,-1.,0.]);angle=(actual*actual[0].inv()).as_rotvec(degrees=True)@direction
report=dict(raw_frames=len(r),status_counts=list(states),commands=len(cmds),all_acknowledged=all(c['acknowledged'] and c['reply'].startswith('0,') for c in cmds),
 send_interval_ms=np.percentile(np.diff([c['sent_ns'] for c in cmds])/1e6,[0,50,95,100]).tolist(),
 controller_target_plateaus=plateaus,rotation_during_send_deg=float(angle[-1]),
 max_plateau_ms=max((p['duration_ms'] for p in plateaus),default=0),
 max_target_actual_joint_difference_deg=np.max(abs(qa-qt),axis=0).tolist(),
 error=s['error'],stop_error=s['stop_error'],stop_acknowledged=s.get('stop_reply','').startswith('0,'),
 stationary_detected=s.get('stationary_ns') is not None)
(d/'micro-analysis.json').write_text(json.dumps(report,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(12,4))
axes[0].plot(t,angle,label='Actual rotation');axes[0].plot(t,.25*t,label='Requested slope')
axes[0].set_ylabel('Base Ry- angle, degrees');axes[0].legend()
for ax in axes:ax.grid(alpha=.3);ax.set_xlabel('Seconds from first command')
j=int(np.argmax(np.ptp(qt,axis=0)))
axes[1].plot(t,qt[:,j]-qt[0,j],label='Controller target');axes[1].plot(t,qa[:,j]-qt[0,j],label='Actual joint')
axes[1].set_ylabel(f'J{j+1} relative angle, degrees');axes[1].legend();fig.suptitle(d.name)
fig.tight_layout();fig.savefig(d/'micro-following.png',dpi=140);plt.close(fig)
print(json.dumps(report,indent=2))
