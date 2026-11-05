"""Read-only formal-page low-speed comparison from bounded audit snapshots."""
from pathlib import Path
import argparse,base64,collections,gzip,json,sys
import numpy as np
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'teleop-platform/upstream/dobot-sdk'))
from dobot_sdk.protocol.data_types import MyType,TEST_VALUE_MAGIC
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);args=p.parse_args();d=args.directory
source=d/'session/x5/audit' if (d/'session').exists() else d
with gzip.open(source/'events.jsonl.gz','rt',encoding='utf-8') as f:
 events=[e for l in f if (e:=json.loads(l))['event'] in ('input','motion_submitted','motion_confirmed','stop_requested','stop_result')]
commands=[e for e in events if e['event']=='motion_submitted'];confirmed=[e for e in events if e['event']=='motion_confirmed']
groups=[];active=None
for e in events:
 if e['event']!='input' or e['data']['op']!='intent':continue
 i=e['data']['intent'];moving=i['deadman'] and any(i['linear_m_s']+i['angular_rad_s']+i['joint_rad_s'])
 # Separate intentional reversals; otherwise a correct reverse movement would
 # be counted as oscillation against the first phase's direction.
 changed=active is not None and (i['angular_rad_s']!=active['intent']['angular_rad_s'] or i['frame']!=active['intent']['frame'])
 if active is not None and (not moving or changed):active['end']=e['at_ns'];groups.append(active);active=None
 if moving and active is None:active=dict(start=e['at_ns'],intent=i)
rows=[];states=collections.Counter();sequence=[]
with gzip.open(source/'raw.jsonl.gz','rt') as f:
 for l in f:
  e=json.loads(l);wire=base64.b64decode(e['raw_b64']);assert len(wire)==1440
  raw=np.frombuffer(wire,dtype=MyType,count=1)[0];assert raw['len']==1440 and raw['TestValue']==TEST_VALUE_MAGIC
  rows.append(np.r_[e['received_ns'],raw['QActual'],raw['QTarget'],raw['ToolVectorActual']]);sequence.append(e['sequence'])
  states[str((int(raw['RobotMode']),int(raw['EnableStatus']),int(raw['ErrorStatus']),int(raw['CollisionState'])))]+=1
r=np.array(rows);results=[]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
for n,g in enumerate(groups):
 cmds=[c for c in commands if g['start']<=c['at_ns']<g['end']]
 if not cmds or not any(g['intent']['angular_rad_s']):continue
 start=cmds[0]['receipt']['sent_ns'];end=g['end'];sub=r[(r[:,0]>=start)&(r[:,0]<end)]
 if len(sub)<2:continue
 t=(sub[:,0]-start)/1e9;qa=sub[:,1:7];qt=sub[:,7:13]
 bound=np.r_[0,np.flatnonzero(np.max(abs(np.diff(qt,axis=0)),axis=1)>1e-7)+1,len(qt)]
 plateau=[dict(from_s=float(t[l]),to_s=float(t[h-1]),duration_ms=float((t[h-1]-t[l])*1000)) for l,h in zip(bound[:-1],bound[1:]) if h-l>2]
 direction=np.array(g['intent']['angular_rad_s']);speed=np.rad2deg(np.linalg.norm(direction));direction/=np.linalg.norm(direction)
 rot=Rotation.from_euler('xyz',sub[:,16:19],degrees=True)
 if g['intent']['frame']=='tool':direction=rot[0].apply(direction)
 angle=(rot*rot[0].inv()).as_rotvec(degrees=True)@direction
 cts=np.array([(c['receipt']['sent_ns']-start)/1e9 for c in cmds]);cr=Rotation.from_quat([c['candidate']['quaternion_xyzw'] for c in cmds])
 ca=(cr*cr[0].inv()).as_rotvec(degrees=True)@direction
 result=dict(group=n,commands=len(cmds),requested_deg_s=float(speed),intent=g['intent'],
   send_interval_ms=np.percentile(np.diff(cts)*1000,[0,50,95,100]).tolist() if len(cts)>1 else [],
   controller_target_plateaus=plateau,max_plateau_after_250ms_ms=max((p['duration_ms'] for p in plateau if p['from_s']>.25),default=0),
   observed_angle_at_release_deg=float(angle[-1]),candidate_angle_deg=float(ca[-1]),
   negative_orientation_steps_over_001deg=int(np.sum((np.diff(angle)<-.01)&(t[1:]>.25))),
   max_joint_tracking_difference_deg=float(np.max(abs(qa-qt))))
 results.append(result)
 fig,axes=plt.subplots(1,2,figsize=(12,4));axes[0].plot(t,angle,label='Actual rotation');axes[0].plot(cts,ca,'o-',ms=3,label='MoveIt candidate (own zero)')
 axes[0].set_ylabel('Angle deg');j=int(np.argmax(np.ptp(qt,axis=0)))
 axes[1].plot(t,qt[:,j]-qt[0,j],label='Controller target');axes[1].plot(t,qa[:,j]-qt[0,j],label='Actual joint')
 axes[1].set_ylabel(f'J{j+1} relative angle deg')
 for ax in axes:ax.grid(alpha=.3);ax.legend();ax.set_xlabel('Seconds from first send')
 fig.suptitle(f'{d.name}, group {n}, requested {speed:g} deg/s');fig.tight_layout();fig.savefig(d/f'group-{n}.png',dpi=140);plt.close(fig)
report=dict(frames=len(r),status_counts=states,sequence_gaps=int(np.sum(np.maximum(np.diff(sequence)-1,0))),
  events=collections.Counter(e['event'] for e in events),motion_confirmed=len(confirmed),groups=results)
(d/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,ensure_ascii=False,indent=2))
