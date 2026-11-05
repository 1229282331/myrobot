from pathlib import Path
import argparse,base64,collections,gzip,json,sys
import numpy as np
from scipy.spatial.transform import Rotation
from release_classification import classify_release
root=Path(__file__).resolve().parents[2]
cli=argparse.ArgumentParser();cli.add_argument('directory',type=Path);cli.add_argument('--output',type=Path)
cli.add_argument('--motion-engine', choices=['moveit_servo','native_servop'], default='moveit_servo')
args=cli.parse_args();base=args.directory.resolve()
session=base/'session' if (base/'session').exists() else base
audit=session/'x5/audit' if (session/'x5/audit').exists() else session
deployment=json.loads((session/'deployment.json').read_text())
assert deployment.get('motion_engine','moveit_servo')==args.motion_engine
sys.path.insert(0,str(root/'teleop-platform/upstream/dobot-sdk'))
from dobot_sdk.protocol.data_types import MyType,TEST_VALUE_MAGIC
from dobot_sdk.protocol.feedback import FeedbackParser
parser=FeedbackParser();rows=[];counts=collections.Counter();seqs=[]
with gzip.open(audit/'raw.jsonl.gz','rt') as f:
 for line in f:
  e=json.loads(line);data=base64.b64decode(e['raw_b64']);p=np.frombuffer(data,dtype=MyType,count=1)[0]
  assert len(data)==1440 and p['len']==1440 and p['TestValue']==TEST_VALUE_MAGIC
  s=parser.parse(data);assert s is not None and np.array_equal(s.joint_state.q_actual,p['QActual'])
  assert np.all(np.isfinite(p['QActual'])) and np.all(np.isfinite(p['ToolVectorActual']))
  rows.append([e['received_ns'],*p['QActual'],*p['ToolVectorActual'],int(p['RobotMode']),int(p['RunQueuedCmd'])]);seqs.append(e['sequence'])
  counts[str((int(p['RobotMode']),int(p['EnableStatus']),int(p['ErrorStatus']),int(p['CollisionState'])))]+=1
raw=np.array(rows);times=raw[:,0];q=raw[:,1:7];poses=raw[:,7:13]
events=collections.Counter();groups=[];active=None;submitted=[];confirmed=[];stops=[];boundaries=[];can=collections.Counter();faults=[];reasons=collections.Counter();final=None;inputs={};processed={}
with gzip.open(audit/'events.jsonl.gz','rt',encoding='utf-8') as f:
 for line in f:
  e=json.loads(line);kind=e['event'];events[kind]+=1
  if kind=='input' and e['data']['op']=='intent':
   inputs[e['data']['seq']]=e
   intent=e['data']['intent'];moving=intent['deadman'] and any(intent['linear_m_s']+intent['angular_rad_s']+intent['joint_rad_s'])
   if moving and active is None:active={'start_ns':e['at_ns'],'intent':intent,'first_sequence':e['data']['seq']}
   if not moving and active is not None:
    active['release_ns']=e['at_ns'];active['release_sequence']=e['data']['seq'];groups.append(active);active=None
  elif kind=='input_processed':processed[e['received_ns']]=e
  elif kind=='motion_submitted':
   if args.motion_engine=='native_servop':
    candidate=e['candidate'];source=candidate['source_input'];original=inputs[candidate['input_sequence']]
    assert source['sequence']==candidate['input_sequence']
    assert source['intent']==original['data']['intent'] and source['intent']['deadman']
    assert original['at_ns']<=source['received_ns']<=e['receipt']['sent_ns']
    assert e['receipt']['sent_ns']-source['received_ns']<=250_000_000
    assert np.allclose(candidate['observed_joints_rad'],np.deg2rad(e['pre_action_observation']['status']['joint_state']['q_actual']),rtol=0,atol=1e-12)
    from dobot_sdk.api.motion import Motion
    from types import SimpleNamespace
    # Rebuild exact wire bytes through the pinned official SDK; do not invent
    # a joint action for a Cartesian command whose internal IK is unobserved.
    sdk=Motion(SimpleNamespace(send_receive_text=lambda command:command))
    options=dict(t=deployment.get('servo_duration_s',.05),aheadtime=deployment.get('servo_aheadtime',50),gain=deployment.get('servo_gain',500))
    if candidate['kind']=='cartesian':
     assert candidate['joints_rad'] is None
     pose=[*(np.array(candidate['position_m'])*1000),*Rotation.from_quat(candidate['quaternion_xyzw']).as_euler('xyz',degrees=True)]
     command=sdk.ServoP(pose,**options)
    else:
     assert candidate['kind']=='joint'
     command=sdk.ServoJ(np.rad2deg(candidate['joints_rad']),**options)
    assert e['receipt']['command']==command
   submitted.append(e)
  elif kind=='motion_confirmed':confirmed.append(e)
  elif kind=='stop_requested':stops.append(e)
  elif kind=='workspace_blocked':boundaries.append(e)
  elif kind in ('motion_uncertain','gripper_fault','enable_uncertain'):faults.append(e)
  elif kind=='gripper_feedback':can[str((e['feedback'].get('state'),e['feedback'].get('fault')))]+=1
  elif kind=='cycle':
   if e['state'].get('reason'):reasons[e['state']['reason']]+=1
  elif kind=='shutdown_state':final=e
assert active is None
for e in submitted:
 assert e['pre_action_observation']['received_ns']<e['receipt']['sent_ns']
 if args.motion_engine=='moveit_servo':
  assert e['candidate']['kind']=='moveit_servoj'
  assert e['candidate']['moveit']['source_input']['intent']['deadman']
assert len(submitted)==len(confirmed)
assert all(e['target']['receipt']['accepted'] and e['target']['receipt']['acknowledged'] for e in confirmed)
motion=[]
for index,g in enumerate(groups):
 start=g['start_ns'];release=g['release_ns'];intent=g['intent'];a=np.searchsorted(times,start);b=np.searchsorted(times,release)
 observation_end=release+1_500_000_000
 if index+1<len(groups):observation_end=min(observation_end,groups[index+1]['start_ns'])
 end=min(np.searchsorted(times,observation_end,side='left')-1,len(raw)-1)
 g['post_release_observation_ms']=(times[end]-release)/1e6
 baseline=np.median(q[max(0,a-12):a+1],axis=0);p0=np.median(poses[max(0,a-12):a+1],axis=0)
 lastq=np.median(q[max(b,end-12):end+1],axis=0);lastp=np.median(poses[max(b,end-12):end+1],axis=0)
 g.update(duration_ms=(release-start)/1e6,joint_delta_deg=(lastq-baseline).tolist(),position_delta_mm=(lastp[:3]-p0[:3]).tolist(),rotation_delta_deg=(Rotation.from_euler('xyz',lastp[3:],degrees=True)*Rotation.from_euler('xyz',p0[3:],degrees=True).inv()).as_rotvec(degrees=True).tolist())
 if not any(intent['linear_m_s']+intent['angular_rad_s']+intent['joint_rad_s']):continue
 # Use the actual adopted input identity. A later pulse within one second
 # must not make an ignored input appear to have driven hardware.
 cmds=[e for e in submitted if g['first_sequence']<=e['candidate']['input_sequence']<g['release_sequence']]
 g['servo_commands']=len(cmds)
 if not cmds:continue  # Shadow inputs are not physical motion or stopping trials.
 detect=np.flatnonzero(np.max(np.abs(q[a:end+1]-baseline),axis=1)>.01)
 g['x5_input_to_position_change_ms']=(times[a+detect[0]]-start)/1e6 if len(detect) else None
 stop_end=min(release+1_000_000_000,observation_end)
 st=next((s for s in stops if release<=s['at_ns']<stop_end),None)
 early_stops=[s for s in stops if start<=s['at_ns']<release]
 g['normal_release_observed'],g['release_classification_reason']=classify_release(release,st,early_stops,processed,boundaries)
 g['release_processing']=processed.get(release)
 g['early_stops_ns']=[s['at_ns'] for s in early_stops]
 # Retain aborted groups, but never describe their already-stopped tail as
 # qualification of an ordinary operator release.
 g['release_to_stop_send_ms']=(st['receipt']['send_completed_ns']-release)/1e6 if st else None
 # Start of a 250 ms stationary window, all joints within .01 degree peak-to-peak,
 # mode5 and empty queue. Report proof arrival separately from estimated onset.
 stationary=None;idle=None
 for j in range(b,end):
  k=np.searchsorted(times,times[j]+250_000_000)
  if k>end:break
  if np.max(np.ptp(q[j:k+1],axis=0))<=.01:
   if stationary is None:stationary=j
   if np.all(raw[j:k+1,13]==5) and np.all(raw[j:k+1,14]==0):idle=j;break
 g['release_to_stationary_window_start_ms']=(times[stationary]-release)/1e6 if stationary is not None else None
 g['stationary_proof_lag_ms']=250
 g['release_to_mode5_empty_stationary_ms']=(times[idle]-release)/1e6 if idle is not None else None
 g['stop_travel_joint_deg']=np.max(np.abs(q[b:end+1]-q[b]),axis=0).tolist()
 g['stop_travel_position_mm']=float(np.max(np.linalg.norm(poses[b:end+1,:3]-poses[b,:3],axis=1)))
 # Central constant-input portion only, acceleration/transients excluded explicitly.
 lo=np.searchsorted(times,start+350e6);hi=np.searchsorted(times,release-100e6)
 if hi>lo:
  dt=(times[hi]-times[lo])/1e9
  g['middle_window_s']=dt;g['middle_linear_mm_s']=((poses[hi,:3]-poses[lo,:3])/dt).tolist()
  g['middle_joint_deg_s']=((q[hi]-q[lo])/dt).tolist()
  g['middle_angular_deg_s']=((Rotation.from_euler('xyz',poses[hi,3:],degrees=True)*Rotation.from_euler('xyz',poses[lo,3:],degrees=True).inv()).as_rotvec(degrees=True)/dt).tolist()
 motion.append(g)
unacted=[g['first_sequence'] for g in groups if not g['servo_commands']]
assert sum(g['servo_commands'] for g in groups)==len(submitted)
summary=dict(raw_frames=len(raw),raw_invalid=0,raw_sequence_gaps=int(np.sum(np.maximum(np.diff(seqs)-1,0))),status_counts=counts,events=events,motion_submitted=len(submitted),motion_confirmed=len(confirmed),faults=faults,cycle_reasons=reasons,gripper_feedback=can,groups=groups,motion_groups=motion,inputs_without_commands=unacted,shutdown=final,clock='X5 same kernel monotonic; does not measure browser-to-X5 network delay',stationary_definition='First 250ms window, all joint positions peak-to-peak <=0.01 degree; mode5/empty-queue reported separately. Not certified physical braking time')
for key in ('release_to_stop_send_ms','release_to_stationary_window_start_ms','x5_input_to_position_change_ms'):
 vals=[g[key] for g in motion if g[key] is not None and (not key.startswith('release_') or g['normal_release_observed'])]
 summary[key]={'n':len(vals),'p50':float(np.percentile(vals,50)),'p95':float(np.percentile(vals,95)),'p99':float(np.percentile(vals,99)),'max':max(vals)} if vals else None
summary['release_definition']='First received intent that removes arm velocity or deadman. Normal release also requires its exact input_processed event to retain a ready, reason-free session. Stop after socket receipt but after a fault inside the control mutex is not a normal release. Observation ends before the next motion input (or after 1.5 s). Release quantiles exclude early-stop, faulted and unobserved-processing groups; all raw groups remain included.'
(args.output or base/'analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('groups','motion_groups','shutdown')},ensure_ascii=False,indent=2))
print('directions',json.dumps([{k:g.get(k) for k in ('first_sequence','duration_ms','joint_delta_deg','position_delta_mm','rotation_delta_deg','release_to_stationary_window_start_ms')} for g in motion]))
