"""Audit the existing native candidate route using original formal-page inputs."""
import argparse,collections,gzip,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from e6_phosphobot.trajectory import VelocityTrajectory
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();base=a.directory
inputs={};events=collections.Counter();seen=set();groups=collections.Counter();last=None;active=False;cycles=0;bad=[];smoothed=0
with gzip.open(base/'session/x5/audit/events.jsonl.gz','rt',encoding='utf8') as f:
 for line in f:
  e=json.loads(line);events[e['event']]+=1
  if e['event']=='input' and e['data']['op']=='intent':
   inputs[e['data']['seq']]=e
   if not e['data']['intent']['deadman']:last=None
  if e['event']!='cycle':continue
  s=e['state'];c=s.get('candidate')
  if not c:continue
  key=(c['input_sequence'],c['pre_action_received_ns'])
  if key in seen:continue
  seen.add(key);cycles+=1
  original=inputs[c['input_sequence']];source=c['source_input'];i=original['data']['intent']
  assert source['intent']==i and source['sequence']==c['input_sequence'] and i['deadman']
  assert original['at_ns']<=source['received_ns']<=e['at_ns']
  assert c['kind']=='cartesian' and c['joints_rad'] is None
  groups[str((i['frame'],i['linear_m_s'],i['angular_rad_s']))]+=1
  # Changes within each continuous input group must agree with its physical
  # base/tool axes; do not treat the robot's unchanged shadow state as motion.
  current=(np.array(c['position_m']),Rotation.from_quat(c['quaternion_xyzw']))
  signature=(i['frame'],tuple(i['linear_m_s']),tuple(i['angular_rad_s']))
  trajectory=c.get('trajectory')
  if trajectory and trajectory.get('motion_profile'):
   # Replay the pinned upstream integrator from its original input state.
   # Reversal tails legitimately differ from the new requested direction.
   # This verifies the generated trajectory without dropping those samples.
   smoothed+=1;step=trajectory['step'];delta=np.array(step['delta'])
   previous_rotation=Rotation.from_rotvec(delta[3:]).inv()*current[1]
   expected=np.array(i['linear_m_s']+i['angular_rad_s'])
   if i['frame']=='tool':expected=np.r_[previous_rotation.apply(expected[:3]),previous_rotation.apply(expected[3:])]
   np.testing.assert_allclose(step['requested_velocity'],expected,atol=1e-10)
   replay=VelocityTrajectory(step['period_s'],'cartesian',motion_profile=trajectory['motion_profile'])
   replay.input.current_velocity=step['prior_velocity'];replay.input.current_acceleration=step['prior_acceleration']
   np.testing.assert_allclose(replay.step(expected),delta,atol=1e-10)
   np.testing.assert_allclose(replay.input.current_velocity,trajectory['velocity'],atol=1e-10)
   if last:
    np.testing.assert_allclose(current[0]-last[1][0],delta[:3],atol=1e-10)
    np.testing.assert_allclose((current[1]*last[1][1].inv()).as_rotvec(),delta[3:],atol=1e-10)
  elif last and last[0]==signature:
   prev=last[1];dp=current[0]-prev[0];dr=(current[1]*prev[1].inv()).as_rotvec()
   v=np.array(i['linear_m_s']);w=np.array(i['angular_rad_s'])
   if i['frame']=='tool':v=prev[1].apply(v);w=prev[1].apply(w)
   for delta,expected in ((dp,v),(dr,w)):
    if np.linalg.norm(delta)<1e-10:continue
    assert np.linalg.norm(expected)>0
    cosine=np.dot(delta,expected)/(np.linalg.norm(delta)*np.linalg.norm(expected))
    assert cosine>.999999
  last=(signature,current)
result=dict(unique_candidates=cycles,smoothed_candidates=smoothed,input_groups=groups,events=events,source_and_direction_errors=0,
 method='Original input identity; unsmoothed adjacent direction or independent pinned Ruckig step replay and adjacent quaternion/translation increments. All reversal samples retained; not physical motion or IK feasibility.')
assert events['motion_submitted']==events['stop_requested']==events['can']==0
(base/'candidate-audit.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps(result,indent=2))
