"""Isolated Servo with real tracking residuals, never a hardware client."""
from pathlib import Path
import argparse,subprocess,select,time,json,hashlib,collections
import numpy as np
import yaml
from e6_phosphobot.moveit_config import load_jazzy_bundle
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();root=a.root
out=root/'candidate-real-residual';out.mkdir(exist_ok=False)
params,_=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
cfg=out/'params.yaml';cfg.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
fixtures=json.loads((root/'real-residual-fixtures.json').read_text());rows=[];seq=epoch=0;last=0.;target=None
binary=root/'build/servo_cpp_adapter'
with (out/'servo.log').open('w') as log:
 process=subprocess.Popen([str(binary),'--ros-args','--params-file',str(cfg)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
 def step(q,v,active,g):
  global seq,last
  time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
  wire=[seq,epoch,int(active),1,0,*q,*v,'G3',g['actual_raw'],g['sent_raw'],g['inflight_raw'],g['requested_raw'],g['hold_state']]
  process.stdin.write(' '.join(map(str,wire))+'\n');process.stdin.flush()
  if not select.select([process.stdout],[],[],20)[0]:raise TimeoutError('Solver')
  return json.loads(process.stdout.readline())
 try:
  for repeat in range(3):
   for index,f in enumerate(fixtures):
    if index==0 or f['reset']:
     epoch+=1;target=np.array(f['actual'])
     for _ in range(6):step(target,np.zeros(6),False,f['gripper'])
    # Preserve the measured deviation from the committed seed, while allowing
    # improved candidates to advance the synthetic plant instead of forcing
    # it to follow the old stuttering trajectory. This is not physical motion.
    q=target+(np.array(f['actual'])-f['seed'])
    v=f['effective']['linear_m_s']+f['effective']['angular_rad_s']
    reply=step(q,v,True,f['gripper']);accepted=reply['status'] in (0,1,3,4) and reply['callback_us']<42000 and reply['gripper_check']['arm_safe']
    rows.append(dict(repeat=repeat,index=index,q=q.tolist(),effective=v,requested=f['requested'],reply=reply,accepted=accepted))
    target=np.array(reply['joints_rad']) if accepted else q
 finally:
  process.stdin.close();process.wait(10)
  (out/'frames.json').write_text(json.dumps(rows))
retreat=[x for x in rows if x['requested']['linear_m_s'][0]<0]
summary=dict(passed=all(x['accepted'] for x in rows) and all(x['reply']['collision_retreat']['selected_scale']==1 for x in retreat),hardware_io=False,
 binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),frames=len(rows),accepted=sum(x['accepted'] for x in rows),
 retreat_frames=len(retreat),full_scale_retreat=sum(x['reply']['collision_retreat']['selected_scale']==1 for x in retreat),
 retreat_reasons=collections.Counter(x['reply']['collision_retreat']['reason'] for x in retreat),
 callback_ms=np.percentile([x['reply']['callback_us']/1000 for x in rows],[50,95,100]).tolist())
(out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary));assert summary['passed'],summary
