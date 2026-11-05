"""Replay identical old-mode input/feedback through old and candidate binaries.

Read-only model / isolated stdin. No robot, camera, CAN, or control connection.
"""
from pathlib import Path
import argparse,json,subprocess,select,time,hashlib
import numpy as np
import yaml
from e6_phosphobot.moveit_config import load_jazzy_bundle

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);a=ap.parse_args();root=a.root
 out=root/'legacy-modes';out.mkdir(exist_ok=False)
 params,_=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
 params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
 cfg=out/'params.yaml';cfg.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
 state=json.loads((root/'current-state.json').read_text());initial=np.array(state['actual']['joints_rad']);raw=round(state['actual']['gripper']*255)
 baseline=[];candidate=[];inputs=[];seq=epoch=0
 for variant,binary in [('baseline',root/'baseline/servo_cpp_adapter'),('candidate',root/'build/servo_cpp_adapter')]:
  last=0.
  with (out/(variant+'.log')).open('w') as log:
   p=subprocess.Popen([str(binary),'--ros-args','--params-file',str(cfg)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
   def exchange(wire):
    nonlocal last
    time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic()
    p.stdin.write(wire);p.stdin.flush()
    if not select.select([p.stdout],[],[],20)[0]:raise TimeoutError('Solver response')
    row=json.loads(p.stdout.readline());return row
   try:
    if variant=='baseline':
     for kind,frame,axes in [(1,False,range(3,6)),(1,True,range(3,6)),(0,False,range(6))]:
      for axis in axes:
       for sign in (-1,1):
        epoch+=1;q=initial.copy()
        for step in range(30):
         active=5<=step<29;v=np.zeros(6);v[axis]=sign*np.deg2rad(2) if active else 0
         seq+=1;wire=' '.join(map(str,[seq,epoch,int(active),kind,int(frame),*q,*v,'G3',raw,-1,-1,-1,1]))+'\n'
         row=exchange(wire);inputs.append(wire);baseline.append(row);q=np.array(row['joints_rad'])
    else:
     for wire in inputs:candidate.append(exchange(wire))
   finally:
    p.stdin.close()
    try:p.wait(10)
    except subprocess.TimeoutExpired:p.terminate();p.wait(5)
    (out/(variant+'.json')).write_text(json.dumps(baseline if variant=='baseline' else candidate))
 (out/'inputs.json').write_text(json.dumps(inputs))
 assert len(candidate)==len(baseline)==720
 differences=[float(np.max(np.abs(np.array(a['joints_rad'])-b['joints_rad']))) for a,b in zip(baseline,candidate)]
 statuses=[(a['status'],b['status']) for a,b in zip(baseline,candidate) if a['status']!=b['status']]
 passed=max(differences)<1e-9 and not statuses
 summary=dict(passed=passed,hardware_io=False,frames=len(candidate),cases=24,max_joint_difference_rad=max(differences),status_differences=statuses,
  callback_candidate_ms=np.percentile([x['callback_us']/1000 for x in candidate[1:]],[50,95,100]).tolist(),
  binary_sha256=hashlib.sha256((root/'build/servo_cpp_adapter').read_bytes()).hexdigest())
 (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary));assert passed

if __name__=='__main__':main()
