"""Read-only KDL replay of a failed virtual speed case; no device connection."""
import argparse
import gzip
import json
from pathlib import Path
import subprocess
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--record',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();cfg=json.loads((a.bundle/'jazzy-runtime.json').read_text())
    recorded_profile=json.loads((a.record/'summary.json').read_text()).get('profile',{})
    with gzip.open(a.record/'samples.jsonl.gz','rt') as f:rows=[json.loads(line) for line in f]
    rows=rows[::10];seed=np.array(rows[0]['q']);qs=[row['q'] for row in rows]
    # A separate virtual seed search, never a planned physical move. Keep the
    # archived posture and failure unchanged. Collision checks run in Servo.
    offsets=np.random.default_rng(20261007).uniform(-30,30,(1024,6))
    offsets[:,0]=0;offsets[:,5]=0
    alternatives=seed+np.deg2rad(offsets);qs.extend(alternatives.tolist())
    args=['chroot',cfg['rootfs'],'/bin/bash','-c',
          'source /opt/ros/jazzy/setup.bash; exec "$1" "$2" controller_base controller_flange','bash',
          cfg['bundle']+'/jazzy-adapter-v35/build/kdl_model_probe',cfg['bundle']+'/model/e6.urdf']
    run=subprocess.run(args,input='\n'.join(' '.join(map(str,q)) for q in qs)+'\n',text=True,
                       capture_output=True,check=True,timeout=30)
    models=[json.loads(line) for line in run.stdout.splitlines()]
    assert len(models)==len(qs)
    results=[]
    for q,model in zip(qs,models):
        jac=np.array(model['jacobian']);sv=np.linalg.svd(jac,compute_uv=False)
        inverse=np.linalg.pinv(jac)
        speeds=np.abs(inverse@np.diag([.03]*3+[np.deg2rad(10)]*3))
        results.append(dict(q_deg=np.rad2deg(q).tolist(),condition=float(sv[0]/sv[-1]),
                            singular_values=sv.tolist(),max_joint_deg_s_per_axis=np.rad2deg(speeds.max(axis=0)).tolist(),
                            position_m=model['position_m']))
    best=min(results[len(rows):],key=lambda r:r['condition'])
    workspace=[r for r in results[len(rows):] if np.linalg.norm(r['position_m'][:2])>=.30 and .25<=r['position_m'][2]<=.45]
    workspace_best=min(workspace,key=lambda r:r['condition']) if workspace else None
    result=dict(hardware_io=False,source_record=str(a.record),current_posture=results[0],
                trajectory=results[:len(rows)],virtual_search=dict(samples=len(alternatives),bounds_deg=30,
                collision_or_physical_path_qualified=False,best=best,workspace_seed=workspace_best),
                recorded_thresholds=dict(lower=recorded_profile.get('singularity_slowdown'),
                    hard_stop=recorded_profile.get('singularity_stop'),source='summary.json profile'))
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(dict(current=results[0],best=best,workspace_seed=workspace_best)))


if __name__=='__main__':main()
