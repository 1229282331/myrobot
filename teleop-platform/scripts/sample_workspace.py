"""PC WSL deterministic workspace sample with FK witnesses; no device access."""
from pathlib import Path
import argparse
import json
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from scipy.stats import qmc

def run(power, assets=None, evidence=None):
    root=Path(__file__).resolve().parents[2]
    assets=(assets or root/'teleop-platform/src/e6_phosphobot/assets/workspace').resolve()
    evidence=(evidence or root/'evidence/workspace-v261').resolve();evidence.mkdir(parents=True,exist_ok=True)
    info=json.loads((assets/'manifest.json').read_text())
    model=ET.parse(assets/'robot.urdf')
    for mesh in model.findall('.//mesh'):mesh.set('filename',(assets/mesh.get('filename')).as_uri())
    temp=Path.home()/'.cache/e6-workspace-v261';temp.mkdir(parents=True,exist_ok=True)
    model.write(temp/'model.urdf',encoding='utf8',xml_declaration=True)
    limits=[]
    for i in range(1,7):
        limit=model.find(f"joint[@name='joint{i}']/limit")
        limits.append([float(limit.get('lower'))+np.deg2rad(5),float(limit.get('upper'))-np.deg2rad(5)])
    limits=np.array(limits)
    q=qmc.Sobol(6,scramble=True,seed=261).random_base2(power)
    q=limits[:,0]+q*(limits[:,1]-limits[:,0])
    actual=json.loads((root/'evidence/workspace-v261/pre-change-state.json').read_text())['actual']['joints_rad']
    q=np.vstack([actual,q])
    text=''.join(' '.join(map(str,row))+'\n' for row in q)
    probe=Path.home()/'.cache/e6-dynamic-gripper-v267/build/workspace_probe' if info.get('dynamic_gripper') else temp/'build/workspace_probe'
    process=subprocess.run([str(probe),str(temp/'model.urdf'),str(assets/'robot.srdf')],input=text,text=True,capture_output=True,check=True)
    (evidence/'sampling-stderr.txt').write_text(process.stderr)
    rows=[json.loads(line) for line in process.stdout.splitlines() if line.startswith('{')]
    assert len(rows)==len(q)
    seen=set();points=[];witnesses=[];contacts={};valid=0
    for joint,row in zip(q,rows):
        if row['collision']:
            for pair in row['contacts']:
                key='/'.join(pair);contacts[key]=contacts.get(key,0)+1
            continue
        valid+=1;p=row['position_m'];voxel=tuple(np.floor(np.array(p)/.01).astype(int))
        if voxel in seen:continue
        seen.add(voxel);points.append(p);witnesses.append(joint.tolist())
    result=dict(model_id=info['model_id'],kind='sampled_reachability_not_motion_permission',environment_modeled=False,
                seed=261,sample_count=len(q),valid_count=valid,voxel_m=.01,points_m=points,witness_joints_rad=witnesses,
                collision_counts=contacts,actual_probe=rows[0],sampling='scipy.stats.qmc.Sobol',moveit_version='2.12.4')
    encoded=json.dumps(result,separators=(',',':'))
    cache=temp/'maps'/info['model_id'];cache.mkdir(parents=True,exist_ok=True)
    (cache/'reachability.json').write_text(encoded)
    (assets/'reachability.json').write_text(encoded)
    print(json.dumps({k:v for k,v in result.items() if k not in ('points_m','witness_joints_rad')},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--power',type=int,default=18)
    p.add_argument('--assets',type=Path);p.add_argument('--evidence',type=Path)
    args=p.parse_args();run(args.power,args.assets,args.evidence)
