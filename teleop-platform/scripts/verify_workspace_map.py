"""Replay every saved witness through MoveIt/FCL on the PC, without hardware."""
import argparse,hashlib,json,subprocess
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser();parser.add_argument('--assets',type=Path);parser.add_argument('--evidence',type=Path)
parser.add_argument('--solver-output',type=Path)
args=parser.parse_args()
assets=(args.assets or root/'teleop-platform/src/e6_phosphobot/assets/workspace').resolve()
evidence=(args.evidence or root/'evidence/workspace-v261').resolve();evidence.mkdir(parents=True,exist_ok=True)
info=json.loads((assets/'manifest.json').read_text());path=assets/'reachability.json';value=json.loads(path.read_text())
assert value['model_id']==info['model_id']
cache=Path.home()/'.cache/e6-workspace-v261';tree=ET.parse(assets/'robot.urdf')
probe=Path.home()/'.cache/e6-dynamic-gripper-v267/build/workspace_probe' if info.get('dynamic_gripper') else cache/'build/workspace_probe'
for mesh in tree.findall('.//mesh'):mesh.set('filename',(assets/mesh.get('filename')).as_uri())
tree.write(cache/'verify.urdf',encoding='utf8',xml_declaration=True)
q=np.array(value['witness_joints_rad']);p=np.array(value['points_m'])
for i in range(6):
    limit=tree.find(f"joint[@name='joint{i+1}']/limit")
    assert np.all(q[:,i]>=float(limit.get('lower'))+np.deg2rad(5)) and np.all(q[:,i]<=float(limit.get('upper'))-np.deg2rad(5))
text=''.join(' '.join(map(str,row))+'\n' for row in q)
run=subprocess.run([str(probe),str(cache/'verify.urdf'),str(assets/'robot.srdf')],input=text,text=True,capture_output=True,check=True)
rows=[json.loads(line) for line in run.stdout.splitlines() if line.startswith('{')]
assert len(rows)==len(q)
error=float(np.max(np.linalg.norm(np.array([r['position_m'] for r in rows])-p,axis=1)))
collisions=sum(r['collision'] for r in rows)
assert collisions==0 and error<1e-10
voxels=np.floor(p/value['voxel_m']).astype(int);assert len(np.unique(voxels,axis=0))==len(p)
key=info['model_id'];saved=cache/'maps'/key;saved.mkdir(parents=True,exist_ok=True)
data=path.read_bytes();(saved/'reachability.json').write_bytes(data)
report=dict(model_id=key,map_sha256=hashlib.sha256(data).hexdigest(),witnesses=len(rows),collisions=collisions,max_fk_error_m=error,
    all_joint_margins_deg=5,unique_voxels=True,hardware_io=False,physical_full_range_verified=False)
(evidence/'witness-verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
frames=(args.solver_output or evidence/'offline-solver')/'frames.jsonl'
if frames.is_file():
    rows_in=[json.loads(line) for line in frames.read_text().splitlines()]
    targets=np.array([r['reply']['joints_rad'] for r in rows_in])
    actual=np.array([r['actual_joints_rad'] for r in rows_in])
    # Include midpoints between each feedback state and its proposed target.
    checks=np.vstack([actual,(actual+targets)/2,targets])
    text=''.join(' '.join(map(str,row))+'\n' for row in checks)
    run=subprocess.run([str(probe),str(cache/'verify.urdf'),str(assets/'robot.srdf')],input=text,text=True,capture_output=True,check=True)
    results=[json.loads(line) for line in run.stdout.splitlines() if line.startswith('{')]
    assert len(results)==len(checks)
    collisions=sum(r['collision'] for r in results)
    assert collisions==0
    from scipy.spatial.transform import Rotation
    errors=[]
    for row,checked in zip(rows_in,results[2*len(rows_in):]):
        fk=row['reply']['target_fk'];tcp=np.array(fk['position_m'])+Rotation.from_quat(fk['quaternion_xyzw']).apply([0,0,.1514])
        errors.append(float(np.linalg.norm(tcp-np.array(checked['position_m']))))
    assert max(errors)<1e-9
    source_model=json.loads((frames.parent/'summary.json').read_text())['model_id']
    report=dict(model_id=key,trajectory_source_model_id=source_model,frames_sha256=hashlib.sha256(frames.read_bytes()).hexdigest(),frames=len(rows_in),fk_checks=len(errors),collision_checks=len(results),collisions=collisions,
                max_fk_error_m=max(errors),hardware_io=False,method='MoveIt FCL independent replay of actual, midpoint and candidate')
    (evidence/'solver-collision-verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
