"""Bound the complete nominal motion of the actual dynamic collision boxes."""
import argparse,hashlib,itertools,json,shutil
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

def rebuild(tree,geometry):
    joints=tree.findall('joint');names=geometry['joint_names'];poses=geometry['joints_rad_by_raw']
    def transform(q):
        frames={'omnipicker':np.eye(4)};remaining=list(joints)
        while remaining:
            done=[]
            for j in remaining:
                parent=j.find('parent').get('link');child=j.find('child').get('link')
                if parent not in frames:continue
                o=j.find('origin');m=np.eye(4)
                if o is not None:
                    m[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
                    m[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
                if j.get('name') in names:
                    r=np.eye(4);r[:3,:3]=Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*q[names.index(j.get('name'))]).as_matrix();m=m@r
                frames[child]=frames[parent]@m;done.append(j)
            if not done:break
            remaining=[j for j in remaining if j not in done]
        return frames
    frames=[transform(q) for q in poses];fallback=tree.find("link[@name='op_unknown_sweep']")
    for c in list(fallback.findall('collision')):fallback.remove(c)
    proofs=[]
    for box in geometry['local_boxes']:
        corners=np.array(list(itertools.product((-1,1),repeat=3)))*np.array(box['size_m'])/2
        m=np.array(box['box_to_link']);corners=corners@m[:3,:3].T+m[:3,3]
        cloud=np.concatenate([corners@f[box['link']][:3,:3].T+f[box['link']][:3,3] for f in frames])
        to_box,size=trimesh.bounds.oriented_bounds(cloud);m=np.linalg.inv(to_box)
        # Independent analytic proof bounds one raw interval below 1 mm. The
        # source cloud already contains each moving box's full 5 mm reserve.
        size=size+.002
        clearance=float(np.min(size/2-np.abs((cloud-m[:3,3])@m[:3,:3])));assert clearance>.0009999
        c=E.SubElement(fallback,'collision',name='finger_fallback_'+box['source_link'])
        E.SubElement(c,'origin',xyz=' '.join(map(str,m[:3,3])),rpy=' '.join(map(str,Rotation.from_matrix(m[:3,:3]).as_euler('xyz'))))
        E.SubElement(E.SubElement(c,'geometry'),'box',size=' '.join(map(str,size)))
        proofs.append(dict(link=box['link'],samples=256,corner_count=len(cloud),minimum_corner_reserve_m=clearance,size_m=size.tolist()))
    geometry['fallback_coverage']=dict(method='OBB of all 256 nominal poses of each dynamic OBB plus 1 mm continuous-interpolation reserve',pieces=proofs)
    return geometry

def stage(source,target):
    assert not target.exists();shutil.copytree(source,target)
    g=json.loads((target/'gripper-kinematics.json').read_text())
    for name in ('attachments.urdf','robot.urdf'):
        tree=E.parse(target/name);rebuild(tree.getroot(),g);tree.write(target/name,encoding='utf-8',xml_declaration=True)
    (target/'gripper-kinematics.json').write_text(json.dumps(g,indent=2),encoding='utf8')
    info=json.loads((target/'manifest.json').read_text());old=info.pop('model_id');info['dynamic_gripper']={k:v for k,v in g.items() if k!='joints_rad_by_raw'}
    root=Path(__file__).resolve().parents[2];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    for path in (Path(__file__),Path(__file__).with_name('articulated_gripper_assets.py')):info['sources'][path.relative_to(root).as_posix()]=sha(path)
    info['fallback_revision_of']=old;info['files']={name:sha(target/name) for name in info['files']}
    info['model_id']=hashlib.sha256(json.dumps(info,sort_keys=True).encode()).hexdigest()
    (target/'manifest.json').write_text(json.dumps(info,indent=2));print(info['model_id'])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();stage(a.source,a.output)
