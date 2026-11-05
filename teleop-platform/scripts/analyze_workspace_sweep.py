"""Offline nominal CAD sweep using installed PyBullet FK, never robot commands.

Source meshes and model remain unchanged. Mesh bounding-box corner motion bounds
all source vertices; this is not calibrated physical/accessory collision proof.
"""
import argparse
import hashlib
import importlib.metadata
import itertools
import json
from pathlib import Path
import struct
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as pb
from scipy.spatial.transform import Rotation


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def corners(path):
    data=path.read_bytes()
    count=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+50*count:
        raise ValueError('Require original binary STL, not guessed geometry: '+str(path))
    dtype=np.dtype([('normal','<f4',3),('vertex','<f4',(3,3)),('attribute','<u2')])
    vertices=np.frombuffer(data,dtype=dtype,offset=84,count=count)['vertex'].reshape(-1,3)
    assert np.all(np.isfinite(vertices))
    low,high=vertices.min(axis=0),vertices.max(axis=0)
    return np.array(list(itertools.product(*zip(low,high))))


def analyze(path,output):
    root=Path(__file__).resolve().parents[1]
    model=root/'config/e6-controller-model/e6.urdf'
    source=ET.parse(model)
    mesh_root=root/'upstream/dobot-ros2/dobot_rviz/meshes/me6'
    boxes={};hashes={}
    for link in source.findall('./link'):
        visual=link.find('visual')
        if visual is not None:
            mesh=visual.find('geometry/mesh')
            mesh_path=mesh_root/Path(mesh.attrib['filename']).name
            origin=visual.find('origin')
            xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
            rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
            boxes[link.attrib['name']]=Rotation.from_euler('xyz',rpy).apply(corners(mesh_path))+xyz
            hashes[mesh_path.name]=sha(mesh_path)
    for mesh in source.findall('.//mesh'):
        mesh.set('filename',(mesh_root/Path(mesh.attrib['filename']).name).as_posix())
    data=json.loads(path.read_text(encoding='utf8'))
    assert data['last']['stage']=='shadow' and not data['counts'].get('motion_submitted')
    assert data['base'] and data['candidates']
    initial=np.array(data['base']['joints_rad'])
    points=[initial]+[np.array(c['joints_rad']) for c in data['candidates']]
    cid=pb.connect(pb.DIRECT)
    try:
        with tempfile.TemporaryDirectory(prefix='e6-cad-sweep-') as temp:
            copy=Path(temp)/'model.urdf';source.write(copy,encoding='utf8')
            robot=pb.loadURDF(str(copy),useFixedBase=True,physicsClientId=cid)
        joints={};links={}
        for i in range(pb.getNumJoints(robot,physicsClientId=cid)):
            info=pb.getJointInfo(robot,i,physicsClientId=cid)
            joints[info[1].decode()]=i;links[info[12].decode()]=i
        def pose(name):
            state=pb.getLinkState(robot,links[name],computeForwardKinematics=True,physicsClientId=cid)
            return np.array(state[4]),Rotation.from_quat(state[5])
        def put(q):
            for n,v in enumerate(q,1):pb.resetJointState(robot,joints['joint'+str(n)],float(v),physicsClientId=cid)
        def world_boxes():
            result={}
            for name,box in boxes.items():
                p,r=pose(name);result[name]=r.apply(box)+p
            return result
        put(initial);original=world_boxes();p0,r0=pose('controller_flange')
        base_error=float(np.linalg.norm(p0-data['base']['position_m'])*1000)
        assert base_error<.1, ('PyBullet / measured base alignment',base_error)
        maxima={name:0. for name in boxes};flange_mm=0.;angle=0.;steps=0;candidate_fk_error=0.;candidate_rotation_error=0.
        for c in data['candidates']:
            put(c['joints_rad']);p,r=pose('controller_flange')
            candidate_fk_error=max(candidate_fk_error,float(np.linalg.norm(p-c['position_m'])*1000))
            candidate_rotation_error=max(candidate_rotation_error,float((r*Rotation.from_quat(c['quaternion_xyzw']).inv()).magnitude()*180/np.pi))
        assert candidate_fk_error<.01 and candidate_rotation_error<.01
        for a,b in zip(points,points[1:]):
            segments=max(1,int(np.ceil(np.max(np.abs(b-a))/np.deg2rad(.1))))
            for q in np.linspace(a,b,segments+1)[1:]:
                put(q);steps+=1
                for name,box in world_boxes().items():
                    maxima[name]=max(maxima[name],float(np.max(np.linalg.norm(box-original[name],axis=1))*1000))
                p,r=pose('controller_flange')
                flange_mm=max(flange_mm,float(np.linalg.norm(p-p0)*1000))
                angle=max(angle,float((r*r0.inv()).magnitude()))
        result=dict(qualification='Nominal rigid CAD displacement only; not certified physical clearance or stop overshoot',
            hardware_commands=0,pybullet_version=importlib.metadata.version('pybullet'),
            model_sha256=sha(model),mesh_sha256=hashes,source_sha256=sha(path),script_sha256=sha(Path(__file__)),
            candidate_count=len(data['candidates']),interpolated_samples=steps,max_joint_step_deg=.1,
            measured_base_fk_error_mm=base_error,candidate_kdl_fk_error_mm=candidate_fk_error,
            candidate_kdl_rotation_error_deg=candidate_rotation_error,
            per_link_corner_displacement_mm=maxima,flange_displacement_mm=flange_mm,
            flange_rotation_deg=angle*180/np.pi,
            assumed_attachment_radius_m=.30,attachment_displacement_bound_mm=flange_mm+600*np.sin(angle/2),
            attachment_radius_physically_measured=False,
            joint_min_deg=np.rad2deg(np.min(points,axis=0)).tolist(),joint_max_deg=np.rad2deg(np.max(points,axis=0)).tolist(),
            phase_boundaries='Full sequential candidate selection from one prepared session; all failed partial paths retained in source')
        output.write_text(json.dumps(result,indent=2),encoding='utf8');print(json.dumps(result))
    finally:pb.disconnect(physicsClientId=cid)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();analyze(args.path,args.output)
