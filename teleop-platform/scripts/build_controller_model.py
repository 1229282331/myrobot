"""Create a reviewable controller-equivalent URDF from exact FK identities.

Not a fitted local pose correction and not an IK implementation. The 13 zero /
single-axis basis queries identify serial-chain geometric totals. All other
poses are excluded from parameter derivation and must independently validate.
Nominal CAD is not thereby certified as physical collision/accessory geometry.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PIN='93502aa14c640bf2de576ef5fa7410373dc9cb1db9ccbb2867c21eea66220cf4'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def derive_basis(samples):
    basis=samples[:13]
    expected=[[0.]*6]
    for axis in range(6):
        for sign in (-1,1):
            q=[0.]*6;q[axis]=45.*sign;expected.append(q)
    if len(basis)!=13 or any(s['q_deg']!=q for s,q in zip(basis,expected)):
        raise ValueError('Require recorded zero and each single-axis +/-45 degree basis')
    poses=[s['controller_pose_mm_deg'] for s in basis]
    if any(len(p)!=6 or not all(math.isfinite(x) for x in p) for p in poses):
        raise ValueError('Invalid FK basis')
    k=math.sin(math.pi/4)
    # Independently derive the same distances from both +/- rotations.
    plus=[-poses[i][0]/k for i in (4,6,8)]
    minus=[poses[i][0]/k for i in (3,5,7)]
    if max(abs(x-y) for x,y in zip(plus,minus))>.001:
        raise ValueError('Single-axis FK symmetry inconsistent')
    r2,r3,r4=minus
    flange=-poses[9][0]/k
    if abs(flange-poses[10][0]/k)>.001:raise ValueError('Wrist FK symmetry inconsistent')
    # Round derived lengths to 1 um; do not promote reply quantization into
    # spurious extra calibration digits after division by sin(45).
    values=dict(shoulder_height=poses[0][2]-r2,upper_arm=r2-r3,forearm=r3-r4,
                wrist_height=r4,flange_axial=flange,parallel_axes_total=-poses[0][1]-flange)
    return {k:round(v,3)/1000 for k,v in values.items()}


def build(source,basis_file,srdf_file,output):
    if sha(source)!=PIN:raise ValueError('Unreviewed official URDF; review topology and identity derivation first')
    basis=json.loads(basis_file.read_text(encoding='utf-8'))
    values=derive_basis(basis['samples'])
    root=ET.parse(source).getroot()
    def xyz(name):return [float(x) for x in root.find(f"joint[@name='{name}']/origin").get('xyz').split()]
    old={name:xyz(name) for name in ('joint1','joint2','joint3','joint4','joint5','joint6')}
    changes=[]
    def change(name,index,value,reason):
        vector=xyz(name);previous=vector[index];vector[index]=value
        root.find(f"joint[@name='{name}']/origin").set('xyz',' '.join(format(x,'.12g') for x in vector))
        changes.append(dict(joint=name,component=index,old_m=previous,new_m=value,reason=reason))
    change('joint1',2,values['shoulder_height']-old['joint2'][2],'Base-to-J2 height identified by zero and J2 rotations')
    change('joint3',1,values['upper_arm'],'J2/J3 rotation radius difference')
    change('joint4',1,values['forearm'],'J3/J4 rotation radius difference')
    change('joint5',1,values['wrist_height']-old['joint6'][2],'J4 radius minus unchanged internal CAD wrist offset')
    # J2/J3/J4 axes are parallel. Their along-axis origin allocation is a
    # coordinate gauge: retain upstream allocation until the final wrist joint.
    # Only their identifiable sum is updated, with that convention explicit.
    change('joint5',2,values['parallel_axes_total']-(-old['joint2'][0]+old['joint3'][2]+old['joint4'][2]),
           'Identified parallel-axis total; keep upstream internal along-axis allocation')
    ET.SubElement(root,'link',name='controller_base')
    joint=ET.SubElement(root,'joint',name='controller_base_definition',type='fixed')
    ET.SubElement(joint,'parent',link='controller_base');ET.SubElement(joint,'child',link='dummy_link')
    ET.SubElement(joint,'origin',xyz='0 0 0',rpy=f'0 0 {math.pi/2:.17g}')
    flange_offset=values['flange_axial']-(-old['joint6'][0])
    ET.SubElement(root,'link',name='controller_flange')
    joint=ET.SubElement(root,'joint',name='controller_flange_definition',type='fixed')
    ET.SubElement(joint,'parent',link='Link6');ET.SubElement(joint,'child',link='controller_flange')
    ET.SubElement(joint,'origin',xyz=f'0 0 {flange_offset:.12g}',rpy=f'0 0 {math.pi:.17g}')
    output.mkdir(parents=True,exist_ok=False)
    urdf=output/'e6.urdf';ET.indent(root);ET.ElementTree(root).write(urdf,encoding='utf-8',xml_declaration=True)
    srdf=ET.parse(srdf_file);chain=srdf.getroot().find("group[@name='me6_group']/chain")
    chain.set('base_link','controller_base');chain.set('tip_link','controller_flange')
    srdf.write(output/'e6.srdf',encoding='utf-8',xml_declaration=True)
    profile=dict(kind='controller_equivalent_serial_kinematics',version=1,production_eligible=False,
        source_urdf_sha256=sha(source),basis_evidence_sha256=sha(basis_file),basis_indices=list(range(13)),
        geometry_m=values,changes=changes,flange_origin_from_Link6_m=[0,0,flange_offset],
        base_yaw_rad=math.pi/2,flange_yaw_rad=math.pi,urdf_sha256=sha(urdf),srdf_sha256=sha(output/'e6.srdf'),
        joint_mapping='Upstream joint1..6 order, same signs and zero angles; controller degrees converted to radians',
        collision_geometry='Nominal CAD only; internal parallel-axis frame origins are not physically measured',
        validation='Must validate held-out global poses and Jacobians with native FK; not a local residual offset')
    (output/'profile.json').write_text(json.dumps(profile,indent=2),encoding='utf-8');print(json.dumps(profile))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--basis',type=Path,required=True);p.add_argument('--srdf',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();build(a.source,a.basis,a.srdf,a.output)
