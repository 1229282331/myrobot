"""Pinned URDF joint origins for conservative sweep estimates, not collision meshes."""
import math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
MODEL=Path(__file__).resolve().parents[1]/'research/sources/me6_robot_ec201c40.urdf'

def transform(xyz,rpy):
    x,y,z=rpy;cx,sx=math.cos(x),math.sin(x);cy,sy=math.cos(y),math.sin(y);cz,sz=math.cos(z),math.sin(z)
    t=np.eye(4);t[:3,3]=xyz
    t[:3,:3]=[[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]]
    return t

def arm_origins(q):
    joints=[j for j in ET.parse(MODEL).getroot().findall('joint') if j.attrib['type']=='revolute']
    if len(joints)!=6 or len(q)!=6:raise ValueError('Require pinned six-axis model')
    t=np.eye(4);out=[]
    for j,v in zip(joints,q):
        o=j.find('origin');t=t@transform([float(x) for x in o.attrib['xyz'].split()],[float(x) for x in o.attrib['rpy'].split()])
        out.append((t[:3,3]*1000).tolist());t=t@transform([0,0,0],[0,0,math.radians(v)])
    return out
