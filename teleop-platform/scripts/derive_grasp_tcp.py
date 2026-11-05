"""Offline nominal datum from fixed upstream finger meshes; no device access.

Use the opposing distal inner pads in the official zero assembly. This is
not a calibrated aperture model. Reproduce the selection and keep its bounds,
centroids and source hashes alongside the rounded control datum.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import numpy as np

root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root))
from omnipicker.studio.model import Assembly,stl_vertices

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--adapter-profile',choices=['r2','r4'],default='r4')
args=parser.parse_args()
m=Assembly();transforms=m.transforms({});pads=[];sources={}
for name,path,visual in m.visuals:
    if name not in ('hand_narrow3_Link','hand_wide3_Link'):continue
    vertices=stl_vertices(path)[:,:3].reshape(-1,3,3)/1000
    transform=transforms[name]@visual
    vertices=vertices@transform[:3,:3].T+transform[:3,3]
    cross=np.cross(vertices[:,1]-vertices[:,0],vertices[:,2]-vertices[:,0])
    area=np.linalg.norm(cross,axis=1)/2
    normals=cross/np.maximum(2*area[:,None],1e-20)
    centers=vertices.mean(1)
    selected=(centers[:,2]>.110)&(np.abs(centers[:,1])<.001)&(np.abs(normals[:,1])>.98)
    assert selected.any()
    points=vertices[selected].reshape(-1,3)
    pads.append(dict(link=name,triangle_count=int(selected.sum()),
        bounds_mm=(np.array([points.min(0),points.max(0)])*1000).tolist(),
        centroid_mm=(np.average(centers[selected],weights=area[selected],axis=0)*1000).tolist()))
    sources[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
for name in ('omnipicker/assets/official/robot_description/urdf/robot_urdf.xacro',
             'output/cad/E6_OmniPicker_adapter_R2/design_parameters_and_checks.json'):
    sources[name]=hashlib.sha256((root/name).read_bytes()).hexdigest()
height=float(np.mean([pad['centroid_mm'][2] for pad in pads]))
if args.adapter_profile=='r4':
    directory=root/'research/grippers/e6_omnipicker_pd01_r4_2026-10-12'
    properties=json.loads((directory/'mass_properties.json').read_text(encoding='utf-8'))
    for name in ('PD-01_R4.step','PD-01_R4_加工图.pdf'):
        path=directory/name
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        expected=[value for key,value in properties['sources_sha256'].items() if key.replace('\\','/').endswith('/'+name)]
        if expected!=[digest]:raise ValueError('R4 geometry source hash mismatch: '+name)
        sources[str(path.relative_to(root))]=digest
    adapter={'thickness':properties['plate']['dimensions_mm'][2]}
else:
    adapter=json.loads((root/'output/cad/E6_OmniPicker_adapter_R2/design_parameters_and_checks.json').read_text())
result=dict(kind='nominal_cad_control_datum',pads=pads,mean_pad_height_mm=height,
    profile='omnipicker-'+args.adapter_profile,
    adapter_thickness_mm=adapter['thickness'],translation_m=[0,0,round(height+adapter['thickness'],1)/1000],
    quaternion_xyzw=[0,0,0,1],sources=sources,
    definition='Opposing distal pad mean height at official zero pose; lateral symmetry axis; axes parallel to flange',
    physical_calibration=False,aperture_tracking=False,
    limitations='Fixed reference point, not instantaneous contact centroid of the adaptive fingers; no aperture-to-link calibration exists.')
print(json.dumps(result,ensure_ascii=False,indent=2))
