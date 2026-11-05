"""Conservative convex hulls for offline MoveIt; visual meshes stay unchanged.

Uses trimesh / SciPy QHull, verifies every source vertex is enclosed. These
are nominal CAD envelopes, not calibrated physical/accessory collision models.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import trimesh


def build(source,output):
    output.mkdir(parents=True,exist_ok=False);rows=[]
    for path in sorted(source.glob('*.STL')):
        mesh=trimesh.load(path,force='mesh',process=False)
        # CAD files contain almost coincident vertices that leave degenerate
        # hull faces after Trimesh cleanup. Quantize to a 10 um grid, then
        # conservatively expand ALL supporting planes by at least 100 um.
        # Verify against the original, unquantized vertices below.
        hull=trimesh.convex.convex_hull(np.unique(np.round(mesh.vertices,5),axis=0))
        center=hull.center_mass
        radii=np.sum((hull.triangles_center-center)*hull.face_normals,axis=1)
        if np.min(radii)<=0:raise ValueError('Invalid convex supporting planes')
        hull.vertices=center+(hull.vertices-center)*(1+.0001/np.min(radii))
        # Signed outward halfspaces, tested in bounded chunks.
        normals=hull.face_normals;origins=hull.triangles_center
        maxima=[]
        for chunk in np.array_split(mesh.vertices,max(1,len(mesh.vertices)//2000)):
            maxima.append(float(np.max(chunk@normals.T-np.sum(origins*normals,axis=1))))
        excess=max(maxima)
        if excess>1e-7 or not hull.is_watertight:raise ValueError('Non-conservative hull: '+path.name)
        data=hull.export(file_type='stl');target=output/path.name;target.write_bytes(data)
        rows.append(dict(name=path.name,original_faces=len(mesh.faces),hull_faces=len(hull.faces),
            max_outside_m=excess,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            hull_sha256=hashlib.sha256(data).hexdigest()))
    if len(rows)!=7:raise ValueError('Require all seven official E6 meshes')
    result=dict(trimesh_version=trimesh.__version__,nominal_geometry_only=True,
                quantization_m=.00001,minimum_outward_padding_m=.0001,meshes=rows)
    (output/'manifest.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();build(a.source,a.output)
