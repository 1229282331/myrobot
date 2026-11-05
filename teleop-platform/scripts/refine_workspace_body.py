"""Derive a versioned body model from an existing verified assembly, offline.

Keep attachment geometry, transforms, TCP, limits and ACM byte-identical. Only
the two CAD housings identified by recorded false proximity are decomposed.
Every original triangle is enclosed; CoACD approximations are never colliders.
"""
from pathlib import Path
import argparse,hashlib,json,shutil,sys,xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import trimesh
from rigid_collision_geometry import decompose

LINKS=('base_link','Link2')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def build(source,output,cache):
    if output.exists():raise ValueError('New staging directory required')
    manifest=json.loads((source/'manifest.json').read_text())
    for name,digest in manifest['files'].items():
        if sha(source/name)!=digest:raise ValueError('Source assembly hash mismatch: '+name)
    shutil.copytree(source,output)
    tree=ET.parse(output/'robot.urdf');proofs={}
    for name in LINKS:
        original=trimesh.load(source/('e6-'+name+'-cad.stl'),force='mesh',process=False)
        files,proof=decompose(original,'e6-'+name,output,cache,tight=True)
        node=tree.find(f"link[@name='{name}']")
        from copy import deepcopy
        old=node.findall('collision');assert len(old)==1
        origin=old[0].find('origin')
        node.remove(old[0])
        for file in files:
            collision=ET.SubElement(node,'collision',name=file[:-4])
            if origin is not None:collision.append(deepcopy(origin))
            shape=ET.SubElement(collision,'geometry')
            ET.SubElement(shape,'mesh',filename=file)
        proofs[name]=dict(proof,source_file='e6-'+name+'-cad.stl')
    tree.write(output/'robot.urdf',encoding='utf-8',xml_declaration=True)
    manifest.pop('model_id');manifest.update(parent_model_id=json.loads((source/'manifest.json').read_text())['model_id'],
        rigid_collision='source_hulls_component_boxes_and_source_triangle_decomposition',body_refinement=proofs)
    for name in ('refine_workspace_body.py','rigid_collision_geometry.py'):
        path=Path(__file__).with_name(name)
        manifest['sources']['teleop-platform/scripts/'+name]=sha(path)
    # Old sample points belong to the parent model; never relabel their cache.
    (output/'reachability.json').unlink(missing_ok=True)
    manifest['files']={p.name:sha(p) for p in sorted(output.iterdir()) if p.is_file() and p.name!='manifest.json'}
    manifest['model_id']=hashlib.sha256(json.dumps(manifest,sort_keys=True).encode()).hexdigest()
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps({'model_id':manifest['model_id'],'proofs':proofs}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source','output','cache'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();build(a.source,a.output,a.cache)
