"""Independent serialized-polytope containment check, no robot access."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import trimesh
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from omnipicker.studio.model import Assembly
from gripper_collision_geometry import linkage_angles
from box_collision_geometry import clip_slab


def equations(path):
    hull=trimesh.load(path,force='mesh')
    # Use exported float32 STL, not the generator's float64 planes.
    return hull.face_normals,np.sum(hull.triangles_center*hull.face_normals,axis=1)


def coverage(triangles,parts):
    worst=-float('inf')
    for block in np.array_split(triangles,max(1,len(triangles)//250)):
        minima=np.full(len(block),np.inf)
        for normals,support in parts:
            excess=(block.reshape(-1,3)@normals.T-support).reshape(len(block),3,-1).max(axis=(1,2))
            minima=np.minimum(minima,excess)
        worst=max(worst,float(minima.max()))
    return worst


def run(assets,output):
    manifest=json.loads((assets/'manifest.json').read_text())
    if manifest['version']==3:
        return verify_boxes(assets,manifest,output)
    results=[]
    for name,proof in manifest['rigid_collision_proofs'].items():
        source='gripper-base_link-collision.stl' if name=='gripper-body' else name+'-collision.stl'
        mesh=trimesh.load(assets/source,force='mesh',process=False)
        parts=[equations(assets/row['file']) for row in proof['parts']]
        worst=coverage(mesh.triangles,parts)
        assert worst<1e-7,(name,worst)
        results.append(dict(name=name,source_triangles=len(mesh.faces),max_excess_m=worst,parts=len(parts)))
    assembly=Assembly();rng=np.random.default_rng(262)
    angles=rng.uniform(*manifest['gripper_collision']['driver_range_deg'],80)
    finger_results=[]
    for name,path,origin in assembly.visuals:
        if name=='base_link':continue
        # Check entire convex-support triangles against the union of exported
        # pieces, including edges crossing between angular partitions.
        mesh=trimesh.load(path,force='mesh').convex_hull
        parts=[equations(p) for p in sorted(assets.glob(f'gripper-{name}-sweep-*.stl'))]
        worst=-float('inf')
        for angle in angles:
            t=assembly.transforms(linkage_angles(assembly,float(angle)))[name]@origin
            triangles=mesh.triangles@t[:3,:3].T+t[:3,3]
            worst=max(worst,coverage(triangles,parts))
        assert worst<-.004,(name,worst)
        finger_results.append(dict(name=name,random_poses=len(angles),max_excess_m=worst))
    report=dict(model_id=manifest['model_id'],rigid=results,fingers=finger_results,
        source_triangle_count=sum(r['source_triangles'] for r in results),
        hardware_io=False,physical_calibration=False,passed=True)
    output.write_text(json.dumps(report,indent=2));print(json.dumps(report))


def verify_boxes(assets,manifest,output):
    """Read exported URDF primitives; independently clip CAD at declared cuts."""
    urdf=ET.parse(assets/'robot.urdf')
    def bounds(row):
        part=urdf.find(f".//collision[@name='{row['name']}']")
        center=np.fromstring(part.find('origin').get('xyz'),sep=' ')
        size=np.fromstring(part.find('geometry/box').get('size'),sep=' ')
        return np.array([center-size/2,center+size/2])
    def check_slabs(mesh,rows):
        assert rows[0]['slab_m'][0] is None and rows[-1]['slab_m'][1] is None
        for a,b in zip(rows[:-1],rows[1:]):assert a['slab_m'][1]==b['slab_m'][0]
        worst=-float('inf')
        for row in rows:
            fragment=clip_slab(mesh,row['axis'],*row['slab_m'])
            if not len(fragment.faces):continue
            box=bounds(row)
            excess=max(np.max(box[0]-fragment.vertices),np.max(fragment.vertices-box[1]))
            worst=max(worst,float(excess))
        return worst
    rigid=[]
    for name,proof in manifest['rigid_collision_proofs'].items():
        mesh=trimesh.load(assets/proof['source_file'],force='mesh')
        normals,support=equations(assets/proof['file'])
        worst=max(float(np.max(v@normals.T-support)) for v in np.array_split(mesh.vertices,max(1,len(mesh.vertices)//300)))
        assert worst<1e-7,(name,worst)
        rigid.append(dict(name=name,source_triangles=len(mesh.faces),max_excess_m=worst))
    box_results=[]
    for key,filename in [('r4_plate','r4-plate-collision.stl'),('gripper_body','gripper-base_link-collision.stl')]:
        mesh=trimesh.load(assets/filename,force='mesh')
        rows=manifest['box_collision_proofs'][key]
        worst=check_slabs(mesh,rows)
        assert worst<-.00099,(key,worst)
        box_results.append(dict(name=key,source_triangles=len(mesh.faces),boxes=len(rows),max_excess_m=worst))
    assembly=Assembly();rng=np.random.default_rng(263)
    low,high=manifest['gripper_collision']['driver_range_deg']
    angles=np.r_[low,high,rng.uniform(low,high,120)]
    fingers=[]
    for name,path,origin in assembly.visuals:
        if name=='base_link':continue
        side='narrow' if 'narrow' in name else 'wide'
        section='root' if ('1_Link' in name or 'loop_Link' in name) else 'tip'
        row=next(r for r in manifest['gripper_collision']['pieces']
                 if r.get('source_link')==name or ('source_link' not in r and r['side']==side and r['section']==section))
        part=urdf.find(f".//collision[@name='{row['name']}']")
        center=np.fromstring(part.find('origin').get('xyz'),sep=' ')
        rotation=Rotation.from_euler('xyz',np.fromstring(part.find('origin').get('rpy'),sep=' ')).as_matrix()
        size=np.fromstring(part.find('geometry/box').get('size'),sep=' ')
        mesh=trimesh.load(path,force='mesh').convex_hull;worst=-float('inf')
        for angle in angles:
            matrix=assembly.transforms(linkage_angles(assembly,float(angle)))[name]@origin
            vertices=mesh.vertices@matrix[:3,:3].T+matrix[:3,3]
            local=(vertices-center)@rotation
            worst=max(worst,float(np.max(np.abs(local)-size/2)))
        assert worst<-.0049,(name,worst)
        fingers.append(dict(name=name,poses=len(angles),max_excess_m=worst))
    union_check=None
    if manifest['gripper_collision'].get('display_union'):
        proof=manifest['gripper_collision']['display_union']
        rendered=trimesh.load(assets/proof['file'],force='mesh')
        assert rendered.is_watertight
        assert abs(rendered.volume-proof['volume_mm3']*1e-9)<1e-10
        points=np.random.default_rng(266).uniform(rendered.bounds[0]-.001,rendered.bounds[1]+.001,(10000,3))
        inside=np.zeros(len(points),bool)
        for row in manifest['gripper_collision']['pieces']:
            pose=np.array(row['box_to_link']);local=(points-pose[:3,3])@pose[:3,:3]
            inside|=np.all(np.abs(local)<=np.array(row['size_m'])/2,axis=1)
        mismatch=int(np.count_nonzero(inside!=rendered.contains(points)))
        assert mismatch==0
        union_check=dict(points=len(points),mismatch=mismatch,watertight=True,
            mesh_volume_m3=rendered.volume,cad_volume_m3=proof['volume_mm3']*1e-9)
    report=dict(model_id=manifest['model_id'],rigid=rigid,boxes=box_results,fingers=fingers,display_union=union_check,
        passed=True,hardware_io=False,physical_calibration=False,
        method='exported URDF boxes; rigid surface clipping and independent nominal angles in oriented finger boxes')
    output.write_text(json.dumps(report,indent=2));print(json.dumps(report))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--assets',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();run(args.assets,args.output)
