"""Reproducible nominal assembly; full CAD visuals and separate collision surfaces.

Run in .venv-cad. Does not connect to devices. The fixed nominal free-stroke
envelope never uses the display-only aperture mapping as measured joint state.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import cadquery as cq
import shutil
import tempfile
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'teleop-platform/src')]
from omnipicker.studio.model import Assembly
from e6_phosphobot.tool_frame import get_tool


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(output, dynamic_gripper=False):
    ASSETS = output.resolve()
    if ASSETS.exists() and any(ASSETS.iterdir()):
        raise ValueError("Build into a new empty staging directory; never overwrite a live model")
    ASSETS.mkdir(parents=True, exist_ok=True)
    sources = {}
    collision_proofs = {}
    from prepare_collision_meshes import build as build_arm_hulls
    hull_cache=tempfile.TemporaryDirectory(prefix='e6-box-build-')
    hull_dir=Path(hull_cache.name)/'hulls'
    arm_sources=ROOT/'teleop-platform/upstream/dobot-ros2/dobot_rviz/meshes/me6'
    build_arm_hulls(arm_sources,hull_dir)
    hull_proofs={row['name']:row for row in json.loads((hull_dir/'manifest.json').read_text())['meshes']}
    def mesh(source, name, transform=None):
        sources[str(source.relative_to(ROOT)).replace('\\','/')] = sha(source)
        original = trimesh.load(source, force='mesh', process=False)
        if transform is not None:
            original.apply_transform(transform)
        # Keep the source CAD silhouette. The former aggressive QEM pass
        # collapsed narrow housings and linked finger parts in the viewer.
        # Collision hulls remain separate; visual meshes are transferred once.
        visual = original.copy()
        visual.export(ASSETS/(name+'.stl'))
        # Retain the full CAD separately for coverage verification. The URDF
        # references the conservative collision primitives, not a visual LOD.
        original.export(ASSETS/(name+'-collision.stl'))
        return original
    model=ROOT/'teleop-platform/config/e6-controller-model/e6.urdf'
    sources[str(model.relative_to(ROOT))]=sha(model)
    tree=ET.parse(model);robot=tree.getroot()
    for link in robot.findall('link'):
        visual=link.find('visual/geometry/mesh')
        if visual is None:continue
        name='e6-'+link.get('name')
        original=mesh(ROOT/'teleop-platform/upstream/dobot-ros2/dobot_rviz/meshes/me6'/Path(visual.get('filename')).name,name)
        raw=ASSETS/(name+'-collision.stl')
        raw.rename(ASSETS/(name+'-cad.stl'))
        original_name=Path(visual.get('filename')).name
        shutil.copyfile(hull_dir/original_name,raw)
        collision_proofs[name]=dict(hull_proofs[original_name],source_file=name+'-cad.stl',file=raw.name)
        visual.set('filename',name+'.stl')
        link.find('collision/geometry/mesh').set('filename',raw.name)
    hull_cache.cleanup()
    additions=ET.Element('robot',name='e6_attachments')
    def link(name, xyz, visual_file=None, collision_file=None, rpy='0 0 0',color='.18 .2 .23 1'):
        node=ET.SubElement(additions,'link',name=name)
        joint=ET.SubElement(additions,'joint',name=name+'_mount',type='fixed')
        ET.SubElement(joint,'parent',link='controller_flange');ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz=xyz,rpy=rpy)
        for kind,file in [('visual',visual_file),('collision',collision_file)]:
            if file:
                part=ET.SubElement(node,kind);g=ET.SubElement(part,'geometry');ET.SubElement(g,'mesh',filename=file)
                if kind=='visual':ET.SubElement(ET.SubElement(part,'material',name=name),'color',rgba=color)
        return node
    step=ROOT/'research/grippers/e6_omnipicker_pd01_r4_2026-10-12/PD-01_R4.step'
    sources[str(step.relative_to(ROOT))]=sha(step)
    shape=cq.importers.importStep(str(step)).val()
    vertices,faces=shape.tessellate(.15)
    plate=trimesh.Trimesh(vertices=np.array([v.toTuple() for v in vertices])*.001,faces=np.array(faces),process=False)
    plate.export(ASSETS/'r4-plate.stl');plate.export(ASSETS/'r4-plate-collision.stl')
    plate_link=link('r4_plate','0 0 0','r4-plate.stl',rpy='0 0 3.141592653589793',color='.65 .67 .7 1')
    gripper_urdf=ROOT/'omnipicker/assets/official/robot_description/urdf/robot_urdf.xacro'
    sources[str(gripper_urdf.relative_to(ROOT))]=sha(gripper_urdf)
    assembly=Assembly();transforms=assembly.transforms({})
    visual_meshes=[]
    for name,path,origin in assembly.visuals:
        source_mesh=mesh(path,'gripper-'+name,transforms[name]@origin)
        visual_meshes.append(source_mesh)
    combined=trimesh.util.concatenate(visual_meshes)
    combined.export(ASSETS/'gripper-visual.stl')
    gripper=link('omnipicker','0 0 .008','gripper-visual.stl')
    from apply_gripper_mount import mounting, apply as apply_mount, CONFIG as mount_config
    confirmed_mount=mounting()
    apply_mount(additions,confirmed_mount)
    sources[mount_config.relative_to(ROOT).as_posix()]=sha(mount_config)
    body=trimesh.load(ASSETS/'gripper-base_link-collision.stl',force='mesh',process=False)
    from box_collision_geometry import build_attachment_boxes
    box_proofs,gripper_collision=build_attachment_boxes(plate,body,assembly)
    def add_box(node,row):
        if 'box_to_link' in row:
            xyz=np.array(row['xyz_m']);rpy=np.array(row['rpy_rad']);size=np.array(row['size_m'])
        else:
            bounds=np.array(row['bounds_m']);xyz=bounds.mean(axis=0);rpy=np.zeros(3);size=bounds[1]-bounds[0]
        part=ET.SubElement(node,'collision',name=row['name'])
        ET.SubElement(part,'origin',xyz=' '.join(f'{v:.12g}' for v in xyz),rpy=' '.join(f'{v:.12g}' for v in rpy))
        ET.SubElement(ET.SubElement(part,'geometry'),'box',size=' '.join(f'{v:.12g}' for v in size))
    for row in box_proofs['r4_plate']:add_box(plate_link,row)
    for row in [*box_proofs['gripper_body'],*gripper_collision['pieces']]:add_box(gripper,row)
    # The control model still uses these individual boxes. Render their exact
    # OpenCascade Boolean union so internal overlaps do not draw a wire tangle.
    solids=[]
    for row in gripper_collision['pieces']:
        size=np.array(row['size_m'])*1000
        solid=cq.Solid.makeBox(*size,cq.Vector(*(-size/2)))
        matrix=np.array(row['box_to_link']);rot=Rotation.from_matrix(matrix[:3,:3]).as_rotvec()
        angle=np.linalg.norm(rot)
        if angle>1e-12:solid=solid.rotate((0,0,0),tuple(rot/angle),np.degrees(angle))
        solids.append(solid.translate(tuple(matrix[:3,3]*1000)))
    union=solids[0].fuse(*solids[1:]).clean()
    assert union.isValid()
    vertices,faces=union.tessellate(.01)
    display_union=trimesh.Trimesh(vertices=np.array([v.toTuple() for v in vertices])*.001,faces=np.array(faces),process=True)
    display_union.export(ASSETS/'finger-sweep-union.stl')
    gripper_collision['display_union']=dict(file='finger-sweep-union.stl',
        method='CadQuery OpenCascade Boolean union of the same URDF boxes',
        volume_mm3=union.Volume(),triangles=len(display_union.faces))
    dynamic=None;dynamic_contacts=[]
    if dynamic_gripper:
        from articulated_gripper_assets import build as articulate
        dynamic,dynamic_contacts=articulate(assembly,additions,gripper,gripper_collision,ASSETS,add_box)
        gripper_collision.pop('display_union',None)
    for i in (1,2,3):
        p=ROOT/f'evidence/workspace-v261/omnipicker-dimensions-{i}.png'
        sources[str(p.relative_to(ROOT))]=sha(p)
    for script in ('build_workspace_assets.py','gripper_collision_geometry.py','box_collision_geometry.py','prepare_collision_meshes.py','apply_gripper_mount.py'):
        p=Path(__file__).with_name(script);sources[str(p.relative_to(ROOT))]=sha(p)
    camera=link('gemini_e','0 -.060 .02055',color='.15 .2 .28 1')
    for kind in ('visual','collision'):
        part=ET.SubElement(camera,kind);ET.SubElement(ET.SubElement(part,'geometry'),'box',size='.08982 .0251 .0251')
        if kind=='visual':ET.SubElement(ET.SubElement(part,'material',name='camera'),'color',rgba='.12 .16 .22 1')
    ET.ElementTree(additions).write(ASSETS/'attachments.urdf',encoding='utf-8',xml_declaration=True)
    # The viewer gets the same fixed transforms and the same collision shapes.
    from copy import deepcopy
    for child in additions:robot.append(deepcopy(child))
    tcp=get_tool('omnipicker-r4')
    srdf_path=ROOT/'teleop-platform/config/e6-controller-model/e6.srdf'
    sources[str(srdf_path.relative_to(ROOT))]=sha(srdf_path)
    srdf=srdf_path.read_text()
    base=dict(robot_description=ET.tostring(robot,encoding='unicode'),robot_description_semantic=srdf)
    base=tcp.attach(base)
    (ASSETS/'robot.urdf').write_text(base['robot_description'],encoding='utf-8')
    semantic=ET.fromstring(base['robot_description_semantic'])
    contacts=[['r4_plate','Link6'],['r4_plate','omnipicker'],['r4_plate','gemini_e'],['omnipicker','gemini_e']]
    contacts+=dynamic_contacts
    for a,b in contacts:ET.SubElement(semantic,'disable_collisions',link1=a,link2=b,reason='Attached mounting contact')
    if dynamic:
        group=ET.SubElement(semantic,'group',name='e6_gripper_geometry')
        for name in [*dynamic['movable_links'],dynamic['fallback_link']]:ET.SubElement(group,'link',name=name)
    ET.ElementTree(semantic).write(ASSETS/'robot.srdf',encoding='utf-8',xml_declaration=True)
    info=dict(version=3,frame='controller_base',tcp=tcp.description(),environment_modeled=False,
              calibration='nominal_cad_unmeasured',gripper_pose='nominal_zero_not_joint_telemetry',
              allowed_contacts=contacts,sources=sources,mounting=confirmed_mount,
              rigid_collision='source_convex_hulls_and_component_boxes',rigid_collision_proofs=collision_proofs,
              box_collision_proofs=box_proofs,
              gripper_collision=gripper_collision,
              joint_margin_deg=5.,voxel_m=.01,files={})
    if dynamic:
        p=Path(__file__).with_name('articulated_gripper_assets.py');sources[str(p.relative_to(ROOT))]=sha(p)
        p=Path(__file__).with_name('dynamic_gripper_fallback.py');sources[str(p.relative_to(ROOT))]=sha(p)
        info.update(version=4,gripper_pose='feedback_driven_nominal_linkage',
                    dynamic_gripper={k:v for k,v in dynamic.items() if k!='joints_rad_by_raw'})
    for p in sorted(ASSETS.iterdir()):
        if p.name not in ('manifest.json','reachability.json'):info['files'][p.name]=sha(p)
    info['model_id']=hashlib.sha256(json.dumps(info,sort_keys=True).encode()).hexdigest()
    (ASSETS/'manifest.json').write_text(json.dumps(info,indent=2),encoding='utf-8')
    print(json.dumps(dict(model_id=info['model_id'],gripper_pieces=len(gripper_collision['pieces']),assets=len(info['files']))))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--dynamic-gripper',action='store_true')
    args=p.parse_args();build(args.output,args.dynamic_gripper)
