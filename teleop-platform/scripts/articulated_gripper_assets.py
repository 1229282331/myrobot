"""Official rigid links, local OBBs and a single versioned nominal pose table.

No hardware writes. Raw CAN aperture is measured; linkage angles derived from
it are nominal, not independent joint telemetry or contact deformation.
"""
import copy
import json
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from gripper_collision_geometry import linkage_angles

PREFIX = 'op_'


def build(assembly, additions, root_link, sweep, output, add_box):
    # The stationary motor/housing boxes remain on the original mounting link.
    for node in list(root_link.findall('visual')):
        root_link.remove(node)
    for node in list(root_link.findall('collision')):
        if node.get('name', '').startswith('finger_'):
            root_link.remove(node)
    visual = ET.SubElement(root_link, 'visual')
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh', filename='gripper-base_link.stl')
    names = {name: 'omnipicker' if name == 'base_link' else PREFIX + name for name in assembly.links}
    nodes = {'base_link': root_link}
    for name in assembly.links:
        if name != 'base_link':
            nodes[name] = ET.SubElement(additions, 'link', name=names[name])
    joints = []
    contacts = []
    for joint in assembly.joints:
        j = ET.SubElement(additions, 'joint', name=PREFIX+joint['name'], type='fixed' if joint['fixed'] else 'revolute')
        ET.SubElement(j, 'parent', link=names[joint['parent']])
        ET.SubElement(j, 'child', link=names[joint['child']])
        pose = joint['matrix']
        ET.SubElement(j, 'origin', xyz=' '.join(map(str, pose[:3, 3])), rpy=' '.join(map(str, Rotation.from_matrix(pose[:3,:3]).as_euler('xyz'))))
        if not joint['fixed']:
            ET.SubElement(j, 'axis', xyz=' '.join(map(str, joint['axis'])))
            # Not mechanical limits: these variables are read-only geometry.
            ET.SubElement(j, 'limit', lower='-3.141592653589793', upper='3.141592653589793', effort='1', velocity='1')
            joints.append(PREFIX+joint['name'])
        contacts.append([names[joint['parent']], names[joint['child']]])
    pieces=[]
    for name,path,origin in assembly.visuals:
        if name=='base_link': continue
        mesh=trimesh.load(path,force='mesh',process=False).apply_transform(origin)
        file=PREFIX+name+'.stl';mesh.export(output/file)
        visual=ET.SubElement(nodes[name],'visual')
        ET.SubElement(ET.SubElement(visual,'geometry'),'mesh',filename=file)
        ET.SubElement(ET.SubElement(visual,'material',name='op_gray'),'color',rgba='.22 .24 .27 1')
        to_box,size=trimesh.bounds.oriented_bounds(mesh.convex_hull.vertices)
        pose=np.linalg.inv(to_box)
        row=dict(name='finger_'+name,source_link=name,link=names[name],box_to_link=pose.tolist(),
                 xyz_m=pose[:3,3].tolist(),rpy_rad=Rotation.from_matrix(pose[:3,:3]).as_euler('xyz').tolist(),
                 size_m=(size+.01).tolist(),source_box_size_m=size.tolist(),reserve_m=.005)
        add_box(nodes[name],row);pieces.append(row)
    # Exact old free-stroke fallback, used only while feedback is unavailable.
    fallback=ET.SubElement(additions,'link',name='op_unknown_sweep')
    j=ET.SubElement(additions,'joint',name='op_unknown_mount',type='fixed')
    ET.SubElement(j,'parent',link='omnipicker');ET.SubElement(j,'child',link='op_unknown_sweep')
    for row in sweep['pieces']:add_box(fallback,row)
    contacts += [['op_unknown_sweep',n] for n in names.values()]
    contacts += [['op_unknown_sweep','r4_plate'],['op_unknown_sweep','gemini_e']]
    # Four-bar closing pivots (not adjacent in the tree), plus the two mating
    # finger pads. No exemptions against E6, camera, or mounting plate.
    for side in ('narrow','wide'):
        contacts += [[PREFIX+f'hand_{side}_loop_Link',PREFIX+f'hand_{side}2_Link'],
                     [PREFIX+f'hand_{side}_loop_Link',PREFIX+f'hand_{side}3_Link']]
    contacts += [[PREFIX+'hand_narrow3_Link',PREFIX+'hand_wide3_Link']]
    # Fixed motor pivots and the mechanically coupled four-bar/pad neighbors.
    # Reviewed against all 256 nominal states using original CAD: the fingers
    # themselves do not intersect; these pairs overlap only padded OBBs. Keep
    # the explicit list, never blanket-exempt fingers from E6 or accessories.
    for side in ('narrow','wide'):
        contacts += [['omnipicker',PREFIX+f'hand_{side}2_Link'],
                     [PREFIX+f'hand_{side}1_Link',PREFIX+f'hand_{side}3_Link'],
                     [PREFIX+f'hand_{side}1_Link',PREFIX+f'hand_{side}_loop_Link']]
    for a,b in [('narrow1_Link','wide1_Link'),('narrow3_Link','wide_loop_Link'),
                ('narrow_loop_Link','wide3_Link'),('narrow_loop_Link','wide_loop_Link'),
                ('narrow1_Link','wide_loop_Link'),('narrow_loop_Link','wide1_Link')]:
        contacts.append([PREFIX+'hand_'+a,PREFIX+'hand_'+b])
    table=[]
    for raw in range(256):
        angles=linkage_angles(assembly,-45.*raw/255.)
        table.append([angles[n.removeprefix(PREFIX)] for n in joints])
    description=dict(version=1,qualification='feedback_driven_nominal_unloaded_linkage_not_calibrated',
        source='official_360g_URDF_four_bar',joint_names=joints,joints_rad_by_raw=table,
        raw_range=[0,255],driver_deg_by_raw=[0.,-45.],passive_fingertip_deg=0.,
        movable_links=[row['link'] for row in pieces],fallback_link='op_unknown_sweep',
        local_boxes=pieces,feedback_max_age_ms=200,geometry_reserve_m=.005,
        mapping_note='CAN position is feedback. The 0..-45 degree linkage mapping is nominal; no force-dependent passive finger deflection is measured.')
    from dynamic_gripper_fallback import rebuild
    rebuild(additions,description)
    (output/'gripper-kinematics.json').write_text(json.dumps(description,indent=2),encoding='utf-8')
    return description,contacts
