"""Component AABBs from source CAD and nominal free-stroke geometry.

Trimesh clips source surfaces into contiguous slabs before taking bounds. No
global hull or fitted display aperture is used for the attachment boxes.
"""
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from gripper_collision_geometry import linkage_angles


def clip_slab(mesh, axis, low, high):
    normal = np.eye(3)[axis]
    result = mesh
    def slice_at(current, direction, origin):
        vertices,faces,_=trimesh.intersections.slice_faces_plane(current.vertices,current.faces,direction,origin)
        clipped=trimesh.Trimesh(vertices=vertices,faces=faces,process=False)
        clipped.remove_unreferenced_vertices()
        return clipped
    if low is not None:
        result = slice_at(result, normal, normal * low)
    if high is not None and len(result.faces):
        result = slice_at(result, -normal, normal * high)
    return result


def slab_boxes(meshes, name, axis, cuts, reserve_m):
    limits = [None, *cuts, None]
    rows = []
    for index, (low, high) in enumerate(zip(limits[:-1], limits[1:])):
        fragments = [clip_slab(mesh, axis, low, high) for mesh in meshes]
        vertices = np.concatenate([m.vertices for m in fragments if len(m.faces)])
        raw = np.array([vertices.min(axis=0), vertices.max(axis=0)])
        bounds = raw + np.array([[-reserve_m], [reserve_m]])
        rows.append(dict(name=f'{name}_{index}',axis=axis,slab_m=[low,high],
                         source_bounds_m=raw.tolist(),bounds_m=bounds.tolist(),reserve_m=reserve_m))
    return rows


def build_attachment_boxes(plate, body, assembly):
    rigid = dict(
        r4_plate=slab_boxes([plate],'r4_plate',1,[.0325],.001),
        # The motor neck ends at 28.5 mm. Combining it with the wider housing
        # filled a 12 mm empty shoulder on each side, close to the wrist.
        gripper_body=slab_boxes([body],'gripper_body',2,[.0285,.0565],.001))
    # Same nominal range as v262, derived there from manufacturer endpoint
    # dimensions. Keep explicit uncertainty and reserve; do not tighten limits
    # just to make the drawing look smaller.
    angles = np.linspace(-50,5,221)
    source = [(name,trimesh.load(path,force='mesh').convex_hull,origin)
              for name,path,origin in assembly.visuals if name!='base_link']
    sweeps = {}
    for angle in angles:
        transforms = assembly.transforms(linkage_angles(assembly,float(angle)))
        for name, mesh, origin in source:
            side = 'narrow' if 'narrow' in name else 'wide'
            section='root' if ('1_Link' in name or 'loop_Link' in name) else 'tip'
            transformed=mesh.copy().apply_transform(transforms[name]@origin)
            sweeps.setdefault((side,section),{}).setdefault(name,[]).append(transformed.vertices)
    fingers = []
    for (side,section),members in sweeps.items():
        # Mature oriented-bounds fit follows each mechanical group's swept
        # direction. Axis-aligned height slabs create a false wide root wall.
        cloud=np.concatenate([v for clouds in members.values() for v in clouds])
        to_box,raw_size=trimesh.bounds.oriented_bounds(cloud,angle_digits=1)
        parent=np.linalg.inv(to_box)
        for name,clouds in members.items():
            # Fit each real CAD link in the SAME parent axes. Independently
            # rotating sub-boxes can expand the union despite smaller volumes.
            # These boxes are provably contained in the former parent box.
            vertices=np.concatenate(clouds)
            local=(vertices-parent[:3,3])@parent[:3,:3]
            low,high=local.min(axis=0),local.max(axis=0)
            pose=parent.copy();pose[:3,3]=parent[:3,:3]@((low+high)/2)+parent[:3,3]
            size=high-low
            fingers.append(dict(name=f'finger_{name}',side=side,section=section,source_link=name,
                box_to_link=pose.tolist(),size_m=(size+.01).tolist(),source_box_size_m=size.tolist(),
                parent_box_to_link=parent.tolist(),parent_box_size_m=(raw_size+.01).tolist(),
                xyz_m=pose[:3,3].tolist(),rpy_rad=Rotation.from_matrix(pose[:3,:3]).as_euler('xyz').tolist(),reserve_m=.005))
    return rigid, dict(method='per_cad_link_nested_oriented_boxes',
        driver_range_deg=[-50,5],driver_step_max_deg=.25,passive_fingertip_deg=0,
        supplier_dimensions_mm=[60.5,125,153],reserve_m=.005,
        qualification='nominal_free_open_close_not_measured_contact_deformation',pieces=fingers)
