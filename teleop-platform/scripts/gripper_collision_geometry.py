"""Nominal unloaded linkage sweep from official CAD, independent of aperture UI.

The manufacturer only supplies endpoint dimensions, not calibrated joint limits.
This is an explicitly nominal free-stroke envelope, NOT a contact-deformation
model. Passive fingertip joints use their unloaded CAD zero. The alternative
all +/-pi URDF-angle experiment is retained in collision-v262 evidence; those
placeholder limits are not the mechanical stroke.
"""
import math
import numpy as np
import trimesh
from scipy.optimize import minimize_scalar
from scipy.spatial import HalfspaceIntersection
from omnipicker.studio.model import Assembly, rotation


def linkage_angles(assembly, driver_deg):
    """Close the CAD four-bar on its zero-assembly branch, in absolute degrees."""
    result = {}
    for side, sign in (('narrow', 1), ('wide', -1)):
        prefix = 'hand_' + side
        root = assembly.by_name[prefix+'1_joint']['matrix']
        loop = assembly.by_name[prefix+'_loop_joint']['matrix']
        b = assembly.by_name[prefix+'2_joint']['matrix'][:2, 3]
        c = assembly.by_name[prefix+'3_joint']['matrix'][:2, 3]
        e = assembly.by_name[prefix+'4_joint']['matrix'][:2, 3]
        d = (np.linalg.inv(root) @ loop)[:2, 3]
        e0 = b+c+e
        q1 = sign*math.radians(driver_deg)
        bn = rotation([0,0,-1], q1)[:2,:2] @ b
        be, de = np.linalg.norm(c+e), np.linalg.norm(e0-d)
        delta = d-bn
        distance = np.linalg.norm(delta)
        along = (be*be-de*de+distance*distance)/(2*distance)
        height2 = be*be-along*along
        if height2 < 0:
            raise ValueError('Nominal linkage cannot close')
        unit = delta/distance
        center = bn+along*unit
        offset = math.sqrt(height2)*np.array([-unit[1],unit[0]])
        end = min((center+offset,center-offset),key=lambda p:np.linalg.norm(p-e0))
        angle = lambda p: math.atan2(p[1],p[0])
        wrap = lambda v: math.atan2(math.sin(v),math.cos(v))
        result[prefix+'1_joint'] = q1
        result[prefix+'2_joint'] = wrap(-(angle(end-bn)-angle(c+e))-q1)
        result[prefix+'3_joint'] = 0.
        result[prefix+'_loop_joint'] = wrap(-(angle(end-d)-angle(e0-d)))
    return result


def outer_polytope(points, reserve_m):
    """Mature SciPy/Qhull halfspace intersection: every source point is inside.

    Plane offsets reserve 5 mm in EVERY direction; finite direction resolution
    can only enlarge this envelope, unlike unsafe mesh decimation of a hull.
    """
    # 42 support directions give <=156 triangular faces per piece. The 162
    # direction version passed coverage but exceeded a 100 ms collision tick
    # on X5; retain decomposition and conservative containment with fewer faces.
    normals = trimesh.creation.icosphere(subdivisions=1).vertices
    support = np.full(len(normals),-np.inf)
    for block in np.array_split(points,max(1,len(points)//2000)):
        support = np.maximum(support,np.max(block@normals.T,axis=0))
    support += reserve_m
    vertices = HalfspaceIntersection(np.column_stack([normals,-support]),points.mean(axis=0)).intersections
    # STL stores float32: remove sub-micron halfspace-intersection slivers
    # before triangulation, otherwise nearly coincident vertices collapse on
    # export. At 10 um quantization the maximum displacement is 8.7 um, below
    # even the rigid pieces' 100 um numerical reserve. Verify the actual STL
    # separately, rather than only these ideal support planes.
    mesh = trimesh.convex.convex_hull(np.unique(np.round(vertices,5),axis=0))
    assert mesh.is_watertight and mesh.volume>0
    return mesh, normals, support


def build_sweeps(assembly, output):
    # Support vertices of each official rigid mesh suffice for any linear
    # halfspace containment check, without approximating its convex support.
    pieces = {n:(trimesh.load(p,force='mesh').convex_hull.vertices,o) for n,p,o in assembly.visuals}
    def points(driver):
        t = assembly.transforms(linkage_angles(assembly,driver))
        return {n:v@(t[n]@o)[:3,:3].T+(t[n]@o)[:3,3] for n,(v,o) in pieces.items()}
    def dimensions(driver):
        p=np.concatenate(list(points(driver).values()))
        return np.ptp(p,axis=0)*1000
    # Fit each published dimension independently and keep the union, rather
    # than treating conflicting endpoint drawings as an exact calibration.
    width=minimize_scalar(lambda d:(dimensions(d)[1]-125)**2,bounds=(-50,-25),method='bounded').x
    height=minimize_scalar(lambda d:(dimensions(d)[2]-143)**2,bounds=(-50,-25),method='bounded').x
    # Round outwards and add an explicit 5 degree engineering reserve.
    lower=math.floor(min(width,height)/5)*5-5
    upper=5.
    cuts=np.linspace(lower,upper,4)
    files=[];checks=[]
    for i,(lo,hi) in enumerate(zip(cuts[:-1],cuts[1:])):
        steps=math.ceil((hi-lo)/.25)
        angles=np.linspace(lo,hi,steps+1)
        samples=[points(d) for d in angles]
        midpoints=[points(d) for d in (angles[:-1]+angles[1:])/2]
        for name in pieces:
            if name=='base_link':continue
            cloud=np.concatenate([p[name] for p in samples])
            mesh,normals,support=outer_polytope(cloud,.005)
            # An independent half-step grid verifies interpolation clearance;
            # retain measured margin, not a claim of physical calibration.
            check=np.concatenate([p[name] for p in midpoints])
            excess=max(float(np.max(block@normals.T-support)) for block in np.array_split(check,max(1,len(check)//2000)))
            if excess>1e-7:raise ValueError('Sweep undercoverage: '+name)
            filename=f'gripper-{name}-sweep-{i}.stl'
            mesh.export(output/filename);files.append(filename)
            checks.append(dict(link=name,driver_deg=[float(lo),float(hi)],faces=len(mesh.faces),
                               half_step_max_excess_m=excess,bounds_m=mesh.bounds.tolist()))
    return files,dict(method='per_rigid_link_three_arc_support_polytopes',
        driver_range_deg=[lower,upper],driver_step_max_deg=.25,passive_fingertip_deg=0,
        endpoint_fit_deg=dict(width=width,height=height),endpoint_fit_dimensions_mm=dict(width=dimensions(width).tolist(),height=dimensions(height).tolist()),
        supplier_dimensions_mm=[60.5,125,153],reserve_m=.005,angular_reserve_deg=5,
        qualification='nominal_free_open_close_not_measured_contact_deformation',
        source_url='https://www.agibot.com.cn/DOCS/PM/X1',pieces=checks)
