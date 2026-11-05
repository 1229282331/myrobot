"""Offline geometry primitives for the first grasp. Units are millimetres.

No hardware imports or commands. Fit results alone never authorize motion.
"""
import math
import numpy as np

class GeometryError(ValueError):
    pass

def checked_transform(value):
    t=np.asarray(value,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1],atol=1e-8):
        raise GeometryError('Require a finite 4x4 homogeneous transform')
    r=t[:3,:3]
    if not np.allclose(r.T@r,np.eye(3),atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6:
        raise GeometryError('Rotation must be proper and orthonormal')
    return t

def pose_transform(pose):
    p=np.asarray(pose,dtype=float)
    if p.shape!=(6,) or not np.isfinite(p).all():raise GeometryError('Require XYZ mm and RxRyRz degrees')
    x,y,z=np.deg2rad(p[3:]);cx,sx=math.cos(x),math.sin(x);cy,sy=math.cos(y),math.sin(y);cz,sz=math.cos(z),math.sin(z)
    rx=np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
    ry=np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
    rz=np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
    t=np.eye(4);t[:3,:3]=rz@ry@rx;t[:3,3]=p[:3];return t

def deproject_mm(depth,intrinsic):
    a=np.asarray(depth,dtype=float)
    if a.shape!=(intrinsic['height'],intrinsic['width']) or not np.isfinite(a).all() or np.any(a<0):
        raise GeometryError('Depth shape/value mismatch')
    if not all(math.isfinite(intrinsic[k]) for k in ['fx','fy','cx','cy']) or min(intrinsic['fx'],intrinsic['fy'])<=0:
        raise GeometryError('Missing camera intrinsics')
    v,u=np.indices(a.shape)
    points=np.stack([(u-intrinsic['cx'])*a/intrinsic['fx'],(v-intrinsic['cy'])*a/intrinsic['fy'],a],axis=-1)
    points[a==0]=np.nan  # Missing depth never becomes a point at the origin.
    return points


def intersect_plane_pixels(pixels_xy, intrinsic, normal, offset_mm):
    """Intersect measured RGB pixels with a plane, in the camera frame.

    A bottle's visible table contact can use this model. Its cap cannot:
    projecting an elevated point onto the table introduces a systematic bias.
    No missing depth is filled and no robot target is generated here.
    """
    pixels = np.asarray(pixels_xy, dtype=float)
    n = np.asarray(normal, dtype=float)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or not np.isfinite(pixels).all():
        raise GeometryError('Require finite Nx2 image pixels')
    if n.shape != (3,) or not np.isfinite(n).all() or abs(np.linalg.norm(n)-1) > 1e-6:
        raise GeometryError('Plane normal must be a finite unit vector')
    if not math.isfinite(offset_mm):
        raise GeometryError('Invalid plane offset')
    if not all(math.isfinite(intrinsic[k]) for k in ['fx','fy','cx','cy']) or min(intrinsic['fx'], intrinsic['fy']) <= 0:
        raise GeometryError('Missing camera intrinsics')
    if np.any(pixels < 0) or np.any(pixels[:, 0] >= intrinsic['width']) or np.any(pixels[:, 1] >= intrinsic['height']):
        raise GeometryError('Pixel outside calibrated image')
    rays = np.column_stack([(pixels[:,0]-intrinsic['cx'])/intrinsic['fx'],
                            (pixels[:,1]-intrinsic['cy'])/intrinsic['fy'], np.ones(len(pixels))])
    denominator = rays @ n
    if np.any(np.abs(denominator)/np.linalg.norm(rays, axis=1) < .15):
        raise GeometryError('Ray is too close to parallel with the table')
    z = -offset_mm / denominator
    if np.any(z < 100) or np.any(z > 2000):
        raise GeometryError('Intersection behind camera or outside initial working range')
    return rays * z[:, None]

def fit_plane(points,threshold_mm=3,min_support=.7,iterations=220,seed=17):
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3:raise GeometryError('Require Nx3 points')
    p=p[np.isfinite(p).all(axis=1)]
    if len(p)<100 or not 0<threshold_mm<=5 or not .5<=min_support<=1:raise GeometryError('Insufficient data or invalid plane bounds')
    rng=np.random.default_rng(seed)
    small=p[rng.choice(len(p),min(len(p),4000),replace=False)]
    best=None;score=-1
    for _ in range(iterations):
        a,b,c=small[rng.choice(len(small),3,replace=False)]
        n=np.cross(b-a,c-a);norm=np.linalg.norm(n)
        if norm<1e-6:continue
        n/=norm;d=-n@a;mask=np.abs(small@n+d)<=threshold_mm;s=int(mask.sum())
        if s>score:best=(n,d);score=s
    if best is None:raise GeometryError('No plane hypothesis')
    mask=np.abs(p@best[0]+best[1])<=threshold_mm
    if mask.mean()<min_support:raise GeometryError('Selected ROI is not predominantly one plane')
    inliers=p[mask];center=inliers.mean(axis=0)
    _,sv,vh=np.linalg.svd(inliers-center,full_matrices=False)
    if sv[1]<1:raise GeometryError('Plane points are collinear or spatially degenerate')
    n=vh[-1]
    if n[2]>0:n=-n
    d=-n@center;residual=inliers@n+d
    return {'normal':n.tolist(),'offset_mm':float(d),'support':float(mask.mean()),'inlier_count':len(inliers),'rms_mm':float(np.sqrt(np.mean(residual**2))),'residual_p95_mm':float(np.percentile(np.abs(residual),95)),'scope':'Observed surface continuity, not absolute distance calibration'}

def solve_pivot(transforms,max_rms_mm=2):
    if len(transforms)<6:raise GeometryError('At least six independently oriented contact poses required')
    ts=[checked_transform(t) for t in transforms]
    a=np.concatenate([np.column_stack([t[:3,:3],-np.eye(3)]) for t in ts])
    b=np.concatenate([-t[:3,3] for t in ts])
    solution,_,rank,sv=np.linalg.lstsq(a,b,rcond=None)
    if rank!=6 or sv[-1]/sv[0]<.01:raise GeometryError('Pivot orientations do not identify all TCP components')
    residual=(a@solution-b).reshape(-1,3);rms=float(np.sqrt(np.mean(np.sum(residual**2,axis=1))))
    if rms>max_rms_mm or np.linalg.norm(solution[:3])>300:raise GeometryError('Pivot fit fails residual/physical bounds')
    return {'flange_to_tcp_mm':solution[:3].tolist(),'fixed_point_base_mm':solution[3:].tolist(),'rms_mm':rms,'condition_number':float(sv[0]/sv[-1]),'status':'fit_only','validated_for_motion':False}

def fit_hand_eye(gripper_to_base,target_to_camera):
    import cv2
    if len(gripper_to_base)!=len(target_to_camera) or len(gripper_to_base)<8:
        raise GeometryError('Require at least eight paired stationary observations')
    gs=[checked_transform(t) for t in gripper_to_base];cs=[checked_transform(t) for t in target_to_camera]
    vectors=np.array([cv2.Rodrigues(g[:3,:3]@gs[0][:3,:3].T)[0][:,0] for g in gs[1:]])
    sv=np.linalg.svd(vectors,compute_uv=False)
    if sv[1]<math.radians(5):raise GeometryError('Hand-eye rotations need excitation around two independent axes')
    r,t=cv2.calibrateHandEye([g[:3,:3] for g in gs],[g[:3,3] for g in gs],[c[:3,:3] for c in cs],[c[:3,3] for c in cs],method=cv2.CALIB_HAND_EYE_PARK)
    x=np.eye(4);x[:3,:3]=r;x[:3,3]=t[:,0];checked_transform(x)
    if np.linalg.norm(x[:3,3])>300:raise GeometryError('Camera offset outside physical sanity bound')
    targets=[g@x@c for g,c in zip(gs,cs)]
    positions=np.array([v[:3,3] for v in targets]);center=positions.mean(axis=0)
    rms=float(np.sqrt(np.mean(np.sum((positions-center)**2,axis=1))))
    angles=[math.degrees(np.linalg.norm(cv2.Rodrigues(v[:3,:3]@targets[0][:3,:3].T)[0])) for v in targets]
    if rms>3 or max(angles)>2:raise GeometryError('Hand-eye fixed-target consistency failed')
    # Reference comes exclusively from fitting observations; validation must
    # compare new observations to it without re-estimating the hand-eye matrix.
    u,_,vh = np.linalg.svd(sum(v[:3,:3] for v in targets))
    correction = np.diag([1,1,np.linalg.det(u@vh)])
    reference = np.eye(4)
    reference[:3,:3] = u @ correction @ vh
    reference[:3,3] = center
    return {'T_flange_camera_mm':x.tolist(),'T_base_target_reference_mm':reference.tolist(),'fixed_target_position_rms_mm':rms,'fixed_target_rotation_max_deg':max(angles),'rotation_excitation_singular_values_rad':sv.tolist(),'status':'fit_only','validated_for_motion':False,'required_next':'Independent held-out target/position checks before robot target generation'}


def validate_hand_eye(fit, gripper_to_base, target_to_camera):
    """Check held-out fixed-target observations without refitting.

    Dataset provenance/disjointness is checked by the CLI. Passing geometric
    closure alone does not check TCP, physical scale, or collision clearance.
    """
    import cv2
    if fit.get('status') != 'fit_only' or len(gripper_to_base) != len(target_to_camera) or len(gripper_to_base) < 3:
        raise GeometryError('Require a fit and at least three held-out observation pairs')
    x = checked_transform(fit['T_flange_camera_mm'])
    reference = checked_transform(fit['T_base_target_reference_mm'])
    gs = [checked_transform(g) for g in gripper_to_base]
    cs = [checked_transform(c) for c in target_to_camera]
    rotation_spread = [np.linalg.norm(cv2.Rodrigues(g[:3,:3] @ gs[0][:3,:3].T)[0]) for g in gs[1:]]
    if max(rotation_spread) < math.radians(5):
        raise GeometryError('Held-out views must include a different orientation')
    predicted = [g @ x @ c for g,c in zip(gs,cs)]
    errors = [float(np.linalg.norm(t[:3,3]-reference[:3,3])) for t in predicted]
    angles = [float(math.degrees(np.linalg.norm(cv2.Rodrigues(t[:3,:3] @ reference[:3,:3].T)[0]))) for t in predicted]
    if max(errors) > 3 or max(angles) > 1:
        raise GeometryError('Held-out fixed-target closure exceeds 3 mm or 1 degree')
    return {'status':'heldout_consistency_only','position_errors_mm':errors,
            'rotation_errors_deg':angles,'independent_consistency_passed':True,
            'validated_for_motion':False,
            'required_next':'Physical scale/TCP verification and reviewed collision-free hover path'}
