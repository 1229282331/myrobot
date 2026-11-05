"""Directional Cartesian velocity damper for the existing TCP test sphere.

MoveIt still owns IK, smoothing, joint and collision limits. Reduce speed along
the requested direction near the site boundary; never slide along the sphere
by introducing axes the operator did not request.
"""
from copy import deepcopy
import numpy as np


def damp_translation(intent, position, rotation, workspace):
    effective=deepcopy(intent)
    if not workspace or not any(intent['linear_m_s']):return effective,None
    origin=np.asarray(workspace['origin_m'],dtype=float)
    radius=float(workspace['radius_m'])
    if origin.shape!=(3,) or not np.isfinite(origin).all() or not np.isfinite(radius) or radius<=0:
        raise ValueError('Invalid workspace velocity boundary')
    delta=np.asarray(position)-origin;distance=float(np.linalg.norm(delta))
    if distance<1e-12:return effective,None
    normal=delta/distance
    velocity=np.asarray(intent['linear_m_s'],dtype=float)
    if intent['frame']=='tool':velocity=rotation.apply(velocity)
    outward=float(velocity@normal)
    # At the normal 30 mm/s profile this starts 10.5 mm before the boundary.
    # A 0.5 mm inner reserve accommodates numerical/servo settling; the
    # independent candidate check retains the original hard sphere.
    reserve=min(.0005,radius*.01)
    permitted=3.*max(0.,radius-distance-reserve)
    if outward<=permitted:return effective,None
    scale=permitted/outward
    # Apply one scalar in the ORIGINAL input frame. Radial vector subtraction
    # changes a pure X/Y/Z command into sideways motion at an oblique boundary.
    # Scaling retains direction, exact zero axes and immediate inward retreat.
    effective['linear_m_s']=(np.asarray(intent['linear_m_s'],dtype=float)*scale).tolist()
    return effective,dict(kind='tcp_outward_velocity_damper',distance_m=distance,
        radius_m=radius,reserve_m=reserve,gain_s_inv=3.,
        outward_requested_m_s=outward,outward_permitted_m_s=permitted,
        policy='preserve_input_direction',velocity_scale=scale,
        reference_position_m=np.asarray(position).tolist(),
        reference_quaternion_xyzw=rotation.as_quat().tolist(),workspace=deepcopy(workspace))
