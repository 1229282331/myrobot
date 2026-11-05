"""Validate a narrowly scoped CAD-backed reserve exit; no hardware writes."""
import math
import numpy as np


def validate_reply(reply, pending):
    proof=reply.get('margin_recovery') or {}
    check=reply.get('gripper_check') or {}
    if check.get('arm_safe') and check.get('arm_reason')=='finger_reserve_retreat' and proof.get('verified') is not True:
        raise ValueError('Missing reserve exit proof')
    if proof.get('verified') is not True:return
    joint=proof.get('joint')
    counts=pending.get('gripper_counts',[-1]*4)
    raw,sent,inflight,requested=counts
    if (proof.get('version')!=1 or type(joint) is not int or not 0<=joint<6 or
        pending['kind']!='j' or not pending['active'] or pending.get('gripper_hold')!=1 or
        raw<0 or inflight>=0 or (sent>=0 and abs(sent-raw)>1) or
        (requested>=0 and not (sent>=0 and requested==sent)) or reply['status']!=4 or
        check.get('allowed') is not False or check.get('arm_safe') is not True or
        check.get('arm_reason')!='finger_reserve_retreat' or proof.get('reason')!='finger_reserve_retreat'):
        raise ValueError('Invalid reserve exit provenance')
    for name,minimum in [('source_clearance_m',.002),('outward_derivative_m_rad',1e-5)]:
        value=proof.get(name)
        if type(value) not in (int,float) or not math.isfinite(value) or value<minimum:
            raise ValueError('Invalid reserve exit clearance')
    if type(proof.get('pairs')) is not int or proof['pairs']<=0:raise ValueError('Missing reserve pair coverage')
    delta=np.asarray(reply['joints_rad'])-np.asarray(pending['q'])
    requested=np.asarray(pending['effective_intent']['joint_rad_s'])
    active=np.flatnonzero(np.abs(requested)>1e-12)
    if (list(active)!=[joint] or delta[joint]*requested[joint]<=0 or
        abs(delta[joint])>math.radians(.025)+1e-12 or
        np.max(np.abs(np.delete(delta,joint)))>1e-12 or
        not np.allclose(reply['velocities_rad_s'],delta/.05,atol=1e-12,rtol=0)):
        raise ValueError('Invalid reserve exit direction or step')
    if check['arm_range_raw']!=[max(0,raw-1),min(255,raw+1)]:
        raise ValueError('Invalid reserve exit aperture interval')


def writer_allows(row, sample, gripper, now, last_target=None):
    """Reserve >=2 mm includes <=0.5 mm off-path feedback/target drift."""
    from .gripper_geometry import retreat_hold
    if retreat_hold(gripper,now)!=1:return False
    if not 0<=now-sample.get('received_ns',0)<=100_000_000:return False
    try:
        seed=np.asarray(row['cpp']['actual_q_rad'],dtype=float)
        actual=np.deg2rad(sample['status']['joint_state']['q_actual'])
        weights=np.array([1.,.9,.7,.5,.4,.3])
        for point in [actual]+([] if last_target is None else [np.asarray(last_target,dtype=float)]):
            if point.shape!=(6,) or not np.isfinite(point).all() or np.dot(np.abs(point-seed),weights)>.0005:return False
    except (KeyError,TypeError,ValueError):return False
    return True
