from copy import deepcopy
import math
import pytest
from e6_phosphobot.margin_recovery import validate_reply,writer_allows


def example():
    d=math.radians(.005)
    pending=dict(kind='j',active=True,gripper_hold=1,gripper_counts=[0,-1,-1,-1],q=[0.]*6,
        effective_intent=dict(joint_rad_s=[0,0,0,.01,0,0]))
    reply=dict(status=4,joints_rad=[0,0,0,d,0,0],velocities_rad_s=[0,0,0,d/.05,0,0],
        margin_recovery=dict(version=1,verified=True,joint=3,pairs=4,reason='finger_reserve_retreat',
            source_clearance_m=.003,outward_derivative_m_rad=.08),
        gripper_check=dict(arm_safe=True,allowed=False,arm_reason='finger_reserve_retreat',arm_range_raw=[0,1]))
    return pending,reply


def test_verified_outward_single_joint_step():
    p,r=example();validate_reply(r,p)


@pytest.mark.parametrize('bad',['missing','kind','inactive','unknown','moving','contact','inflight','commitment','status',
    'allowed','unverified','clearance','nan','derivative','pairs','step','reverse','multi','velocity','range'])
def test_bad_or_partial_proofs_fail_closed(bad):
    p,r=example();proof=r['margin_recovery'];check=r['gripper_check']
    if bad=='missing':r.pop('margin_recovery')
    elif bad=='kind':p['kind']='t'
    elif bad=='inactive':p['active']=False
    elif bad=='unknown':p['gripper_counts'][0]=-1
    elif bad=='moving':p['gripper_hold']=0
    elif bad=='contact':p['gripper_hold']=2
    elif bad=='inflight':p['gripper_counts'][2]=0
    elif bad=='commitment':p['gripper_counts'][3]=10
    elif bad=='status':r['status']=0
    elif bad=='allowed':check['allowed']=True
    elif bad=='unverified':proof['verified']=False
    elif bad=='clearance':proof['source_clearance_m']=.00199
    elif bad=='nan':proof['source_clearance_m']=float('nan')
    elif bad=='derivative':proof['outward_derivative_m_rad']=0
    elif bad=='pairs':proof['pairs']=0
    elif bad=='step':r['joints_rad'][3]=.01
    elif bad=='reverse':p['effective_intent']['joint_rad_s'][3]=-.01
    elif bad=='multi':r['joints_rad'][2]=.00001
    elif bad=='velocity':r['velocities_rad_s'][3]=.1
    elif bad=='range':check['arm_range_raw']=[0,0]
    with pytest.raises(ValueError):validate_reply(r,p)


def test_recovery_writer_checks_new_feedback_and_held_fingers():
    now=1_000_000_000
    grip=dict(valid=True,actual=0.,received_ns=now,phase='ready',reported_velocity_raw=0,reported_state=0,arrived=True)
    arm=dict(received_ns=now,status=dict(joint_state=dict(q_actual=[0.]*6)))
    row=dict(cpp=dict(actual_q_rad=[0.]*6))
    assert writer_allows(row,arm,grip,now)
    assert not writer_allows(row,arm,grip,now+100_000_001)
    bad=deepcopy(arm);bad['status']['joint_state']['q_actual'][3]=.1
    assert not writer_allows(row,bad,grip,now)
    assert not writer_allows(row,arm,dict(grip,reported_velocity_raw=1),now)
    assert not writer_allows(row,arm,dict(grip,reported_state=2),now)
    assert not writer_allows(row,arm,grip,now,last_target=[0,0,0,.002,0,0])
