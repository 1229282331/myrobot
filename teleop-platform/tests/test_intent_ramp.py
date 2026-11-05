from copy import deepcopy
import math
import numpy as np
import pytest
from e6_phosphobot.intent_ramp import IntentRamp
from e6_phosphobot.cpp_cycle import CppCycle
from test_cpp_cycle import context,response


def intent():
    return context(0)['command']['intent']


def test_step_matches_critical_damping_without_speed_overshoot():
    f=IntentRamp(.2);i=intent();values=[]
    for step in range(1,81):
        shaped,trace=f.step(i,1);values.append(shaped['joint_rad_s'][-1])
        t=step*.05;expected=.01*(1-(1+t/.2)*math.exp(-t/.2))
        assert values[-1]==pytest.approx(expected,abs=1e-15)
        assert shaped['linear_m_s']==[0]*3 and shaped['angular_rad_s']==[0]*3
    assert all(a<b<=.01 for a,b in zip(values,values[1:]))
    assert .95<values[19]/.01<1
    # Acceleration starts small, rises, then falls instead of stepping to a ceiling.
    acceleration=np.diff([0]+values)/.05
    assert acceleration[0]<acceleration[1]<acceleration[2]<acceleration[3]
    assert acceleration[10]<acceleration[3]
    assert i==intent()


def test_reversal_is_continuous_and_variable_vectors_remain_in_convex_speed_envelope():
    f=IntentRamp(.2);i=intent()
    for _ in range(30):f.step(i,1)
    prior=f.state.copy();i['joint_rad_s'][-1]=-.01
    shaped,trace=f.step(i,1)
    assert not trace['reset'] and 0<shaped['joint_rad_s'][-1]<prior[1,-1]
    rng=np.random.default_rng(252)
    f.reset()
    for _ in range(1000):
        vector=rng.normal(size=3);vector*=.03/np.linalg.norm(vector)
        i['linear_m_s']=vector.tolist();i['joint_rad_s']=[0]*6
        shaped,_=f.step(i,2)
        assert np.linalg.norm(shaped['linear_m_s'])<=.03+1e-14
        assert shaped['angular_rad_s']==[0]*3


def test_epoch_coordinate_and_release_reset_all_filter_history():
    f=IntentRamp(.2);i=intent();first=f.step(i,1)[0]
    for _ in range(20):f.step(i,1)
    assert f.step(i,2)[0]==first
    i['frame']='tool';changed,trace=f.step(i,2)
    assert trace['reset'] and changed['joint_rad_s']==first['joint_rad_s']
    f.reset();assert not f.state.any()


def test_release_bypasses_ramp_and_source_remains_auditable():
    c=CppCycle(input_ramp_time_constant_s=.2);now=1_000_000_000
    c.update(context(now,active=False));c.request(now);c.complete(response(c),now+1)
    now+=50_000_000;c.update(context(now));line=c.request(now)
    assert 0<float(line.split()[-1])<.001
    row=c.complete(response(c),now+1)
    assert row['source_input']['intent']['joint_rad_s'][-1]==.01
    assert row['source_input']['input_ramp']['reset']
    # Release inside the current 50 ms interval is not delayed by the filter.
    c.update(context(now+1_000_000,active=False));line=c.request(now+1_000_000)
    assert line.split()[2]=='0' and all(float(v)==0 for v in line.split()[-6:])
    assert not c.ramp.state.any()
    c.complete(response(c),now+1_000_001)
    c.update(context(now+50_000_000));c.request(now+50_000_000)
    assert c.pending['input_ramp']['reset']
    c.reject('solver_deadline');assert not c.ramp.state.any()


@pytest.mark.parametrize('corrupt',[False,True])
def test_audit_reconstructs_input_ramp_from_raw_input_and_prior_state(corrupt):
    import importlib.util
    from pathlib import Path
    path=Path(__file__).parents[1]/'scripts/audit_moveit_page.py'
    spec=importlib.util.spec_from_file_location('ramp_audit',path)
    audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    c=CppCycle(input_ramp_time_constant_s=.2);now=1_000_000_000
    c.update(context(now,active=False));c.request(now);c.complete(response(c),now+1)
    now+=50_000_000;c.update(context(now));c.request(now);row=c.complete(response(c),now+1)
    if corrupt:row['source_input']['input_ramp']['state'][1][-1]+=.001
    errors=audit.cpp_errors(row,{now:([0.]*6,[0,0,400,0,0,0])},{row['id']:row})
    assert bool(errors)==corrupt


@pytest.mark.parametrize('frame',['base','tool'])
@pytest.mark.parametrize('axis',[0,1,2],ids=['Rx','Ry','Rz'])
@pytest.mark.parametrize('direction',[-1,1],ids=['negative','positive'])
def test_rotation_reaches_solver_smoothly_and_release_discards_history(frame,axis,direction):
    """Verify the production request boundary, not the fake solver's motion."""
    from e6_phosphobot.motion_profiles import get_profile
    profile=get_profile('normal')
    c=CppCycle(tcp_profile='omnipicker-r4',
               input_ramp_time_constant_s=profile.input_ramp_time_constant_s)
    now=1_000_000_000
    c.update(context(now,active=False));c.request(now);c.complete(response(c),now+1)
    requested=direction*math.radians(profile.angular_deg_s)*.8

    def rotate(stamp,speed,epoch=1):
        ctx=context(stamp,epoch=epoch)
        ctx['command']['intent'].update(joint_rad_s=[0]*6,frame=frame,
            angular_rad_s=[speed if k==axis else 0. for k in range(3)])
        c.update(ctx)
        wire=c.request(stamp).split()
        assert wire[2:5]==['1','1',str(int(frame=='tool'))]
        twist=np.asarray(wire[-6:],dtype=float)
        assert np.array_equal(twist[:3],np.zeros(3))
        assert all(twist[3+k]==0 for k in range(3) if k!=axis)
        row=c.complete(response(c),stamp+1)
        assert row['source_input']['intent']==ctx['command']['intent']
        assert row['source_input']['effective_intent']['angular_rad_s']==twist[3:].tolist()
        assert row['source_input']['input_ramp']['time_constant_s']==.2
        return twist[3+axis]

    speeds=[]
    for step in range(20):
        now+=50_000_000
        speeds.append(rotate(now,requested))
    ratios=np.asarray(speeds)/requested
    assert np.all(np.diff(ratios)>0) and 0<ratios[0]<.03
    assert .95<ratios[-1]<1
    acceleration=np.diff([0.]+ratios)/.05
    assert acceleration[0]<acceleration[1]<acceleration[2]<acceleration[3]
    assert acceleration[-1]<acceleration[3]

    reversal=[]
    for _ in range(12):
        now+=50_000_000
        reversal.append(rotate(now,-requested)/requested)
    assert 0<reversal[0]<ratios[-1] and reversal[-1]<0
    assert np.all(np.diff(reversal)<0) and np.max(np.abs(reversal))<1

    # Releasing inside the 50 ms period must bypass the ramp immediately.
    c.update(context(now+1_000_000,active=False))
    released=c.request(now+1_000_000).split()
    assert released[2]=='0' and all(float(v)==0 for v in released[-6:])
    assert not c.ramp.state.any()
    c.complete(response(c),now+1_000_001)
    now+=50_000_000
    assert rotate(now,requested,epoch=2)==pytest.approx(speeds[0])
