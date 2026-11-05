"""Regression for held-input candidate skipping and bounded workspace approach."""
from copy import deepcopy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from test_gateway import setup
from test_moveit_bridge import Bridge,row
from test_cpp_cycle import initialized,context,response
from e6_phosphobot.gateway import wait_control_cycle
from e6_phosphobot.workspace_velocity import damp_translation


def test_uneven_arrival_of_uniform_solver_steps_never_alternates_one_and_two_steps():
    e,d,send,step,now=setup(moveit_bundle='fixture',motion_profile='normal');e.moveit=Bridge();e.moveit_api='cpp'
    origin=now[0]
    # Uniform 50 ms generation, variable callback/pipe arrival. The previous
    # second gate loses points whenever delay becomes smaller than last time.
    delays=[8,3,7,4,9,3,6,4,8,3,9,5]*5
    for i,delay in enumerate(delays):
        now[0]=origin+(i*50+delay)*1_000_000
        send(deadman=True,angular_rad_s=[0,0,np.deg2rad(2)])
        target=row(now[0],e.session.sequence);target['joints_rad'][5]=np.deg2rad(.05+i*.1)
        observed=max(0.,.05+(i-1)*.1)
        d.data['status']['joint_state'].update(q_actual=[0]*5+[observed],q_target=[0]*5+[observed])
        e.moveit.row=target;e.tick()
    sent=np.array([call[1][0][5] for call in d.calls if call[0]=='ServoJ'])
    assert len(sent)==len(delays)
    np.testing.assert_allclose(np.diff(sent),.1,atol=1e-12)
    send();assert d.calls[-1][0]=='Stop'


def test_candidate_notification_wakes_without_padding_but_health_timeout_remains():
    class Event:
        def __init__(self,signaled=False):self.signaled=signaled;self.waits=[]
        def wait(self,t):self.waits.append(t);return self.signaled
        def clear(self):self.signaled=False
    halt,wake=Event(),Event(True)
    wait_control_cycle(halt,wake,50_000_000,.05,55_000_000,clock=lambda:60_000_000,producer_paced=True)
    assert wake.waits==[.04] and not halt.waits


def test_fast_release_repress_does_not_burst_producer_and_delays_do_not_catch_up():
    c,now=initialized();c.update(context(now));c.request(now);c.complete(response(c),now+1)
    c.update(context(now+10_000_000,active=False));assert c.request(now+10_000_000)
    c.complete(response(c),now+10_000_001)
    c.update(context(now+20_000_000,epoch=2));assert c.request(now+20_000_000) is None
    c.update(context(now+50_000_000,epoch=2));assert c.request(now+50_000_000)
    c.complete(response(c),now+50_000_001)
    c.update(context(now+180_000_000,epoch=2));assert c.request(now+180_000_000)
    c.complete(response(c),now+180_000_001)
    assert c.request(now+180_000_002) is None


def intent(v):
    return dict(linear_m_s=v,angular_rad_s=[0,0,0],joint_rad_s=[0]*6,
                gripper_rate_s=0,deadman=True,frame='base')


def test_continuous_outward_input_approaches_without_crossing_or_stop_restart():
    raw=intent([.03,0,0]);saved=deepcopy(raw);p=np.zeros(3)
    velocities=[];positions=[]
    for _ in range(200):
        effective,report=damp_translation(raw,p,Rotation.identity(),dict(origin_m=[0,0,0],radius_m=.05))
        v=np.array(effective['linear_m_s']);p+=v*.05;velocities.append(v[0]);positions.append(p[0])
    assert raw==saved
    assert .0494<p[0]<.05
    assert max(positions)<.05 and min(velocities)>=0
    assert np.max(np.diff(velocities))<1e-12
    # Immediate inward retreat preserves its requested speed, not a latched
    # boundary Stop requiring neutral/reprepare.
    retreat=intent([-.03,0,0]);effective,report=damp_translation(retreat,p,Rotation.identity(),dict(origin_m=[0,0,0],radius_m=.05))
    assert effective==retreat and report is None


@pytest.mark.parametrize('frame',['base','tool'])
def test_oblique_input_direction_and_rotation_are_preserved(frame):
    r=Rotation.from_euler('z',90,degrees=True)
    raw=intent([.02,.01,0] if frame=='base' else r.inv().apply([.02,.01,0]).tolist())
    raw.update(frame=frame,angular_rad_s=[.01,.02,.03])
    eff,report=damp_translation(raw,[.049,0,0],r,dict(origin_m=[0,0,0],radius_m=.05))
    v=np.array(eff['linear_m_s']);v=r.apply(v) if frame=='tool' else v
    np.testing.assert_allclose(v,[.0015,.00075,0],atol=1e-12)
    assert eff['angular_rad_s']==raw['angular_rad_s'] and report['radius_m']==.05
    assert report['policy']=='preserve_input_direction'
    assert report['velocity_scale']==pytest.approx(.075)


@pytest.mark.parametrize('frame',['base','tool'])
@pytest.mark.parametrize('axis',[0,1,2])
@pytest.mark.parametrize('sign',[-1,1])
def test_single_axis_never_acquires_an_unrequested_axis_at_spherical_boundary(frame,axis,sign):
    rotation=Rotation.from_euler('xyz',[23,41,-18],degrees=True)
    v=np.zeros(3);v[axis]=sign*.03
    world=rotation.apply(v) if frame=='tool' else v
    # An oblique sphere normal, rather than one conveniently aligned to input.
    tangent=np.cross(world,[.3,.7,.2]);tangent*=np.linalg.norm(world)/np.linalg.norm(tangent)
    position=(world+tangent)/np.linalg.norm(world+tangent)*.049
    raw=intent(v.tolist());raw['frame']=frame
    eff,report=damp_translation(raw,position,rotation,dict(origin_m=[0,0,0],radius_m=.05))
    assert report is not None and 0<report['velocity_scale']<1
    assert all(eff['linear_m_s'][j]==0 for j in range(3) if j!=axis)
    assert np.sign(eff['linear_m_s'][axis])==sign
    # Reversing the SAME input is immediately permitted at full requested speed.
    raw['linear_m_s']=(-v).tolist()
    retreat,report=damp_translation(raw,position,rotation,dict(origin_m=[0,0,0],radius_m=.05))
    assert retreat==raw and report is None


def test_recorded_minus_y_input_does_not_slide_sideways():
    workspace=dict(origin_m=[.09522215933515633,-.31067052969243963,.40827273616554105],radius_m=.05)
    p=np.array([.084882124198796,-.3589795764965288,.40810320985478005])
    start=p.copy();raw=intent([0,-.012,0]);speeds=[]
    for _ in range(200):
        effective,report=damp_translation(raw,p,Rotation.identity(),workspace)
        v=np.array(effective['linear_m_s']);speeds.append(-v[1]);p+=.05*v
        assert np.linalg.norm(p-workspace['origin_m'])<.05
        assert p[0]==start[0] and p[2]==start[2]
    assert max(np.diff(speeds))<=1e-12


def test_tangent_and_inward_commands_and_pure_rotation_are_unchanged():
    for v in ([0,.02,0],[-.03,.02,0],[0,0,0]):
        raw=intent(v);raw['angular_rad_s']=[.01,.02,.03]
        effective,report=damp_translation(raw,[.049,0,0],Rotation.identity(),dict(origin_m=[0,0,0],radius_m=.05))
        assert effective==raw and report is None


def test_cpp_retains_raw_input_and_uses_damped_velocity_in_official_request():
    c,now=initialized();ctx=context(now);ctx['command']['intent']=intent([.03,0,0])
    ctx['command']['workspace']=dict(origin_m=[-.049,0,.4],radius_m=.05)
    c.update(ctx);request=list(map(float,c.request(now).split()))
    assert request[-6]==pytest.approx(.0015)
    record=c.complete(response(c),now+1)['source_input']
    assert record['intent']['linear_m_s']==[.03,0,0]
    assert record['effective_intent']['linear_m_s'][0]==pytest.approx(.0015)
    assert record['workspace_limit']['kind']=='tcp_outward_velocity_damper'
