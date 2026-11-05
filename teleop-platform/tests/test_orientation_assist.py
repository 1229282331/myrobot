"""Assisted policy must preserve original ownership, expiry and motion gates."""
from copy import deepcopy
import math
import pytest
from pydantic import ValidationError
from e6_phosphobot.intent import Intent,Limits
from e6_phosphobot.cpp_cycle import CppCycle
from test_cpp_cycle import context,response

def assisted():
    return dict(linear_m_s=[.001,0,0],angular_rad_s=[0.]*3,joint_rad_s=[0.]*6,
        gripper_rate_s=0,deadman=True,frame='base',orientation_assist=True,assist_angular_rad_s=.01)

def test_old_intents_keep_strict_orientation_and_cannot_request_hidden_rotation():
    value=assisted();value.pop('orientation_assist');value.pop('assist_angular_rad_s')
    i=Intent(**value);assert not i.orientation_assist and Limits().accepts(i)
    assert not Limits().accepts(Intent(**dict(value,assist_angular_rad_s=.01)))

@pytest.mark.parametrize('changes',[{'angular_rad_s':[.001,0,0]}, {'joint_rad_s':[.001,0,0,0,0,0]},
    {'assist_angular_rad_s':0}, {'assist_angular_rad_s':.02}])
def test_assistance_is_translation_only_and_within_selected_capabilities(changes):
    assert not Limits().accepts(Intent(**dict(assisted(),**changes)))

@pytest.mark.parametrize('rate',[float('nan'),float('inf'),-.1,math.radians(3.01)])
def test_unbounded_assisted_rotation_is_invalid(rate):
    with pytest.raises(ValidationError):Intent(**dict(assisted(),assist_angular_rad_s=rate))

def reply(c):
    r=response(c);enabled=c.pending['kind']=='a'
    r['orientation_assist']=dict(version=1,enabled=enabled,max_angle_deg=15,
        max_rate_rad_s=.01 if enabled else 0,anchor_xyzw=[0,0,0,1],deviation_deg=0,
        reason='assisting' if enabled else 'inactive',twist_base=[.001,0,0,0,0,0] if enabled else [0]*6)
    return r

def pending():
    c=CppCycle(orientation_assist=True);now=1_000_000_000
    c.update(context(now,active=False));c.request(now);c.complete(reply(c),now+1)
    now+=50_000_000;ctx=context(now);ctx['command']['intent']=assisted();c.update(ctx)
    wire=c.request(now).split();assert wire[3]=='2' and wire[-2:]==['A1','0.01']
    return c,now

def test_assisted_binary_attestation_is_required():
    c=CppCycle();now=1_000_000_000;c.update(context(now,active=False));c.request(now);c.complete(response(c),now+1)
    ctx=context(now+50_000_000);ctx['command']['intent']=assisted();c.update(ctx)
    assert c.request(now+50_000_000) is None
    assert c.info['last_rejection']=='orientation_assist_unavailable'

def test_verified_assistance_is_recorded_separately_from_operator_rotation():
    c,now=pending();row=c.complete(reply(c),now+1)
    assert row['cpp']['kind']=='a' and row['cpp']['orientation_assist']['enabled']
    assert row['source_input']['intent']['angular_rad_s']==[0,0,0]
    assert row['valid_until_ns']==now+100_000_000

@pytest.mark.parametrize('corrupt',[{'enabled':False},{'version':2},{'max_rate_rad_s':.02},
    {'deviation_deg':16},{'twist_base':[.001,.0001,0,0,0,0]},
    {'twist_base':[.001,0,0,.011,0,0]},{'anchor_xyzw':[0,0,.1,1]}])
def test_forged_or_out_of_bounds_assistance_is_rejected(corrupt):
    c,now=pending();r=reply(c);r['orientation_assist'].update(corrupt)
    with pytest.raises(ValueError):c.complete(r,now+1)

@pytest.mark.parametrize('case',['release','late','collision','singularity'])
def test_assistance_cannot_override_existing_stop_gates(case):
    c,now=pending();r=reply(c);finish=now+1
    if case=='release':c.update(context(now,active=False))
    elif case=='late':finish=now+50_000_000
    elif case=='collision':r['status']=5
    else:r['status']=2
    assert c.complete(r,finish) is None

def test_switching_to_assistance_while_held_requires_a_new_neutral():
    from test_intent import message
    from e6_phosphobot.intent import IntentSession
    now=[1_000_000_000];s=IntentSession(clock=lambda:now[0])
    s.receive(message(s,1));s.receive(message(s,2,op='prepare',intent=None))
    original=assisted();original.update(orientation_assist=False,assist_angular_rad_s=0)
    assert s.receive(message(s,3,intent=original))
    assert not s.receive(message(s,4,intent=assisted()))
    assert s.ready and s.stop_requested and s.active() is None
    now[0]+=1_000_000
    assert s.receive(message(s,5))
    assert s.receive(message(s,6,intent=assisted()))

def test_final_target_cannot_misreport_its_assisted_angle():
    from scipy.spatial.transform import Rotation
    c,now=pending();r=reply(c)
    r['target_fk']['quaternion_xyzw']=Rotation.from_euler('x',16,degrees=True).as_quat().tolist()
    with pytest.raises(ValueError,match='angular envelope'):c.complete(r,now+1)
